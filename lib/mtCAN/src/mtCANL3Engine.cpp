/**
 * @file mtCANL3Engine.cpp
 * @brief Production Implementation File for the mtCAN Layer 3 Protocol Stack.
 *
 * LAYMAN EXPLANATION:
 * This file is the muscle of our Layer 3 protocol engine. It does the heavy lifting:
 * slicing wide arrays into CAN-ready packets, checking bit masks to track chunks coming in,
 * updating telemetry data structures, protecting memory values via hardware interrupt disables (__disable_irq),
 * finding the oldest active slot to safely recycle ping allocations, and mapping global UNIX times 
 * down to human calendar dates using standard year leap algorithms.
 */

#include "mtCANL3Engine.h"

// External matrix grids defined in Layer 2 memory space that store active packets coming off the wire.
extern CanMatrixFrame g_ApplicationMatrix[TOTAL_APP_FRAMES];
extern CanMatrixFrame g_ControlMatrix[TOTAL_CTRL_FRAMES];

/**
 * @brief Construct a new mtCANL3Engine and automatically reset its internal trackers.
 */
mtCANL3Engine::mtCANL3Engine(mtCANL2Engine& l2Driver) : m_l2Engine(l2Driver) {
    init();
}

/**
 * @brief Clears out internal memory tracking states, diagnostics scoreboards, and active ping registers.
 */
void mtCANL3Engine::init() {
    m_baseUnixEpochSeconds = 0;
    m_baseEpochSyncMillis = 0;
    m_stallGraceStartMs = 0;
    m_isStallClockRunning = false;
    m_timeIsValid = false;

    // Reset tracking state variables across all available application stream slots
    for (uint8_t i = 0; i < TOTAL_APP_CONN; ++i) {
        m_appTrackers[i].lastFrameTimestampMs = 0;
        m_appTrackers[i].expectedFramesCount = 0;
        m_appTrackers[i].receivedFramesMask = 0;
        m_appTrackers[i].processedFramesMask = 0;

        m_laneStats[i].completedPackets = 0;
        m_laneStats[i].timeoutEvents = 0;
        m_laneStats[i].structuralMismatches = 0;
        m_laneStats[i].tokenInconsistencies = 0;
    }

    // Completely clear out the diagnostic ping monitoring array registers
    for (uint8_t i = 0; i < L3_MAX_PING_TRACKERS; ++i) {
        m_pingRegistry[i].echoVerificationToken = 0;
        m_pingRegistry[i].transmitTimestampUs = 0;
        m_pingRegistry[i].isActiveSlot = false;
    }
}


/**
 * @brief Slices a wide application buffer into sequential 8-byte frames and pushes them into the Layer 2 queue.
 * * LAYMAN: This is the core fragmentation algorithm. If you hand it a 20-byte string, it figures out that 
 * 3 physical frames are required. Frame 0 stores the total message length in byte 0, followed by 7 bytes of data. 
 * Frames 1 and 2 carry 8 data bytes each until completed. It loops through, packs the proper chunk sequence index 
 * into the 29-bit header, and passes them to Layer 2 to be queued.
 */
uint8_t mtCANL3Engine::send(uint32_t connection_token, const uint8_t* payload, uint8_t size) {
    if (!validate_outgress_token(connection_token)) return XCAN_L3_STATUS_ERR_INVALID_TOKEN;
    if (m_l2Engine.get_current_flow_state() == L2_FLOW_CONGESTED) return L3_STATUS_ERR_CONGESTED;
    if (size > 63 || (size > 0 && payload == nullptr)) return L3_STATUS_ERR_INVALID_SIZE;

    uint8_t totalFramesNeeded = calculate_expected_frames(size);
    uint8_t bytesRemaining = size;
    uint8_t payloadOffset = 0;

    // Zero out old plane and sequence fields, then bind the target application plane flags and local source address
    uint32_t baseId29 = connection_token & ~(XCAN_MASK_SEQUENCE | XCAN_MASK_MSG_TYPE);
    baseId29 |= ((uint32_t)XCAN_PLANE_APPLICATION << XCAN_SHIFT_MSG_TYPE) & XCAN_MASK_MSG_TYPE;
    baseId29 |= ((uint32_t)m_l2Engine.get_node_id() << XCAN_SHIFT_SRC_NODE) & XCAN_MASK_SRC_NODE;

    // Sequentially assemble and enqueue every frame piece required for this multi-frame block
    for (uint8_t seq = 0; seq < totalFramesNeeded; ++seq) {
        uint8_t frameBuffer[8];
        uint8_t framePayloadLen = 0;

        if (seq == 0) {
            // First frame structural signature layout requirement: Byte 0 contains the true package size
            frameBuffer[0] = size & 0x3F; 
            framePayloadLen = 1;

            uint8_t chunk = (bytesRemaining > 7) ? 7 : bytesRemaining;
            for (uint8_t i = 0; i < chunk; ++i) {
                frameBuffer[framePayloadLen++] = payload[payloadOffset++];
            }
            bytesRemaining -= chunk;
        } else {
            // Subsequent frames carry up to a full 8-byte payload capacity
            uint8_t chunk = (bytesRemaining > 8) ? 8 : bytesRemaining;
            for (uint8_t i = 0; i < chunk; ++i) {
                frameBuffer[framePayloadLen++] = payload[payloadOffset++];
            }
            bytesRemaining -= chunk;
        }

        // Stitch the specific sequence number into the lower 3 bits of this unique sub-frame identifier
        uint32_t frameTxId = baseId29 | ((uint32_t)seq & XCAN_MASK_SEQUENCE);
        
        // Pass to Layer 2 Application Ring Buffer
        if (!m_l2Engine.enqueue_app_frame(frameTxId, frameBuffer, framePayloadLen)) {
            return L3_STATUS_ERR_L2_FAILED;
        }
    }

    return L3_STATUS_OK;
}

/**
 * @brief Constructs and schedules a single-frame management message on the System/Control network plane.
 */
uint8_t mtCANL3Engine::send_ctrl(uint32_t connectionToken, L3CtrlType type, const uint8_t* payload, uint8_t size) {
    if (!validate_outgress_token(connectionToken)) return XCAN_L3_STATUS_ERR_INVALID_TOKEN;
    if (size > 7 || (size > 0 && payload == nullptr)) return L3_STATUS_ERR_INVALID_SIZE;

    uint32_t ctrlId29 = prepare_control_frame_id(connectionToken, 2); // Force priority = 2 for control plane compliance
    uint8_t frameBuffer[8];
    frameBuffer[0] = (uint8_t)type & 0x7F; // Command specifier resides inside the system plane header byte position

    for (uint8_t i = 0; i < size; ++i) {
        frameBuffer[i + 1] = payload[i];
    }

    // Pass directly to the Layer 2 Control Plane ring buffer array pipeline
    if (!m_l2Engine.enqueue_ctrl_frame(ctrlId29, frameBuffer, size + 1)) {
        return L3_STATUS_ERR_L2_FAILED;
    }

    return L3_STATUS_OK;
}

/**
 * @brief This is a special API that allows upper layers to tunnel control messages through the Layer 3 control interface.
 * * LAYMAN: This is a unique escape hatch that allows upper application layers to send small control messages 
 * (up to 7 bytes) through the Layer 3 control interface. It bypasses the standard command type byte and lets you 
 * pack your own custom payload directly into the control frame, which can be useful for advanced users who want 
 * to implement their own custom system plane protocols without being constrained by the predefined L3CtrlType enum.
 */
uint8_t mtCANL3Engine::tunnel_send_ctrl(uint32_t connectionToken, const uint8_t* payload, uint8_t size) {
    if (!validate_outgress_token(connectionToken)) return XCAN_L3_STATUS_ERR_INVALID_TOKEN;
    if (size > 8 || size < 1 || (payload == nullptr)) return L3_STATUS_ERR_INVALID_SIZE;
    if (payload[0] >> 7 == 0) return L3_STATUS_ERR_INVALID_CTRL_TYPE; // Ensure the first byte doesn't conflict with reserved command types

    uint32_t ctrlId29 = prepare_control_frame_id(connectionToken, 2); // Force priority = 2 for control plane compliance
    // Pass directly to the Layer 2 Control Plane ring buffer array pipeline
    if (!m_l2Engine.enqueue_ctrl_frame(ctrlId29, payload, size)) {
        return L3_STATUS_ERR_L2_FAILED;
    }
    return L3_STATUS_OK;
}

/**
 * @brief Generates and manages an outbound latency validation request (Ping).
 * * LAYMAN: This is where we safely find or recycle tracking resources. If all 4 tracking slots 
 * are currently active (waiting for responses), we parse our local list to locate the oldest 
 * active ping based on its timestamp. We select that slot, overwrite it with our new token details, 
 * and dispatch the command over the control plane wire.
 */
uint8_t mtCANL3Engine::send_ping(uint32_t connection_token, uint8_t internalSequenceId) {
    if (!validate_outgress_token(connection_token)) return XCAN_L3_STATUS_ERR_INVALID_TOKEN;
    
    uint8_t targetNode = (uint8_t)((connection_token & XCAN_MASK_DST_NODE) >> XCAN_SHIFT_DST_NODE);
    uint32_t combinedToken = ((uint32_t)targetNode << 16) | internalSequenceId;

    int8_t freeSlotIdx = -1;
    // Look for an explicitly open, empty slot registry item
    for (uint8_t i = 0; i < L3_MAX_PING_TRACKERS; ++i) {
        if (!m_pingRegistry[i].isActiveSlot) {
            freeSlotIdx = i;
            break;
        }
    }

    // NEW STALL TRIGGER: If no slots are open, reject new pings and engage the 500ms group grace clock
    if (freeSlotIdx == -1) {
        if (!m_isStallClockRunning) {
            m_stallGraceStartMs = (uint32_t)millis(); // Start the absolute 500ms countdown right now
            m_isStallClockRunning = true;
        }
        return L3_STATUS_ERR_CONGESTED; // Return congested to tell the upper app: "Engine is full, waiting for grace period"
    }

    // Configure the chosen tracking slot with full lifetime context
    uint32_t currentMs = (uint32_t)millis();
    m_pingRegistry[freeSlotIdx].echoVerificationToken = combinedToken;
    m_pingRegistry[freeSlotIdx].connectionToken = connection_token; // Save token context for error callbacks
    m_pingRegistry[freeSlotIdx].transmitTimestampUs = (uint32_t)micros(); // High resolution round-trip marker
    m_pingRegistry[freeSlotIdx].expirationTimestampMs = currentMs + L3_INTERFRAME_TIMEOUT_MS; // Individual timeout (500ms)
    m_pingRegistry[freeSlotIdx].isActiveSlot = true;

    // If we successfully populated a slot, check if we are no longer stalled
    // (If slots were cleared by incoming replies before hitting this code, turn off stall clock)
    bool stillFull = true;
    for (uint8_t i = 0; i < L3_MAX_PING_TRACKERS; ++i) {
        if (!m_pingRegistry[i].isActiveSlot) stillFull = false;
    }
    if (!stillFull) {
        m_isStallClockRunning = false;
    }

    uint8_t payload[4];
    payload[0] = (uint8_t)(combinedToken & 0xFF);
    payload[1] = (uint8_t)((combinedToken >> 8) & 0xFF);
    payload[2] = (uint8_t)((combinedToken >> 16) & 0xFF);
    payload[3] = (uint8_t)((combinedToken >> 24) & 0xFF);
    
    return send_ctrl(connection_token, L3_CTRL_PING_REQ, payload, 4);
}

void mtCANL3Engine::clear_ping_slot(uint8_t slotIdx) {
    if (slotIdx >= L3_MAX_PING_TRACKERS) return;

    m_pingRegistry[slotIdx].isActiveSlot = false;
    m_pingRegistry[slotIdx].echoVerificationToken = 0;
    m_pingRegistry[slotIdx].connectionToken = 0;
    m_pingRegistry[slotIdx].expirationTimestampMs = 0;
    m_pingRegistry[slotIdx].transmitTimestampUs = 0;
}

/**
 * @brief Dispatches global Unix seconds across the plane to calibrate remote node time baselines.
 */
uint8_t mtCANL3Engine::send_ntp_master(uint32_t connection_token, uint32_t unix_seconds) {
    uint8_t payload[4];
    payload[0] = (uint8_t)(unix_seconds & 0xFF);
    payload[1] = (uint8_t)((unix_seconds >> 8) & 0xFF);
    payload[2] = (uint8_t)((unix_seconds >> 16) & 0xFF);
    payload[3] = (uint8_t)((unix_seconds >> 24) & 0xFF);
    
    return send_ctrl(connection_token, XCAN_L3_CTRL_NTP_MASTER, payload, 4);
}

/**
 * @brief Updates the real-time clock tracker using standard calendar structural ranges.
 * * LAYMAN: Converts years, months, and days down into a raw global 32-bit second counter value. 
 * This contains strict boundary logic constraining years exclusively between 2000 and 2099 to eliminate 
 * any catastrophic calendar roll errors inside our Gregorian epoch calculation loops.
 */
uint8_t mtCANL3Engine::set_system_time(const mtTimeStructure& newTimeSetting) {
    // Tightened RTC Year Validation Fix: Reject any input calendar parameters outside 2000-2099 window
    if (newTimeSetting.year < 2000 || newTimeSetting.year > 2099) {
        return L3_RTC_ERR_INVALID_PARAMS;
    }
    // Perform standard chronological sanity boundary tracking checks
    if (newTimeSetting.seconds > 59 || newTimeSetting.minutes > 59 || newTimeSetting.hours > 23 ||
        newTimeSetting.day == 0 || newTimeSetting.day > 31 || newTimeSetting.month == 0 || newTimeSetting.month > 12) {
        return L3_RTC_ERR_INVALID_PARAMS;
    }

    uint16_t y = newTimeSetting.year;
    uint8_t m = newTimeSetting.month;
    if (m <= 2) {
        m += 12;
        y -= 1;
    }
    
    // Core conversion formula translating Gregorian dates directly into cumulative days since epoch
    uint32_t daysSinceEpoch = (365 * y) + (y / 4) - (y / 100) + (y / 400) + ((306 * (m + 1)) / 10) + newTimeSetting.day - 719468;
    uint32_t totalSecs = (daysSinceEpoch * 86400UL) + (newTimeSetting.hours * 3600UL) + (newTimeSetting.minutes * 60UL) + newTimeSetting.seconds;

    // Critical Section Protection: Temporarily block core microchip hardware interrupts while updating time variables
    __disable_irq();
    m_baseUnixEpochSeconds = totalSecs;
    m_baseEpochSyncMillis = (uint32_t)millis();
    m_timeIsValid = true;
    __enable_irq(); // Re-enable background interrupts safely

    return L3_RTC_STATUS_OK;
}

/**
 * @brief Directly configures base time trackers from a raw epoch second value.
 */
uint8_t mtCANL3Engine::set_system_time_from_epoch(uint32_t unixEpochSeconds) {
    __disable_irq();
    m_baseUnixEpochSeconds = unixEpochSeconds;
    m_baseEpochSyncMillis = (uint32_t)millis();
    m_timeIsValid = true;
    __enable_irq();
    return L3_RTC_STATUS_OK;
}

/**
 * @brief Extracts internal ticking seconds variables back into human-readable calendar structures.
 */
uint8_t mtCANL3Engine::get_system_time(mtTimeStructure& outputTargetBuffer) const {
    if (!m_timeIsValid) return L3_RTC_ERR_NOT_SYNCHRONIZED;

    // Calculate current running seconds since initial base NTP synchronization event anchor point
    uint32_t deltaMs = (uint32_t)millis() - m_baseEpochSyncMillis;
    uint32_t currentUnixTime = m_baseUnixEpochSeconds + (deltaMs / 1000UL);

    uint32_t secondsSinceDay = currentUnixTime % 86400UL;
    outputTargetBuffer.seconds = secondsSinceDay % 60;
    outputTargetBuffer.minutes = (secondsSinceDay / 60) % 60;
    outputTargetBuffer.hours = secondsSinceDay / 3600;

    uint32_t daysSinceEpoch = currentUnixTime / 86400UL;
    outputTargetBuffer.dayOfWeeks = (daysSinceEpoch + 4) % 7; // Epoch began on a Thursday (4)

    // Reverse-engineered algorithmic matrix translating raw days back into calendar date values
    uint32_t l = daysSinceEpoch + 68569 + 2440588;
    uint32_t n = (4 * l) / 146097;
    l = l - ((146097 * n) + 3) / 4;
    uint32_t i = (4000 * (l + 1)) / 1461001;
    l = l - ((1461 * i) / 4) + 31;
    uint32_t j = (80 * l) / 2447;
    outputTargetBuffer.day = l - ((2447 * j) / 80);
    l = j / 11;
    outputTargetBuffer.month = j + 2 - (12 * l);
    outputTargetBuffer.year = 100 * (n - 49) + i + l;

    return L3_RTC_STATUS_OK;
}

/**
 * @brief Scans and compiles fragmented multi-frame packets residing inside the application data matrix.
 * * LAYMAN: This is our background factory pipeline. It continually loops over our 16 parallel data lanes. 
 * If it notices that a lane has received data but hasn't updated within 500ms, it logs an Inter-Frame Timeout 
 * error and wipes the slot. If a packet is actively flowing, it verifies that the incoming sub-frames are numerical 
 * neighbors (Sequence check: 0, 1, 2...) and that their address tokens match perfectly. Once all bits in our tracking 
 * mask are turned on, the letter is complete! It extracts the contents into a final buffer and executes callback_rX.
 */
void mtCANL3Engine::process_application_rx_pipeline() {
    uint32_t currentTick = (uint32_t)millis();

    // Iterate across each parallel reception assembly channel track lane
    for (uint8_t laneIdx = 0; laneIdx < TOTAL_APP_CONN; ++laneIdx) {
        uint16_t baseMatrixIdx = laneIdx * 8;
        uint32_t f0_id29 = g_ApplicationMatrix[baseMatrixIdx].id29; // Check slot 0 of the current lane

        // If slot 0 is empty but the lane tracking shows uncompleted fragments exist, handle timeouts
        if (f0_id29 == 0) {
            if (m_appTrackers[laneIdx].receivedFramesMask > 0) {
                if ((currentTick - m_appTrackers[laneIdx].lastFrameTimestampMs) > L3_INTERFRAME_TIMEOUT_MS) {
                    m_laneStats[laneIdx].timeoutEvents++;
                    
                    // Attempt to locate an active ID in subsequent slots to pass accurate token context to error callback
                    uint32_t trackingRecoveryToken = 0;
                    for (uint8_t k = 0; k < 8; ++k) {
                        if (g_ApplicationMatrix[baseMatrixIdx + k].id29 != 0) {
                            trackingRecoveryToken = g_ApplicationMatrix[baseMatrixIdx + k].id29;
                            break;
                        }
                    }
                    if (trackingRecoveryToken == 0) {
                        trackingRecoveryToken = ((uint32_t)laneIdx << XCAN_SHIFT_DST_NODE);
                    }

                    // Alert upper level code of timeout exception
                    callback_error(trackingRecoveryToken, L3_ERR_INTER_FRAME_TIMEOUT);
                    
                    // Fully clear state trackers back to neutral
                    m_appTrackers[laneIdx].receivedFramesMask = 0;
                    m_appTrackers[laneIdx].expectedFramesCount = 0;
                    m_appTrackers[laneIdx].processedFramesMask = 0;

                    __disable_irq();
                    for (uint8_t i = 0; i < 8; ++i) {
                        g_ApplicationMatrix[baseMatrixIdx + i].id29 = 0;
                    }
                    __enable_irq();
                }
            }
            continue;
        }

        // Initialize state tracking details upon arrival of a brand new multi-frame series chunk 0
        if (m_appTrackers[laneIdx].expectedFramesCount == 0) {
            uint8_t l3Header = (uint8_t)(g_ApplicationMatrix[baseMatrixIdx].dataLow & 0xFF);
            
            // Safety Check: If the topmost bit is flipped, this is an illegal format configuration. Clear slot.
            if ((l3Header & 0x80) != 0) {
                __disable_irq();
                g_ApplicationMatrix[baseMatrixIdx].id29 = 0;
                __enable_irq();
                continue;
            }

            uint8_t l4Size = l3Header & 0x3F; // Extract target payload size bounds from bit field mapping
            m_appTrackers[laneIdx].expectedFramesCount = calculate_expected_frames(l4Size);
            m_appTrackers[laneIdx].lastFrameTimestampMs = currentTick;
            m_appTrackers[laneIdx].processedFramesMask = 0;
        }

        uint8_t currentMask = 0;
        uint32_t unifiedToken = f0_id29;
        bool structuralBreachDetected = false;
        bool frameTokenInconsistencyDetected = false;

        uint32_t baseProfileToken = f0_id29 & ~XCAN_MASK_SEQUENCE; // Base validation template containing fixed headers

        // Inspect and parse every expected slot allocation item inside this sequence lane
        for (uint8_t i = 0; i < m_appTrackers[laneIdx].expectedFramesCount; ++i) {
            uint32_t currentSlotId29 = g_ApplicationMatrix[baseMatrixIdx + i].id29;
            
            if (currentSlotId29 != 0) {
                // Verification A: Ensure the embedded frame sequence field strictly matches its physical array location index
                uint8_t actualSeq = (uint8_t)(currentSlotId29 & XCAN_MASK_SEQUENCE);
                if (actualSeq != i) {
                    m_laneStats[laneIdx].structuralMismatches++;
                    structuralBreachDetected = true;
                    break;
                }

                // Verification B: Confirm the source and routing identifiers have not changed mid-stream
                if ((currentSlotId29 & ~XCAN_MASK_SEQUENCE) != baseProfileToken) {
                    m_laneStats[laneIdx].tokenInconsistencies++;
                    frameTokenInconsistencyDetected = true;
                    break;
                }

                // Update bitfield marking this index item as validated and landed in local memory
                currentMask |= (1 << i);
            }
        }

        // Anomaly Handling Loop: Wipe lane and fire callback alerts if packet data corruption is found
        if (structuralBreachDetected || frameTokenInconsistencyDetected) {
            uint32_t anomalyToken = unifiedToken;
            if (anomalyToken == 0) {
                for (uint8_t k = 0; k < m_appTrackers[laneIdx].expectedFramesCount; ++k) {
                    if (g_ApplicationMatrix[baseMatrixIdx + k].id29 != 0) {
                        anomalyToken = g_ApplicationMatrix[baseMatrixIdx + k].id29;
                        break;
                    }
                }
            }
            if (anomalyToken == 0) anomalyToken = ((uint32_t)laneIdx << XCAN_SHIFT_DST_NODE);

            uint8_t errCode = structuralBreachDetected ? L3_ERR_SEQUENCE_CORRUPTION : XCAN_L3_ERR_TOKEN_INCONSISTENCY;
            callback_error(anomalyToken, errCode);
            
            m_appTrackers[laneIdx].receivedFramesMask = 0;
            m_appTrackers[laneIdx].expectedFramesCount = 0;
            m_appTrackers[laneIdx].processedFramesMask = 0;

            __disable_irq();
            for (uint8_t i = 0; i < 8; ++i) {
                g_ApplicationMatrix[baseMatrixIdx + i].id29 = 0;
            }
            __enable_irq();
            continue;
        }

        // Keep local timeout watch clocks active if new components are successfully filling slots
        if (currentMask != m_appTrackers[laneIdx].processedFramesMask) {
            uint8_t landedNewBits = currentMask & ~(m_appTrackers[laneIdx].processedFramesMask);
            if (landedNewBits != 0) {
                m_appTrackers[laneIdx].lastFrameTimestampMs = currentTick; 
                m_appTrackers[laneIdx].processedFramesMask = currentMask;
            }
        }
        
        m_appTrackers[laneIdx].receivedFramesMask = currentMask;
        uint8_t targetMask = (1 << m_appTrackers[laneIdx].expectedFramesCount) - 1; // Expected bitmask target matching total chunk limits

        // Success Condition: If current mask matches target mask, reassemble data and deliver up
        if (m_appTrackers[laneIdx].receivedFramesMask == targetMask) {
            uint8_t l4Size = (uint8_t)(g_ApplicationMatrix[baseMatrixIdx].dataLow & 0xFF) & 0x3F;
            uint8_t userBufferArray[64];
            uint8_t bytesCopied = 0;

            // Reconstruct total text stream across the individual CAN frame memory segments
            for (uint8_t i = 0; i < m_appTrackers[laneIdx].expectedFramesCount; ++i) {
                uint32_t dLow = g_ApplicationMatrix[baseMatrixIdx + i].dataLow;
                uint32_t dHigh = g_ApplicationMatrix[baseMatrixIdx + i].dataHigh;

                uint8_t framePayload[8];
                framePayload[0] = (uint8_t)(dLow & 0xFF);
                framePayload[1] = (uint8_t)((dLow >> 8) & 0xFF);
                framePayload[2] = (uint8_t)((dLow >> 16) & 0xFF);
                framePayload[3] = (uint8_t)((dLow >> 24) & 0xFF);
                framePayload[4] = (uint8_t)(dHigh & 0xFF);
                framePayload[5] = (uint8_t)((dHigh >> 8) & 0xFF);
                framePayload[6] = (uint8_t)((dHigh >> 16) & 0xFF);
                framePayload[7] = (uint8_t)((dHigh >> 24) & 0xFF);

                // Frame 0 skips byte position 0 because it was consumed by the protocol size header
                uint8_t startIdx = (i == 0) ? 1 : 0;
                for (uint8_t k = startIdx; k < 8; ++k) {
                    if (bytesCopied < l4Size) {
                        userBufferArray[bytesCopied++] = framePayload[k];
                    }
                }
            }

            m_laneStats[laneIdx].completedPackets++;
            callback_rX(unifiedToken, userBufferArray, l4Size); // Dispatch the full rebuilt letter up the stack!

            // Fully clear tracking context variables before enabling hardware registers to avoid race conditions
            m_appTrackers[laneIdx].receivedFramesMask = 0;
            m_appTrackers[laneIdx].expectedFramesCount = 0;
            m_appTrackers[laneIdx].processedFramesMask = 0;

            __disable_irq();
            for (uint8_t i = 0; i < 8; ++i) {
                g_ApplicationMatrix[baseMatrixIdx + i].id29 = 0;
            }
            __enable_irq();

        } else {
            // Cyclical evaluation backup checking if the timeout clock breached during partial fragment collection gaps
            if ((currentTick - m_appTrackers[laneIdx].lastFrameTimestampMs) > L3_INTERFRAME_TIMEOUT_MS) {
                m_laneStats[laneIdx].timeoutEvents++;
                callback_error(unifiedToken, L3_ERR_INTER_FRAME_TIMEOUT);
                
                m_appTrackers[laneIdx].receivedFramesMask = 0;
                m_appTrackers[laneIdx].expectedFramesCount = 0;
                m_appTrackers[laneIdx].processedFramesMask = 0;

                __disable_irq();
                for (uint8_t i = 0; i < 8; ++i) {
                    g_ApplicationMatrix[baseMatrixIdx + i].id29 = 0;
                }
                __enable_irq();
            }
        }
    }
}

/**
 * @brief Processes incoming system, clock, and diagnostic maintenance frames from the control queue.
 * * LAYMAN: This scans the circular queue managed by Layer 2. When a frame drops in, it parses the command byte.
 * - If it's a PING REQUEST, it builds a matching response packet, flips the routing token, and sends an echo back.
 * - If it's a PING RESPONSE, it grabs the current microsecond clock, finds the matching registry slot, clears it, 
 * calculates total round-trip microsecond latency, and delivers a report via callback_ping_report.
 * - If it's an NTP MASTER command, it updates our local calendar epoch reference baseline immediately.
 */
void mtCANL3Engine::process_control_rx_pipeline() {
    uint32_t currentTickMs = (uint32_t)millis();

    // ========================================================================
    // NEW BACKGROUND TRACKING ENGINE: SCAN FOR INDIVIDUAL AND GROUP STALL TIMEOUTS
    // ========================================================================
    
    // Condition A: If all slots are jammed and our 500ms group grace window expires, drop the hammer
    if (m_isStallClockRunning && (currentTickMs - m_stallGraceStartMs >= 500UL)) {
        for (uint8_t i = 0; i < L3_MAX_PING_TRACKERS; ++i) {
            if (m_pingRegistry[i].isActiveSlot) {
                uint8_t extractedSequenceId = (uint8_t)(m_pingRegistry[i].echoVerificationToken & 0xFF);
                
                // Fire the callback to upper layers returning the 0xFFFFFFFF sentinel error code
                callback_ping_report(m_pingRegistry[i].connectionToken, extractedSequenceId, 0xFFFFFFFFUL);
                
                // Pristine Cleanup
                clear_ping_slot(i);
            }
        }
        m_isStallClockRunning = false; // Reset the master countdown gate
    }
    // Condition B: Routine check for individual item timeouts when not under full group stall freeze
    else {
        for (uint8_t i = 0; i < L3_MAX_PING_TRACKERS; ++i) {
            if (m_pingRegistry[i].isActiveSlot && (currentTickMs >= m_pingRegistry[i].expirationTimestampMs)) {
                uint8_t extractedSequenceId = (uint8_t)(m_pingRegistry[i].echoVerificationToken & 0xFF);
                
                // Report single item death
                callback_ping_report(m_pingRegistry[i].connectionToken, extractedSequenceId, 0xFFFFFFFFUL);
                
                 // Pristine Cleanup
                clear_ping_slot(i);            }
        }
    }

    // Process incoming queue messages from the bus wire
    while (m_l2Engine.m_ctrlMatrixTail != m_l2Engine.m_ctrlMatrixHead) {
        uint8_t tailIdx = m_l2Engine.m_ctrlMatrixTail;
        uint32_t sysId29 = g_ControlMatrix[tailIdx].id29;
        
        uint32_t dLow = g_ControlMatrix[tailIdx].dataLow;
        uint32_t dHigh = g_ControlMatrix[tailIdx].dataHigh;

        uint8_t ctrlPayload[8];
        ctrlPayload[0] = (uint8_t)(dLow & 0xFF);
        ctrlPayload[1] = (uint8_t)((dLow >> 8) & 0xFF);
        ctrlPayload[2] = (uint8_t)((dLow >> 16) & 0xFF);
        ctrlPayload[3] = (uint8_t)(dLow >> 24);
        ctrlPayload[4] = (uint8_t)(dHigh & 0xFF);
        ctrlPayload[5] = (uint8_t)((dHigh >> 8) & 0xFF);
        ctrlPayload[6] = (uint8_t)((dHigh >> 16) & 0xFF);
        ctrlPayload[7] = (uint8_t)(dHigh >> 24);

        uint8_t primaryHeaderByte = ctrlPayload[0];

        if ((primaryHeaderByte & 0x80) != 0) {
            callback_control_plane(sysId29, &ctrlPayload[1], 7);
        } else {
            uint8_t enumCommandType = primaryHeaderByte & 0x7F;

            if (enumCommandType == L3_CTRL_PING_REQ) {
                uint8_t routingMode = (uint8_t)((sysId29 & XCAN_MASK_MODE) >> XCAN_SHIFT_MODE);
                uint8_t sourceNodeId = (uint8_t)((sysId29 & XCAN_MASK_SRC_NODE) >> XCAN_SHIFT_SRC_NODE);
                
                if (routingMode == XCAN_MODE_UNICAST && sourceNodeId <= 63) {
                    uint32_t incomingReplayToken = ((uint32_t)ctrlPayload[4] << 24) | 
                                                   ((uint32_t)ctrlPayload[3] << 16) | 
                                                   ((uint32_t)ctrlPayload[2] << 8)  | 
                                                   ((uint32_t)ctrlPayload[1]);

                    uint32_t echoOutToken = convert_ingress_to_outgress(sysId29);
                    
                    uint8_t echoPayload[4];
                    echoPayload[0] = (uint8_t)(incomingReplayToken & 0xFF);
                    echoPayload[1] = (uint8_t)((incomingReplayToken >> 8) & 0xFF);
                    echoPayload[2] = (uint8_t)((incomingReplayToken >> 16) & 0xFF);
                    echoPayload[3] = (uint8_t)((incomingReplayToken >> 24) & 0xFF);

                    send_ctrl(echoOutToken, L3_CTRL_PING_RESP, echoPayload, 4);
                }
            } else if (enumCommandType == L3_CTRL_PING_RESP) {
                uint32_t receivedToken = ((uint32_t)ctrlPayload[4] << 24) | 
                                         ((uint32_t)ctrlPayload[3] << 16) | 
                                         ((uint32_t)ctrlPayload[2] << 8)  | 
                                         ((uint32_t)ctrlPayload[1]);
                uint32_t rightNowUs = (uint32_t)micros(); 

                // Match response to an active registry item
                for (uint8_t i = 0; i < L3_MAX_PING_TRACKERS; ++i) {
                    if (m_pingRegistry[i].isActiveSlot && m_pingRegistry[i].echoVerificationToken == receivedToken) {
                        uint32_t latencyCalculated = rightNowUs - m_pingRegistry[i].transmitTimestampUs;
                        uint8_t extractedSequenceId = (uint8_t)(receivedToken & 0xFF);
                        
                        // Execute success callback passing back the calculated microsecond RTT
                        callback_ping_report(m_pingRegistry[i].connectionToken, extractedSequenceId, latencyCalculated);
                        
                        // Pristine Cleanup
                        clear_ping_slot(i);
                        
                        // Turn off group stall timer if a clean reply freed a slot naturally
                        m_isStallClockRunning = false; 
                        break;
                    }
                }
            } else if (enumCommandType == XCAN_L3_CTRL_NTP_MASTER) {
                uint32_t masterSecondsEpoch = ((uint32_t)ctrlPayload[4] << 24) | 
                                              ((uint32_t)ctrlPayload[3] << 16) | 
                                              ((uint32_t)ctrlPayload[2] << 8)  | 
                                              ((uint32_t)ctrlPayload[1]);
                set_system_time_from_epoch(masterSecondsEpoch);
            } else {
                callback_ctrl(sysId29, enumCommandType, &ctrlPayload[1], 7);
            }
        }

        m_l2Engine.m_ctrlMatrixTail = (tailIdx + 1) & (TOTAL_CTRL_FRAMES - 1);
    }
}
    