/**
 * @file mtCANL4Engine.cpp
 * @brief High-reliability Segment-by-Segment Transport Engine for mtCAN Layer 4.
 * * Implements Layer 4 transport operations within the mtCAN architecture. 
 * Manages chunked data decomposition, transmission verification, retransmission loops, 
 * data integrity checking via CRC16, and link status telemetry tracking for active sessions.
 *
 * Detailed Architectural Rules:
 * 1. Priority Lane Constraint: Priorities 0 and 1 are reserved for system critical overrides. 
 * Applications must use lanes 2 through 7.
 * 2. Duplicate Session Restriction: Only one active connection can exist between any two nodes 
 * on the same priority lane at any given time.
 * 3. Link Failure Escalation: Telemetry checking operates via rolling windows. If a connection is 
 * silent for over 2.2 seconds, it escalates to an UNRESPONSIVE activity state, triggering incremental 
 * error alerts before a 10-second timeout disconnects the session entirely.
 */

#include "mtCANL4Engine.h"
#include "mtCANL3Engine.h"
#include <string.h>

// Industrial Error and Warning Constant Offsets

/**
 * @brief Telemetry Alert Offset Base. Real-time diagnostic warning reports use this value 
 * combined with the missed heartbeat count to provide live link degradation telemetry.
 */
#define L4_ERR_KEEPALIVE_MISS_BASE 0x7F00

/**
 * @brief Stream Segment Watchdog Exception Code. Triggered if an active multi-frame receiver 
 * process stalls without data activity for 5 full seconds.
 */
#define L4_ERR_RX_TIMEOUT          0x0009  

/**
 * @brief Core Engine Constructor. Establishes cross-layer links to Layer 3 routing 
 * structures and clears internal timing parameters.
 * @param l3Engine Underlying Layer 3 engine instance utilized to send and receive frames across the bus.
 */
mtCANL4Engine::mtCANL4Engine(mtCANL3Engine& l3Engine) : m_l3Engine(l3Engine), m_currentMillis(0) {
    init();
}

/**
 * @brief Global Allocation Table Initialization. Clears and prepares internal slot tracking 
 * fields to guarantee a deterministic system memory layout upon startup.
 */
void mtCANL4Engine::init() {
    for (int i = 0; i < L4_MAX_PARALLEL_SESSIONS; i++) {
        m_slots[i].clear();
        m_slots[i].lastControlActivityMs = 0;
        m_slots[i].lastRxSegmentTimestampMs = 0;
        m_slots[i].lastKeepAliveTxMs = 0;
        m_slots[i].lastReportedMissCount = 0;
    }
}

/**
 * @brief Pre-allocates an internal session slot to act as a passive listener, waiting for remote connection requests.
 * @param priority Targeted operational stream priority allocation lane (Constraint: Must reside in the 2-7 range).
 * @param rxBuffer Target RAM memory block assigned to receive incoming streaming data payloads safely.
 * @param bufferReserve Total capacity limit allocated to protect the receiver buffer space.
 * @param rxCb Callback function triggered when an incoming data stream completes successfully.
 * @param errCb Error callback function utilized for reporting link drops or diagnostic warnings.
 * @return Generated 32-bit unique passive application tracking token handle, or 0 if validation rules fail.
 */
uint32_t mtCANL4Engine::register_passive_listener(uint8_t priority, uint8_t* rxBuffer, uint32_t bufferReserve, L4_RxCallback rxCb, L4_ErrorCallback errCb) {
    // Constraint Rule Validation: Ensure requested priority lane sits within acceptable application bounds.
    if (priority < 2 || priority > 7) return 0;

    int8_t slotIdx = allocate_free_slot();
    if (slotIdx < 0) return 0;

    SessionSlot& slot = m_slots[slotIdx];
    slot.isAllocated = true;
    
    // Bit-Pack Token Structure: Bit 31 set identifies this handle as an inactive passive listener archetype.
    slot.associatedAppToken = 0x80000000 | (slotIdx << 16) | priority; 
    slot.netState = L4_NET_DISCONNECTED;
    slot.actState = L4_ACT_DISCONNECTED; 
    slot.priority = priority;
    slot.destNode = 0; 
    slot.rxBuf = rxBuffer;
    slot.rxBufMax = bufferReserve;
    slot.appRxCallback = rxCb;
    slot.appErrorCb = errCb;
    slot.keepAliveRetryCount = 0;
    slot.lastReportedMissCount = 0;
    slot.lastControlActivityMs = m_currentMillis;
    slot.lastRxSegmentTimestampMs = m_currentMillis;
    slot.lastKeepAliveTxMs = m_currentMillis;
    
    return slot.associatedAppToken;
}

/**
 * @brief Opens an active outbound communication pipeline targeting a specific remote node over the network bus.
 * Compiles structural handshakes and registers internal timers to verify target availability.
 * @param destNode Physical network device destination node address being targeted.
 * @param priority Priority arbitration value assigned to track this stream (Constraint: Must reside in the 2-7 range).
 * @param rxBuffer Target buffer space assigned to capture incoming response data streams.
 * @param bufferReserve Total capacity size limit tracking the provided receiver storage block.
 * @param rxCb Callback function triggered upon completing stream integrity checks.
 * @param errCb Diagnostic tracking callback used for connectivity error monitoring.
 * @return Generated active connection session application handle token, or 0 if rules are violated.
 */
uint32_t mtCANL4Engine::open_connection(uint8_t destNode, uint8_t priority, uint8_t* rxBuffer, uint32_t bufferReserve, L4_RxCallback rxCb, L4_ErrorCallback errCb) {
    // Constraint Rule Validation: Ensure priority assignment respects application boundary constraints.
    if (priority < 2 || priority > 7) return 0;

    // Constraint Rule Validation: Prevent concurrent duplicate sessions between identical nodes on the same priority.
    for (int i = 0; i < L4_MAX_PARALLEL_SESSIONS; i++) {
        if (m_slots[i].isAllocated && 
            m_slots[i].destNode == destNode && 
            m_slots[i].priority == priority &&
            m_slots[i].netState != L4_NET_DISCONNECTED) {
            return 0; 
        }
    }

    int8_t slotIdx = allocate_free_slot();
    if (slotIdx < 0) return 0;

    SessionSlot& slot = m_slots[slotIdx];
    slot.isAllocated = true;
    
    // Bit-Pack Token Structure: Bit 30 set identifies this handle as an active connection initiator.
    slot.associatedAppToken = 0x40000000 | (destNode << 8) | slotIdx;
    slot.netState = L4_NET_CONNECTING;
    slot.actState = L4_ACT_CONNECTING; 
    slot.destNode = destNode;
    slot.priority = priority;
    slot.rxBuf = rxBuffer;
    slot.rxBufMax = bufferReserve;
    slot.appRxCallback = rxCb;
    slot.appErrorCb = errCb;
    
    // Setup chronological retry thresholds using default timeout parameters
    slot.ackDeadlineMs = m_currentMillis + L4_DEFAULT_TIMEOUT_MS;
    slot.lastTxRxTimestampMs = m_currentMillis;
    slot.keepAliveRetryCount = 0;
    slot.lastReportedMissCount = 0;
    slot.lastControlActivityMs = m_currentMillis;
    slot.lastRxSegmentTimestampMs = m_currentMillis;
    slot.lastKeepAliveTxMs = m_currentMillis;
    
    // Assemble and pack the control payload for Handshake Step 1 (Connection Request)
    uint8_t ctrlPayload[2];
    ctrlPayload[0] = 0x80 | ((L4_CMD_CONN_OPEN_REQ & 0x0F) << 3) | (slotIdx & 0x07);
    ctrlPayload[1] = priority;
    
    // Send the control packet over the Layer 3 signaling plane
    m_l3Engine.tunnel_send_ctrl(slot.associatedAppToken, ctrlPayload, sizeof(ctrlPayload));
    
    return slot.associatedAppToken;
}

/**
 * @brief Mounts an active host data array and initiates chunked streaming transmission to a remote recipient node.
 * @param appToken Authorized active session token handle verifying an established connection.
 * @param data Source memory pointer to the data array scheduled for network transmission.
 * @param size Total length in bytes of the data file scheduled for transmission.
 * @return True if data validation passes and stream preparation begins; False if connection states are invalid.
 */
bool mtCANL4Engine::transmit_data(uint32_t appToken, const uint8_t* data, uint32_t size) {
    if (size == 0 || data == nullptr) return false;

    int8_t slotIdx = find_session_by_token(appToken);
    if (slotIdx < 0) return false;

    SessionSlot& slot = m_slots[slotIdx];
    if (slot.netState != L4_NET_CONNECTED) return false;
    
    // Transmission is only allowed if the channel is currently idle or recovering from an unresponsive state
    if (slot.actState != L4_ACT_CONNECTED && slot.actState != L4_ACT_IDLE && slot.actState != L4_ACT_UNRESPONSIVE) {
        return false;
    }

    // Bind source tracking handles to isolate state machine execution fields
    slot.txBuffer = data;
    slot.txTotalStreamSize = size;
    slot.txByteCursor = 0;
    slot.txNextSeq = 0;
    slot.retryCount = 0;
    
    slot.lastTxRxTimestampMs = m_currentMillis;
    
    // Initialize stream processing by transmitting metadata packet descriptors
    send_data_header(slot);
    return true;
}

/**
 * @brief Generates and transmits stream descriptive metadata parameters, including total sizes and CRC16 footprints.
 * Prepares the remote receiver node to verify allocation availability before data streaming begins.
 * @param slot Reference pointer mapping back to the tracking session structure context block.
 */
void mtCANL4Engine::send_data_header(SessionSlot& slot) {
    int8_t slotIdx = &slot - m_slots;
    
    // Calculate full-stream CRC data footprint validation tags
    uint16_t crc = compute_crc16(slot.txBuffer, slot.txTotalStreamSize);
    uint8_t headerPayload[7];
    
    // Formulate packet stream descriptor array layout parameters
    headerPayload[0] = 0x80 | ((L4_CMD_DATA_HEADER & 0x0F) << 3) | (slotIdx & 0x07);
    headerPayload[1] = (slot.txTotalStreamSize >> 24) & 0xFF;
    headerPayload[2] = (slot.txTotalStreamSize >> 16) & 0xFF;
    headerPayload[3] = (slot.txTotalStreamSize >> 8) & 0xFF;
    headerPayload[4] = slot.txTotalStreamSize & 0xFF;
    headerPayload[5] = (crc >> 8) & 0xFF;
    headerPayload[6] = crc & 0xFF;
    
    m_l3Engine.tunnel_send_ctrl(slot.associatedAppToken, headerPayload, sizeof(headerPayload));
    
    // Transition state tracking to wait for receiver memory confirmation flags
    slot.actState = L4_ACT_WAITING_FOR_HEADER_ACK;
    slot.ackDeadlineMs = m_currentMillis + L4_DEFAULT_TIMEOUT_MS;
    slot.lastControlActivityMs = m_currentMillis; 
}

/**
 * @brief Extends data boundaries, slices streaming bytes, and transmits a single data chunk over the bus.
 * Manages sequence numbering and sets timeout limits to handle retransmission requirements.
 * @param slot Reference mapping back to the session structure tracking data array parameters.
 */
void mtCANL4Engine::send_next_segment(SessionSlot& slot) {
    uint32_t remaining = slot.txTotalStreamSize - slot.txByteCursor;
    
    // Clamp chunk size strictly within maximum Layer 3 wire bounds
    uint8_t chunkSize = (remaining > L4_DATA_CHUNK_MAX_SIZE) ? L4_DATA_CHUNK_MAX_SIZE : remaining;

    uint8_t l4Packet[L4_MAX_L3_PAYLOAD_SIZE];
    
    // Inject the rolling 3-bit tracking sequence index into Byte 0
    l4Packet[0] = slot.txNextSeq & 0x07; 
    memcpy(&l4Packet[1], &slot.txBuffer[slot.txByteCursor], chunkSize);

    slot.actState = L4_ACT_WAITING_FOR_ACK;
    slot.ackDeadlineMs = m_currentMillis + L4_DEFAULT_TIMEOUT_MS;
    
    // Dispatch packet downstream onto the application data plane
    m_l3Engine.send(slot.associatedAppToken, l4Packet, chunkSize + 1);
    slot.lastControlActivityMs = m_currentMillis; 
}

/**
 * @brief Main processing engine handling incoming Layer 4 control frames.
 * Evaluates handshakes, validates session allocations, manages keep-alives, and handles data transmission state switches.
 * @param controlToken Raw CAN communication tracking token routing addressing data.
 * @param payload Pointer to the network byte buffer containing control structures.
 * @param size Total length in bytes of the incoming control packet.
 */
void mtCANL4Engine::handle_ingress_control(uint32_t controlToken, const uint8_t* payload, uint8_t size) {
    if (size == 0) return;
    uint8_t header = payload[0];

    // Decode control byte allocations using shift matrices
    uint8_t command = (header >> 3) & 0x0F;
    uint8_t idx = header & 0x07;

    // Route Open Requests through listening profiles to establish new sessions
    if (command == L4_CMD_CONN_OPEN_REQ && size >= 2) {
        uint8_t reqPriority = payload[1];
        int8_t listenerIdx = find_passive_listener(reqPriority);
        
        if (listenerIdx >= 0) {
            int8_t newSlotIdx = allocate_free_slot();
            uint8_t remoteNode = (controlToken >> 24) & 0xFF; 
            
            if (newSlotIdx >= 0) {
                // Duplicate Session Protection: Enforce separation rules before establishing the connection
                for (int i = 0; i < L4_MAX_PARALLEL_SESSIONS; i++) {
                    if (m_slots[i].isAllocated && 
                        m_slots[i].destNode == remoteNode && 
                        m_slots[i].priority == reqPriority &&
                        m_slots[i].netState != L4_NET_DISCONNECTED) {
                        
                        // Reject duplicate link connection requests immediately
                        uint32_t rejectToken = 0x40000000 | (remoteNode << 8) | 0xFF;
                        uint8_t resp[3];
                        resp[0] = 0x80 | ((L4_CMD_CONN_OPEN_RESP & 0x0F) << 3) | 0x07;
                        resp[1] = (L4_RESP_ERR_REJECTED >> 8) & 0xFF;
                        resp[2] = (L4_RESP_ERR_REJECTED & 0xFF);
                        m_l3Engine.tunnel_send_ctrl(rejectToken, resp, 3);
                        return;
                    }
                }

                // Bind listener configurations to initialize the new active connection slot
                SessionSlot& listener = m_slots[listenerIdx];
                SessionSlot& newSlot  = m_slots[newSlotIdx];
                
                newSlot.isAllocated = true;
                newSlot.destNode    = remoteNode;
                newSlot.priority    = reqPriority;
                newSlot.netState    = L4_NET_CONNECTED;
                newSlot.actState    = L4_ACT_CONNECTED; 
                
                newSlot.rxBuf         = listener.rxBuf;
                newSlot.rxBufMax      = listener.rxBufMax;
                newSlot.appRxCallback = listener.appRxCallback;
                newSlot.appErrorCb    = listener.appErrorCb;
                newSlot.keepAliveRetryCount = 0;
                newSlot.lastReportedMissCount = 0;
                
                newSlot.associatedAppToken = 0x40000000 | (newSlot.destNode << 8) | newSlotIdx;
                newSlot.lastTxRxTimestampMs = m_currentMillis; 
                newSlot.lastControlActivityMs = m_currentMillis; 
                newSlot.lastRxSegmentTimestampMs = m_currentMillis;
                newSlot.lastKeepAliveTxMs = m_currentMillis;
                
                // Transmit Step 2 Response: Acknowledge the connection open request
                uint8_t resp[8] = {0}; 
                resp[0] = 0x80 | ((L4_CMD_CONN_OPEN_RESP & 0x0F) << 3) | (newSlotIdx & 0x07);
                resp[1] = (L4_RESP_SUCCESS_OK >> 8) & 0xFF;
                resp[2] = L4_RESP_SUCCESS_OK & 0xFF;
                m_l3Engine.tunnel_send_ctrl(newSlot.associatedAppToken, resp, 8);
                
                // Notify the local application that a new channel connection has opened
                if (newSlot.appRxCallback) {
                    newSlot.appRxCallback(newSlot.associatedAppToken, nullptr, 0); 
                }
            } else {
                // Refuse connection if internal resources are exhausted and slots are full
                uint32_t rejectToken = 0x40000000 | (remoteNode << 8) | 0xFF;
                uint8_t resp[3];
                resp[0] = 0x80 | ((L4_CMD_CONN_OPEN_RESP & 0x0F) << 3) | 0x07; 
                resp[1] = (L4_RESP_ERR_REJECTED >> 8) & 0xFF;      
                resp[2] = (L4_RESP_ERR_REJECTED & 0xFF);          
                m_l3Engine.tunnel_send_ctrl(rejectToken, resp, 3);
            }
        }
        return;
    }

    int8_t slotIdx = find_session_by_token(controlToken);
    if (slotIdx < 0) return;

    // Route packet verification arrays to confirm slot identity parameters
    if (idx != (slotIdx & 0x07)) {
        return; 
    }

    SessionSlot& slot = m_slots[slotIdx];
    slot.lastControlActivityMs = m_currentMillis;

    // Reset telemetry metrics upon receiving valid activity from the remote node
    slot.keepAliveRetryCount = 0;
    slot.lastReportedMissCount = 0;
    
    if (slot.actState == L4_ACT_UNRESPONSIVE) {
        slot.actState = L4_ACT_CONNECTED; 
    }

    // Process specific Layer 4 functional control commands
    switch (command) {
        case L4_CMD_CONN_OPEN_RESP: {
            if (size >= 3) {
                uint16_t status = ((uint16_t)payload[1] << 8) | payload[2]; 
                if (status == L4_RESP_SUCCESS_OK) {
                    slot.netState = L4_NET_CONNECTED;
                    slot.actState = L4_ACT_CONNECTED; 
                } else if (status == L4_RESP_ERR_REJECTED) {
                    force_terminate_session(slotIdx, L4_RESP_ERR_REJECTED, false);
                }
            }
            break;
        }

        case L4_CMD_DATA_HEADER: {
            if (size >= 7) {
                // Extract target stream boundaries and integrity verification tags
                slot.rxTotalStreamSize = ((uint32_t)payload[1] << 24) | ((uint32_t)payload[2] << 16) | 
                                         ((uint32_t)payload[3] << 8)  | payload[4];
                slot.rxExpectedCRC16   = ((uint16_t)payload[5] << 8)  | payload[6];
                
                // Enforce buffer capacity tracking rules to prevent allocation overflows
                if (slot.rxTotalStreamSize > slot.rxBufMax) {
                    force_terminate_session(slotIdx, L4_ERR_BUFFER_OVERFLOW, true);
                    return;
                }

                // Reset stream indexing states to begin receiving segments
                slot.rxByteCursor = 0;
                slot.rxExpectedSeq = 0;
                slot.actState = L4_ACT_RECEIVING;
                slot.lastRxSegmentTimestampMs = m_currentMillis; 
                
                // Confirm metadata initialization by sending a header acknowledgment
                uint8_t ack[2];
                ack[0] = 0x80 | ((L4_CMD_DATA_HEADER_ACK & 0x0F) << 3) | (slotIdx & 0x07);
                ack[1] = 0x00;
                m_l3Engine.tunnel_send_ctrl(slot.associatedAppToken, ack, sizeof(ack));
            }
            break;
        }

        case L4_CMD_DATA_HEADER_ACK: {
            // Robustness Guard: Validate packet size allocation boundaries before checking headers
            if (size >= 2 && slot.actState == L4_ACT_WAITING_FOR_HEADER_ACK) {
                uint8_t status = payload[1];
                if (status == 0x00) {
                    slot.retryCount = 0;
                    send_next_segment(slot); // Authorization received; begin streaming segments
                } else {
                    force_terminate_session(slotIdx, L4_RESP_ERR_REJECTED, true);
                }
            }
            break;
        }

        case L4_CMD_DATA_ACK_NACK: {
            if (slot.actState == L4_ACT_WAITING_FOR_ACK && size >= 3) {
                uint8_t statusByte      = payload[1]; 
                uint8_t nextExpectedSeq = payload[2];
                uint8_t targetNextSeq   = (uint8_t)((slot.txNextSeq + 1) & 0x07);

                // Process successful segment confirmations (ACK)
                if (statusByte == 0x00 && nextExpectedSeq == targetNextSeq) {
                    uint32_t remaining = slot.txTotalStreamSize - slot.txByteCursor;
                    uint32_t lastChunkSize = (remaining > L4_DATA_CHUNK_MAX_SIZE) ? L4_DATA_CHUNK_MAX_SIZE : remaining;
                    
                    slot.txByteCursor += lastChunkSize;
                    slot.txNextSeq = (slot.txNextSeq + 1) & 0x07; 
                    slot.retryCount = 0;
                    slot.lastTxRxTimestampMs = m_currentMillis; 

                    if (slot.txByteCursor >= slot.txTotalStreamSize) {
                        // Stream data fully transmitted; waiting for final remote integrity checks
                    } else {
                        send_next_segment(slot);
                    }
                } 
                // Process sequence failure flags (NACK); initiate segment retransmissions
                else if (statusByte == 0x01 && nextExpectedSeq == slot.txNextSeq) {
                    if (++slot.retryCount <= L4_MAX_RETRANSMIT_ATTEMPTS) {
                        send_next_segment(slot);
                    } else {
                        force_terminate_session(slotIdx, L4_ERR_SEQUENCE_MISMATCH, true);
                    }
                }
            }
            break;
        }

        case L4_CMD_RX_INTEGRITY: {
            if (size >= 3) {
                uint16_t status = ((uint16_t)payload[1] << 8) | payload[2];
                if (status == L4_RESP_SUCCESS_OK) {
                    slot.txBuffer = nullptr; // Stream successfully delivered and verified by remote node
                    slot.lastTxRxTimestampMs = m_currentMillis; 
                } else {
                    force_terminate_session(slotIdx, status, false);
                }
            }
            break;
        }

        case L4_CMD_CONN_CLOSE_REQ: {
            uint8_t resp = 0x80 | ((L4_CMD_CONN_CLOSE_RESP & 0x0F) << 3) | (slotIdx & 0x07);
            m_l3Engine.tunnel_send_ctrl(slot.associatedAppToken, &resp, 1);
            force_terminate_session(slotIdx, L4_RESP_SUCCESS_OK, false);
            break;
        }

        case L4_CMD_CONN_CLOSE_RESP:
        case L4_CMD_RESET_CONNECTION: { 
            force_terminate_session(slotIdx, L4_RESP_SUCCESS_OK, false);
            break;
        }

        case L4_CMD_KEEP_ALIVE_REQ: {
            uint8_t echo[8] = {0};
            echo[0] = 0x80 | ((L4_CMD_KEEP_ALIVE_RESP & 0x0F) << 3) | (slotIdx & 0x07); 
            m_l3Engine.tunnel_send_ctrl(slot.associatedAppToken, echo, 8);
            break;
        }

        case L4_CMD_KEEP_ALIVE_RESP: {
            break;
        }

        default:
            break;
    }
}

/**
 * @brief High-throughput processing engine handling incoming application data segments.
 * Tracks rolling sequence metrics, reassembles chunks into application memory buffers, 
 * and handles error tracking for missing segments.
 * @param appToken Streaming identification handle token routing data.
 * @param payload Pointer to raw incoming network data bytes.
 * @param size Total length of the immediate data segment frame.
 */
void mtCANL4Engine::handle_ingress_data(uint32_t appToken, const uint8_t* payload, uint8_t size) {
    if (size == 0) return;
    
    int8_t slotIdx = find_session_by_token(appToken);
    if (slotIdx < 0) return;

    SessionSlot& slot = m_slots[slotIdx];
    if (slot.actState != L4_ACT_RECEIVING) return;

    slot.lastControlActivityMs = m_currentMillis;
    slot.lastRxSegmentTimestampMs = m_currentMillis;

    // Reset telemetry miss metrics upon receiving valid stream data frames
    slot.keepAliveRetryCount = 0;
    slot.lastReportedMissCount = 0;
    
    if (slot.actState == L4_ACT_UNRESPONSIVE) {
        slot.actState = L4_ACT_CONNECTED;
    }

    // Extract the sequence tracking index from Byte 0 of the packet payload
    uint8_t rxSeq = payload[0] & 0x07;
    uint8_t dataLen = size - 1;

    // Handle Duplicate Frames: If the received sequence matches the previous chunk, re-send the ACK
    uint8_t prevExpectedSeq = (uint8_t)((slot.rxExpectedSeq - 1) & 0x07);
    if (rxSeq == prevExpectedSeq) {
        uint8_t ack[3];
        ack[0] = 0x80 | ((L4_CMD_DATA_ACK_NACK & 0x0F) << 3) | (slotIdx & 0x07);
        ack[1] = 0x00;
        ack[2] = slot.rxExpectedSeq;
        m_l3Engine.tunnel_send_ctrl(slot.associatedAppToken, ack, sizeof(ack));
        return;
    }

    // Handle Out-of-Order Packets: If sequence numbers desynchronize, issue a NACK request to trigger retransmission
    if (rxSeq != slot.rxExpectedSeq) {
        uint8_t nack[3];
        nack[0] = 0x80 | ((L4_CMD_DATA_ACK_NACK & 0x0F) << 3) | (slotIdx & 0x07);
        nack[1] = 0x01; 
        nack[2] = slot.rxExpectedSeq;
        m_l3Engine.tunnel_send_ctrl(slot.associatedAppToken, nack, sizeof(nack));
        return;
    }

    // Enforce buffer boundaries to protect memory segments against overflow attempts
    if (slot.rxByteCursor + dataLen > slot.rxTotalStreamSize || slot.rxByteCursor + dataLen > slot.rxBufMax) {
        force_terminate_session(slotIdx, L4_ERR_BUFFER_OVERFLOW, true);
        return;
    }

    // Copy segment data into the allocated application memory buffer space
    memcpy(&slot.rxBuf[slot.rxByteCursor], &payload[1], dataLen);
    slot.rxByteCursor += dataLen;
    slot.rxExpectedSeq = (slot.rxExpectedSeq + 1) & 0x07; // Increment rolling 3-bit tracking sequence index

    // Construct and dispatch segment transmission confirmation flags
    uint8_t ack[8] = {0};
    ack[0] = 0x80 | ((L4_CMD_DATA_ACK_NACK & 0x0F) << 3) | (slotIdx & 0x07);
    ack[1] = 0x00; 
    ack[2] = slot.rxExpectedSeq; 

    m_l3Engine.tunnel_send_ctrl(slot.associatedAppToken, ack, 8);

    // Stream Completion Verification: Evaluate overall data integrity once all bytes are received
    if (slot.rxByteCursor >= slot.rxTotalStreamSize) {
        slot.lastTxRxTimestampMs = m_currentMillis; 
        
        // Compute CRC16 across the reassembled memory buffer
        uint16_t localCRC = compute_crc16(slot.rxBuf, slot.rxTotalStreamSize);
        uint8_t integrityPayload[3];
        integrityPayload[0] = 0x80 | ((L4_CMD_RX_INTEGRITY & 0x0F) << 3) | (slotIdx & 0x07);

        // Deliver stream to host callbacks if CRC16 check passes successfully
        if (localCRC == slot.rxExpectedCRC16) {
            integrityPayload[1] = (L4_RESP_SUCCESS_OK >> 8) & 0xFF;
            integrityPayload[2] = L4_RESP_SUCCESS_OK & 0xFF;
            m_l3Engine.tunnel_send_ctrl(slot.associatedAppToken, integrityPayload, sizeof(integrityPayload));
            
            if (slot.appRxCallback) {
                slot.appRxCallback(slot.associatedAppToken, slot.rxBuf, slot.rxTotalStreamSize);
            }
        } else {
            // Terminate session and report data corruption if CRC16 verification fails
            integrityPayload[1] = (L4_ERR_CHECKSUM_MISMATCH >> 8) & 0xFF;
            integrityPayload[2] = L4_ERR_CHECKSUM_MISMATCH & 0xFF;
            m_l3Engine.tunnel_send_ctrl(slot.associatedAppToken, integrityPayload, sizeof(integrityPayload));
            force_terminate_session(slotIdx, L4_ERR_CHECKSUM_MISMATCH, false);
        }
    }
}

/**
 * @brief Cyclically executed engine update check. Processes data timeouts, 
 * manages retransmission pacing, and monitors link telemetry watchdogs.
 * @param currentMillis Active high-resolution running system up-time monitor log expressed in milliseconds.
 */
void mtCANL4Engine::tick(uint32_t currentMillis) {
    m_currentMillis = currentMillis;

    // Iterate through all slots to process active timeout counters
    for (int i = 0; i < L4_MAX_PARALLEL_SESSIONS; i++) {
        if (!m_slots[i].isAllocated) continue;
        SessionSlot& slot = m_slots[i];

        // 1. Dedicated Rx Transfer Watchdog Protection (Segment Timeout)
        if (slot.actState == L4_ACT_RECEIVING) {
            if (m_currentMillis - slot.lastRxSegmentTimestampMs >= 5000) {
                force_terminate_session(i, L4_ERR_RX_TIMEOUT, true);
                continue; 
            }
        }

        // 2. Process active data transfer timeouts (Retransmit Strategy)
        if (slot.actState == L4_ACT_WAITING_FOR_ACK || slot.actState == L4_ACT_WAITING_FOR_HEADER_ACK) {
            if (m_currentMillis > slot.ackDeadlineMs) {
                if (++slot.retryCount <= L4_MAX_RETRANSMIT_ATTEMPTS) {
                    if (slot.actState == L4_ACT_WAITING_FOR_HEADER_ACK) {
                        send_data_header(slot);
                    } else {
                        send_next_segment(slot);
                    }
                } else {
                    force_terminate_session(i, L4_ERR_TIMEOUT, true);
                    continue;
                }
            }
        }

        // 3. Connection Handshake Timeout
        if (slot.netState == L4_NET_CONNECTING && m_currentMillis > slot.ackDeadlineMs) {
            force_terminate_session(i, L4_ERR_TIMEOUT, false);
            continue;
        }

        // 4. Industrial Link-Plane Activity & Telemetry Synchronization
        if (slot.netState == L4_NET_CONNECTED) {
            bool isConnectionOpener = ((slot.associatedAppToken & 0xF0000000) == 0x40000000);
            
            if (isConnectionOpener) {
                // Connection Initiator: Transmit periodic telemetry pings every 1000ms during quiet windows
                if (m_currentMillis - slot.lastKeepAliveTxMs >= 1000) {
                    uint8_t ping[8] = {0};
                    ping[0] = 0x80 | ((L4_CMD_KEEP_ALIVE_REQ & 0x0F) << 3) | (i & 0x07);
                    m_l3Engine.tunnel_send_ctrl(slot.associatedAppToken, ping, 8);
                    
                    slot.lastKeepAliveTxMs = m_currentMillis;
                }

                // Escalation Protocol: Transition to UNRESPONSIVE if silence exceeds 2200ms
                if (m_currentMillis - slot.lastControlActivityMs >= 2200) {
                    if (m_currentMillis - slot.lastControlActivityMs >= 10000) {
                        force_terminate_session(i, L4_ERR_TIMEOUT, true); // Tear down link after 10s of total silence
                        continue;
                    }

                    slot.actState = L4_ACT_UNRESPONSIVE;

                    uint32_t deadTimeOverBase = (m_currentMillis - slot.lastControlActivityMs) - 2200;
                    slot.keepAliveRetryCount = (deadTimeOverBase / 1000) + 1;

                    // Edge-Trigger Protection: Only trigger the error callback when the missed count increases
                    if (slot.keepAliveRetryCount > slot.lastReportedMissCount) {
                        slot.lastReportedMissCount = slot.keepAliveRetryCount;
                        if (slot.appErrorCb) {
                            slot.appErrorCb(slot.associatedAppToken, (uint16_t)(L4_ERR_KEEPALIVE_MISS_BASE + slot.keepAliveRetryCount));
                        }
                    }
                } else {
                    // Update standard idle or connection tracking metrics based on recent activity
                    if (m_currentMillis - slot.lastTxRxTimestampMs >= 1000) {
                        slot.actState = L4_ACT_IDLE;
                    } else {
                        slot.actState = L4_ACT_CONNECTED;
                    }
                }
            } else {
                // Passive Server Side Tracker: Monitor connection life without initiating pings
                if (m_currentMillis - slot.lastControlActivityMs >= 10000) {
                    force_terminate_session(i, L4_ERR_TIMEOUT, true); // Terminate session if silence crosses the 10s limit
                    continue;
                }
                
                // Server-Side Telemetry Escalation: Transition to UNRESPONSIVE if silence exceeds 2200ms
                if (m_currentMillis - slot.lastControlActivityMs >= 2200) {
                    slot.actState = L4_ACT_UNRESPONSIVE;

                    // Server-Side Edge-Trigger: Calculate missed intervals to report connection degradation
                    uint32_t deadTimeOverBase = (m_currentMillis - slot.lastControlActivityMs) - 2200;
                    uint16_t simulatedMissCount = (deadTimeOverBase / 1000) + 1;

                    if (simulatedMissCount > slot.lastReportedMissCount) {
                        slot.lastReportedMissCount = simulatedMissCount;
                        if (slot.appErrorCb) {
                            slot.appErrorCb(slot.associatedAppToken, (uint16_t)(L4_ERR_KEEPALIVE_MISS_BASE + simulatedMissCount));
                        }
                    }
                } else if (m_currentMillis - slot.lastTxRxTimestampMs >= 1000) {
                    slot.actState = L4_ACT_IDLE; 
                } else {
                    slot.actState = L4_ACT_CONNECTED;
                }
            }
        }
    }
}

/**
 * @brief Utility Scanners: Maps a unique 32-bit tracking token identifier back to its active matrix slot index.
 * @param token The application token handle to locate.
 * @return Internal table slot index position (0 to 3), or -1 if no matching allocated token is found.
 */
int8_t mtCANL4Engine::find_session_by_token(uint32_t token) {
    for (int i = 0; i < L4_MAX_PARALLEL_SESSIONS; i++) {
        if (m_slots[i].isAllocated && m_slots[i].associatedAppToken == token) {
            return i;
        }
    }
    return -1;
}

/**
 * @brief Utility Scanners: Locates an active passive listener slot awaiting incoming connection requests on a priority lane.
 * @param priority The targeted operational priority lane to check.
 * @return Internal table slot index position (0 to 3), or -1 if no matching listener is available.
 */
int8_t mtCANL4Engine::find_passive_listener(uint8_t priority) {
    for (int i = 0; i < L4_MAX_PARALLEL_SESSIONS; i++) {
        if (m_slots[i].isAllocated && 
            m_slots[i].netState == L4_NET_DISCONNECTED && 
            m_slots[i].priority == priority) {
            return i;
        }
    }
    return -1;
}

/**
 * @brief Utility Scanners: Searches the tracking array for an available, unallocated session slot.
 * @return Available internal table slot index position (0 to 3), or -1 if all system allocation channels are busy.
 */
int8_t mtCANL4Engine::allocate_free_slot() {
    for (int i = 0; i < L4_MAX_PARALLEL_SESSIONS; i++) {
        if (!m_slots[i].isAllocated) return i;
    }
    return -1;
}

/**
 * @brief Standard CRC16 Data Integrity Calculator. Uses bitwise shifting loops to 
 * generate mathematical verification signatures for data payloads.
 * @param data Pointer to the memory buffer array to evaluate.
 * @param length Absolute size in bytes of the target data block.
 * @return Calculated 16-bit unsigned validation signature code integer.
 */
uint16_t mtCANL4Engine::compute_crc16(const uint8_t* data, uint32_t length) {
    uint16_t crc = 0xFFFF;
    for (uint32_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else crc >>= 1;
        }
    }
    return crc;
}

/**
 * @brief Forcefully terminates a session channel slot, clears memory tracking parameters, 
 * and notifies host application error hooks.
 * @param slotIdx Target internal index position inside the session matrix table.
 * @param errorCode Diagnostic status code reported to application error callback hooks.
 * @param issueResetCommand Trigger flag determining if an emergency connection reset command is sent over the bus.
 */
void mtCANL4Engine::force_terminate_session(uint8_t slotIdx, uint16_t errorCode, bool issueResetCommand) {
    if (slotIdx >= L4_MAX_PARALLEL_SESSIONS) return;
    SessionSlot& slot = m_slots[slotIdx];
    
    // Dispatch an emergency link override reset packet if requested and the channel remains active
    if (issueResetCommand && slot.netState != L4_NET_DISCONNECTED) {
        uint8_t cmd = 0x80 | ((L4_CMD_RESET_CONNECTION & 0x0F) << 3) | (slotIdx & 0x07);
        m_l3Engine.tunnel_send_ctrl(slot.associatedAppToken, &cmd, 1);
    }
    
    // Trigger diagnostic callback notification hooks if errors are present
    if (errorCode != L4_RESP_SUCCESS_OK && slot.appErrorCb) {
        slot.appErrorCb(slot.associatedAppToken, errorCode);
    }
    
    // Clear slot states completely to prepare the structure for future allocations
    slot.clear();
    slot.lastControlActivityMs = 0;
    slot.lastRxSegmentTimestampMs = 0;
    slot.lastKeepAliveTxMs = 0;
    slot.lastReportedMissCount = 0;
    slot.netState = L4_NET_DISCONNECTED;
    slot.actState = L4_ACT_DISCONNECTED;
}