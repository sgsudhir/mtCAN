/**
 * @file mtCAN.cpp
 * @brief Application Connection Manager (ACM) Implementation.
 * Governs full composition layer orchestration, blocking boots, and shutdowns.
 */

#include "mtCAN.h"

// Static routing instance reference to capture external C-style Interrupt vectors
static mtCAN* g_acmInstance = nullptr;

// Safe fallback tick stub to completely immunize the stack from Null-Pointer hardware faults
static uint32_t stub_tick_provider() {
    return 0;
}

extern "C" {
    static void static_l4_rx_bridge(uint32_t appToken, const uint8_t* payload, uint32_t size) {
        if (g_acmInstance) g_acmInstance->forward_l4_rx(appToken, payload, size);
    }
    static void static_l4_err_bridge(uint32_t appToken, uint16_t errorCode) {
        if (g_acmInstance) g_acmInstance->forward_l4_err(appToken, errorCode);
    }
}

// Composition initialization sequences downstream layers tightly via reference binding pass-through
mtCAN::mtCAN(uint8_t localNodeId, HardwareTickCallback tickProvider) 
    : m_localNodeId(localNodeId),
      m_getSystemTimeMs(tickProvider ? tickProvider : stub_tick_provider),
      m_cachedMillis(0),
      m_lastSpeedCalcMs(0),
      m_previousWindowBytes(0),
      m_l2(),
      m_l3(m_l2), 
      m_l4(m_l3),
      m_globalTelemetry{0, 0, 0, 0, 0, 0, BUS_OFFLINE} 
{
    // Robustness Improvement: Guarantee complete internal initialization across all channel structures
    for (int i = 0; i < L4_MAX_PARALLEL_SESSIONS; i++) {
        m_channels[i].hdl = (conn_hdl_t)i;
        m_channels[i].isAllocated = false;
        reset_channel_struct(m_channels[i]);
    }

    // Validation Guard: Lock the object state out of the bus grid if the tick provider is null
    if (tickProvider == nullptr) {
        m_globalTelemetry.busState = BUS_FAULT;
    }

    // Singleton Protection Guard: Enforce single-node instantiation rules safely without undefined states
    if (g_acmInstance != nullptr) {
        m_globalTelemetry.busState = BUS_FAULT;
        return; 
    }
    
    // Assign instance routing token only if this is the validated primary allocation
    if (m_globalTelemetry.busState != BUS_FAULT) {
        g_acmInstance = this;
    }
}

/**
 * @brief Synchronous, sequential bottom-up protocol startup engine.
 */
bool mtCAN::connect_to_bus() {
    // Architectural Guard: Prevent duplicate initialization or fault bypass of a running stack
    if (m_globalTelemetry.busState == BUS_FAULT || m_globalTelemetry.busState == BUS_ONLINE) {
        return (m_globalTelemetry.busState == BUS_ONLINE);
    }

    // 1. Fire Layer 2 Hardware with a 5000ms allocation settling threshold boundary
    bool l2Status = m_l2.initialize_hardware(m_localNodeId, 5000);
    if (!l2Status) {
        m_globalTelemetry.busState = BUS_FAULT;
        return false;
    }

    // 2. Clear out L3 network registries, Unix epochs, and ping matrices
    m_l3.init();

    // 3. Purge and assign clean execution matrices for L4 segments
    m_l4.init();

    m_cachedMillis = m_getSystemTimeMs();
    m_lastSpeedCalcMs = m_cachedMillis;
    m_globalTelemetry.busState = BUS_ONLINE;
    
    return true;
}

/**
 * @brief Synchronous, sequential top-down network termination logic.
 */
bool mtCAN::disconnect_from_bus() {
    if (m_globalTelemetry.busState == BUS_FAULT) return false;
    m_globalTelemetry.busState = BUS_OFFLINE;

    // 1. Force down active handle connections gracefully at application bounds
    for (int i = 0; i < L4_MAX_PARALLEL_SESSIONS; i++) {
        if (m_channels[i].isAllocated) {
            disconnect((conn_hdl_t)i);
        }
    }

    // 2. Shut down transport window allocations completely
    m_l4.init();
    m_l3.init();

    // 3. Put peripheral registers to low power mode and separate interrupt vectors
    m_l2.shutdown_hardware();
    
    return true;
}

conn_hdl_t mtCAN::connect(uint8_t destNode, uint8_t priority, bool autoConnect,
                          uint8_t* rxBuffer, uint32_t rxBufferSize,
                          uint32_t remoteRequestBufferSize,
                          ACM_RxCallback rxCb, ACM_ErrorCallback errCb,
                          ACMSendMode sendMode) {
    if (m_globalTelemetry.busState != BUS_ONLINE) return (conn_hdl_t)ACM_ERR_LINK_DROPPED;
    if (rxBuffer == nullptr || rxBufferSize == 0) return (conn_hdl_t)ACM_ERR_INVALID_HANDLE;
    
    conn_hdl_t hdl = allocate_channel();
    if (hdl < 0) return (conn_hdl_t)ACM_ERR_BUFFER_OVERFLOW;
    
    ACMConnection& ch = m_channels[hdl];
    ch.destNode = destNode;
    ch.priority = priority;
    ch.autoConnect = autoConnect;
    ch.defaultSendMode = sendMode;
    ch.rxBuffer = rxBuffer;
    ch.rxBufferSize = rxBufferSize;
    ch.remoteRequestBufferSize = remoteRequestBufferSize;
    ch.onDataArrive = rxCb;
    ch.onError = errCb;
    ch.reconnectPending = false; 
    
    ch.state = ACM_STATE_CONNECTING;
    ch.telemetry.state = ACM_STATE_CONNECTING;
    ch.lastActivityMs = m_getSystemTimeMs();
    ch.reconnectAttemptStartMs = ch.lastActivityMs;
    
    ch.currentL4Token = m_l4.open_connection(destNode, priority, rxBuffer, rxBufferSize, 
                                             static_l4_rx_bridge, static_l4_err_bridge);
    
    if (ch.currentL4Token == 0) {
        ch.isAllocated = false;
        ch.state = ACM_STATE_OFFLINE;
        return (conn_hdl_t)ACM_ERR_BUSY;
    }

    uint32_t startWait = m_getSystemTimeMs();
    m_cachedMillis = startWait;

    while (ch.state == ACM_STATE_CONNECTING) {
        m_cachedMillis = m_getSystemTimeMs(); 
        m_l2.process_tx_management();
        m_l4.tick(m_cachedMillis);
        
        if ((m_cachedMillis - startWait) >= 5000) { 
            m_l4.close_connection(ch.currentL4Token);
            ch.state = ACM_STATE_DISCONNECTING;
            ch.telemetry.state = ACM_STATE_DISCONNECTING;
            
            uint32_t abortWaitStart = m_cachedMillis;
            while (ch.state == ACM_STATE_DISCONNECTING) {
                m_cachedMillis = m_getSystemTimeMs();
                m_l2.process_tx_management();
                m_l4.tick(m_cachedMillis);
                if ((m_cachedMillis - abortWaitStart) >= 2000) {
                    m_l4.abort_transfer(ch.currentL4Token, 0x0009);
                    break; 
                }
            }

            // Cleanup Improvement: Clear out all tracking counters immediately on timeout failures
            m_globalTelemetry.nodeTotalErrorsCaught++;
            ch.isAllocated = false;
            reset_channel_struct(ch);
            return (conn_hdl_t)ACM_ERR_TIMEOUT;
        }
    }
    
    return hdl;
}

ACMError mtCAN::send_data(conn_hdl_t hdl, const uint8_t* payload, uint32_t size) {
    if (hdl < 0 || hdl >= L4_MAX_PARALLEL_SESSIONS || !m_channels[hdl].isAllocated) return ACM_ERR_INVALID_HANDLE;
    ACMConnection& ch = m_channels[hdl];
    
    if (ch.state != ACM_STATE_CONNECTED) return ACM_ERR_LINK_DROPPED;
    if (ch.isTxActive) return ACM_ERR_BUSY;
    
    ch.isTxActive = true;
    ch.txExpectedTotalSize = size;
    ch.txBytesConfirmed = 0;
    ch.telemetry.currentTxProgressPct = 0;
    ch.txAttemptsCount++; 
    
    bool l4Status = m_l4.transmit_data(ch.currentL4Token, payload, size);
    if (!l4Status) {
        ch.isTxActive = false;
        ch.txFailuresCount++;
        if (ch.txAttemptsCount > 0) {
            ch.telemetry.txErrorRate = ((float)ch.txFailuresCount / (float)ch.txAttemptsCount) * 100.0f;
        }
        ch.lastActivityMs = m_getSystemTimeMs(); 
        return ACM_ERR_BUSY;
    }
    
    if (ch.defaultSendMode == ACM_SEND_BLOCKING) {
        uint32_t startWait = m_getSystemTimeMs();
        m_cachedMillis = startWait;

        while (ch.isTxActive) {
            m_cachedMillis = m_getSystemTimeMs();
            m_l2.process_tx_management();
            m_l4.tick(m_cachedMillis);
            
            if ((m_cachedMillis - startWait) >= 10000) { 
                ch.isTxActive = false;
                ch.txFailuresCount++;
                if (ch.txAttemptsCount > 0) {
                    ch.telemetry.txErrorRate = ((float)ch.txFailuresCount / (float)ch.txAttemptsCount) * 100.0f;
                }
                m_l4.abort_transfer(ch.currentL4Token, 0x0009);
                ch.lastActivityMs = m_cachedMillis; 
                return ACM_ERR_TIMEOUT;
            }
        }
        if (ch.state != ACM_STATE_CONNECTED) return ACM_ERR_LINK_DROPPED;
    }
    
    return ACM_SUCCESS;
}

ACMError mtCAN::async_send(conn_hdl_t hdl, const uint8_t* payload, uint32_t size) {
    if (hdl < 0 || hdl >= L4_MAX_PARALLEL_SESSIONS || !m_channels[hdl].isAllocated) return ACM_ERR_INVALID_HANDLE;
    ACMConnection& ch = m_channels[hdl];
    
    if (ch.state != ACM_STATE_CONNECTED) return ACM_ERR_LINK_DROPPED;
    if (ch.isTxActive) return ACM_ERR_BUSY;
    
    ch.isTxActive = true;
    ch.txExpectedTotalSize = size;
    ch.txBytesConfirmed = 0;
    ch.telemetry.currentTxProgressPct = 0;
    ch.txAttemptsCount++;
    
    bool l4Status = m_l4.transmit_data(ch.currentL4Token, payload, size);
    if (!l4Status) {
        ch.isTxActive = false;
        ch.txFailuresCount++;
        if (ch.txAttemptsCount > 0) {
            ch.telemetry.txErrorRate = ((float)ch.txFailuresCount / (float)ch.txAttemptsCount) * 100.0f;
        }
        ch.lastActivityMs = m_getSystemTimeMs();
        return ACM_ERR_BUSY;
    }
    
    return ACM_SUCCESS;
}

ACMError mtCAN::disconnect(conn_hdl_t hdl) {
    if (hdl < 0 || hdl >= L4_MAX_PARALLEL_SESSIONS || !m_channels[hdl].isAllocated) return ACM_ERR_INVALID_HANDLE;
    ACMConnection& ch = m_channels[hdl];
    
    ch.state = ACM_STATE_DISCONNECTING;
    ch.lastActivityMs = m_getSystemTimeMs(); 
    m_l4.close_connection(ch.currentL4Token);
    
    uint32_t startWait = m_getSystemTimeMs();
    m_cachedMillis = startWait;
    bool isGraceful = true;

    while (ch.state == ACM_STATE_DISCONNECTING) {
        m_cachedMillis = m_getSystemTimeMs();
        m_l2.process_tx_management();
        m_l4.tick(m_cachedMillis);
        if ((m_cachedMillis - startWait) >= 2000) {
            m_l4.abort_transfer(ch.currentL4Token, 0x0009);
            isGraceful = false; 
            break;
        }
    }
    
    ch.isAllocated = false;
    reset_channel_struct(ch);
    
    return isGraceful ? ACM_SUCCESS : ACM_ERR_TIMEOUT;
}

void mtCAN::tick(uint32_t currentMillis) {
    m_cachedMillis = currentMillis;
    
    if (m_globalTelemetry.busState != BUS_ONLINE) return;

    m_l2.process_tx_management();
    m_l4.tick(m_cachedMillis);
    
    for (int i = 0; i < L4_MAX_PARALLEL_SESSIONS; i++) {
        ACMConnection& ch = m_channels[i];
        if (!ch.isAllocated) continue;
        
        if (ch.state == ACM_STATE_RECONNECT_BACKOFF) {
            if ((m_cachedMillis - ch.backoffTimerStartMs) >= ch.backoffDurationMs) {
                ch.state = ACM_STATE_CONNECTING;
                ch.telemetry.state = ACM_STATE_CONNECTING;
                ch.telemetry.reconnectAttempts++; 
                ch.reconnectPending = true; 
                ch.reconnectAttemptStartMs = m_cachedMillis; 
                ch.lastActivityMs = m_cachedMillis; 
                
                ch.currentL4Token = m_l4.open_connection(ch.destNode, ch.priority, ch.rxBuffer, ch.rxBufferSize,
                                                         static_l4_rx_bridge, static_l4_err_bridge);
                if (ch.currentL4Token == 0) {
                    ch.state = ACM_STATE_RECONNECT_BACKOFF;
                    ch.telemetry.state = ACM_STATE_RECONNECT_BACKOFF;
                    ch.reconnectPending = false; 
                    ch.backoffTimerStartMs = m_cachedMillis;
                    ch.backoffDurationMs = (ch.backoffDurationMs * 2 > 60000) ? 60000 : ch.backoffDurationMs * 2;
                }
            }
        }
        else if (ch.state == ACM_STATE_CONNECTING && ch.autoConnect && ch.reconnectAttemptStartMs != 0) {
            if ((m_cachedMillis - ch.reconnectAttemptStartMs) >= 5000) {
                // Tracked Cleanup: Synchronously tear down down-stack transport loops to prevent orphan callbacks
                m_l4.abort_transfer(ch.currentL4Token, 0x0009);
                m_globalTelemetry.nodeTotalErrorsCaught++;
                
                ch.currentL4Token = 0;
                ch.reconnectPending = false; 
                ch.state = ACM_STATE_RECONNECT_BACKOFF;
                ch.telemetry.state = ACM_STATE_RECONNECT_BACKOFF;
                ch.backoffTimerStartMs = m_cachedMillis;
                ch.backoffDurationMs = (ch.backoffDurationMs * 2 > 60000) ? 60000 : ch.backoffDurationMs * 2;
                ch.lastActivityMs = m_cachedMillis; 
            }
        }
    }
    
    recalculate_throughput();
}

void mtCAN::forward_l4_rx(uint32_t appToken, const uint8_t* payload, uint32_t size) {
    int8_t idx = find_channel_by_token(appToken);
    if (idx < 0) return;
    
    ACMConnection& ch = m_channels[idx];
    ch.lastActivityMs = m_cachedMillis; 
    
    if (payload == nullptr) {
        if (ch.isTxActive) {
            ch.isTxActive = false;
            ch.txBytesConfirmed = ch.txExpectedTotalSize;
            ch.telemetry.currentTxProgressPct = 100; 
            
            ch.telemetry.totalTxBytes += ch.txExpectedTotalSize;
            ch.telemetry.payloadsSent++;
            m_globalTelemetry.nodeTotalTxBytes += ch.txExpectedTotalSize;
            m_globalTelemetry.nodeTotalTxPackets += ((ch.txExpectedTotalSize + 61) / 62);
            
            if (ch.txAttemptsCount > 0) {
                ch.telemetry.txErrorRate = ((float)ch.txFailuresCount / (float)ch.txAttemptsCount) * 100.0f;
            }
        }
        return; 
    }

    if (ch.state == ACM_STATE_CONNECTING) {
        if (ch.reconnectPending) {
            ch.telemetry.reconnectCount++; 
            ch.reconnectPending = false; 
        }
        ch.state = ACM_STATE_CONNECTED;
        ch.telemetry.state = ACM_STATE_CONNECTED;
        ch.reconnectAttemptStartMs = 0; 
        ch.telemetry.lastConnectTimestampMs = m_cachedMillis;
        ch.backoffDurationMs = 5000;
        ch.currentTransferRxBytes = 0; 
        ch.telemetry.currentRxProgressPct = 0;
        return;
    }
    
    if (ch.currentTransferRxBytes == 0) {
        ch.telemetry.currentRxProgressPct = 0;
    }

    ch.telemetry.totalRxBytes += size;
    ch.telemetry.payloadsReceived++;
    m_globalTelemetry.nodeTotalRxBytes += size;
    m_globalTelemetry.nodeTotalRxPackets += ((size + 61) / 62);
    
    ch.currentTransferRxBytes += size;
    if (ch.remoteRequestBufferSize > 0) {
        uint32_t currentProgress = (uint32_t)((ch.currentTransferRxBytes * 100) / ch.remoteRequestBufferSize);
        ch.telemetry.currentRxProgressPct = (currentProgress > 100) ? 100 : (uint8_t)currentProgress;
    } else {
        ch.telemetry.currentRxProgressPct = 100;
    }
    
    if (ch.onDataArrive) {
        ch.onDataArrive(ch.hdl, payload, size);
    }

    if (ch.currentTransferRxBytes >= ch.remoteRequestBufferSize) {
        ch.currentTransferRxBytes = 0;
    }
}

void mtCAN::forward_l4_err(uint32_t appToken, uint16_t errorCode) {
    int8_t idx = find_channel_by_token(appToken);
    if (idx < 0) return;
    
    ACMConnection& ch = m_channels[idx];
    ch.lastActivityMs = m_cachedMillis; 
    m_globalTelemetry.nodeTotalErrorsCaught++;
    ch.currentTransferRxBytes = 0; 
    ch.telemetry.currentRxProgressPct = 0;
    ch.reconnectAttemptStartMs = 0;
    ch.reconnectPending = false;
    
    ACMError translatedErr = ACM_ERR_LINK_DROPPED;
    if (errorCode == 0x0003) translatedErr = ACM_ERR_CRC_MISMATCH;
    if (errorCode == 0x0009) translatedErr = ACM_ERR_TIMEOUT;
    
    if (ch.isTxActive) {
        ch.isTxActive = false;
        ch.txFailuresCount++;
        if (ch.txAttemptsCount > 0) {
            ch.telemetry.txErrorRate = ((float)ch.txFailuresCount / (float)ch.txAttemptsCount) * 100.0f; 
        }
    }

    if (ch.state == ACM_STATE_CONNECTING || ch.state == ACM_STATE_DISCONNECTING) {
        if (!ch.autoConnect) {
            ch.state = ACM_STATE_OFFLINE;
            ch.telemetry.state = ACM_STATE_OFFLINE;
            ch.currentL4Token = 0;
            return;
        }
    }
    
    ch.telemetry.lastDisconnectTimestampMs = m_cachedMillis;
    
    if (ch.onError) {
        ch.onError(ch.hdl, translatedErr, errorCode);
    }
    
    if (ch.autoConnect) {
        ch.state = ACM_STATE_RECONNECT_BACKOFF;
        ch.telemetry.state = ACM_STATE_RECONNECT_BACKOFF;
        ch.currentL4Token = 0; 
        ch.backoffTimerStartMs = m_cachedMillis;
    } else {
        ch.state = ACM_STATE_OFFLINE;
        ch.telemetry.state = ACM_STATE_OFFLINE;
        ch.currentL4Token = 0;
    }
}

ChannelTelemetry mtCAN::get_channel_telemetry(conn_hdl_t hdl) {
    if (hdl < 0 || hdl >= L4_MAX_PARALLEL_SESSIONS) return ChannelTelemetry{};
    return m_channels[hdl].telemetry;
}

ConnectionMatrix mtCAN::get_connection_matrix() {
    ConnectionMatrix mx;
    mx.activeConnectionsCount = 0;
    
    for (int i = 0; i < L4_MAX_PARALLEL_SESSIONS; i++) {
        if (m_channels[i].isAllocated) {
            // Matrix Telemetry Mapping: Protect users from misinterpreting the state machine during backoffs
            ACMState visibleState = m_channels[i].state;
            
            mx.rows[mx.activeConnectionsCount] = {
                m_channels[i].destNode,
                m_channels[i].priority,
                visibleState,
                m_channels[i].currentL4Token
            };
            mx.activeConnectionsCount++;
        }
    }
    return mx;
}

conn_hdl_t mtCAN::allocate_channel() {
    for (int i = 0; i < L4_MAX_PARALLEL_SESSIONS; i++) {
        if (!m_channels[i].isAllocated) {
            m_channels[i].isAllocated = true;
            return (conn_hdl_t)i;
        }
    }
    return -1;
}

int8_t mtCAN::find_channel_by_token(uint32_t token) {
    for (int i = 0; i < L4_MAX_PARALLEL_SESSIONS; i++) {
        if (m_channels[i].isAllocated && m_channels[i].currentL4Token == token) return i;
    }
    return -1;
}

void mtCAN::reset_channel_struct(ACMConnection& ch) {
    ch.destNode = 0;
    ch.priority = 0;
    ch.autoConnect = false;
    ch.state = ACM_STATE_OFFLINE;
    ch.currentL4Token = 0;
    ch.backoffTimerStartMs = 0;
    ch.backoffDurationMs = 5000;
    ch.reconnectAttemptStartMs = 0;
    ch.lastActivityMs = 0;
    ch.isTxActive = false;
    ch.txExpectedTotalSize = 0;
    ch.txBytesConfirmed = 0;
    ch.currentTransferRxBytes = 0;
    ch.txAttemptsCount = 0;
    ch.txFailuresCount = 0;
    ch.lastWindowChannelBytes = 0;
    ch.reconnectPending = false;
    ch.rxBuffer = nullptr;
    ch.rxBufferSize = 0;
    ch.remoteRequestBufferSize = 0;
    ch.onDataArrive = nullptr;
    ch.onError = nullptr;
    ch.telemetry = {ACM_STATE_OFFLINE, 0, 0, 0, 0, 0, 0, 0, 0, 0.0f, 0, 0, 0};
}

void mtCAN::recalculate_throughput() {
    if ((m_cachedMillis - m_lastSpeedCalcMs) >= 1000) {
        uint64_t currentLifetimeBytes = m_globalTelemetry.nodeTotalTxBytes + m_globalTelemetry.nodeTotalRxBytes;
        if (currentLifetimeBytes >= m_previousWindowBytes) {
            m_globalTelemetry.nodeAverageDataRate = (uint32_t)(currentLifetimeBytes - m_previousWindowBytes);
        }
        m_previousWindowBytes = currentLifetimeBytes;
        
        for (int i = 0; i < L4_MAX_PARALLEL_SESSIONS; i++) {
            ACMConnection& ch = m_channels[i];
            if (ch.isAllocated) {
                uint64_t channelCurrentBytes = ch.telemetry.totalTxBytes + ch.telemetry.totalRxBytes;
                if (channelCurrentBytes >= ch.lastWindowChannelBytes) {
                    ch.telemetry.averageSpeedBytesPerSec = (uint32_t)(channelCurrentBytes - ch.lastWindowChannelBytes);
                }
                ch.lastWindowChannelBytes = channelCurrentBytes;
            }
        }
        m_lastSpeedCalcMs = m_cachedMillis;
    }
}