/**
 * @file mtCAN.h
 * @brief Application Connection Manager (ACM) for the mtCAN Architecture.
 * Orchestrates encapsulated, sequential lower-layer initializations and shutdowns.
 */

#ifndef MT_CAN_ACM_H
#define MT_CAN_ACM_H

#pragma once

#include "mtCANTypes.h"
#include "mtCANL2Engine.h"
#include "mtCANL3Engine.h"
#include "mtCANL4Engine.h"

typedef uint32_t (*HardwareTickCallback)();
typedef int8_t conn_hdl_t;

enum ACMState : uint8_t {
    ACM_STATE_OFFLINE,
    ACM_STATE_CONNECTING,
    ACM_STATE_CONNECTED,
    ACM_STATE_DISCONNECTING,
    ACM_STATE_RECONNECT_BACKOFF
};

enum ACMSendMode : uint8_t {
    ACM_SEND_BLOCKING,
    ACM_SEND_NON_BLOCKING
};

enum ACMError : int16_t {
    ACM_SUCCESS             = 0,
    ACM_ERR_TIMEOUT         = -1,
    ACM_ERR_CRC_MISMATCH    = -2,
    ACM_ERR_BUFFER_OVERFLOW = -3,
    ACM_ERR_BUSY            = -4,
    ACM_ERR_INVALID_HANDLE  = -5,
    ACM_ERR_LINK_DROPPED    = -6
};

enum ACMBusState : uint8_t {
    BUS_OFFLINE,
    BUS_ONLINE,
    BUS_FAULT,
    BUS_RECOVERY
};

typedef void (*ACM_RxCallback)(conn_hdl_t hdl, const uint8_t* payload, uint32_t size);
typedef void (*ACM_ErrorCallback)(conn_hdl_t hdl, ACMError error, uint16_t rawL4Code);

inline uint32_t system_ms_tick_provider() {
    return millis(); 
}

struct ChannelTelemetry {
    ACMState state;
    uint32_t reconnectAttempts;         
    uint32_t reconnectCount;            
    uint64_t totalTxBytes;
    uint64_t totalRxBytes;
    uint32_t payloadsSent;
    uint32_t payloadsReceived;
    uint32_t lastConnectTimestampMs;
    uint32_t lastDisconnectTimestampMs;
    float    txErrorRate;               
    uint32_t averageSpeedBytesPerSec;   
    uint8_t  currentTxProgressPct;      // Indication of transaction completion boundary only (0% or 100%)
    uint8_t  currentRxProgressPct;      
};

struct GlobalNodeTelemetry {
    uint64_t nodeTotalTxBytes;
    uint64_t nodeTotalRxBytes;
    uint32_t nodeTotalTxPackets;
    uint32_t nodeTotalRxPackets;
    uint32_t nodeTotalErrorsCaught;
    uint32_t nodeAverageDataRate;       
    ACMBusState busState;
};

struct ConnectionMatrixRow {
    uint8_t    destNode;
    uint8_t    priority;
    ACMState   state;
    uint32_t   activeToken;
};

struct ConnectionMatrix {
    uint8_t activeConnectionsCount;
    ConnectionMatrixRow rows[L4_MAX_PARALLEL_SESSIONS];
};

struct ACMConnection {
    conn_hdl_t         hdl;
    bool               isAllocated;
    uint8_t            destNode;
    uint8_t            priority;
    bool               autoConnect;
    ACMSendMode        defaultSendMode;
    
    ACMState           state;
    uint32_t           currentL4Token;
    uint32_t           backoffTimerStartMs;
    uint32_t           backoffDurationMs;
    uint32_t           reconnectAttemptStartMs;   
    uint32_t           lastActivityMs;            
    
    bool               isTxActive;
    uint32_t           txExpectedTotalSize;
    uint32_t           txBytesConfirmed;
    uint32_t           currentTransferRxBytes;    
    uint32_t           txAttemptsCount;          
    uint32_t           txFailuresCount;          
    uint64_t           lastWindowChannelBytes;    
    
    bool               reconnectPending;          
    
    uint8_t* rxBuffer;
    uint32_t           rxBufferSize;
    uint32_t           remoteRequestBufferSize;
    
    ACM_RxCallback     onDataArrive;
    ACM_ErrorCallback  onError;
    
    ChannelTelemetry   telemetry;
};

class mtCAN {
public:
    /**
     * @brief Instantiates the ACM and fully contains all downstream engine layers.
     * @param localNodeId Unique hardware node identifier for L2/L3 addressing filters.
     * @param tickProvider Runtime system microsecond or millisecond clock hook pointer.
     */
    mtCAN(uint8_t localNodeId, HardwareTickCallback tickProvider);
    ~mtCAN() = default;

    /**
     * @brief Blocking sequential activation of the complete protocol stack (L2 -> L3 -> L4).
     * @return true if all layers pass validation and settle, false otherwise.
     */
    bool connect_to_bus();

    /**
     * @brief Blocking sequential teardown and low-power isolation of the stack.
     * @return true when shutdown confirms successfully.
     */
    bool disconnect_from_bus();

    void tick(uint32_t currentMillis);

    conn_hdl_t connect(uint8_t destNode, uint8_t priority, bool autoConnect,
                       uint8_t* rxBuffer, uint32_t rxBufferSize,
                       uint32_t remoteRequestBufferSize,
                       ACM_RxCallback rxCb, ACM_ErrorCallback errCb,
                       ACMSendMode sendMode);

    ACMError send_data(conn_hdl_t hdl, const uint8_t* payload, uint32_t size);
    ACMError async_send(conn_hdl_t hdl, const uint8_t* payload, uint32_t size);
    ACMError disconnect(conn_hdl_t hdl);

    ChannelTelemetry get_channel_telemetry(conn_hdl_t hdl);
    GlobalNodeTelemetry get_global_telemetry() const { return m_globalTelemetry; }
    ConnectionMatrix get_connection_matrix();

    void forward_l4_rx(uint32_t appToken, const uint8_t* payload, uint32_t size);
    void forward_l4_err(uint32_t appToken, uint16_t errorCode);

private:
    uint8_t              m_localNodeId;
    HardwareTickCallback m_getSystemTimeMs;
    uint32_t             m_cachedMillis;
    uint32_t             m_lastSpeedCalcMs;
    uint64_t             m_previousWindowBytes;

    // --- ENCAPSULATED PIPELINE STACK MATRIX ---
    mtCANL2Engine        m_l2;
    mtCANL3Engine        m_l3;
    mtCANL4Engine        m_l4;
    
    ACMConnection        m_channels[L4_MAX_PARALLEL_SESSIONS];
    GlobalNodeTelemetry  m_globalTelemetry;
    
    conn_hdl_t allocate_channel();
    int8_t find_channel_by_token(uint32_t token);
    void reset_channel_struct(ACMConnection& ch);
    void recalculate_throughput();

};

#endif // MT_CAN_ACM_H