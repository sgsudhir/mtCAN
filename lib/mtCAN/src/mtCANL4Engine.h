/**
 * @file mtCANL4Engine.h
 * @brief High-reliability Segment-by-Segment Transport Engine for mtCAN Layer 4.
 *
 * This file defines the operational interface, memory tracking slots, session constraints,
 * and command matrices governing Layer 4 connection state tracking. Layer 4 manages segment-by-segment
 * data decomposition, flow control validations, streaming sequence numbers, data integrity verification (CRC16),
 * and link telemetry watchdog heartbeats.
 * * Non-technical Overview:
 * Think of this file as a multi-line switchboard controller. It partitions data streams into manageable chunks, 
 * transmits them across a noisy wire, checks for errors at the destination, and automatically resends parts 
 * that are lost or garbled, all while monitoring whether the remote end is still alive.
 */

#ifndef MTCAN_L4_ENGINE_H
#define MTCAN_L4_ENGINE_H

#include <stdint.h>
#include <stddef.h>

/**
 * @brief Safety macro verifying consistent assembly of control frame headers.
 * Combines commands and session indices into an identical control byte signature.
 * @param cmd The specific L4_Command code enum being transmitted.
 * @param slot The active tracked internal session allocation table slot index.
 */
#define PACK_HEADER(cmd, slot) (0x80 | (((cmd) & 0x0F) << 3) | ((slot) & 0x07))

// Architectural Engineering Limit Boundaries
#define L4_MAX_PARALLEL_SESSIONS            4       /**< Max concurrent communication sessions supported in memory. */
#define L4_CLOSE_TOKEN_KEYWORD              0x5A5A  /**< Magic safety validation constant used to authorize disconnects. */
#define L4_DEFAULT_REMOTE_REQUEST_SIZE      262144  /**< Upper streaming threshold boundary constraint (256 KB). */
#define L4_MAX_L3_PAYLOAD_SIZE              63      /**< Absolute maximum raw capacity envelope allowed per Layer 3 frame. */
#define L4_DATA_CHUNK_MAX_SIZE              62      /**< Effective payload capacity per frame (L3 Limit minus 1-byte L4 overhead sequence header). */
#define L4_DEFAULT_TIMEOUT_MS               500UL   /**< Active retry acknowledgment waiting window before escalating a retransmit. */
#define L4_MAX_RETRANSMIT_ATTEMPTS          3       /**< Retransmit threshold before declaring a session dead and force-terminating. */

/**
 * @brief Functional Layer 4 command signaling codes packed into control headers.
 * Governs state tracking, connection establishment, data flow checkpoints, and link telemetry.
 */
enum L4_Command : uint8_t {
    L4_CMD_CONN_OPEN_REQ      = 0x01,  /**< Handshake step 1: Requesting a dedicated link session with a target node. */
    L4_CMD_CONN_OPEN_RESP     = 0x02,  /**< Handshake step 2: Responding to an open request with an acceptance or rejection code. */
    L4_CMD_CONN_CLOSE_REQ     = 0x03,  /**< Graceful Teardown step 1: Asking the remote endpoint to close the connection. */
    L4_CMD_CONN_CLOSE_RESP    = 0x04,  /**< Graceful Teardown step 2: Final response confirming the slot has been cleared. */
    L4_CMD_DATA_HEADER        = 0x05,  /**< Stream Preparation: Transmitting the size, bounds, and total stream CRC16 layout. */
    L4_CMD_DATA_ACK_NACK      = 0x06,  /**< Segment Validation: Signals successful reception (ACK) or sequence mismatch error (NACK). */
    L4_CMD_REQUEST_TO_WAIT    = 0x07,  /**< Flow Control Override: Tells the transmitter to stall temporarily due to full receiver buffers. */
    L4_CMD_KEEP_ALIVE_REQ     = 0x08,  /**< Telemetry Probe: Injected ping request to measure link responsiveness during quiet periods. */
    L4_CMD_KEEP_ALIVE_RESP    = 0x09,  /**< Telemetry Echo: Return reply verifying that the remote application is still processing frames. */
    L4_CMD_RX_INTEGRITY       = 0x0B,  /**< Stream Completion: Final status notification confirming overall stream CRC16 match. */
    L4_CMD_RESET_CONNECTION   = 0x0C,  /**< Emergency Override: Demands instant host connection clearance due to unrecoverable errors. */
    L4_CMD_ABORT_SEND         = 0x0D,  /**< Transaction Interruption: Signals that the application layer has abruptly canceled the stream. */
    L4_CMD_DATA_HEADER_ACK    = 0x0E   /**< Stream Preparation ACK: Confirms receipt of metadata bounds before data streaming begins. */
};

/**
 * @brief Standardized diagnostic status codes returned in network control blocks.
 * Used to report transfer status, buffer alerts, or error tracking parameters.
 */
enum L4_ResponseStatus : uint16_t {
    L4_RESP_SUCCESS_OK        = 0x0000, /**< Operation succeeded perfectly. */
    L4_RESP_ERR_REJECTED      = 0x0001, /**< Link request rejected due to priority conflicts, busy slots, or access restrictions. */
    L4_ERR_BUFFER_OVERFLOW    = 0x0002, /**< Inbound data volume exceeds the application's allocated reserve buffer memory. */
    L4_ERR_CHECKSUM_MISMATCH  = 0x0003, /**< Reassembled stream failed the CRC16 verification; data is corrupted. */
    L4_ERR_TIMEOUT            = 0x0004, /**< The remote node stopped responding within the allowed time limits. */
    L4_ERR_DESYNCHRONIZED     = 0x0005, /**< Internal sequence state machine has slipped or tracking metrics have lost alignment. */
    L4_ERR_SEQUENCE_MISMATCH  = 0x0006  /**< Received a segment sequence number that outpaced or lagged the expected state tracking index. */
};

/**
 * @brief High-level structural tracking states for connection lifecycle management.
 */
enum NetworkSessionState : uint8_t {
    L4_NET_DISCONNECTED,  /**< Slot is inactive; unallocated or fully reset. */
    L4_NET_CONNECTING,    /**< Handshake requested; waiting for response confirmation from the network target. */
    L4_NET_CONNECTED,     /**< Session verified and active; link is authorized to transmit or receive streams. */
    L4_NET_DISCONNECTING  /**< Tearing down gracefully; awaiting final closing handshake confirmations. */
};

/**
 * @brief Fine-grained inner state tracking flags defining immediate channel data activity.
 */
enum ActivityState : uint8_t {
    L4_ACT_DISCONNECTED,           /**< Inactive idle state; zero operational tracking processing. */
    L4_ACT_CONNECTING,             /**< Synchronizing control layers with the target device. */
    L4_ACT_IDLE,                   /**< Connection healthy, but no data transfers are currently active. */
    L4_ACT_SENDING,                /**< Currently actively processing and pushing streaming data across the wire. */
    L4_ACT_RECEIVING,              /**< Actively buffering incoming multi-segment data arrays. */
    L4_ACT_WAITING_FOR_HEADER_ACK, /**< Stream metadata block sent; waiting for target node storage authorization. */
    L4_ACT_WAITING_FOR_ACK,        /**< Segment burst sent; waiting for the remote device to verify sequence alignment. */
    L4_ACT_UNRESPONSIVE,           /**< Watchdog Alert: Remote host quiet time has crossed safety thresholds; telemetry warnings are active. */
    L4_ACT_CONNECTED               /**< Connection verified; link ready for state transition mapping. */
};

/**
 * @brief Callback signature triggered upon successful stream completion and integrity verification.
 * @param appToken The unique session token matching the data-delivery slot.
 * @param buffer Pointer to the contiguous memory block containing fully reassembled stream data.
 * @param size The absolute final byte count size of the completed stream.
 */
typedef void (*L4_RxCallback)(uint32_t appToken, const uint8_t* buffer, uint32_t size);

/**
 * @brief Callback signature triggered when a session encounters a failure or warning condition.
 * @param appToken The unique session token matching the failed slot context.
 * @param errorCode Standardized diagnostic status code tracking the failure condition.
 */
typedef void (*L4_ErrorCallback)(uint32_t appToken, uint16_t errorCode);

class mtCANL3Engine; // Forward declaration linking down to the underlying network frame router engine

/**
 * @class mtCANL4Engine
 * @brief The Layer 4 Transport Engine state machine.
 *
 * Manages parallel active multi-client communication slots. Handles connection negotiation, 
 * data chunk segmentation, transmission confirmations, error detection protocols, and connection loss tracking.
 */
class mtCANL4Engine {
public:
    /**
     * @struct SessionSlot
     * @brief Context isolation structure tracking a single independent communication pipeline.
     */
    struct SessionSlot {
        bool isAllocated;                /**< Allocation flag indicating if this slot is active or free. */
        uint32_t associatedAppToken;     /**< Unique tracking identification code routing this session context. */
        NetworkSessionState netState;    /**< Macro lifecycle status flag tracking connection level. */
        ActivityState actState;          /**< Immediate data operation state tracking flag. */
        uint8_t priority;                /**< The assigned bus priority level (2 to 7). */
        uint8_t destNode;                /**< The destination network node address identifier. */
        
        // RX State Tracking Parameters
        uint8_t* rxBuf;                  /**< Destination application memory buffer for incoming streams. */
        uint32_t rxBufMax;               /**< Maximum size boundary limit allocated to protect memory space. */
        uint32_t rxByteCursor;           /**< Running count index of bytes successfully written to the buffer. */
        uint32_t rxTotalStreamSize;      /**< Expected file transfer target byte size defined by data headers. */
        uint16_t rxExpectedCRC16;        /**< Expected composite mathematical signature payload validation tag. */
        uint8_t rxExpectedSeq;           /**< Target tracking value used to check incoming segment numbers (0-7). */
        
        // TX State Tracking Parameters
        const uint8_t* txBuffer;         /**< Pointer to the source memory block staged for transmission. */
        uint32_t txTotalStreamSize;      /**< Total length in bytes of the data file scheduled for transmission. */
        uint32_t txByteCursor;           /**< Running index position tracking bytes pushed to lower drivers. */
        uint8_t txNextSeq;               /**< 3-bit counter index embedded inside next outbound packet data headers. */
        uint8_t retryCount;              /**< Re-transmission try counter tracking attempts for a single unconfirmed block. */
        
        // Application Interface Hook Callbacks
        L4_RxCallback appRxCallback;     /**< Success callback hook fired on successful data receipt. */
        L4_ErrorCallback appErrorCb;     /**< Failure callback hook triggered for diagnostics and warnings. */
        
        // Internal Timing Metrics and Watchdog Parameters
        uint32_t ackDeadlineMs;           /**< Target clock timestamp deadline before a segment is marked lost. */
        uint32_t lastTxRxTimestampMs;     /**< Clock millisecond tracker logging the most recent transaction transfer. */
        uint32_t lastControlActivityMs;   /**< Clock milestone monitor tracking overall remote network life. */
        uint32_t lastRxSegmentTimestampMs;/**< Dedicated stream segment watchdog logging active stream packet history. */
        uint32_t lastKeepAliveTxMs;       /**< Master counter scheduling outbound telemetry ping operations. */
        uint8_t failedKeepAliveCount;     /**< Counter tracking unacknowledged outbound pings. */
        uint8_t keepAliveRetryCount;      /**< Live count tracking consecutive missed telemetry window milestones. */
        uint8_t lastReportedMissCount;    /**< State latch tracking reported warnings to prevent repetitive callbacks. */

        /**
         * @brief Resets and clears the internal session state variables to restore a slot to its baseline state.
         */
        void clear() {
            isAllocated = false;
            associatedAppToken = 0;
            netState = L4_NET_DISCONNECTED;
            actState = L4_ACT_DISCONNECTED;
            priority = 0;
            destNode = 0;
            rxBuf = nullptr;
            rxBufMax = 0;
            rxByteCursor = 0;
            rxTotalStreamSize = 0;
            rxExpectedCRC16 = 0;
            rxExpectedSeq = 0;
            txBuffer = nullptr;
            txTotalStreamSize = 0;
            txByteCursor = 0;
            txNextSeq = 0;
            retryCount = 0;
            appRxCallback = nullptr;
            appErrorCb = nullptr;
            ackDeadlineMs = 0;
            lastTxRxTimestampMs = 0;
            lastControlActivityMs = 0;
            lastRxSegmentTimestampMs = 0;
            failedKeepAliveCount = 0;
            keepAliveRetryCount = 0;
            lastReportedMissCount = 0;
        }
    };

    /**
     * @brief Explicit transport initialization engine constructor. Logs a reference to Layer 3 drivers.
     * @param l3Engine Core lower routing interface layer tracking physical network hardware paths.
     */
    explicit mtCANL4Engine(mtCANL3Engine& l3Engine);
    
    /**
     * @brief Default secure class destructor. Maintains baseline lifecycle management.
     */
    ~mtCANL4Engine() = default;

    /**
     * @brief Resets all session tracking arrays to ensure clean memory spaces prior to network activation.
     */
    void init();

    /**
     * @brief Cyclically called background engine heartbeat. Evaluates timeouts, retries, and watchdogs.
     * @param currentMillis Active high-resolution running system up-time monitor log expressed in milliseconds.
     */
    void tick(uint32_t currentMillis);
    
    /**
     * @brief Registers an application allocation pattern as a passive listener, waiting for remote connections.
     * @param priority Targeted operational stream priority level lane assignment (Must follow the 2-7 constraint rules).
     * @param rxBuffer Pre-allocated host RAM address pool assigned to safely receive incoming data streams.
     * @param bufferReserve Absolute storage boundary limit tracking the capacity of the provided RAM block.
     * @param rxCb Callback hook used to hand off successfully reassembled data arrays to the system application.
     * @param errCb Diagnostic notification callback hook used to report link errors and connection warnings.
     * @return Generated 32-bit application token integer tracking this listener instance, or 0 if allocation fails.
     */
    uint32_t register_passive_listener(uint8_t priority, uint8_t* rxBuffer, uint32_t bufferReserve, L4_RxCallback rxCb, L4_ErrorCallback errCb);
    
    /**
     * @brief Initiates an outbound point-to-point handshake sequence targeting a specific remote node.
     * @param destNode The network node identifier address being targeted for connection.
     * @param priority Priority tracking lane assigned to govern arbitration urgency (Must follow the 2-7 constraint rules).
     * @param rxBuffer Memory address pool assigned to handle data streams received from this node.
     * @param bufferReserve Total capacity limit assigned to the tracking receiver buffer array.
     * @param rxCb App-level callback hook triggered upon completing data streaming integrity checks.
     * @param errCb Diagnostic reporting callback hook utilized for connection loss or fault tracking.
     * @return Generated active session application access tracking token, or 0 if allocation fails.
     */
    uint32_t open_connection(uint8_t destNode, uint8_t priority, uint8_t* rxBuffer, uint32_t bufferReserve, L4_RxCallback rxCb, L4_ErrorCallback errCb);
    
    /**
     * @brief Gracefully terminates an active tracking stream channel slot using a standardized disconnection protocol.
     * @param appToken The authorized session reference key mapping back to an active tracking slot.
     * @return True if teardown handshake command structures are successfully dispatched; False if the token is invalid.
     */
    bool close_connection(uint32_t appToken);
    
    /**
     * @brief Stages an application data array for chunked streaming transmission across the network bus.
     * @param appToken Authorized session reference key confirming an established and healthy connection.
     * @param data Source memory array tracking bytes prepared for network streaming.
     * @param size Absolute byte array measurement size tracking the complete block volume.
     * @return True if streaming structures initialize successfully; False if parameters violate boundary rules.
     */
    bool transmit_data(uint32_t appToken, const uint8_t* data, uint32_t size);
    
    /**
     * @brief Forces an abrupt termination of a transmission due to an internal exception or application override.
     * @param appToken Authorized tracking handle defining the targeted communication channel slot.
     * @param reasonCode Error reporting status parameter passed to the remote recipient node.
     */
    void abort_transfer(uint32_t appToken, uint16_t reasonCode);
    
    /**
     * @brief Router entry method capturing and processing control frames targeted at this layer.
     * Handles signaling elements, including session requests, acknowledgments, and pings.
     * @param controlToken Raw CAN frame packet token identifier tracking routing metadata parameters.
     * @param payload Memory address mapping back to incoming control byte patterns.
     * @param size Payload length constraint tracking control block formatting size boundaries.
     */
    void handle_ingress_control(uint32_t controlToken, const uint8_t* payload, uint8_t size);
    
    /**
     * @brief Router entry method tracking streaming segment blocks carrying fragmented application payloads.
     * Evaluates packet sequence markers, copies payloads into buffers, and issues tracking acknowledgments.
     * @param appToken Streaming identification token mapping back to an active tracked communication session.
     * @param payload Memory block reference pointing to data bytes pulled from physical network drivers.
     * @param size Absolute packet format size tracking the immediate incoming segment frame payload.
     */
    void handle_ingress_data(uint32_t appToken, const uint8_t* payload, uint8_t size);

private:
    mtCANL3Engine& m_l3Engine;                                  /**< Core reference interface linking back to the lower Layer 3 routing engine. */
    SessionSlot m_slots[L4_MAX_PARALLEL_SESSIONS];              /**< Statically allocated execution matrix managing parallel connection state blocks. */
    uint32_t m_currentMillis;                                   /**< Master local cache holding the latest system timer clock measurement. */

    // Core Private Internal Utility Framework Mapping Engine Methods
    int8_t find_session_by_token(uint32_t token);               /**< Maps a 32-bit token integer back to its internal slot index. */
    int8_t find_passive_listener(uint8_t priority);              /**< Locates an unlinked passive listener matching a priority lane allocation requirement. */
    int8_t allocate_free_slot();                                /**< Scans the internal matrix table for an unused session structure slot. */
    uint16_t compute_crc16(const uint8_t* data, uint32_t length);/**< Calculates a standard CRC16 checksum for data integrity verification. */
    
    /**
     * @brief Forcefully terminates a session slot, clears memory footprints, and optionally alerts the remote node.
     * @param slotIdx The target array tracking index inside the execution matrix.
     * @param errorCode The diagnostics error status parameter reported to the host application callback.
     * @param issueResetCommand Trigger flag determining if an emergency reset control frame is dispatched over the bus.
     */
    void force_terminate_session(uint8_t slotIdx, uint16_t errorCode, bool issueResetCommand);
    
    void send_next_segment(SessionSlot& slot);                  /**< Packages and transmits the next sequential data chunk from a stream. */
    void send_data_header(SessionSlot& slot);                   /**< Compiles and sends stream initialization metadata to prepare the remote target. */
};

#endif /* MTCAN_L4_ENGINE_H */