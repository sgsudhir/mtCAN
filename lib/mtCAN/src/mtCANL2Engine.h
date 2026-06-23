/**
 * @file mtCANL2Engine.h
 * @brief Object-Oriented Interface for the Isolated Dual-Plane bxCAN Driver Engine.
 *
 * Implements a bifurcated software queuing architecture, locking control plane operations
 * to hardware transmission mailbox 0, and multiplexing application data frames across mailboxes 1 and 2.
 * Includes explicit specifications for the parallel active routing data sets.
 */

#ifndef MT_CAN_L2_ENGINE_H
#define MT_CAN_L2_ENGINE_H

#pragma once

#include <Arduino.h>
#include "mtCANTypes.h"

/* ============================================================================
 * QUEUING BOUNDARIES AND MEMORY MASK BIT CONSTRUCTS
 * ============================================================================ */

/** * @brief Allocation ceiling for the Application Layer matrix frame slots (16 connections * 8 frames/seq) at max. 
 *  This defines the absolute maximum number of sub-frames we can store for larger messages.
 * 16 parallel channels multiplied by 8 sequential slots each equals 128 total tracking frames.
 */
#define TOTAL_APP_FRAMES        (TOTAL_APP_CONN * 8U)

/** * @brief Allocation ceiling for the System/Control Layer matrix circular queue frame buffer. 
 *  A dedicated pool of 32 slots reserved strictly for network management tasks like 
 * checking connections (pings) and setting clocks (time synchronization).
 */
#define TOTAL_CTRL_FRAMES       ((TOTAL_APP_CONN * 2U) + 4U)

/** @brief Allocation size of the software application data queue buffer. Must remain a power of two. */
#define M_L2_TX_APP_QUEUE_SIZE  (TOTAL_APP_CONN * 8U)

/** @brief Bitwise wrap mask utilizing power-of-two constraints for fast pointer evaluation. */
#define M_L2_TX_APP_QUEUE_MASK  (M_L2_TX_APP_QUEUE_SIZE - 1U)

/** @brief Dedicated safety queue size for isolated system control data frames. */
#define M_L2_TX_CTRL_QUEUE_SIZE ((TOTAL_APP_CONN * 2U) + 4U)

/** @brief Bitwise wrap mask utilizing power-of-two constraints for the control ring buffer. */
#define M_L2_TX_CTRL_QUEUE_MASK (M_L2_TX_CTRL_QUEUE_SIZE - 1U)


/* ============================================================================
 * BACKPRESSURE FLOW CONTROL AND TIMEOUT TIMING LIMITS
 * ============================================================================ */

/** @brief Upper queue depth limit which triggers backpressure protection and blocks incoming application inputs. */
#define M_L2_TX_APP_QUEUE_HIGH_WATERMARK    24

/** @brief Lower queue depth clearing point where application data ingestion is safely re-enabled. */
#define M_L2_TX_APP_QUEUE_LOW_WATERMARK     16

/** @brief Maximum lifespan (in milliseconds) a queue can remain blocked before forcing a software flush. */
#define M_L2_CONGESTION_TIMEOUT_MS          500

/** @brief Allowed time slice for a hardware transmission mailbox to sit empty/pending before triggering a reset. */
#define M_L2_QUEUE_STUCK_TIMEOUT_MS         500

/** @brief Threshold check of consecutive arbitration loss events before forcing a hardware mailbox clear. */
#define M_L2_MAX_ALLOWED_ALST_STRIKES       25


/* ============================================================================
 * CORE ENGINE CLASS DECLARATION
 * ============================================================================ */
class mtCANL2Engine {
public:
    /**
     * @brief Instantiates the Layer 2 engine controller, binding instance references and clearing statistics.
     */
    mtCANL2Engine();
    
    /**
     * @brief Default destructor cleaning up peripheral handles during teardown instances.
     */
    ~mtCANL2Engine() = default;

    /**
     * @brief Initialized peripheral configuration for bxCAN hardware and internal routing vectors.
     * @param assignedNodeId The local network address of this node (clamped strictly between 0 and 64).
     * @param commLossTimeoutThresholdMs The time slice limit before reporting a total loss of signal on the wire.
     * @return true if hardware successfully registers initialization mode and leaves without errors.
     */
    bool initialize_hardware(uint8_t assignedNodeId, uint32_t commLossTimeoutThresholdMs = 5000);
    
    /**
     * @brief Disconnects interrupt parameters and requests a total hardware sleep step.
     */
    void shutdown_hardware();
    
    /**
     * @brief Safety unbinding routine that disables NVIC masks and releases interrupt lines.
     */
    void disconnect();
    
    /**
     * @brief Periodic background supervisor routine handling stuck mailboxes, timeouts, and tracking.
     */
    void process_tx_management();
    
    /**
     * @brief Pushes an outgoing data package into the Application Plane queue (Mailbox 1 or 2).
     * @param id29 Complete 29-bit identifier compiled to fit application plane protocols.
     * @param dataPayload Point referencing the first source element of the byte array block.
     * @param dataLengthCode Dimensional bound of payload tracking parameters [0 to 8].
     * @return true if space was available and the package was safely appended.
     */
    bool enqueue_app_frame(uint32_t id29, const uint8_t* dataPayload, uint8_t dataLengthCode);

    /**
     * @brief Pushes an outgoing command package into the high-priority System Control Plane queue (Mailbox 0).
     * @param id29 Complete 29-bit identifier compiled to fit system plane protocols.
     * @param dataPayload Pointer referencing the first source element of the control structure.
     * @param dataLengthCode Dimensional bound of tracking parameter dimensions [0 to 8].
     * @return true if appended successfully, unaffected by application backpressure.
     */
    bool enqueue_ctrl_frame(uint32_t id29, const uint8_t* dataPayload, uint8_t dataLengthCode);

    /**
     * @brief Registers a routing entry into the Parallel Search arrays using an Insertion Sort algorithm.
     * @param ingressToken The base 29-bit identifier with sequence tracking bits masked out.
     * @param connIdx The assigned internal storage coordinate index row within g_ApplicationMatrix.
     * @return true if entry fits and inserts into table slots correctly without duplicates.
     */
    bool L2_route_add(uint32_t ingressToken, uint8_t connIdx);

    /**
     * @brief Drops a routing entry from the Parallel Search arrays, maintaining a continuous block structure.
     * @param ingressToken The base tracking token to scan and purge from memory arrays.
     * @return true if entry was found and removed, shift execution tracking safely finalized.
     */
    bool L2_route_drop(uint32_t ingressToken);

    /**
     * @brief Forces backpressure indicators clear and resets congestion trackers immediately.
     */
    void reset_flow_engine();

    /**
     * @brief Total hard-reset cycle for the peripheral chip block to pull it out of lockups or state errors.
     */
    bool recover_hardware();

    /**
     * @brief Performs queue flush steps and triggers a full device re-initialization sequence.
     */
    bool rejoin_network();

    /**
     * @brief Core outbound hardware dispatch supervisor processing pending items in software lines.
     */
    void handle_tx_interrupt_service_routine();

    /**
     * @brief Core ingress hardware processing logic linked to incoming frame interrupt updates.
     * @param hardwareFifoIndex Targeting point identifying source vector (FIFO 0 or FIFO 1).
     */
    void handle_rx_interrupt_service_routine(uint8_t hardwareFifoIndex);

    /* --- TELEMETRY READ-ONLY INTERFACES --- */
    uint8_t  get_node_id() const { return m_nodeId; }
    uint32_t get_boot_counter() const { return m_nodeBootCounter; }
    uint32_t get_total_transmitted_frames() const { return m_totalTransmittedFrames; }
    uint32_t get_total_received_frames() const { return m_totalReceivedFrames; }
    uint32_t get_tx_arbitration_lost_events() const { return m_txArbitrationLostEvents; }
    uint32_t get_tx_error_hardware_events() const { return m_txErrorHardwareEvents; }
    uint32_t get_bus_off_transitions() const { return m_busOffTransitions; }
    uint32_t get_error_passive_transitions() const { return m_errorPassiveTransitions; }
    uint32_t get_error_warning_transitions() const { return m_errorWarningTransitions; }
    uint32_t get_mailbox_stall_recoveries() const { return m_mailboxStallRecoveries; }
    uint32_t get_fifo0_overrun_events() const { return m_fifo0OverrunEvents; }
    uint32_t get_fifo1_overrun_events() const { return m_fifo1OverrunEvents; }
    uint32_t get_telemetry_firewall_drops() const { return m_telemetryFirewallDrops; }
    uint32_t get_control_matrix_dropped_frames() const { return m_controlMatrixDroppedFrames; }

    /**
     * @brief Diagnostic poll verifying network overflow occurrences and logging offending targets.
     * @param outPlaneState Returns the plane value that triggered the overload fault.
     * @param outOffendingNodeId Out parameter recording the transmitter that caused the overflow condition.
     * @return true if an active fault was present and cleared during the call event.
     */
    bool check_and_clear_quench_condition(uint8_t& outPlaneState, uint8_t& outOffendingNodeId);
    
    CanBusErrorState get_current_bus_error_state() const { return m_cachedErrorState; }
    L2FlowControlState get_current_flow_state() const { return m_flowState; }

    /* --- HIGH SPEED INLINE QUEUE DEPTH EVALUATORS --- */
    inline uint16_t get_tx_app_queue_count() const { return (m_txAppHead - m_txAppTail) & M_L2_TX_APP_QUEUE_MASK; }
    inline uint16_t get_tx_app_queue_free_slots() const { return (M_L2_TX_APP_QUEUE_SIZE - 1) - get_tx_app_queue_count(); }
    inline bool is_tx_app_queue_full() const { return (((m_txAppHead + 1) & M_L2_TX_APP_QUEUE_MASK) == m_txAppTail); }
    inline bool is_tx_app_queue_empty() const { return (m_txAppHead == m_txAppTail); }

    inline uint16_t get_tx_ctrl_queue_count() const { return (m_txCtrlHead - m_txCtrlTail) & M_L2_TX_CTRL_QUEUE_MASK; }
    inline uint16_t get_tx_ctrl_queue_free_slots() const { return (M_L2_TX_CTRL_QUEUE_SIZE - 1) - get_tx_ctrl_queue_count(); }
    inline bool is_tx_ctrl_queue_full() const { return (((m_txCtrlHead + 1) & M_L2_TX_CTRL_QUEUE_MASK) == m_txCtrlTail); }
    inline bool is_tx_ctrl_queue_empty() const { return (m_txCtrlHead == m_txCtrlTail); }

    /** @brief Head marker offset track for the global control matrix line storage paths. */
    volatile uint8_t m_ctrlMatrixHead;
    
    /** @brief Tail marker offset track for the global control matrix line storage paths. */
    volatile uint8_t m_ctrlMatrixTail;

private:
    void flush_software_queues();
    void reconfigure_hardware_filters();
    uint32_t get_portable_system_timestamp_ms();

    uint8_t  m_nodeId;
    uint32_t m_nodeBootCounter;
    uint32_t m_commLossTimeoutThresholdMs;

    /* --- BIFURCATED QUEUE BACKING ARRAY FIELDS --- */
    CanFrameL2 m_txAppRingBuffer[M_L2_TX_APP_QUEUE_SIZE];
    volatile uint16_t m_txAppHead;
    volatile uint16_t m_txAppTail;

    CanFrameL2 m_txCtrlRingBuffer[M_L2_TX_CTRL_QUEUE_SIZE];
    volatile uint16_t m_txCtrlHead;
    volatile uint16_t m_txCtrlTail;

    volatile L2FlowControlState m_flowState;
    uint32_t m_blockedTimestamp;
    volatile uint32_t m_lastWireActivityTimestamp;
    volatile bool m_congestionWarningActive;
    uint32_t m_congestionWarningStrikes;

    uint8_t  m_activeRouteCount;

    /* --- HARDWARE TRACKING TIMESTAMPS AND ARBITRATION REGISTERS --- */
    uint32_t m_mailboxTxTimestamp[3];
    volatile uint32_t m_mailboxArbitrationLossCount[3];

    /* --- PERSISTENT ANALYTICS AND COUNTER TRACKS --- */
    volatile uint32_t m_totalTransmittedFrames;
    volatile uint32_t m_totalReceivedFrames;
    volatile uint32_t m_fifo0OverrunEvents;
    volatile uint32_t m_fifo1OverrunEvents;
    volatile uint32_t m_txArbitrationLostEvents;
    volatile uint32_t m_txErrorHardwareEvents;
    volatile uint32_t m_busOffTransitions;
    volatile uint32_t m_errorPassiveTransitions;
    volatile uint32_t m_errorWarningTransitions;
    volatile uint32_t m_mailboxStallRecoveries;
    volatile uint32_t m_telemetryFirewallDrops;
    volatile uint32_t m_controlMatrixDroppedFrames;

    /**
     * @enum L2QuenchState
     * @brief Internal tracking codes evaluating internal queue overflow exceptions.
     */
    enum L2QuenchState : uint8_t {
        QUENCH_STATE_NONE        = 0,
        QUENCH_STATE_CONTROL     = 1,
        QUENCH_STATE_APPLICATION = 2
    };

    volatile L2QuenchState m_quenchState;
    volatile uint8_t       m_quenchNodeId;

    uint32_t m_lastTelemetrySampleTick;
    uint32_t m_lastTotalFrameCount;

    volatile CanBusErrorState m_cachedErrorState;
};

#endif /* MT_CAN_L2_ENGINE_H */