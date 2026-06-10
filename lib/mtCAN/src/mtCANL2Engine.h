/**
 * @file mtCANL2Engine.h
 * @brief Class definition for the STM32 bxCAN Direct-Mapped Layer 2 Preemptive Processing Engine.
 * * Provides class declarations, buffer metrics, watermarks, flow structures, and object fields
 * required to operate the double-FIFO interrupt-driven matrix-mapped CAN engine.
 */

#ifndef MT_CAN_L2_ENGINE_H
#define MT_CAN_L2_ENGINE_H

#pragma once

#include <Arduino.h>
#include "mtCANTypes.h"

/** * @section Queue Invariant Geometries
 * Parameters sizing and restricting the software circular ring buffers.
 * Sizes must always equal a strict power of two to allow ultra-fast logical bitwise masking
 * instead of expensive division operations to loop circular index boundaries.
 */

/// @brief Number of storage frame allocations inside the software transmission ring buffer.
#define M_L2_TX_QUEUE_SIZE 32

/// @brief Bitwise wrapper used to handle ring buffer index wrapping without using an explicit modulo (%) operator.
#define M_L2_TX_QUEUE_MASK (M_L2_TX_QUEUE_SIZE - 1)

/** * @section Congestion Management Boundaries
 * Watermark indicators evaluated inside enqueue loops to trip outbound engine throttling structures.
 */

/// @brief Depth threshold matching 85% of total queue allocation. Reaching this forces a congestion state lock.
#define M_L2_TX_QUEUE_HIGH_WATERMARK 27

/// @brief Clear threshold matching 55% of total queue allocation. Falling below this unlocks the system.
#define M_L2_TX_QUEUE_LOW_WATERMARK  18

/// @brief Maximum allowed time window a TX queue can remain blocked before forcing a hard engine flush.
#define M_L2_CONGESTION_TIMEOUT_MS   250

/// @brief Maximum life duration allowed for a single frame stuck inside a hardware mailbox before manual revocation.
#define M_L2_QUEUE_STUCK_TIMEOUT_MS  500

/// @brief Threshold tracking consecutive Arbitration Loss events before hardware forces mailbox teardown.
#define M_L2_MAX_ALLOWED_ALST_STRIKES 25

/**
 * @enum L2FlowControlState
 * @brief Internal states representing back-pressure condition status of the transmission pipeline.
 */
enum L2FlowControlState {
    L2_FLOW_OK = 0,         ///< Transmission stream is clear; software layers may submit packets freely.
    L2_FLOW_CONGESTED = 1   ///< Throttling activated; queues are saturated, incoming packets are rejected.
};

/**
 * @class mtCANL2Engine
 * @brief Driver class controlling STM32 Register Configuration and Matrix Packet Routing.
 * * Manages the underlying STM32 bxCAN registers directly while providing a zero-copy direct memory-mapped 
 * matrix interface to upper network layers. It treats incoming data frames as real-time absolute coordinate state 
 * nodes rather than a collection of volatile items inside traditional, sequential circular queues.
 */
class mtCANL2Engine {
public:
    /**
     * @brief Construct a new mtCANL2Engine instance.
     * Sets defaults, binds singleton reference pointers, and processes soft restart counters.
     */
    mtCANL2Engine();
    
    /// @brief Default destructor.
    ~mtCANL2Engine() = default;

    /**
     * @brief Direct configuration entry point to assign node boundaries and bring up bxCAN clocks.
     * * Configures the physical GPIO pins, sets timing profiles, turns on interrupt lines, 
     * and clears global matrix planes.
     * * @param assignedNodeId The physical node index assigned to this board instance (0 to 126).
     * @param commLossTimeoutThresholdMs Lifespan boundary window defining network node drop conditions.
     * @return true Configuration verified, clocks stable, hardware is running.
     * @return false Hardware initialization timed out or node ID bounds check failed.
     */
    bool initialize_hardware(uint8_t assignedNodeId, uint32_t commLossTimeoutThresholdMs = 5000);
    
    /// @brief Disables internal interrupt lines and drops hardware into a low-power shutdown configuration.
    void shutdown_hardware();
    
    /// @brief Severs interface mappings with the bxCAN hardware controller, forcing a state isolation.
    void disconnect();

    /**
     * @brief Periodic scheduling point driving outbound mailbox sanity checks, recovery passes, and timeouts.
     * Must be evaluated regularly inside the primary processing execution pipeline.
     */
    void process_tx_management();

    /**
     * @brief Enqueues a raw layer 2 frame into the local software transmission loop.
     * * Validates protocol signatures, bounds-checks parameters, handles priority metrics,
     * and triggers mailbox loads if resources are idle.
     * * @param id29 Pre-compiled 29-bit CAN Extended identifier containing complete routing fields.
     * @param dataPayload Source buffer sequence holding byte blocks to transmit.
     * @param dataLengthCode Count specifying total bytes to copy on wire (0 to 8).
     * @return true Packet accepted and loaded into software ring buffer.
     * @return false Congested state active, input criteria invalid, or memory structure saturated.
     */
    bool enqueue_tx_frame(uint32_t id29, const uint8_t* dataPayload, uint8_t dataLengthCode);

    /// @brief Clears back-pressure tracking locks and resets the congestion timeout timers.
    void reset_flow_engine();
    
    /// @brief Performs a hard power cycle and reconfiguration across bxCAN hardware networks.
    bool recover_hardware();
    
    /// @brief Flushes pending frames and executes a complete initialization recovery to rejoin the live network.
    bool rejoin_network();

    /**
     * @brief Low-level service router executing inside the TX interrupt execution frame.
     * Inspects mailbox clear state bitmasks, increments counters, and streams the software queue.
     */
    void handle_tx_interrupt_service_routine();

    /**
     * @brief Direct routing parser executing inside hardware RX interrupt context frames.
     * * Slices IDs, routes system frames to FIFO0/Circular matrix spaces, and parses application frames 
     * directly into Layer 4 spatial coordinates across FIFO1 boundaries.
     * * @param hardwareFifoIndex Defines active processing source (0 = System, 1 = Application).
     */
    void handle_rx_interrupt_service_routine(uint8_t hardwareFifoIndex);

    /* --- Telemetry and Diagnostic Metrics API Surface --- */
    
    /// @brief Fetches local assigned node ID.
    uint8_t  get_node_id() const { return m_nodeId; }
    /// @brief Fetches total soft-restarts undergone since power-on.
    uint32_t get_boot_counter() const { return m_nodeBootCounter; }
    /// @brief Fetches running total of frames pushed onto physical wire.
    uint32_t get_total_transmitted_frames() const { return m_totalTransmittedFrames; }
    /// @brief Fetches running total of error-free packages parsed from wire.
    uint32_t get_total_received_frames() const { return m_totalReceivedFrames; }
    /// @brief Fetches historical counts of mailbox collisions resolved by retry drops.
    uint32_t get_tx_arbitration_lost_events() const { return m_txArbitrationLostEvents; }
    /// @brief Fetches raw counts indicating physical hardware error line triggers.
    uint32_t get_tx_error_hardware_events() const { return m_txErrorHardwareEvents; }
    /// @brief Fetches total transitions dropped into catastrophic absolute isolation.
    uint32_t get_bus_off_transitions() const { return m_busOffTransitions; }
    /// @brief Fetches transitions where internal error tracking exceeded 127 counts.
    uint32_t get_error_passive_transitions() const { return m_errorPassiveTransitions; }
    /// @brief Fetches transitions where tracking crossed warning ceilings (96 counts).
    uint32_t get_error_warning_transitions() const { return m_errorWarningTransitions; }
    /// @brief Fetches counts showing forced mailbox flushes due to stuck frame drops.
    uint32_t get_mailbox_stall_recoveries() const { return m_mailboxStallRecoveries; }
    /// @brief Fetches count of physical hardware overruns reported on FIFO 0.
    uint32_t get_fifo0_overrun_events() const { return m_fifo0OverrunEvents; }
    /// @brief Fetches count of physical hardware overruns reported on FIFO 1.
    uint32_t get_fifo1_overrun_events() const { return m_fifo1OverrunEvents; }
    /// @brief Fetches frame drop counts induced by space saturation within Application Plane matrix arrays.
    uint32_t get_telemetry_firewall_drops() const { return m_telemetryFirewallDrops; }
    /// @brief Fetches frame drop counts induced by queue saturation within System Plane circular arrays.
    uint32_t get_system_matrix_dropped_frames() const { return m_systemMatrixDroppedFrames; }
    
    /**
     * @brief Thread-safe atomic extraction function to inspect and clear the Source Quench request flag.
     * @return true Quench condition was active and has been cleared down.
     * @return false No quench pending.
     */
    bool     check_and_clear_quench_condition();

    /// @brief Returns cached representation of physical error layer states.
    CanBusErrorState get_current_bus_error_state() const { return m_cachedErrorState; }
    /// @brief Returns current flow control status restriction profile.
    L2FlowControlState get_current_flow_state() const { return m_flowState; }

    /* --- Inline Queue Geometry Calculations --- */
    
    /// @brief Measures current frame count remaining inside software transmission buffer.
    inline uint16_t get_tx_queue_count() const { return (m_txHead - m_txTail) & M_L2_TX_QUEUE_MASK; }
    /// @brief Calculates open array space remaining inside the software transmission buffer.
    inline uint16_t get_tx_queue_free_slots() const { return (M_L2_TX_QUEUE_SIZE - 1) - get_tx_queue_count(); }
    /// @brief Safety calculation to intercept index overlaps before advancing head.
    inline bool is_tx_queue_full() const { return (((m_txHead + 1) & M_L2_TX_QUEUE_MASK) == m_txTail); }
    /// @brief Standard emptiness validation flag.
    inline bool is_tx_queue_empty() const { return (m_txHead == m_txTail); }

    /// @brief Volatile tracking indices exposing System Plane queue boundaries to execution environments.
    volatile uint8_t m_sysMatrixHead;
    volatile uint8_t m_sysMatrixTail;

private:
    /// @brief Resets queue internal indexing trackers to base state under interrupt isolation masks.
    void flush_software_queues();
    /// @brief Slices filtering rules directly into hardware bxCAN tracking blocks using bit shifts.
    void reconfigure_hardware_filters();
    /// @brief Portable layer wrapping the low-level timer microsecond/millisecond ticker source.
    uint32_t get_portable_system_timestamp_ms();

    /* --- Internal Core Engine Variables --- */
    uint8_t  m_nodeId;                     ///< Physical address identifier bound to local node board.
    uint32_t m_nodeBootCounter;            ///< Running history tracking reset survivals across execution spans.
    uint32_t m_commLossTimeoutThresholdMs; ///< Duration tracking threshold used to scrap dead connection entries.

    /* --- Outbound Storage Framework --- */
    CanFrameL2 m_txRingBuffer[M_L2_TX_QUEUE_SIZE]; ///< Memory array buffering frames bound for wire execution.
    volatile uint16_t m_txHead;                    ///< Producer allocation index tracking next open TX frame block.
    volatile uint16_t m_txTail;                    ///< Consumer allocation index tracking next frame targeting hardware.

    /* --- Congestion State Telemetry Matrix --- */
    volatile L2FlowControlState m_flowState;    ///< Backpressure state tracking context block.
    uint32_t m_blockedTimestamp;                ///< Timestamp capture marking initialization of current lock phase.
    volatile uint32_t m_lastWireActivityTimestamp; ///< Timestamp logging last verified clean bus TX/RX block.
    volatile bool m_congestionWarningActive;    ///< High-priority signal indicator indicating queue congestion.
    uint32_t m_congestionWarningStrikes;       ///< Cumulative metrics tracking total congestion lock operations.

    /* --- Hardware Mailbox Tracking Registries --- */
    uint32_t m_mailboxTxTimestamp[3];               ///< Timestamp records logging exactly when frame entered each mailbox.
    volatile uint32_t m_mailboxArbitrationLossCount[3]; ///< Tracking sequential collisions encountered by mailbox channels.

    /* --- Diagnostic Metric Storage Aggregates --- */
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
    volatile uint32_t m_systemMatrixDroppedFrames;
    volatile bool     m_sourceQuenchTriggerPending; ///< Atomic status flag flagging immediate network throttling demands.

    /* --- Telemetry Window Tracking Intervals --- */
    uint32_t m_lastTelemetrySampleTick; ///< Logs processing window timelines across interval loops.
    uint32_t m_lastTotalFrameCount;     ///< Captured data benchmark used to calculate bus loading coefficients.

    volatile CanBusErrorState m_cachedErrorState; ///< Current status profile tracking internal bxCAN error states.
};

#endif