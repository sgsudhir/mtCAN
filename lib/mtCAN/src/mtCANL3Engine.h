/**
 * @file mtCANL3Engine.h
 * @brief Layer 3 Engine for mtCAN Architecture with RTC and Ping RTT.
 *
 * LAYMAN EXPLANATION:
 * Think of standard CAN hardware as a simple postman that can only carry tiny postcard envelopes holding 
 * up to 8 bytes of text. If you want to send a long letter (like a 50-byte configuration string), 
 * it won't fit on one postcard. 
 * * Layer 3 (this file) is the smart manager that chops long application messages up into multiple sequential 
 * postcards on transmission, and re-assembles them back into a single continuous letter when receiving them.
 * It also handles network pings to calculate message latency (RTT) and manages an internal real-time wall clock 
 * (RTC) synced to global Unix Epoch seconds.
 */

#ifndef MT_CAN_L3_ENGINE_H
#define MT_CAN_L3_ENGINE_H

#pragma once

#include <Arduino.h>
#include "mtCANL2Engine.h"
#include "mtCANTypes.h"

// --- Maintainability Configuration Parameters ---
/** @brief Max time allowed between individual incoming packet fragments before we consider the stream broken. */
#define L3_INTERFRAME_TIMEOUT_MS            500UL
/** @brief Maximum number of parallel outbound pings we can monitor at the exact same time. */
#define L3_MAX_PING_TRACKERS                4

// --- Layer 3 Status Operational Return Codes ---
#define L3_STATUS_OK                        0x00  /**< Action successfully executed. */
#define L3_STATUS_ERR_CONGESTED             0x01  /**< Outgoing pipelines are full. Transmission blocked. */
#define L3_STATUS_ERR_INVALID_SIZE          0x02  /**< Message payload size requested is too big (above 63 bytes) or null. */
#define L3_STATUS_ERR_L2_FAILED             0x03  /**< Layer 2 hardware injection ring buffer refused frame allocation. */
#define L3_STATUS_ERR_INVALID_CTRL_TYPE     0x04  /**< The control frame command type byte is unrecognized or out of range. */
#define XCAN_L3_STATUS_ERR_INVALID_TOKEN    0x05  /**< The addressing tracking token failed architectural layout tests. */

// --- RTC Error Operational Codes ---
#define L3_RTC_STATUS_OK                    0x00  /**< Real-time clock successfully configured or updated. */
#define L3_RTC_ERR_NOT_SYNCHRONIZED         0xE0  /**< System clock requested before an NTP master ever initialized it. */
#define L3_RTC_ERR_INVALID_PARAMS           0xE1  /**< Provided calendar parameters failed structural range boundaries. */

// --- Layer 3 Network Exception Indicators ---
#define L3_ERR_INTER_FRAME_TIMEOUT          0x10  /**< Remote device started sending a big block but stopped halfway through. */
#define L3_ERR_SEQUENCE_CORRUPTION          0x11  /**< Packets landed out of sequence order (e.g., received slot 2 before slot 1). */
#define XCAN_L3_ERR_TOKEN_INCONSISTENCY     0x12  /**< Middle sub-frame matching identifiers unexpectedly mutated mid-stream. */

// --- System Plane Protocol Command Enumerations ---
/**
 *  Internal command system codes for the Control Plane. These tell the background system handler 
 * what kind of maintenance task is happening inside a received control envelope.
 */
enum L3CtrlType : uint8_t {
    L3_CTRL_RESERVED       = 0,
    L3_CTRL_PING_REQ       = 1, /**< Diagnostic latency request verification. Bytes 1-4: Unique token. */
    L3_CTRL_PING_RESP      = 2, /**< Diagnostic latency echo reply. Echoes back the precise verification token. */
    L3_CTRL_HEARTBEAT      = 5, /**< Cyclical uptime packet. Bytes 1-4: System uptime tracked in continuous seconds. */
    XCAN_L3_CTRL_NTP_MASTER = 6  /**< Time synchronization broadcast. Bytes 1-4: Standard Global Unix Epoch time in seconds. */
};

// --- Real-time Clock Structural Definitions ---
/**
 *  A human-readable calendar layout structure used to set or read the system clock.
 */
struct mtTimeStructure {
    uint8_t  seconds;      /**< Seconds tracking window [0 to 59] */
    uint8_t  minutes;      /**< Minutes tracking window [0 to 59] */
    uint8_t  hours;        /**< Hours tracking window [0 to 23] */
    uint8_t  dayOfWeeks;   /**< Day index of the active week [0 = Sunday to 6 = Saturday] */
    uint8_t  day;          /**< Day index of the active month [1 to 31] */
    uint8_t  month;        /**< Month index of the active year [1 to 12] */
    uint16_t year;         /**< Full absolute calendar year notation [Strictly Validated Range: 2000 to 2099] */
};

// --- Microsecond Diagnostics Ping Structural Elements ---
/**
 *  A tracker structure stored inside the local memory matrix to log details of an ongoing ping.
 * When we send a ping, we record a unique random token number and save the exact microsecond timestamp 
 * of when it left. When the receiver echoes it back, we subtract the old timestamp from the current clock 
 * to find the exact round-trip travel time.
 */
struct PingTracker {
    uint32_t echoVerificationToken; /**< Unique sequence check code sent in the frame payload */
    uint32_t transmitTimestampUs;   /**< Microsecond capture clock marking the transmission exit point */
    uint32_t expirationTimestampMs; /**< Absolute millisecond timestamp when this individual ping officially expires */
    uint32_t connectionToken;       /**< Saved original outgress token so we can pass it back to the callback during timeouts */
    bool     isActiveSlot;          /**< True if this tracker slot is waiting for a reply; false if empty/free. */
};

// --- Layer 3 Channel Diagnostics Telemetry Block ---
/**
 *  Built-in scoreboard counters tracking the overall health, performance, and line error stats 
 * for each individual multi-frame routing lane.
 */
struct L3ChannelStats {
    uint32_t completedPackets;     /**< Total number of large fragmented letters successfully reassembled with no errors. */
    uint32_t timeoutEvents;        /**< Total times a remote device went silent mid-transmission and dumped the stream. */
    uint32_t structuralMismatches; /**< Total times frames showed up out of structural sequencing order. */
    uint32_t tokenInconsistencies; /**< Total times address validation headers randomly shifted mid-stream. */
};

// --- Layer 3 Tracking States (Zero-Allocation Pipeline Layout) ---
/**
 * @struct L3SessionControl
 * @brief Manages the active step-by-step assembly tracking information for a multi-frame reception lane.
 * *  When multi-frame packets hit a lane, we need to remember state without pausing execution. 
 * This records when the last chunk arrived (to catch timeouts), how many frames we are expecting in total, 
 * and uses bit-masks (where each bit represents a frame slot index) to check off frames as they arrive.
 */
struct L3SessionControl {
    uint32_t lastFrameTimestampMs;  /**< Millisecond log of the last valid frame entry for timeout tracking. */
    uint8_t  expectedFramesCount;   /**< Absolute total sub-frame puzzle pieces required to declare completion. */
    uint8_t  receivedFramesMask;    /**< Dynamic bitfield marking which slots have physically landed in memory. */
    uint8_t  processedFramesMask;   /**< Bitfield tracking background updates to avoid repeating parsing routines. */
};


class mtCANL3Engine {
public:
    // --- Constructor & Lifecycle Constraints ---
    /** @brief Connects the Layer 3 management engine to its underlying Layer 2 hardware network frame router. */
    explicit mtCANL3Engine(mtCANL2Engine& l2Driver);
    ~mtCANL3Engine() = default;

    // Disallow copies of this engine to protect internal tracking states from duplication bugs
    mtCANL3Engine(const mtCANL3Engine&) = delete;
    mtCANL3Engine& operator=(const mtCANL3Engine&) = delete;

    /** @brief Resets and initializes trackers, statistics, real-time variables, and diagnostic ping logs to zero. */
    void init();

    // --- Core High-Performance Transmit Interfaces ---
    /** @brief Main API to slice and send a large data letter (up to 63 bytes) across the Application Plane. */
    uint8_t send(uint32_t connectionToken, const uint8_t* payload, uint8_t size);
    /** @brief Sends a small management system packet (up to 7 bytes) across the Control Plane. */
    uint8_t send_ctrl(uint32_t connectionToken, L3CtrlType type, const uint8_t* payload, uint8_t size);
    /** @brief This is a special API that allows upper layers to tunnel control messages through the Layer 3 control interface. */
    uint8_t tunnel_send_ctrl(uint32_t connectionToken, const uint8_t* payload, uint8_t size); 

    // --- Core Autonomous Protocol Handlers ---
    /** @brief Dispatches a diagnostic latency tracking request frame to a designated node. 
     * Maximum outstanding pings: 4
     * Ping sequence IDs must be unique among active requests targeting the same node. */
    uint8_t send_ping(uint32_t connectionToken, uint8_t internalSequenceId);

    void clear_ping_slot(uint8_t slotIdx);

    /** @brief Transmits a master synchronization unix clock payload to all network devices. */
    uint8_t send_ntp_master(uint32_t connectionToken, uint32_t unix_seconds);

    // --- Continuous Ingress Background Processing Engines ---
    /** @brief Scans and reconstructs fragmented multi-frame packets residing inside the application data matrix. */
    void process_application_rx_pipeline();
    /** @brief Checks the system matrix circular queue queue for incoming ping requests, replies, or clock synchronizations. */
    void process_control_rx_pipeline();

    // --- High-Fidelity Real-Time Clock (RTC) API ---
    /** @brief Calibrates the internal tracking clock using standard calendar date configurations. */
    uint8_t set_system_time(const mtTimeStructure& newTimeSetting);
    /** @brief Directly sets the internal clock reference line using a raw 32-bit global Unix timestamp. */
    uint8_t set_system_time_from_epoch(uint32_t unixEpochSeconds);
    /** @brief Calculates and extracts current time back into human-readable calendar format. */
    uint8_t get_system_time(mtTimeStructure& outputTargetBuffer) const;
    /** @brief Returns true if an external NTP master has validated the system clock since boot. */
    bool    is_time_synchronized() const { return m_timeIsValid; }

    // --- Diagnostic Telemetry Portal ---
    /** @brief Provides thread-safe read access to structural error logs and scoreboard counters for an active lane. */
    inline const L3ChannelStats& get_lane_statistics(uint8_t laneIdx) const {
        return m_laneStats[laneIdx & 0x0F]; // Clamped safety bitmask bound
    }

private:
    mtCANL2Engine& m_l2Engine; /**< Reference linkage to physical layer 2 frame transmission driver engine. */
    L3SessionControl m_appTrackers[TOTAL_APP_CONN]; /**< Tracking arrays for the 16 parallel assembly streams. */
    L3ChannelStats   m_laneStats[TOTAL_APP_CONN];   /**< Performance tracking scoreboards for the 16 assembly lanes. */

    // --- High-Precision Internal Temporal Variables ---
    volatile uint32_t m_baseUnixEpochSeconds;  /**< Base anchor timestamp loaded from last synchronization event. */
    volatile uint32_t m_baseEpochSyncMillis;   /**< Local microchip clock tick count caught at the moment of synchronization. */
    volatile bool     m_timeIsValid;            /**< Flag tracking if calendar synchronization is confirmed active. */

    // --- Diagnostics Storage ---
    PingTracker m_pingRegistry[L3_MAX_PING_TRACKERS]; /**< Storage registry for up to 4 concurrent outbound pings. */

    // --- New Core State Tracking Variables for Stall Handling ---
    uint32_t m_stallGraceStartMs;  /**< Tracks the millisecond clock when the 4-slot jam condition was first reached */
    bool     m_isStallClockRunning; /**< Tracks whether the 500ms group purge countdown is currently active */

    /** * @brief Math conversion calculation to figure out exactly how many individual 8-byte postcards are needed.
     *  The first frame loses 1 byte to the Layer 3 size header, leaving 7 bytes for content. 
     * Subsequent frames have the full 8 bytes available. This arithmetic handles that calculation exactly.
     */
    inline uint8_t calculate_expected_frames(uint8_t l4Size) const {
        return (l4Size + 1 + 7) / 8;
    }

};

// --- Upper Layer Abstract Callback Contracts ---
/**
 *  These are "hooks" or event notification targets. When Layer 3 finishes assembling a whole letter, 
 * caught an error, or evaluated a ping, it calls these functions to hand the results up to the higher application code.
 */
extern void callback_L3_ctrl(uint32_t connectionToken, uint8_t type, const uint8_t* payload, uint8_t size);
extern void callback_L4_ctrl(uint32_t connectionToken, uint8_t baseHeader, const uint8_t* payload, uint8_t size);
extern void callback_L4_rX(uint32_t connectionToken, uint8_t* targetBuffer, uint8_t size);
extern void callback_L4_error(uint32_t connectionToken, uint8_t errorCode);
extern void callback_ping_report(uint32_t connectionToken, uint8_t sequenceId, uint32_t roundTripTimeUs);

#endif // MT_CAN_L3_ENGINE_H