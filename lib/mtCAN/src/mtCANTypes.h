/**
 * @file mtCANTypes.h
 * @brief High-Density, Firmware Type Definitions and Bit-Packing Macros for the mtCAN Architecture.
 *
 * LAYMAN EXPLANATION:
 * This file acts as a global master dictionary for our custom CAN network. When electronics
 * talk to each other over a CAN bus, they share data in small chunks called "frames". This file 
 * describes the exact structure of those frames, the rules of who gets to speak first (Priority),
 * and how we pack a bunch of information (like who sent the message, who is supposed to receive it,
 * and what part of a larger multi-frame puzzle it belongs to) into a single 29-bit identification number.
 * * Think of it as defining the size of the envelopes and the formatting rules for the addresses 
 * written on them.
 */

#ifndef MT_CAN_TYPES_H
#define MT_CAN_TYPES_H

#pragma once

#include <stdint.h>
#include <stddef.h>

/* ============================================================================
 * MATRIX STORAGE DIMENSIONS & CONFIGURATION CONSTANTS
 * ============================================================================ */

/** * @brief Allocation ceiling for the Application Layer matrix frame slots (16 connections * 8 frames/seq). 
 * LAYMAN: This defines the absolute maximum number of sub-frames we can store for larger messages.
 * 16 parallel channels multiplied by 8 sequential slots each equals 128 total tracking frames.
 */
#define TOTAL_APP_FRAMES   128

/** * @brief Allocation ceiling for the System/Control Layer matrix circular queue frame buffer. 
 * LAYMAN: A dedicated pool of 32 slots reserved strictly for network management tasks like 
 * checking connections (pings) and setting clocks (time synchronization).
 */
#define TOTAL_CTRL_FRAMES  32

/** * @brief Total network address resolution capacity for individual nodes on the bus grid. 
 * LAYMAN: This represents the total number of hardware devices (nodes) allowed on this CAN network grid (0 to 63).
 */
#define MAX_NETWORK_NODES  64

/** * @brief Allocation ceiling for the concurrent Parallel Ingress Routing Table Engine. 
 * LAYMAN: This allows up to 16 completely separate, simultaneous multi-frame message streams 
 * to be actively received and assembled at the exact same time without breaking order.
 */
#define TOTAL_APP_CONN     16


/* ============================================================================
 * NETWORK PROTOCOL 2-BIT PRIORITIZATION SCHEMES
 * ============================================================================ */
/**
 * LAYMAN: When multiple devices try to speak at the same exact millisecond on a CAN bus, 
 * the lower priority number automatically wins the hardware tug-of-war and goes first.
 */
#define XCAN_PRIORITY_HIGH       0x01  /**< High priority level indicator for urgent system notifications. */
#define XCAN_PRIORITY_MID        0x02  /**< Mid priority level indicator for routine operational parameters. */
#define XCAN_PRIORITY_LOW        0x03  /**< Low priority level indicator for bulk non-critical data transfers. */


/* ============================================================================
 * STREAM TYPE INDICATOR BITS
 * ============================================================================ */
/**
 * LAYMAN: Determines if a message is a continuous raw data stream (like audio/sensor dumps)
 * or if it's a standard standalone packet that requires strict data boundaries.
 */
#define XCAN_ST_STREAM         0      
#define XCAN_ST_STANDARD       1      


/* ============================================================================
 * ISOLATED ARCHITECTURAL NETWORK PLANES
 * ============================================================================ */
/**
 * LAYMAN: To keep things running smoothly, we split our network into two virtual "freeways":
 * 1. Control Plane: Handles commands, heartbeats, and coordination. Uses Hardware FIFO 0.
 * 2. Application Plane: Handles bulk data, user payload messages. Uses Hardware FIFO 1.
 */
#define XCAN_PLANE_CONTROL        0     /**< System/control Plane identifier for control/handshake/system traffic (FIFO 0). */
#define XCAN_PLANE_APPLICATION   1     /**< Application Plane identifier for data streaming blocks (FIFO 1). */


// --- Direct Node Address Properties ---
#define M_MAX_UNICAST_NODE_ID   31         /**< Maximum address bound for 00b prefix Unicast paths */
#define M_GLOBAL_BROADCAST_NODE_ID 127     /**< LAYMAN: The magical target ID that means "Everyone listen up! This message is for everyone." */

/* ============================================================================
 * PROTOCOL SYSTEM STRUCTURE LAYOUTS & STATUS ENUMS
 * ============================================================================ */

/**
 * @enum L2FlowControlState
 * @brief Represents the software-level transmit backpressure state of the application plane ring buffer.
 * * LAYMAN: This is our internal traffic light system. If our hardware transmission buffers are wide open, 
 * the state is OK. If they get filled up with pending outgoing messages, we set it to CONGESTED 
 * so our application layer slows down and stops trying to jam more data into the pipe.
 */
enum L2FlowControlState {
    L2_FLOW_OK = 0,         /**< Outbound pipeline under nominal threshold capacity; data acceptance allowed. */
    L2_FLOW_CONGESTED = 1   /**< Outbound queue exceeded high watermark limit; application flow choked. */
};

/**
 * @enum CanBusErrorState
 * @brief Categorizes the current state of the bxCAN physical bus line error counters (TEC and REC).
 *
 * LAYMAN: Microcontrollers have dedicated physical hardware monitoring the health of the CAN wires. 
 * If a wire gets snipped, loose, or suffers electromagnetic interference, the chip increases error counters. 
 * This tracking enum lets our code monitor those counts so we know if the bus is perfectly healthy (ACTIVE), 
 * glitching slightly (WARNING), heavily failing (PASSIVE), or completely shut down by the chip's internal breakers (OFF).
 */
enum CanBusErrorState {
    CAN_BUS_ERROR_ACTIVE = 0,   /**< Normal operating condition; hardware error counters are below 96. */
    CAN_BUS_ERROR_WARNING = 1,  /**< Error Warning flag; at least one internal counter has breached 96. */
    CAN_BUS_ERROR_PASSIVE = 2,  /**< Error Passive status; at least one counter has breached 127. Node cannot send dominate bits. */
    CAN_BUS_ERROR_OFF = 3       /**< Bus-Off condition; Transmit Error Counter has exceeded 255. Hardware is offline. */
};

/**
 * @struct CanMatrixFrame
 * @brief Hardware-independent memory footprint representation of data frames inside the global storage matrices.
 *
 * LAYMAN: A raw CAN frame can hold up to 8 bytes of raw content. For maximum execution speed on modern 
 * 32-bit microcontrollers, it is much faster to slice those 8 bytes into two 32-bit words (dataLow for 
 * the first 4 bytes, and dataHigh for the remaining 4 bytes). This layout mirrors that high-speed optimization 
 * alongside the 29-bit CAN tracking ID.
 */
struct CanMatrixFrame {
    uint32_t id29;          /**< Raw 29-bit Extended CAN Identifier containing bit-packed protocol metadata. */
    uint32_t dataLow;       /**< Least Significant 4 Bytes of the payload data field (Bytes 0, 1, 2, 3). */
    uint32_t dataHigh;      /**< Most Significant 4 Bytes of the payload data field (Bytes 4, 5, 6, 7). */
};

/**
 * @struct CanFrameL2
 * @brief Raw architectural format utilized by the Layer 2 Software Ring Buffers before hardware transmission.
 * * LAYMAN: This is standard format for an outbound envelope right before it is loaded into the microcontroller's 
 * physical radio transmitter registers. It keeps the ID, the true size of the payload (Length: 0 to 8 bytes), 
 * and a direct, standard byte array containing the data.
 */
struct CanFrameL2 {
    uint32_t id29;          /**< Clean 29-bit Extended CAN Identifier for direct injection into hardware registers. */
    uint8_t  length;        /**< Data Length Code (DLC) bounding the physical frame size payload [0 to 8]. */
    uint8_t  data[8];       /**< Continuous 8-byte array buffer carrying the actual message fragment payload. */
};


/* ============================================================================
 * MASK, SHIFT, AND GLOBAL SIGNATURE ARCHITECTURAL SPECIFICATIONS
 * ============================================================================ */
/**
 * LAYMAN: This is where the magic layout occurs. A 29-bit integer is just a sequence of 29 ones and zeros. 
 * To pack multiple pieces of information into a single number, we slice up those bits like slices of pie.
 * * Visual Layout of the 29-bit Header:
 * [ Signature (5b) ][ Priority (3b) ][ MsgType (1b) ][ StreamType (1b) ][ Mode (2b) ][ DstNode (7b) ][ SrcNode (7b) ][ Sequence (3b) ]
 * * The "SHIFT" macros tell the computer how many slots to move a value to the left to put it in its right slice.
 * The "MASK" macros act as templates to clean out everything else when we try to read that specific slice.
 */

#define M_MAX_MULTICAST_GROUPS 64              /**< Dimensional bounds for multicast tracking allocations. */
#define XCAN_PROTOCOL_SIGNATURE 0x1F           /**< Topmost 5-bit validation token defining mtCAN architecture. All our packets must start with this signature! */
#define M_HW_SALT_CONSTANT 0xDEADBEEFUL        /**< Verification magic value used to evaluate warm boot contexts. */

#define XCAN_SHIFT_SEQUENCE 0                  /**< Sequence number occupies bits 0 to 2 (Positions 0, 1, 2). */
#define XCAN_SHIFT_SRC_NODE 3                  /**< Source Node ID occupies bits 3 to 9 (7 bits wide). */
#define XCAN_SHIFT_DST_NODE 10                 /**< Destination Node ID occupies bits 10 to 16 (7 bits wide). */
#define XCAN_SHIFT_MODE 17                     /**< Routing Mode (Unicast/Multicast/Broadcast) occupies bits 17 and 18 (2 bits wide). */
#define XCAN_SHIFT_STREAM_TYPE 19              /**< Stream indicator flag occupies bit position 19 (1 bit wide). */
#define XCAN_SHIFT_MSG_TYPE 20                 /**< Message type / Plane selection (Control vs App) occupies bit position 20 (1 bit wide). */
#define XCAN_SHIFT_PRIORITY 21                 /**< Priority level occupies bits 21 to 23 (3 bits wide). */
#define XCAN_SHIFT_SIGNATURE 24                /**< System Verification Signature occupies bits 24 to 28 (5 bits wide). */

#define XCAN_MASK_SEQUENCE 0x00000007UL        /**< Binary mask: 00000000000000000000000000000111 (Isolates Sequence) */
#define XCAN_MASK_SRC_NODE 0x000003F8UL        /**< Binary mask: 00000000000000000000001111111000 (Isolates Source) */
#define XCAN_MASK_DST_NODE 0x0001FC00UL        /**< Binary mask: 00000000000000011111110000000000 (Isolates Destination) */
#define XCAN_MASK_MODE 0x00060000UL            /**< Binary mask: 00000000000001100000000000000000 (Isolates Mode) */
#define XCAN_MASK_STREAM_TYPE 0x00080000UL     /**< Binary mask: 00000000000010000000000000000000 (Isolates Stream Flag) */
#define XCAN_MASK_MSG_TYPE 0x00100000UL        /**< Binary mask: 00000000000100000000000000000000 (Isolates Plane Flag) */
#define XCAN_MASK_PRIORITY 0x00E00000UL        /**< Binary mask: 00000000111000000000000000000000 (Isolates Priority) */
#define XCAN_MASK_SIGNATURE 0x1F000000UL       /**< Binary mask: 00011111000000000000000000000000 (Isolates Protocol Signature) */

#define XCAN_MODE_UNICAST 1                    /**< 1 = Message is point-to-point (one specific sender to one specific receiver). */
#define XCAN_MODE_MULTICAST 2                  /**< 2 = Message is point-to-group (sent to a designated subset group of devices). */
#define XCAN_MODE_BROADCAST 3                  /**< 3 = Message is global broadcast (sent to every single device on the bus wiring). */


/* ============================================================================
 * FAST INLINE CONSTRUCTORS AND TRANSLATION PROTOCOLS
 * ============================================================================ */

/**
 * @brief Bit-packs individual protocol fields into a standardized 29-bit Extended CAN Identifier.
 *
 * LAYMAN: This utility function is our "envelope address builder". You hand it all the distinct items—like 
 * who you are, who you're talking to, how important the message is, and what chunk index it is—and it shifts 
 * and glues them together using bitwise OR operations into one massive 29-bit integer that CAN hardware accepts.
 */
inline uint32_t mtCAN_PacketId29(
    uint8_t signature,
    uint8_t priority,
    uint8_t msgType,
    uint8_t streamType,
    uint8_t mode,
    uint8_t dstNode,
    uint8_t srcNode,
    uint8_t seqNum)
{
    return
        (((uint32_t)(signature  & 0x1F)) << XCAN_SHIFT_SIGNATURE)   |
        (((uint32_t)(priority   & 0x07)) << XCAN_SHIFT_PRIORITY)    |
        (((uint32_t)(msgType    & 0x01)) << XCAN_SHIFT_MSG_TYPE)    |
        (((uint32_t)(streamType & 0x01)) << XCAN_SHIFT_STREAM_TYPE) |
        (((uint32_t)(mode       & 0x03)) << XCAN_SHIFT_MODE)        |
        (((uint32_t)(dstNode    & 0x7F)) << XCAN_SHIFT_DST_NODE)    |
        (((uint32_t)(srcNode    & 0x7F)) << XCAN_SHIFT_SRC_NODE)    |
        (((uint32_t)(seqNum     & 0x07)) << XCAN_SHIFT_SEQUENCE);
}

extern uint8_t localNodeId;

/**
 * @brief Creates a standardized outgress token targeting a specific remote destination node.
 * * Automatically handles local node identification, plane routing defaults, 
 * streaming types, sequencing defaults, and routing topologies.
 * * @param destNode The targeted remote receiving node ID (0-63).
 * @param priority Application priority value (typically 2-7).
 * @return uint32_t The fully packed 29-bit outgress CAN token.
 */
inline uint32_t create_outgress_token(uint8_t destNode, uint8_t priority) {
    // Defaults: ST = 0 (XCAN_ST_STREAM), MT = 1 (XCAN_PLANE_APPLICATION)
    // Mode = XCAN_MODE_UNICAST (1), Seq = 0
    return mtCAN_PacketId29(
        XCAN_PROTOCOL_SIGNATURE, // 5-bit native verification signature
        priority,                // User-defined arbitration priority
        XCAN_PLANE_APPLICATION,  // MT default = 1
        XCAN_ST_STREAM,          // ST default = 0
        XCAN_MODE_UNICAST,       // Mode is always unicast (1)
        destNode,                // Destination node
        localNodeId,             // Internal source node injection
        0                        // Sequence field defaults to 0
    );
}

/**
 * @brief Validates an outbound/outgress token against the strict structural rules.
 * * Checks source node context, destination boundaries, priority constraints, 
 * signature correctness, unicast mode, and a zeroed sequence field. Ignores MT and ST entirely.
 * * @param token The raw 29-bit CAN token to evaluate.
 * @return true If the token satisfies all outgress rules; false otherwise.
 */
inline bool validate_outgress_token(uint32_t token) {
    // 1. Verify Protocol Signature
    if ((token & XCAN_MASK_SIGNATURE) != ((uint32_t)XCAN_PROTOCOL_SIGNATURE << XCAN_SHIFT_SIGNATURE)) {
        return false;
    }

    // 2. Verify Mode is strictly Unicast
    uint8_t mode = (uint8_t)((token & XCAN_MASK_MODE) >> XCAN_SHIFT_MODE);
    if (mode != XCAN_MODE_UNICAST) {
        return false;
    }

    // 3. Verify Sequence field is strictly 0
    if ((token & XCAN_MASK_SEQUENCE) != 0) {
        return false;
    }

    // 4. Verify srcNode belongs to this specific local node
    uint8_t srcNode = (uint8_t)((token & XCAN_MASK_SRC_NODE) >> XCAN_SHIFT_SRC_NODE);
    if (srcNode != localNodeId) {
        return false;
    }

    // 5. Verify destNode fits within valid address boundaries
    uint8_t destNode = (uint8_t)((token & XCAN_MASK_DST_NODE) >> XCAN_SHIFT_DST_NODE);
    if (destNode >= MAX_NETWORK_NODES) {
        return false;
    }

    // 6. Verify priority matches the Application space constraints (2 to 7)
    uint8_t priority = (uint8_t)((token & XCAN_MASK_PRIORITY) >> XCAN_SHIFT_PRIORITY);
    if (priority < 2 || priority > 7) {
        return false;
    }

    return true;
}

/**
 * @brief Validates an inbound/ingress token received from the bus.
 * * Mirror validation matching outgress token requirements, ensuring the target 
 * is the local node (or a valid broad/multicast routing) and signature matches.
 */
inline bool validate_ingress_token(uint32_t token) {
    if ((token & XCAN_MASK_SIGNATURE) != ((uint32_t)XCAN_PROTOCOL_SIGNATURE << XCAN_SHIFT_SIGNATURE)) return false;
    if ((token & XCAN_MASK_SEQUENCE) != 0) return false;
    
    uint8_t destNode = (uint8_t)((token & XCAN_MASK_DST_NODE) >> XCAN_SHIFT_DST_NODE);
    // Ingress targets this node explicitly
    if (destNode != localNodeId) {
        return false;
    }
    
    return true;
}

/**
 * @brief Swaps the Source and Destination fields of a verified outgress token to create an ingress token.
 * * @param outgressToken The outbound token to flip.
 * @return uint32_t The resulting ingress token, or 0 if validation fails.
 */
inline uint32_t convert_outgress_to_ingress(uint32_t outgressToken) {
    if (!validate_outgress_token(outgressToken)) {
        return 0; // Guard against blind or corrupt conversions
    }

    uint32_t srcNodeBits = (outgressToken & XCAN_MASK_SRC_NODE) >> XCAN_SHIFT_SRC_NODE;
    uint32_t dstNodeBits = (outgressToken & XCAN_MASK_DST_NODE) >> XCAN_SHIFT_DST_NODE;

    // Clear old routing lanes
    uint32_t flippedToken = outgressToken & ~(XCAN_MASK_SRC_NODE | XCAN_MASK_DST_NODE);

    // Re-inject reversed mappings
    flippedToken |= (srcNodeBits << XCAN_SHIFT_DST_NODE); // Old source becomes new destination
    flippedToken |= (dstNodeBits << XCAN_SHIFT_SRC_NODE); // Old destination becomes new source

    return flippedToken;
}

/**
 * @brief Swaps the Source and Destination fields of a verified ingress token to target the sender back.
 * * @param ingressToken The incoming token to flip.
 * @return uint32_t The resulting outgress token, or 0 if validation fails.
 */
inline uint32_t convert_ingress_to_outgress(uint32_t ingressToken) {
    if (!validate_ingress_token(ingressToken)) {
        // Enforce checking instead of blind switching
        return 0; 
    }

    uint32_t srcNodeBits = (ingressToken & XCAN_MASK_SRC_NODE) >> XCAN_SHIFT_SRC_NODE;
    uint32_t dstNodeBits = (ingressToken & XCAN_MASK_DST_NODE) >> XCAN_SHIFT_DST_NODE;

    // Clear old routing lanes
    uint32_t flippedToken = ingressToken & ~(XCAN_MASK_SRC_NODE | XCAN_MASK_DST_NODE);

    // Re-inject reversed mappings
    flippedToken |= (srcNodeBits << XCAN_SHIFT_DST_NODE);
    flippedToken |= (dstNodeBits << XCAN_SHIFT_SRC_NODE);

    // Ensure priority complies with the outgress app boundary rules (default fallback if needed)
    uint8_t priority = (uint8_t)((flippedToken & XCAN_MASK_PRIORITY) >> XCAN_SHIFT_PRIORITY);
    if (priority < 2) {
        flippedToken &= ~XCAN_MASK_PRIORITY;
        flippedToken |= ((uint32_t)2 << XCAN_SHIFT_PRIORITY); // Force to app-compliant default priority 2
    }

    return flippedToken;
}


/**
 * @brief Prepares an Application Token for transmitting Control/System frames.
 * * Forces MT = 0 (Control Plane) and constraints priority to 2 (Control Default).
 * * @param appToken The clean application token to convert.
 * * @param priority Optional override for control frame priority (default = 2).
 * @return uint32_t System-ready transmission identifier.
 */
inline uint32_t prepare_control_frame_id(uint32_t appToken, uint8_t priority = 2) {
    // 1. Strip the Sequence field, MT field, and Priority field
    uint32_t ctrlToken = appToken & ~(XCAN_MASK_SEQUENCE | XCAN_MASK_MSG_TYPE | XCAN_MASK_PRIORITY);

    // 2. Force MT = 0 (XCAN_PLANE_CONTROL)
    // 3. Force Priority = 2 (Control space defaults to 2, allows 1 for explicit overrides)
    ctrlToken |= ((uint32_t)XCAN_PLANE_CONTROL << XCAN_SHIFT_MSG_TYPE);
    ctrlToken |= ((uint32_t)priority << XCAN_SHIFT_PRIORITY);
    
    // 4. Sequence defaults to 0 (already cleared above)
    return ctrlToken;
}




#endif /* MT_CAN_TYPES_H */