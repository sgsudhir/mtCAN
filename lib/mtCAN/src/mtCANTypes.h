/**
 * @file mtCANTypes.h
 * @brief High-Density, Firmware Type Definitions and Bit-Packing Macros for the mtCAN Architecture.
 *
 * This header encapsulates the configuration constants, status enumerations, structural layouts,
 * and bit-manipulation macros used across the isolated dual-plane bxCAN driver engine.
 * Every macro definition maintains absolute operational equivalence to the baseline configuration.
 * It contains the critical definition for CanBusErrorState to resolve cross-compilation errors.
 */

#ifndef MT_CAN_TYPES_H
#define MT_CAN_TYPES_H

#pragma once

#include <stdint.h>
#include <stddef.h>

/* ============================================================================
 * MATRIX STORAGE DIMENSIONS & CONFIGURATION CONSTANTS
 * ============================================================================ */

/** @brief Allocation ceiling for the Application Layer matrix frame slots (16 connections * 8 frames/seq). */
#define TOTAL_APP_FRAMES   128

/** @brief Allocation ceiling for the System/Control Layer matrix circular queue frame buffer. */
#define TOTAL_CTRL_FRAMES  32

/** @brief Total network address resolution capacity for individual nodes on the bus grid. */
#define MAX_NETWORK_NODES  64

/** @brief Allocation ceiling for the concurrent Parallel Ingress Routing Table Engine. */
#define TOTAL_APP_CONN     16


/* ============================================================================
 * NETWORK PROTOCOL 2-BIT PRIORITIZATION SCHEMES
 * ============================================================================ */
#define XCAN_PRIORITY_HIGH       0x01  /**< High priority level indicator for urgent system notifications. */
#define XCAN_PRIORITY_MID        0x02  /**< Mid priority level indicator for routine operational parameters. */
#define XCAN_PRIORITY_LOW        0x03  /**< Low priority level indicator for bulk non-critical data transfers. */


/* ============================================================================
 * STREAM TYPE INDICATOR BITS
 * ============================================================================ */
#define XCAN_ST_STREAM         0      
#define XCAN_ST_STANDARD       1      


/* ============================================================================
 * ISOLATED ARCHITECTURAL NETWORK PLANES
 * ============================================================================ */
#define XCAN_PLANE_CONTROL        0     /**< System/control Plane identifier for control/handshake/system traffic (FIFO 0). */
#define XCAN_PLANE_APPLICATION   1     /**< Application Plane identifier for data streaming blocks (FIFO 1). */


/* ============================================================================
 * PROTOCOL SYSTEM STRUCTURE LAYOUTS & STATUS ENUMS
 * ============================================================================ */

/**
 * @enum L2FlowControlState
 * @brief Represents the software-level transmit backpressure state of the application plane ring buffer.
 */
enum L2FlowControlState {
    L2_FLOW_OK = 0,         /**< Outbound pipeline under nominal threshold capacity; data acceptance allowed. */
    L2_FLOW_CONGESTED = 1   /**< Outbound queue exceeded high watermark limit; application flow choked. */
};

/**
 * @enum CanBusErrorState
 * @brief Categorizes the current state of the bxCAN physical bus line error counters (TEC and REC).
 *
 * This enumeration directly corresponds to the hardware operational status bits from the 
 * bxCAN Error Status Register (ESR) and mirrors the states logged within the telemetry variables.
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
 * Optimized to match standard 32-bit internal MCU alignment, splitting the 8-byte CAN payload into
 * two separate word registers (dataLow and dataHigh) to achieve maximum copy speed inside ISR routines.
 */
struct CanMatrixFrame {
    uint32_t id29;          /**< Raw 29-bit Extended CAN Identifier containing bit-packed protocol metadata. */
    uint32_t dataLow;       /**< Least Significant 4 Bytes of the payload data field (Bytes 0, 1, 2, 3). */
    uint32_t dataHigh;      /**< Most Significant 4 Bytes of the payload data field (Bytes 4, 5, 6, 7). */
};

/**
 * @struct CanFrameL2
 * @brief Raw architectural format utilized by the Layer 2 Software Ring Buffers before hardware transmission.
 */
struct CanFrameL2 {
    uint32_t id29;          /**< Clean 29-bit Extended CAN Identifier for direct injection into hardware registers. */
    uint8_t  length;        /**< Data Length Code (DLC) bounding the physical frame size payload [0 to 8]. */
    uint8_t  data[8];       /**< Continuous 8-byte array buffer carrying the actual message fragment payload. */
};


/* ============================================================================
 * MASK, SHIFT, AND GLOBAL SIGNATURE ARCHITECTURAL SPECIFICATIONS
 * ============================================================================ */
#define M_MAX_MULTICAST_GROUPS 64              /**< Dimensional bounds for multicast tracking allocations. */
#define M_GLOBAL_BROADCAST_NODE_ID 127         /**< Hardware destination address mapping all network points. */
#define XCAN_PROTOCOL_SIGNATURE 0x1F           /**< Topmost 5-bit validation token defining mtCAN architecture. */
#define M_HW_SALT_CONSTANT 0xDEADBEEFUL        /**< Verification magic value used to evaluate warm boot contexts. */

#define XCAN_SHIFT_SEQUENCE 0                  /**< Bit offset positioning for the 3-bit sequence field. */
#define XCAN_SHIFT_SRC_NODE 3                  /**< Bit offset positioning for the 7-bit transmitter node ID. */
#define XCAN_SHIFT_DST_NODE 10                 /**< Bit offset positioning for the 7-bit receiver destination ID. */
#define XCAN_SHIFT_MODE 17                     /**< Bit offset positioning for the 2-bit routing topology layout. */
#define XCAN_SHIFT_STREAM_TYPE 19              /**< Bit offset positioning for the 1-bit Stream Status bit flag. */
#define XCAN_SHIFT_MSG_TYPE 20                 /**< Bit offset positioning for the 1-bit Dual-Plane mapping flag. */
#define XCAN_SHIFT_PRIORITY 21                 /**< Bit offset positioning for the 3-bit frame priority field. */
#define XCAN_SHIFT_SIGNATURE 24                /**< Bit offset positioning for the 5-bit protocol verification signature. */

#define XCAN_MASK_SEQUENCE 0x00000007UL        /**< Extraction mask selecting the lower 3 bits for frame sequencing. */
#define XCAN_MASK_SRC_NODE 0x000003F8UL        /**< Extraction mask isolating bits 3-9 for node source identification. */
#define XCAN_MASK_DST_NODE 0x0001FC00UL        /**< Extraction mask isolating bits 10-16 for receiver targeting maps. */
#define XCAN_MASK_MODE 0x00060000UL            /**< Extraction mask isolating bits 17-18 for structural routing configurations. */
#define XCAN_MASK_STREAM_TYPE 0x00080000UL     /**< Extraction mask isolating bit 19 for streaming state checking. */
#define XCAN_MASK_MSG_TYPE 0x00100000UL        /**< Extraction mask isolating bit 20 for plane routing configuration. */
#define XCAN_MASK_PRIORITY 0x00E00000UL        /**< Extraction mask isolating bits 21-23 for priority mapping. */
#define XCAN_MASK_SIGNATURE 0x1F000000UL       /**< Extraction mask isolating bits 24-28 for protocol verification. */

#define XCAN_MODE_UNICAST 1                    /**< Bitwise configuration evaluation tag indicating Unicast mode. */
#define XCAN_MODE_MULTICAST 2                  /**< Bitwise configuration evaluation tag indicating Multicast mode. */
#define XCAN_MODE_BROADCAST 3                  /**< Bitwise configuration evaluation tag indicating Broadcast mode. */


/* ============================================================================
 * FAST INLINE CONSTRUCTORS AND TRANSLATION PROTOCOLS
 * ============================================================================ */

/**
 * @brief Bit-packs individual protocol fields into a standardized 29-bit Extended CAN Identifier.
 *
 * Executes raw bit shifting and logical OR operations to compile the hardware-ready ID.
 * Extracted ranges are clamped via bitwise masks to guarantee zero overlap corruption.
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


#endif /* MT_CAN_TYPES_H */