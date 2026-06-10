/**
 * @file mtCANTypes.h
 * @brief Layer 2 Protocol Frame Definitions and Bitfield Layout Configurations.
 * * This file acts as the single source of truth for the custom 29-bit Extended 
 * CAN identifier protocol mapping used across the network architecture. It handles 
 * the exact bit-shifting and bitmask boundaries required to slice and pack a standard 
 * CAN 29-bit arbitration field into distinct logical networking domains (Priority, 
 * Message Type, Mode, Destination, Source, and Frame Sequence Numbers).
 */

#ifndef MT_CAN_TYPES_H
#define MT_CAN_TYPES_H

#pragma once

#include <stdint.h>
#include <stddef.h>

/** * @section Networking Topology Constants
 * These definitions establish the physical limits and verification patterns 
 * of the custom Layer 2 protocol implementation on the wire.
 */

/// @brief Maximum allowable concurrent multicast subscription groups tracking per node.
#define M_MAX_MULTICAST_GROUPS 64

/// @brief Dedicated physical node identifier reserved to signal a global network broadcast.
#define M_GLOBAL_BROADCAST_NODE_ID 127

/// @brief Protocol Verification Token placed in the most significant bits of the 29-bit ID.
#define XCAN_PROTOCOL_SIGNATURE 0x1F

/// @brief Magic verification token used to validate the sanity of non-initialized SRAM locations across soft-resets.
#define M_HW_SALT_CONSTANT 0xDEADBEEFUL

/** * @section Bitfield Position Shift Offsets
 * These define the exact bit positions where each logical protocol component begins 
 * within the raw 32-bit container holding the 29-bit Extended identifier.
 */

/// @brief Bits [2:0] - Frame Sequence Offset within a multi-packet fragmented stream.
#define XCAN_SHIFT_SEQUENCE 0

/// @brief Bits [9:3] - Unique Identifier of the transmitting physical node.
#define XCAN_SHIFT_SRC_NODE 3

/// @brief Bits [16:10] - Unique Identifier of the target receiving node (or multicast group ID).
#define XCAN_SHIFT_DST_NODE 10

/// @brief Bits [18:17] - Transmission Type identifier (Unicast, Multicast, or Broadcast).
#define XCAN_SHIFT_MODE 17

/// @brief Bit [19] - Stream Classifier flag separating burst sequences from single commands.
#define XCAN_SHIFT_STREAM_TYPE 19

/// @brief Bit [20] - Message Domain separation flag (System vs Application plane).
#define XCAN_SHIFT_MSG_TYPE 20

/// @brief Bits [23:21] - CAN Arbitration Priority field (0x00 is highest priority on wire).
#define XCAN_SHIFT_PRIORITY 21

/// @brief Bits [28:24] - Protocol Verification Token position.
#define XCAN_SHIFT_SIGNATURE 24

/// @brief Explicit offset helper targeting the second bit bitfield of the priority group.
#define XCAN_SHIFT_PRIORITY_BIT2 23

/** * @section Bitfield Isolation Masks
 * Unshifted masks matching the exact width and spacing of each specific field domain.
 * Applying these via bitwise AND allows extraction or isolation of single properties.
 */

/// @brief Extracts 3 bits allocated to packet grouping tracking sequences.
#define XCAN_MASK_SEQUENCE 0x00000007UL

/// @brief Extracts 7 bits allocated to identify the original source node (0 to 127).
#define XCAN_MASK_SRC_NODE 0x000003F8UL

/// @brief Extracts 7 bits allocated to identify the target node destination address.
#define XCAN_MASK_DST_NODE 0x0001FC00UL

/// @brief Extracts 2 bits defining structural routing mode context.
#define XCAN_MASK_MODE 0x00060000UL

/// @brief Extracts 1 bit managing streaming vs single transaction frames.
#define XCAN_MASK_STREAM_TYPE 0x00080000UL

/// @brief Extracts 1 bit establishing the data plane (0=System, 1=Application).
#define XCAN_MASK_MSG_TYPE 0x00100000UL

/// @brief Extracts 3 bits governing physical arbitration priority inside CAN mailboxes.
#define XCAN_MASK_PRIORITY 0x00E00000UL

/// @brief Extracts 5 bits comprising the foundational protocol verification code.
#define XCAN_MASK_SIGNATURE 0x1F000000UL

/** * @section Core Communication Modes
 * Logical mappings encoded inside the XCAN_MASK_MODE field.
 */

/// @brief Specifies a point-to-point direct message intended for a single node ID.
#define XCAN_MODE_UNICAST 1

/// @brief Specifies a group-addressed message targeting nodes subscribed to a shared bucket.
#define XCAN_MODE_MULTICAST 2

/// @brief Specifies an open network frame intended for consumption by every online host.
#define XCAN_MODE_BROADCAST 3

/**
 * @enum CanBusErrorState
 * @brief Categorizes physical CAN bus health states derived from hardware error counters.
 * * Mirrors the internal state machine of the bxCAN controller based on the Transmit
 * Error Counter (TEC) and Receive Error Counter (REC) registers.
 */
enum CanBusErrorState {
    /// @brief Normal Operation: Both error counters are below 96.
    CAN_BUS_ERROR_ACTIVE  = 0,
    /// @brief Threshold Exceeded: At least one error counter has crossed 96.
    CAN_BUS_ERROR_WARNING = 1,
    /// @brief Passive Restriction: At least one error counter passed 127. Node cannot actively destroy frames.
    CAN_BUS_ERROR_PASSIVE = 2,
    /// @brief Bus Off Condition: Transmit counter passed 255. Node is chemically isolated from the differential wire.
    CAN_BUS_ERROR_OFF     = 3
};

/**
 * @struct CanFrameL2
 * @brief Standardized Software Layout for an Outbound Transmission Queue Frame.
 * * Decouples raw hardware registers from software buffering mechanisms during data transit passes.
 */
struct CanFrameL2 {
    uint32_t id29;    ///< Raw 29-bit identifier compiled using protocol shifts and bit fields.
    uint8_t  length;  ///< Data Length Code (DLC) expressing payload length from 0 to 8 bytes.
    uint8_t  data[8]; ///< Static memory footprint allocated to hold the CAN frame payload payload.
};

/**
 * @struct CanWarmPersistentL2Context
 * @brief Preservation Layout for Hot-Restart In-Flight Telemetry Assets.
 * * Maps variables that must survive temporary physical micro-controller resets without losing 
 * established network group identities.
 */
struct CanWarmPersistentL2Context {
    uint8_t  activeGroupIds[10];           ///< Active physical multicast group memberships retained during reset.
    uint16_t allocatedSlotsBitmask;         ///< Spatial allocations mapped across structural tracking arrays.
    uint8_t  multicastArrayInitializedToken; ///< Validation byte indicating context survival vs fresh boot.
};

/**
 * @brief Synthesizes a raw, hardware-compliant 29-bit Extended CAN ID from individual domains.
 * * Takes distinct protocol elements, bounds-checks them using logical AND configurations, 
 * shifts them to their correct positional slot, and logically combines them.
 * * @param signature Protocol validation field (must be 0x1F).
 * @param priority Frame prioritization rank (0=Highest, 7=Lowest).
 * @param msgType Frame plane target (0=System, 1=Application).
 * @param streamType Transfer pattern flag.
 * @param mode Core structural routing type (Unicast/Multicast/Broadcast).
 * @param dstNode Target recipient address node ID.
 * @param srcNode Generating node ID.
 * @param seqNum Packet order tracker index within stream.
 * @return uint32_t Compiled 29-bit CAN arbitration field identifier ready for bxCAN register entry.
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

#endif