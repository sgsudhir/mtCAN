/**
 * @file mtCANTypes.h
 * @brief High-Density, Firmware Type Definitions and Bit-Packing Macros for the mtCAN Architecture.
 *
 * This file acts as the foundational layer defining how data frames are formatted, structured, 
 * and verified across the network bus. It defines the structural layout of 29-bit Extended 
 * CAN Identifiers (CAN ID) and provides rapid translation utilities for moving between network 
 * driver tokens, source/destination node addressing, and operational priority lanes.
 * * Designed for high-reliability embedded control environments where network synchronization,
 * message ordering, and collision avoidance are non-negotiable prerequisites.
 */

#ifndef MT_CAN_TYPES_H
#define MT_CAN_TYPES_H

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string.h>

/* ============================================================================
 * MATRIX STORAGE DIMENSIONS & CONFIGURATION CONSTANTS
 * ============================================================================ */

/**
 * @brief The absolute maximum number of multi-client communication slots managed simultaneously.
 * Denotes how many independent sessions the software stack can track in parallel memory.
 */
#define L4_MAX_PARALLEL_SESSIONS            8

/**
 * @brief Mirror mapping of total application layer connections allowed at any one instance.
 * Directly dependent on the hardware slot limitations.
 */
#define TOTAL_APP_CONN                      (L4_MAX_PARALLEL_SESSIONS)

/**
 * @brief Total pool size for unique physical nodes addressable within this bus network topology.
 * Allocates room for node identifiers ranging sequentially from index 0 to 63.
 */
#define MAX_NETWORK_NODES                   64

/**
 * @brief Specific revision signature identifier tracking the wire format specification.
 * Used during handshake protocols to ensure protocol compatibility across different nodes.
 */
#define XCAN_PROTOCOL_VERSION_V1            0x01

#define M_HW_SALT_CONSTANT                  0xDEADBEEFUL        

/* ============================================================================
 * NETWORK PROTOCOL 2-BIT PRIORITIZATION SCHEMES
 * ============================================================================ */

/**
 * @brief Urgent/Real-Time message priority allocation flag (Highest transmission preference).
 * Used for critical telemetry updates, diagnostic overrides, or emergency events.
 */
#define XCAN_PRIORITY_HIGH                  0x01

/**
 * @brief Nominal/Standard operational message priority allocation flag.
 * Used for standard data telemetry frames that require consistent tracking but lack strict real-time deadlines.
 */
#define XCAN_PRIORITY_MID                   0x02

/**
 * @brief Background/Bulk message priority allocation flag (Lowest transmission preference).
 * Used for massive file transfers, debugging blocks, or secondary telemetry logs.
 */
#define XCAN_PRIORITY_LOW                   0x03

/* ============================================================================
 * STREAM TYPE INDICATOR BITS
 * ============================================================================ */

/**
 * @brief Standard Data Frame Indicator. Represents single-packet transmissions or independent messages.
 */
#define XCAN_ST_STANDARD                    0      

/**
 * @brief Streaming Segment Indicator. Indicates the packet belongs to a continuous data stream.
 */
#define XCAN_ST_STREAM                      1

/* ============================================================================
 * ISOLATED ARCHITECTURAL NETWORK PLANES
 * ============================================================================ */

/**
 * @brief Plane assigned for handshake commands, session negotiations, and keep-alive tracking.
 * Maps to hardware FIFO 0 to isolate network control planes from throughput traffic bottlenecks.
 */
#define XCAN_PLANE_CONTROL                  0     

/**
 * @brief Plane optimized for heavy data payload streams, multi-segment arrays, and file transfers.
 * Maps to hardware FIFO 1 to handle maximum throughput without interrupting signaling mechanisms.
 */
#define XCAN_PLANE_APPLICATION              1     

/**
 * @brief Maximum permitted ID for a single, uniquely addressable recipient device.
 */
#define M_MAX_UNICAST_NODE_ID               31          

/**
 * @brief Universal broadcast address targeting all active listening nodes on the bus.
 */
#define M_GLOBAL_BROADCAST_NODE_ID          127         

/**
 * @brief Flow Control Status reporting options utilized by Layer 2 driver abstractions.
 */
enum L2FlowControlState {
    L2_FLOW_OK = 0,         /**< Transmitter/Receiver buffer is clear; communication may proceed normally. */
    L2_FLOW_CONGESTED = 1   /**< Hardware/Software queues are full; backing off transmission immediately. */
};

/**
 * @brief Hardware-level error monitoring flags tracking physical CAN bus electrical stability.
 */
enum CanBusErrorState {
    CAN_BUS_ERROR_ACTIVE = 0,   /**< Normal healthy bus; device actively participates in error flag signaling. */
    CAN_BUS_ERROR_WARNING = 1,  /**< Error counters elevated; slight wire noise or transient signal distortion detected. */
    CAN_BUS_ERROR_PASSIVE = 2,  /**< Error counters critical; device listens but cannot forcefully interrupt the bus. */
    CAN_BUS_ERROR_OFF = 3       /**< Controller has physically disconnected from the bus due to excessive transmission failure. */
};

/**
 * @brief Raw hardware-aligned memory map representation of a 29-bit CAN frame identifier split across registers.
 */
struct CanMatrixFrame {
    uint32_t id29;          /**< The packed 29-bit Extended Identifier integer. */
    uint32_t dataLow;       /**< Least Significant 32 bits (Bytes 0-3) of the physical packet data payload. */
    uint32_t dataHigh;      /**< Most Significant 32 bits (Bytes 4-7) of the physical packet data payload. */
};

/**
 * @brief High-level application driver structure containing a standard parsed Layer 2 data element.
 */
struct CanFrameL2 {
    uint32_t id29;          /**< Comprehensive 29-bit identification token containing packed protocol headers. */
    uint8_t  length;        /**< Actual payload data length size (ranges safely from 0 to 8 bytes for standard CAN). */
    uint8_t  data[8];       /**< Safe static array container holding raw byte arrays received or staged for TX. */
};

/* ============================================================================
 * MASK, SHIFT, AND GLOBAL SIGNATURE ARCHITECTURAL SPECIFICATIONS
 * ============================================================================ */
#define M_MAX_MULTICAST_GROUPS              64              
#define XCAN_PROTOCOL_SIGNATURE             0x1F           /**< Unique 5-bit signature checking valid mtCAN frames. */

// Bit-position shift alignments defining fields inside the 29-bit integer block
#define XCAN_SHIFT_SEQUENCE                 0                  
#define XCAN_SHIFT_SRC_NODE                 3                  
#define XCAN_SHIFT_DST_NODE                 10                 
#define XCAN_SHIFT_MODE                     17                 
#define XCAN_SHIFT_STREAM_TYPE              19                 
#define XCAN_SHIFT_MSG_TYPE                 20                 
#define XCAN_SHIFT_PRIORITY                 21                 
#define XCAN_SHIFT_SIGNATURE                24                 

// Extraction bit-masks matching the corresponding field widths inside the 29-bit frame
#define XCAN_MASK_SEQUENCE                  0x00000007UL        /**< 3 bits: Tracks packet rolling sequence numbers (0-7). */
#define XCAN_MASK_SRC_NODE                  0x000003F8UL        /**< 7 bits: Extracts source sender hardware node ID. */
#define XCAN_MASK_DST_NODE                  0x0001FC00UL        /**< 7 bits: Extracts target destination node ID. */
#define XCAN_MASK_MODE                      0x00060000UL        /**< 2 bits: Addressing topology (Unicast/Multicast/Broadcast). */
#define XCAN_MASK_STREAM_TYPE               0x00080000UL        /**< 1 bit: Distinguishes Standard packets from Stream segments. */
#define XCAN_MASK_MSG_TYPE                  0x00100000UL        /**< 1 bit: Routes frame to Control or Application planes. */
#define XCAN_MASK_PRIORITY                  0x00E00000UL        /**< 3 bits: Defines message priority hierarchy (0-7 range). */
#define XCAN_MASK_SIGNATURE                 0x1F000000UL        /**< 5 bits: System validity safety tag verification. */

// Protocol transmission topologies defining recipient routing scopes
#define XCAN_MODE_UNICAST                   1                    /**< Direct Point-to-Point dedicated link conversation. */
#define XCAN_MODE_MULTICAST                 2                    /**< Group targeted distribution targeting custom device lists. */
#define XCAN_MODE_BROADCAST                 3                    /**< Global notification read simultaneously by every node on wire. */

/**
 * @brief Bit-packs individual structured protocol fields into a unified 29-bit Extended CAN Identifier.
 * @param signature Unique 5-bit identification key defining the proprietary network profile.
 * @param priority Application routing layer priority value (determines arbiter dominance on the wire).
 * @param msgType Functional plane routing value (0 for Control signaling, 1 for Application data payloads).
 * @param streamType Continuous segmentation flag (0 for independent single frames, 1 for active data streams).
 * @param mode Operational addressing target style (1=Unicast, 2=Multicast, 3=Broadcast).
 * @param dstNode Node network address identifier of the targeted receiving unit.
 * @param srcNode Node network address identifier of the host generating and sending this packet.
 * @param seqNum 3-bit rolling sequence frame indexing counter used to reassemble long payload segments.
 * @return Fully structured uint32_t representation of the packed extended CAN identifier field.
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

/**
 * @brief Global hardware tracking state holding the active assigned physical node address of this local device.
 */
extern uint8_t localNodeId;

/**
 * @brief Creates a standardized outbound 29-bit CAN identification token targeting a remote recipient.
 * @param destNode The targeted destination node address on the network bus.
 * @param priority System priority index applied to steer the arbitration urgency on the wire.
 * @return Ready-to-transmit uint32_t packed application plane token.
 */
inline uint32_t create_outgress_token(uint8_t destNode, uint8_t priority) {
    return mtCAN_PacketId29(
        XCAN_PROTOCOL_SIGNATURE, 
        priority,                
        XCAN_PLANE_APPLICATION,  
        XCAN_ST_STREAM,          
        XCAN_MODE_UNICAST,       
        destNode,                
        localNodeId,             
        0                        
    );
}

/**
 * @brief Validates an outbound 29-bit CAN identifier against strict structural sanity and ownership rules.
 * Ensures that the packet possesses a valid signature framework and matches our local identity ownership.
 * @param token The raw packed 29-bit identification code staged for outbound evaluation.
 * @return True if the token is completely safe to process and dispatch; False if malformed or unauthorized.
 */
inline bool validate_outgress_token(uint32_t token) {
    if ((token & XCAN_MASK_SIGNATURE) != ((uint32_t)XCAN_PROTOCOL_SIGNATURE << XCAN_SHIFT_SIGNATURE)) return false;
    uint8_t srcNode = (uint8_t)((token & XCAN_MASK_SRC_NODE) >> XCAN_SHIFT_SRC_NODE);
    if (srcNode != localNodeId) return false;
    uint8_t destNode = (uint8_t)((token & XCAN_MASK_DST_NODE) >> XCAN_SHIFT_DST_NODE);
    if (destNode >= MAX_NETWORK_NODES && destNode != M_GLOBAL_BROADCAST_NODE_ID) return false;
    return true;
}

/**
 * @brief Validates a raw inbound 29-bit CAN identifier pulled directly from physical transceiver controllers.
 * Confirms system signature validity and that the frame is directed to us (or is a valid network broadcast).
 * @param token The raw packed inbound identifier field evaluated for local stack consumption.
 * @return True if the frame targets this local processor topology; False if it belongs to a different node.
 */
inline bool validate_ingress_token(uint32_t token) {
    if ((token & XCAN_MASK_SIGNATURE) != ((uint32_t)XCAN_PROTOCOL_SIGNATURE << XCAN_SHIFT_SIGNATURE)) return false;
    uint8_t destNode = (uint8_t)((token & XCAN_MASK_DST_NODE) >> XCAN_SHIFT_DST_NODE);
    uint8_t mode = (uint8_t)((token & XCAN_MASK_MODE) >> XCAN_SHIFT_MODE);
    
    if (destNode != localNodeId && destNode != M_GLOBAL_BROADCAST_NODE_ID && mode != XCAN_MODE_BROADCAST) {
        return false;
    }
    return true;
}

/**
 * @brief Swaps the source and destination address fields inside an outbound token to generate its inbound counterpart.
 * Effectively creates the flipped mirror ID required to intercept replies coming back from the remote target.
 * @param outgressToken The active initialized outbound frame identifier tracking target parameters.
 * @return Flipped tracking token matching the incoming response layout, or 0 if validation fails.
 */
inline uint32_t convert_outgress_to_ingress(uint32_t outgressToken) {
    if (!validate_outgress_token(outgressToken)) return 0;
    uint32_t srcNodeBits = (outgressToken & XCAN_MASK_SRC_NODE) >> XCAN_SHIFT_SRC_NODE;
    uint32_t dstNodeBits = (outgressToken & XCAN_MASK_DST_NODE) >> XCAN_SHIFT_DST_NODE;
    uint32_t flippedToken = outgressToken & ~(XCAN_MASK_SRC_NODE | XCAN_MASK_DST_NODE);
    flippedToken |= (srcNodeBits << XCAN_SHIFT_DST_NODE);
    flippedToken |= (dstNodeBits << XCAN_SHIFT_SRC_NODE);
    return flippedToken;
}

/**
 * @brief Reverses address orientations on an incoming packet to construct a matching response identifier.
 * Ensures the responding frame satisfies all structural egress network rules, including fallback priority checking.
 * @param ingressToken The raw network-validated incoming message header token to be replied to.
 * @return A configured outbound token targeting the original sender node safely, or 0 if invalid.
 */
inline uint32_t convert_ingress_to_outgress(uint32_t ingressToken) {
    if (!validate_ingress_token(ingressToken)) return 0;
    uint32_t srcNodeBits = (ingressToken & XCAN_MASK_SRC_NODE) >> XCAN_SHIFT_SRC_NODE;
    uint32_t dstNodeBits = (ingressToken & XCAN_MASK_DST_NODE) >> XCAN_SHIFT_DST_NODE;
    uint32_t flippedToken = ingressToken & ~(XCAN_MASK_SRC_NODE | XCAN_MASK_DST_NODE);
    flippedToken |= (srcNodeBits << XCAN_SHIFT_DST_NODE);
    flippedToken |= (dstNodeBits << XCAN_SHIFT_SRC_NODE);
    
    uint8_t priority = (uint8_t)((flippedToken & XCAN_MASK_PRIORITY) >> XCAN_SHIFT_PRIORITY);
    if (priority < 2) {
        flippedToken &= ~XCAN_MASK_PRIORITY;
        flippedToken |= ((uint32_t)2 << XCAN_SHIFT_PRIORITY); 
    }
    return flippedToken;
}

/**
 * @brief Extracts the priority allocation field (which functions as the Connection ID) from an active stream token.
 * Validates that the requested index resides safely inside standard system operating constraints.
 * @param appToken The packed 29-bit active application plane identification tracking frame.
 * @return The extracted connection/priority ID (2-7), or 0xFF if the token format violates structural masks.
 */
inline uint8_t get_connection_id_from_app_token(uint32_t appToken) {
    if ((appToken & XCAN_MASK_SIGNATURE) != ((uint32_t)XCAN_PROTOCOL_SIGNATURE << XCAN_SHIFT_SIGNATURE)) return 0xFF;
    
    uint8_t connID = (uint8_t)((appToken & XCAN_MASK_PRIORITY) >> XCAN_SHIFT_PRIORITY);
    if (connID < 2 || connID > 7) return 0xFF;
    return connID;
}

/**
 * @brief Transforms a low-overhead Control Plane CAN ID into a high-capacity Application Plane streaming CAN ID.
 * Modifies structural routing flags while preserving vital routing configurations like destination addresses.
 * @param controlToken Valid control tracking token forming the base addressing envelope.
 * @param connID Requested target channel priority lane assignment (must reside strictly in the 2-7 range).
 * @return Configured application plane stream token integer, or 0 if context conditions fail.
 */
inline uint32_t convert_control_token_to_app_token(uint32_t controlToken, uint8_t connID) {
    if (connID < 2 || connID > 7) return 0;
    if ((controlToken & XCAN_MASK_SIGNATURE) != ((uint32_t)XCAN_PROTOCOL_SIGNATURE << XCAN_SHIFT_SIGNATURE)) return 0;
    
    uint32_t appToken = controlToken & ~(XCAN_MASK_PRIORITY | XCAN_MASK_MSG_TYPE | XCAN_MASK_STREAM_TYPE);
    appToken |= ((uint32_t)connID << XCAN_SHIFT_PRIORITY);
    appToken |= ((uint32_t)XCAN_PLANE_APPLICATION << XCAN_SHIFT_MSG_TYPE);
    appToken |= ((uint32_t)XCAN_ST_STREAM << XCAN_SHIFT_STREAM_TYPE);
    return appToken;
}

/**
 * @brief Transforms an Application Plane streaming CAN ID back into a Control Plane system CAN ID.
 * Used when signaling teardowns, abort notifications, or transport reset conditions.
 * @param appToken The operational data streaming identifier token.
 * @param priority The specific priority rank to overwrite inside the new control frame (defaults to 2).
 * @return Configured control signaling plane token integer.
 */
inline uint32_t convert_app_token_to_control_token(uint32_t appToken, uint8_t priority = 2) {
    uint32_t ctrlToken = appToken & ~(XCAN_MASK_SEQUENCE | XCAN_MASK_MSG_TYPE | XCAN_MASK_PRIORITY);
    ctrlToken |= ((uint32_t)XCAN_PLANE_CONTROL << XCAN_SHIFT_MSG_TYPE);
    ctrlToken |= ((uint32_t)priority << XCAN_SHIFT_PRIORITY);
    return ctrlToken;
}

#endif /* MT_CAN_TYPES_H */