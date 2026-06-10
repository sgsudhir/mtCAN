/**
 * @file mtCANL2Engine.cpp
 * @brief Register Implementation for STM32 bxCAN Direct-Mapped Matrix Engine.
 * * Implements the hardware setup operations, filter configurations, 
 * atomic queue operations, and low-level interrupt routing matrices 
 * for the Layer 2 engine.
 */

#include "mtCANL2Engine.h"

/**
 * @struct CanMatrixFrame
 * @brief High-efficiency, zero-copy unpacked structure for direct storage on matrix planes.
 * * Maps raw 64-bit frame data fields into split 32-bit registers (Low/High data components) 
 * matching the hardware register architecture of bxCAN mailbox buffers for rapid zero-copy copies.
 */
struct CanMatrixFrame {
    uint32_t id29;     ///< Contains the full extracted 29-bit architecture protocol identifier.
    uint32_t dataLow;  ///< Maps raw bytes [0:3] exactly as written inside hardware CAN_RDLxR registers.
    uint32_t dataHigh; ///< Maps raw bytes [4:7] exactly as written inside hardware CAN_RDHxR registers.
};

/**
 * @struct NodeStatus
 * @brief In-memory layout representing real-time telemetry metrics of a remote network participant.
 */
struct NodeStatus {
    volatile uint32_t lastSeenTimestampMs; ///< Absolute timestamp (millis) marking last valid wire event from node.
    uint8_t  onlineState;                  ///< Network status state machine tracking flag.
    uint8_t  deviceType;                   ///< Identifier byte tracking class categorization code of node.
    uint16_t firmwareVersion;              ///< Packed representation showing operational code version of node.
};

/* --- Global Network Matrix Planes Layout definitions --- */
#define TOTAL_APP_FRAMES   256
#define TOTAL_SYS_FRAMES   64
#define MAX_NETWORK_NODES  128

/**
 * @brief Matrix tracking plane mapping for application packets.
 * Indexed via calculation coordinate: (g_ActiveSessionRouter[srcNodeId] << 4) | (seqNum & 0x0F)
 */
CanMatrixFrame g_ApplicationMatrix[TOTAL_APP_FRAMES];

/**
 * @brief System plane storage matrix operating as a fast circular buffer.
 * Processes high-priority parameters, configurations, and handshakes via FIFO 0.
 */
CanMatrixFrame g_SystemMatrix[TOTAL_SYS_FRAMES];

/**
 * @brief Active Connection Routing Map.
 * Maps raw physical remote Node IDs (0-127) to internal allocated Session Row Indices (0-15).
 * Unallocated slots must strictly contain default token value 0xFF.
 */
volatile uint8_t g_ActiveSessionRouter[MAX_NETWORK_NODES];

/**
 * @brief Global Node Vital Telemetry Matrix tracking presence and health profiles across the bus.
 */
volatile NodeStatus g_NetworkNodeStatusMatrix[MAX_NETWORK_NODES];

/// @brief Singleton storage tracking running instance to allow routing from standard C interrupt handlers.
static mtCANL2Engine* g_pL2EngineInstance = nullptr;

/** * @section Persistent Non-Volatile RAM Declarations
 * Places core tracking metrics inside a memory region (.noinit) that is skipped over 
 * during standard C-runtime startup reset routines, allowing metric survival across crashes.
 */
__attribute__((section(".noinit"))) static uint32_t persistentNodeRestartCounter;
__attribute__((section(".noinit"))) static uint32_t persistentValidityMagicToken;

/**
 * @brief Standard wrapper matching internal micro-controller execution clock cycles.
 * @return uint32_t Upward counting execution timestamp expressed in milliseconds.
 */
uint32_t mtCANL2Engine::get_portable_system_timestamp_ms() {
    return (uint32_t)millis();
}

/**
 * @brief Class Constructor initializing telemetry structures and recovering non-volatile crash contexts.
 */
mtCANL2Engine::mtCANL2Engine() :
    m_nodeId(0),
    m_nodeBootCounter(0),
    m_commLossTimeoutThresholdMs(5000),
    m_sysMatrixHead(0),
    m_sysMatrixTail(0),
    m_txHead(0),
    m_txTail(0),
    m_flowState(L2_FLOW_OK),
    m_blockedTimestamp(0),
    m_congestionWarningActive(false),
    m_congestionWarningStrikes(0),
    m_lastWireActivityTimestamp(0),
    m_totalTransmittedFrames(0),
    m_totalReceivedFrames(0),
    m_fifo0OverrunEvents(0),
    m_fifo1OverrunEvents(0),
    m_txArbitrationLostEvents(0),
    m_txErrorHardwareEvents(0),
    m_busOffTransitions(0),
    m_errorPassiveTransitions(0),
    m_errorWarningTransitions(0),
    m_mailboxStallRecoveries(0),
    m_telemetryFirewallDrops(0),
    m_systemMatrixDroppedFrames(0),
    m_sourceQuenchTriggerPending(false),
    m_lastTelemetrySampleTick(0),
    m_lastTotalFrameCount(0),
    m_cachedErrorState(CAN_BUS_ERROR_ACTIVE) {

    // Bind instance pointer to provide target referencing during C-coded NVIC triggers
    g_pL2EngineInstance = this;

    // Reset initial hardware tracking contexts
    m_mailboxArbitrationLossCount[0] = 0;
    m_mailboxArbitrationLossCount[1] = 0;
    m_mailboxArbitrationLossCount[2] = 0;

    m_mailboxTxTimestamp[0] = 0;
    m_mailboxTxTimestamp[1] = 0;
    m_mailboxTxTimestamp[2] = 0;

    // Validate if memory survived a warm reset by testing the custom validation token
    if (persistentValidityMagicToken == M_HW_SALT_CONSTANT) {
        persistentNodeRestartCounter++;
    } else {
        // Cold boot state: Memory is invalid; perform hard reset initialization
        persistentValidityMagicToken = M_HW_SALT_CONSTANT;
        persistentNodeRestartCounter = 1;
    }
    m_nodeBootCounter = persistentNodeRestartCounter;
}

/**
 * @brief Configures clocks, configures IO lines, defines filters, and launches bxCAN.
 */
bool mtCANL2Engine::initialize_hardware(uint8_t localNodeId, uint32_t commLossTimeoutMs) {
    // Structural Node validation check; physical IDs cannot exceed network limits
    if (localNodeId > 126) return false;
    m_nodeId = localNodeId;
    m_commLossTimeoutThresholdMs = commLossTimeoutMs;

    // Initialize the Application Matrix storage cells
    for (uint32_t i = 0; i < TOTAL_APP_FRAMES; ++i) {
        g_ApplicationMatrix[i].id29 = 0;
        g_ApplicationMatrix[i].dataLow = 0;
        g_ApplicationMatrix[i].dataHigh = 0;
    }
    // Initialize the System Matrix storage cells
    for (uint32_t i = 0; i < TOTAL_SYS_FRAMES; ++i) {
        g_SystemMatrix[i].id29 = 0;
        g_SystemMatrix[i].dataLow = 0;
        g_SystemMatrix[i].dataHigh = 0;
    }
    // Set Session Tracking mappings to default disconnected state token (0xFF)
    for (uint32_t i = 0; i < MAX_NETWORK_NODES; ++i) {
        g_ActiveSessionRouter[i] = 0xFF;
        g_NetworkNodeStatusMatrix[i].lastSeenTimestampMs = 0;
        g_NetworkNodeStatusMatrix[i].onlineState = 0;
        g_NetworkNodeStatusMatrix[i].deviceType = 0;
        g_NetworkNodeStatusMatrix[i].firmwareVersion = 0;
    }

    // Enable Peripheral Clocks: APB2 handles Alternate Function I/O and Port A; APB1 drives bxCAN1
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;

    // Reset pin configuration fields for PA11 (CAN_RX) and PA12 (CAN_TX)
    GPIOA->CRH &= ~(GPIO_CRH_MODE11 | GPIO_CRH_CNF11 | GPIO_CRH_MODE12 | GPIO_CRH_CNF12);
    // Pin 11 (RX): Input Floating Mode (CNF=01, MODE=00)
    GPIOA->CRH |= GPIO_CRH_CNF11_0;
    // Pin 12 (TX): Alternate Function Output Push-Pull 50MHz max speed (CNF=10, MODE=10)
    GPIOA->CRH |= GPIO_CRH_MODE12_1 | GPIO_CRH_CNF12_1;

    // Enforce default non-remapped pin placement configuration (PA11/PA12) inside AFIO tracking registers
    AFIO->MAPR &= ~AFIO_MAPR_CAN_REMAP;

    // Enter Hardware Initialization Mode to open write access to configuration registers
    CAN1->MCR |= CAN_MCR_INRQ;
    uint32_t timeoutGuard = 0xFFFF;
    // Block execution until bxCAN hardware reports entry authorization confirmation via the INAK flag
    while (!(CAN1->MSR & CAN_MSR_INAK)) {
        if (--timeoutGuard == 0) return false;
    }

    // Ensure sleep mode is disabled
    CAN1->MCR &= ~CAN_MCR_SLEEP;

    // Setup Bit Timing Profile: Target 250 Kbps configuration derived from 36MHz APB1 clock line
    CAN1->BTR = 0;
    // Prescaler=9 (BTR[3:0]=8), TS1=12 (BTR[19:16]=11), TS2=3 (BTR[22:20]=2), SJW=1 (BTR[25:24]=0)
    CAN1->BTR |= (8 << 0) | (11 << 16) | (2 << 20) | (0 << 24);

    // MCR Setup: ABOM=1 (Automatic Bus-Off Recovery Enabled), RFLM=1 (Receive FIFO Locked Mode Enabled)
    CAN1->MCR |= CAN_MCR_ABOM | CAN_MCR_RFLM;
    // NART=0 (Automatic Retransmission on collision enabled; standard wire arbitration protocol rule)
    CAN1->MCR &= ~CAN_MCR_NART;

    // Load filter banks matching protocol topology maps
    reconfigure_hardware_filters();

    // Interrupt Enabling: Turn on FIFO 0 Pending, FIFO 1 Pending, and Mailbox Transmit Empty flags
    CAN1->IER |= CAN_IER_FMPIE0 | CAN_IER_FMPIE1 | CAN_IER_TMEIE;

    // Bind and enable target vector channels inside the Core Nested Vectored Interrupt Controller (NVIC)
    NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0); // FIFO 0 handles high priority system plane traffic
    NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);

    NVIC_SetPriority(CAN1_RX1_IRQn, 1);        // FIFO 1 manages application matrix packet routing
    NVIC_EnableIRQ(CAN1_RX1_IRQn);

    NVIC_SetPriority(USB_HP_CAN1_TX_IRQn, 2);  // TX priority tracking layer
    NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);

    // Request transition out of initialization mode back into normal active wire execution mode
    CAN1->MCR &= ~CAN_MCR_INRQ;
    timeoutGuard = 0xFFFF;
    // Block until hardware drops INAK flag indicating active wire synchronization completed
    while (CAN1->MSR & CAN_MSR_INAK) {
        if (--timeoutGuard == 0) return false;
    }

    // Benchmark base operating timelines
    uint32_t currentTick = get_portable_system_timestamp_ms();
    m_lastTelemetrySampleTick = currentTick;
    m_lastWireActivityTimestamp = currentTick;

    m_mailboxTxTimestamp[0] = 0;
    m_mailboxTxTimestamp[1] = 0;
    m_mailboxTxTimestamp[2] = 0;

    // Cache physical error layer attributes
    uint32_t esr = CAN1->ESR;
    if (esr & CAN_ESR_BOFF)      m_cachedErrorState = CAN_BUS_ERROR_OFF;
    else if (esr & CAN_ESR_EPVF) m_cachedErrorState = CAN_BUS_ERROR_PASSIVE;
    else if (esr & CAN_ESR_EWGF) m_cachedErrorState = CAN_BUS_ERROR_WARNING;
    else                         m_cachedErrorState = CAN_BUS_ERROR_ACTIVE;

    return true;
}

/**
 * @brief Slices internal ID tracking criteria into physical bxCAN Filter Banks.
 * * Hardware Filter Mapping Architecture Logic:
 * - Bank 0: Unicast System Commands -> Direct to FIFO 0
 * - Bank 1: Broadcast System Commands -> Direct to FIFO 0
 * - Bank 2: Unicast Application Frame Data -> Direct to FIFO 1
 * - Bank 3: Broadcast Application Frame Data -> Direct to FIFO 1
 * - Banks 4-11: Multicast Application Streaming Nodes -> Direct to FIFO 1
 */
void mtCANL2Engine::reconfigure_hardware_filters() {
    // Assert Filter Initialization Mode flag to toggle operational parameters safely
    CAN1->FMR |= CAN_FMR_FINIT;

    // Deactivate all 14 filter channels to protect tracking changes during mutation steps
    CAN1->FA1R &= ~0x0FFFUL;

    // Setup configurations across banks 0 to 11: Set Single 32-bit Scale mode and Mask Mode layout
    for (uint8_t bank = 0; bank < 12; ++bank) {
        CAN1->FS1R |= (1UL << bank);  // FS1R bit set = Single 32-bit scale configuration
        CAN1->FM1R &= ~(1UL << bank); // FM1R bit cleared = Identifier Mask Mode layout
    }

    // --- BANK 0: Unicast System Frame Parsing Configuration ---
    // Targets Frame matching Signature=0x1F, MsgType=0 (System), Mode=1 (Unicast), Destination=Local Node ID.
    // Shifted left by 3 bits to align with high position mapping requirements of Extended IDs in FR1/FR2.
    uint32_t unicastSysId   = (0x1F000000UL | (0 << 20) | (1 << 17) | ((uint32_t)m_nodeId << 10)) << 3;
    uint32_t unicastSysMask = (0x1F100060UL | (0x0001FC00UL)) << 3; // Verify signature, plane, routing mode, destination ID
    CAN1->sFilterRegister[0].FR1 = unicastSysId | 4;   // Assertion of bit index 2 forces Extended Identifier tracking rules
    CAN1->sFilterRegister[0].FR2 = unicastSysMask | 4; 
    CAN1->FFA1R &= ~(1UL << 0);                        // Clear bit 0 = Route matches to hardware FIFO 0

    // --- BANK 1: Broadcast System Frame Parsing Configuration ---
    // Targets Frame matching Signature=0x1F, MsgType=0 (System), Mode=3 (Broadcast).
    uint32_t broadcastSysId   = (0x1F000000UL | (0 << 20) | (3 << 17)) << 3;
    uint32_t broadcastSysMask = (0x1F100060UL) << 3;   // Match signature, plane, and explicit broadcast routing mode token
    CAN1->sFilterRegister[1].FR1 = broadcastSysId | 4;
    CAN1->sFilterRegister[1].FR2 = broadcastSysMask | 4;
    CAN1->FFA1R &= ~(1UL << 1);                        // Route matches to hardware FIFO 0

    // --- BANK 2: Unicast Application Frame Parsing Configuration ---
    // Targets Frame matching Signature=0x1F, MsgType=1 (Application Data), Mode=1 (Unicast), Destination=Local Node ID.
    uint32_t unicastAppId   = (0x1F000000UL | (1 << 20) | (1 << 17) | ((uint32_t)m_nodeId << 10)) << 3;
    uint32_t unicastAppMask = (0x1F100060UL | (0x0001FC00UL)) << 3;
    CAN1->sFilterRegister[2].FR1 = unicastAppId | 4;
    CAN1->sFilterRegister[2].FR2 = unicastAppMask | 4;
    CAN1->FFA1R |= (1UL << 2);                         // Set bit 2 = Route matches to hardware FIFO 1

    // --- BANK 3: Broadcast Application Frame Parsing Configuration ---
    // Targets Frame matching Signature=0x1F, MsgType=1 (Application Data), Mode=3 (Broadcast).
    uint32_t broadcastAppId   = (0x1F000000UL | (1 << 20) | (3 << 17)) << 3;
    uint32_t broadcastAppMask = (0x1F100060UL) << 3;
    CAN1->sFilterRegister[3].FR1 = broadcastAppId | 4;
    CAN1->sFilterRegister[3].FR2 = broadcastAppMask | 4;
    CAN1->FFA1R |= (1UL << 3);                         // Route matches to hardware FIFO 1

    // --- BANKS 4-11: Multicast Application Plane Core Matrix Rules ---
    // Targets general application multicast frames (Mode=2, MsgType=1). Asserts open tracking layout.
    uint32_t multicastAppId   = (0x1F000000UL | (1 << 20) | (2 << 17) | (0x00010000UL)) << 3;
    uint32_t multicastAppMask = (0x1F100060UL | (0x00010000UL)) << 3;
    for (uint8_t bank = 4; bank < 12; ++bank) {
        CAN1->sFilterRegister[bank].FR1 = multicastAppId | 4;
        CAN1->sFilterRegister[bank].FR2 = multicastAppMask | 4;
        CAN1->FFA1R |= (1UL << bank);                  // Assign entire block array to hardware FIFO 1
    }

    // Re-engage active operational filtering bitmask sets across altered channels
    CAN1->FA1R |= 0x0FFFUL;
    // Release initialization lock to execute tracking optimizations inside live peripheral networks
    CAN1->FMR &= ~CAN_FMR_FINIT;
}

/**
 * @brief Severs hardware interface operations under explicit global interrupt masking block rules.
 */
void mtCANL2Engine::disconnect() {
    __disable_irq(); // Enforce absolute thread-safe atomic block execution pass
    // Strip peripheral level interrupt lines
    CAN1->IER &= ~(CAN_IER_FMPIE0 | CAN_IER_FMPIE1 | CAN_IER_TMEIE);
    
    // Unbind local vector allocations inside CPU NVIC controller
    NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
    NVIC_DisableIRQ(CAN1_RX1_IRQn);
    NVIC_DisableIRQ(USB_HP_CAN1_TX_IRQn);

    // Command bxCAN peripheral core back into quiescent low-power Initialization mode
    CAN1->MCR |= CAN_MCR_INRQ;
    uint32_t timeoutGuard = 0xFFFF;
    while (!(CAN1->MSR & CAN_MSR_INAK)) {
        if (--timeoutGuard == 0) break; // Escape constraint barrier loop if hardware hangs
    }
    __enable_irq(); // Re-open global processor execution pathways
}

/**
 * @brief Public interface wrapping driver disconnection execution passes.
 */
void mtCANL2Engine::shutdown_hardware() {
    disconnect();
}

/**
 * @brief Inserts an outbound tracking frame into the local software circular allocation ring buffer.
 */
bool mtCANL2Engine::enqueue_tx_frame(uint32_t id29, const uint8_t* sourceBuffer, uint8_t length) {
    // Structural Guard: Reject packet ingestion if the network pipeline is flagged as congested
    if (m_flowState == L2_FLOW_CONGESTED) return false;
    // Parameter Invariant Check: Length declaration must align with valid data references
    if (length > 0 && sourceBuffer == nullptr) return false;
    // Bounds Check: Raw input identifier must fall inside standard 29-bit architecture constraints
    if (id29 > 0x1FFFFFFFUL) return false;

    // Squeeze out specific parameters using protocol bit shifts and masks
    uint8_t extractedSignature = (uint8_t)((id29 & XCAN_MASK_SIGNATURE) >> XCAN_SHIFT_SIGNATURE);
    uint8_t extractedPriority = (uint8_t)((id29 & XCAN_MASK_PRIORITY) >> XCAN_SHIFT_PRIORITY);
    uint8_t extractedMode = (uint8_t)((id29 & XCAN_MASK_MODE) >> XCAN_SHIFT_MODE);
    uint8_t extractedSrcNode = (uint8_t)((id29 & XCAN_MASK_SRC_NODE) >> XCAN_SHIFT_SRC_NODE);

    // Protocol Validation Guards: Enforce strict structural criteria matching layer design specs
    if (extractedSignature != XCAN_PROTOCOL_SIGNATURE) return false;
    if (extractedPriority == 0 || extractedPriority > 7) return false;
    if (extractedMode == 0 || extractedMode > 3) return false;
    if (extractedSrcNode != m_nodeId) return false; // Spoofing Guard: Source ID field must match this local node address
    if (length > 8) return false;

    __disable_irq(); // Enter atomic state; guard indices from preemptive race modifications inside TX/RX vectors

    uint16_t currentHead = m_txHead;
    uint16_t nextHead = (currentHead + 1) & M_L2_TX_QUEUE_MASK; // Fast mask replaces standard modulo loops

    // Saturated Overrun Condition Check: Intercept head overtaking tail structure boundary indices
    if (nextHead == m_txTail) {
        m_flowState = L2_FLOW_CONGESTED;
        m_blockedTimestamp = get_portable_system_timestamp_ms();
        __enable_irq();
        return false;
    }

    // Dynamic High-Watermark Telemetry Check: Trigger backpressure if buffer utilization exceeds 85%
    uint16_t activeQueueDepth = (nextHead - m_txTail) & M_L2_TX_QUEUE_MASK;
    if (activeQueueDepth >= M_L2_TX_QUEUE_HIGH_WATERMARK) {
        m_flowState = L2_FLOW_CONGESTED;
        m_blockedTimestamp = get_portable_system_timestamp_ms();
    }

    // Perform copy of payload bytes into the software ring buffer cell allocation slot
    m_txRingBuffer[currentHead].id29 = id29;
    m_txRingBuffer[currentHead].length = length;
    for (uint8_t idx = 0; idx < length; ++idx) {
        m_txRingBuffer[currentHead].data[idx] = sourceBuffer[idx];
    }

    // Advance head index to confirm item insertion validation across consumers
    m_txHead = nextHead;
    
    // Manually push scheduling layer pass to drive hardware mailbox loading without waiting for standard vector trip
    handle_tx_interrupt_service_routine();

    __enable_irq(); // Relinquish atomic isolation block
    return true;
}

/**
 * @brief Core low-level hardware feeder routing frames from software buffers directly into bxCAN registers.
 */
void mtCANL2Engine::handle_tx_interrupt_service_routine() {
    uint32_t tsrRegister = CAN1->TSR; // Read hardware Transmit Status Register once to cache execution state flags
    uint32_t currentTick = get_portable_system_timestamp_ms();

    // --- Mailbox 0 Execution Complete Check ---
    if (tsrRegister & CAN_TSR_RQCP0) { // Request Completed Flag for Mailbox 0 active
        if (tsrRegister & CAN_TSR_TXOK0) { // Transmission matched clean hardware verification on wire
            m_totalTransmittedFrames++;
            m_lastWireActivityTimestamp = currentTick;
        }
        m_mailboxArbitrationLossCount[0] = 0; // Clear collision sequence counters on success paths
        CAN1->TSR |= CAN_TSR_RQCP0;           // Clear hardware flag by writing a logical 1 directly to register bit
    }
    // --- Mailbox 1 Execution Complete Check ---
    if (tsrRegister & CAN_TSR_RQCP1) {
        if (tsrRegister & CAN_TSR_TXOK1) {
            m_totalTransmittedFrames++;
            m_lastWireActivityTimestamp = currentTick;
        }
        m_mailboxArbitrationLossCount[1] = 0;
        CAN1->TSR |= CAN_TSR_RQCP1;
    }
    // --- Mailbox 2 Execution Complete Check ---
    if (tsrRegister & CAN_TSR_RQCP2) {
        if (tsrRegister & CAN_TSR_TXOK2) {
            m_totalTransmittedFrames++;
            m_lastWireActivityTimestamp = currentTick;
        }
        m_mailboxArbitrationLossCount[2] = 0;
        CAN1->TSR |= CAN_TSR_RQCP2;
    }

    // Re-sample TSR status flags to locate open hardware channels following clearing cycles
    tsrRegister = CAN1->TSR;

    // Loop through software queue records, streaming data down into the 3 bxCAN physical mailbox registers
    while (m_txTail != m_txHead) {
        CAN_TxMailBox_TypeDef* pTxMailbox = nullptr;
        uint8_t selectedMailboxIdx = 0xFF;

        // Assign processing pointers targeting lowest available free hardware mailbox index
        if (tsrRegister & CAN_TSR_TME0) { // Mailbox 0 Transmit Mailbox Empty flag active
            pTxMailbox = &CAN1->sTxMailBox[0];
            selectedMailboxIdx = 0;
        } else if (tsrRegister & CAN_TSR_TME1) { // Mailbox 1 Transmit Mailbox Empty flag active
            pTxMailbox = &CAN1->sTxMailBox[1];
            selectedMailboxIdx = 1;
        } else if (tsrRegister & CAN_TSR_TME2) { // Mailbox 2 Transmit Mailbox Empty flag active
            pTxMailbox = &CAN1->sTxMailBox[2];
            selectedMailboxIdx = 2;
        }

        // Hardware constraint boundary hit: All 3 physical transmission registers are loaded with packets
        if (pTxMailbox == nullptr) break;

        uint16_t currentTail = m_txTail;
        CanFrameL2* sourceFrame = &m_txRingBuffer[currentTail];

        // Benchmark exact timestamp marking initialization entry step to drive timeout watchdogs
        m_mailboxTxTimestamp[selectedMailboxIdx] = currentTick;

        // Configure frame Data Length Code fields inside the hardware Transmit Data Length register
        pTxMailbox->TDTR &= ~CAN_TDT0R_DLC; // Strip structural configuration width mask bits
        pTxMailbox->TDTR |= (sourceFrame->length & CAN_TDT0R_DLC);

        // Pack individual 8-bit software array bytes into high/low split 32-bit registers via bitwise logical shifts
        uint32_t dataLowValue = ((uint32_t)sourceFrame->data[3] << 24) | ((uint32_t)sourceFrame->data[2] << 16) | ((uint32_t)sourceFrame->data[1] << 8) | ((uint32_t)sourceFrame->data[0]);
        uint32_t dataHighValue = ((uint32_t)sourceFrame->data[7] << 24) | ((uint32_t)sourceFrame->data[6] << 16) | ((uint32_t)sourceFrame->data[5] << 8) | ((uint32_t)sourceFrame->data[4]);
        pTxMailbox->TDLR = dataLowValue;   // Load bytes [0:3] directly to peripheral hardware Low Data Register
        pTxMailbox->TDHR = dataHighValue;  // Load bytes [4:7] directly to peripheral hardware High Data Register

        // Compile and write identifier maps to peripheral Transmit Identifier Register.
        // Bit 2 (IDE Extended Identifier Selection) forced active; TXRQ (Transmit Request Bit) tripped 
        // to pass arbitration priority command mechanics over to the hardware scheduler engine.
        uint32_t tirRegisterValue = (sourceFrame->id29 << 3) | CAN_TI0R_IDE | CAN_TI0R_TXRQ;
        pTxMailbox->TIR = tirRegisterValue;

        // Advance tail tracker to clear current frame block entry across tracking allocations
        m_txTail = (currentTail + 1) & M_L2_TX_QUEUE_MASK;

        // Explicitly flip internal structural bitmasks to signal that selected mailbox is now tracking data
        if (selectedMailboxIdx == 0)      tsrRegister &= ~CAN_TSR_TME0;
        else if (selectedMailboxIdx == 1) tsrRegister &= ~CAN_TSR_TME1;
        else if (selectedMailboxIdx == 2) tsrRegister &= ~CAN_TSR_TME2;
    }

    // Low-Watermark Backpressure Check: Release congestion blocks once queue usage drops below 55%
    if (m_flowState == L2_FLOW_CONGESTED) {
        uint16_t currentCount = (m_txHead - m_txTail) & M_L2_TX_QUEUE_MASK;
        if (currentCount <= M_L2_TX_QUEUE_LOW_WATERMARK) {
            m_flowState = L2_FLOW_OK;
        }
    }
}

/**
 * @brief High-efficiency, direct memory-mapped routing engine splitting incoming packets across double bxCAN FIFOs.
 */
void mtCANL2Engine::handle_rx_interrupt_service_routine(uint8_t hardwareFifoIndex) {
    if (hardwareFifoIndex > 1) return; // Escape check targeting impossible hardware allocation profiles

    // Select register references matching active source processing stream
    CAN_FIFOMailBox_TypeDef* pFifoMailbox = &CAN1->sFIFOMailBox[hardwareFifoIndex];
    uint32_t currentTick = get_portable_system_timestamp_ms();

    // --- Critical Hardware Overrun Diagnostics Check ---
    if (hardwareFifoIndex == 0) {
        if (CAN1->RF0R & CAN_RF0R_FOVR0) { // Hardware FIFO 0 Overrun condition flag tripped on wire
            m_fifo0OverrunEvents++;
            CAN1->RF0R |= CAN_RF0R_FOVR0;   // Clear overflow error flag bit directly in register layout
        }
    } else {
        if (CAN1->RF1R & CAN_RF1R_FOVR1) { // Hardware FIFO 1 Overrun condition flag tripped on wire
            m_fifo1OverrunEvents++;
            CAN1->RF1R |= CAN_RF1R_FOVR1;
        }
    }

    // Squeeze out current packet allocation density counts waiting within physical peripheral registers
    uint32_t rfrRegister = (hardwareFifoIndex == 0) ? CAN1->RF0R : CAN1->RF1R;
    uint8_t pendingFramesCount = (hardwareFifoIndex == 0) ? (rfrRegister & CAN_RF0R_FMP0) : (rfrRegister & CAN_RF1R_FMP1);

    // Drain every structural packet element waiting inside hardware mailbox allocations
    while (pendingFramesCount > 0) {
        uint32_t nirRegister = pFifoMailbox->RIR; // Read raw Receive Identifier Register structure

        if (nirRegister & CAN_RI0R_IDE) { // Invariant Check: Verify frame uses Extended ID layout
            uint32_t incomingId29 = (nirRegister >> 3) & 0x1FFFFFFFUL; // Extract clean 29-bit architecture field
            uint8_t srcNodeId = (uint8_t)((incomingId29 & XCAN_MASK_SRC_NODE) >> XCAN_SHIFT_SRC_NODE);

            // If incoming packet matches a valid physical network footprint node, refresh its vital heartbeat timer
            if (srcNodeId < MAX_NETWORK_NODES) {
                g_NetworkNodeStatusMatrix[srcNodeId].lastSeenTimestampMs = currentTick;
            }

            m_totalReceivedFrames++;
            m_lastWireActivityTimestamp = currentTick;

            // =================================================================
            // --- BRANCH 0: SYSTEM PLANE DETECTED (MAPPED VIA HARDWARE FIFO 0) ---
            // =================================================================
            if (hardwareFifoIndex == 0) {
                uint8_t sysHead = m_sysMatrixHead;
                uint8_t nextSysHead = (sysHead + 1) & (TOTAL_SYS_FRAMES - 1);
                
                // System Queue Overrun Validation: Intercept boundary overlaps within System circular array space
                if (nextSysHead == m_sysMatrixTail) {
                    m_systemMatrixDroppedFrames++;
                    m_sourceQuenchTriggerPending = true; // Set atomic flag to request backpressure choke on next loop pass
                } else {
                    // Execute zero-copy snapshot load of data directly from peripheral register map assets
                    g_SystemMatrix[sysHead].id29     = incomingId29;
                    g_SystemMatrix[sysHead].dataLow  = pFifoMailbox->RDLR; // Fetch low data bytes [0:3]
                    g_SystemMatrix[sysHead].dataHigh = pFifoMailbox->RDHR; // Fetch high data bytes [4:7]
                    m_sysMatrixHead = nextSysHead;                        // Commit producer sequence index advance
                }
            } 
            // =======================================================================
            // --- BRANCH 1: APPLICATION PLANE DETECTED (MAPPED VIA HARDWARE FIFO 1) ---
            // =======================================================================
            else {
                if (srcNodeId < MAX_NETWORK_NODES) {
                    // Extract the Session Row Index allocated to this specific remote source node ID via Layer 4
                    uint8_t connIdx = g_ActiveSessionRouter[srcNodeId];

                    if (connIdx < 16) { // Valid assigned tracking slot confirmed ($0 \dots 15$)
                        uint8_t seqNum = (uint8_t)(incomingId29 & XCAN_MASK_SEQUENCE);
                        // Compute direct spatial matrix slot coordinate map index inside Layer 4 matrix layout
                        uint16_t matrixTargetIndex = (connIdx << 4) | (seqNum & 0x0F);

                        if (matrixTargetIndex < TOTAL_APP_FRAMES) {
                            // --- CRITICAL DIRECT-MAPPED MATRIX CONCURRENCY INVARIANT ---
                            // If target slot field already contains an active tracking ID, Layer 4 has failed to ingest 
                            // the previous frame cycle in time. This is an In-Place Application Frame Overrun.
                            if (g_ApplicationMatrix[matrixTargetIndex].id29 != 0) {
                                m_telemetryFirewallDrops++;          // Log structural drop metrics
                                m_sourceQuenchTriggerPending = true; // Assert Quench Flag to choke the runaway sender
                            } else {
                                // Clean, zero-copy slot acquisition: Write directly to memory map slot
                                g_ApplicationMatrix[matrixTargetIndex].id29     = incomingId29;
                                g_ApplicationMatrix[matrixTargetIndex].dataLow  = pFifoMailbox->RDLR;
                                g_ApplicationMatrix[matrixTargetIndex].dataHigh = pFifoMailbox->RDHR;
                            }
                        } else {
                            m_telemetryFirewallDrops++;
                        }
                    } else {
                        // Drop frame: Node is streaming application bytes without a session allocation row initialized by L4
                        m_telemetryFirewallDrops++;
                    }
                } else {
                    m_telemetryFirewallDrops++;
                }
            }
        }

        // Release current hardware mailbox allocation cell back to the bxCAN ring layer to clear hardware space
        if (hardwareFifoIndex == 0) {
            CAN1->RF0R |= CAN_RF0R_RFOM0; // Release FIFO 0 Output Mailbox bit trigger
            pendingFramesCount = CAN1->RF0R & CAN_RF0R_FMP0; // Re-evaluate count metrics
        } else {
            CAN1->RF1R |= CAN_RF1R_RFOM1; // Release FIFO 1 Output Mailbox bit trigger
            pendingFramesCount = CAN1->RF1R & CAN_RF1R_FMP1;
        }
    }

    // --- Dynamic Physical Error Layer Extraction Sequence ---
    uint32_t esr = CAN1->ESR;
    CanBusErrorState parsedState = CAN_BUS_ERROR_ACTIVE;
    if (esr & CAN_ESR_BOFF)      parsedState = CAN_BUS_ERROR_OFF;
    else if (esr & CAN_ESR_EPVF) parsedState = CAN_BUS_ERROR_PASSIVE;
    else if (esr & CAN_ESR_EWGF) parsedState = CAN_BUS_ERROR_WARNING;

    // Track historical transitions across diagnostic error ceilings
    if (parsedState != m_cachedErrorState) {
        if (parsedState == CAN_BUS_ERROR_OFF)     m_busOffTransitions++;
        else if (parsedState == CAN_BUS_ERROR_PASSIVE) m_errorPassiveTransitions++;
        else if (parsedState == CAN_BUS_ERROR_WARNING) m_errorWarningTransitions++;
        m_cachedErrorState = parsedState; // Save current state baseline metrics
    }
}

/**
 * @brief Scheduling supervisor task managing arbitration loss metrics, stalls, and stuck frame conditions.
 */
void mtCANL2Engine::process_tx_management() {
    uint32_t tsrRegister = CAN1->TSR;
    uint32_t currentTick = get_portable_system_timestamp_ms();

    // Outbound Squeeze Safety Lock Watchdog: Reset TX structure tracking elements if tail stays locked down 
    // beyond 250 milliseconds during a congestion choke phase.
    if (m_flowState == L2_FLOW_CONGESTED) {
        if (currentTick - m_blockedTimestamp > M_L2_CONGESTION_TIMEOUT_MS) {
            __disable_irq();
            m_txHead = 0;
            m_txTail = 0;
            m_flowState = L2_FLOW_OK;
            m_congestionWarningActive = true;
            m_congestionWarningStrikes++;
            __enable_irq();
            return;
        }
    }

    // --- MAILBOX 0 SAFETY TIMEOUT AND ARBITRATION MONITORING ---
    if (!(tsrRegister & CAN_TSR_TME0)) { // Mailbox 0 contains a pending unsent frame profile
        if (tsrRegister & CAN_TSR_ALST0) { // Arbitration Lost Flag active for Mailbox 0
            m_txArbitrationLostEvents++;
            m_mailboxArbitrationLossCount[0]++;
        }
        if (tsrRegister & CAN_TSR_TERR0) { // Transmission Error reported by hardware
            m_txErrorHardwareEvents++;
        }
        // Evaluation of structural stall rules: If frame is stuck > 500ms, or crossed 25 consecutive collisions, 
        // or reported a hard physical line error, execute manual hardware mailbox revocation.
        if ((currentTick - m_mailboxTxTimestamp[0] > M_L2_QUEUE_STUCK_TIMEOUT_MS) || (m_mailboxArbitrationLossCount[0] > M_L2_MAX_ALLOWED_ALST_STRIKES) || (tsrRegister & CAN_TSR_TERR0)) {
            CAN1->TSR |= CAN_TSR_ABRQ0; // Abort Request bit trigger forced active for Mailbox 0
            m_mailboxStallRecoveries++;
            m_mailboxTxTimestamp[0] = currentTick; // Refresh benchmark tracker to protect loop sequences
        }
    }

    // --- MAILBOX 1 SAFETY TIMEOUT AND ARBITRATION MONITORING ---
    if (!(tsrRegister & CAN_TSR_TME1)) {
        if (tsrRegister & CAN_TSR_ALST1) {
            m_txArbitrationLostEvents++;
            m_mailboxArbitrationLossCount[1]++;
        }
        if (tsrRegister & CAN_TSR_TERR1) {
            m_txErrorHardwareEvents++;
        }
        if ((currentTick - m_mailboxTxTimestamp[1] > M_L2_QUEUE_STUCK_TIMEOUT_MS) || (m_mailboxArbitrationLossCount[1] > M_L2_MAX_ALLOWED_ALST_STRIKES) || (tsrRegister & CAN_TSR_TERR1)) {
            CAN1->TSR |= CAN_TSR_ABRQ1;
            m_mailboxStallRecoveries++;
            m_mailboxTxTimestamp[1] = currentTick;
        }
    }

    // --- MAILBOX 2 SAFETY TIMEOUT AND ARBITRATION MONITORING ---
    if (!(tsrRegister & CAN_TSR_TME2)) {
        if (tsrRegister & CAN_TSR_ALST2) {
            m_txArbitrationLostEvents++;
            m_mailboxArbitrationLossCount[2]++;
        }
        if (tsrRegister & CAN_TSR_TERR2) {
            m_txErrorHardwareEvents++;
        }
        if ((currentTick - m_mailboxTxTimestamp[2] > M_L2_QUEUE_STUCK_TIMEOUT_MS) || (m_mailboxArbitrationLossCount[2] > M_L2_MAX_ALLOWED_ALST_STRIKES) || (tsrRegister & CAN_TSR_TERR2)) {
            CAN1->TSR |= CAN_TSR_ABRQ2;
            m_mailboxStallRecoveries++;
            m_mailboxTxTimestamp[2] = currentTick;
        }
    }
}

/**
 * @brief Thread-safe atomic status evaluation routine to extract and reset current Source Quench requests.
 * * Closes potential race windows by executing interrupt mask locks BEFORE inspecting variable states,
 * preventing mid-line preemptions from stripping event signals out of scheduling tasks.
 */
bool mtCANL2Engine::check_and_clear_quench_condition() {
    __disable_irq(); // Lock context lines; prevent interrupt handlers from altering flag state during read pass
    bool statusCaptured = m_sourceQuenchTriggerPending; // Capture clean snapshot state
    if (statusCaptured) {
        m_sourceQuenchTriggerPending = false; // Reset atomic state flag down to base condition
    }
    __enable_irq(); // Release interrupt mask lock lines
    return statusCaptured; // Return original captured snapshot metric back to caller
}

/**
 * @brief Synchronous queue index reset layer operating under strict global interrupt isolation masks.
 */
void mtCANL2Engine::flush_software_queues() {
    __disable_irq();
    m_txHead = 0;
    m_txTail = 0;
    m_sysMatrixHead = 0;
    m_sysMatrixTail = 0;
    m_flowState = L2_FLOW_OK;
    m_sourceQuenchTriggerPending = false;
    __enable_irq();
}

/**
 * @brief Clears back-pressure tracking locks and resets congestion tracking attributes.
 */
void mtCANL2Engine::reset_flow_engine() {
    __disable_irq();
    m_flowState = L2_FLOW_OK;
    m_blockedTimestamp = 0;
    m_congestionWarningActive = false;
    __enable_irq();
}

/**
 * @brief Executes a physical power teardown cycle across bxCAN peripherals followed by a fresh initialization pass.
 */
bool mtCANL2Engine::recover_hardware() {
    shutdown_hardware();
    return initialize_hardware(m_nodeId, m_commLossTimeoutThresholdMs);
}

/**
 * @brief High-level network integration recovery asset tracking clean queue flushes and initialization cycles.
 */
bool mtCANL2Engine::rejoin_network() {
    flush_software_queues();
    return recover_hardware();
}

/**
 * @section Low-Level ISR Vector Mapping Hooks
 * Re-routes raw hardware execution vectors allocated inside the micro-controller's vector 
 * table into the corresponding C++ class instance handler functions via the local class instance pointer.
 */

/**
 * @brief High Priority CAN Transmit Interrupt Handler Vector Entry Point.
 */
extern "C" void USB_HP_CAN1_TX_IRQHandler(void) {
    if (g_pL2EngineInstance != nullptr) {
        g_pL2EngineInstance->handle_tx_interrupt_service_routine();
    }
}

/**
 * @brief Low Priority CAN Receive FIFO 0 Interrupt Handler Vector Entry Point.
 * Handles high priority system plane traffic.
 */
extern "C" void USB_LP_CAN1_RX0_IRQHandler(void) {
    if (g_pL2EngineInstance != nullptr) {
        g_pL2EngineInstance->handle_rx_interrupt_service_routine(0);
    }
}

/**
 * @brief CAN Receive FIFO 1 Interrupt Handler Vector Entry Point.
 * Handles application matrix packet routing.
 */
extern "C" void CAN1_RX1_IRQHandler(void) {
    if (g_pL2EngineInstance != nullptr) {
        g_pL2EngineInstance->handle_rx_interrupt_service_routine(1);
    }
}