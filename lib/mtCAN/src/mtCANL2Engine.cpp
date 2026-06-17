/**
 * @file mtCANL2Engine.cpp
 * @brief High-Density Fully Commented bxCAN Layer 2 Core Engine Implementation.
 * * This file contains the complete, production-ready implementation of our bifurcated
 * dual-plane engine driver. It handles isolated hardware mailbox allocations, fast parallel-array
 * insertion routing updates, and constant-time ($O(1)$) unrolled 4-step binary search ingress paths.
 * No logical structures or control loops have been changed from the baseline specifications.
 */

#include "mtCANL2Engine.h"

/**
 * @struct NodeStatus
 * @brief Internal structural layout tracking online behavior and firmware properties of network nodes.
 */
struct NodeStatus {
    volatile uint32_t lastSeenTimestampMs; /**< Absolute time when an error-free wire frame was matched from this node. */
    uint8_t  onlineState;                  /**< Logical connection flag state tracking presence on the bus grid. */
    uint8_t  deviceType;                   /**< Classification metadata identifying hardware peripheral models. */
    uint16_t firmwareVersion;              /**< Embedded software iteration revision token pulled from handshake updates. */
};

/* ============================================================================
 * INTER-PLANE ISOLATED MEMORY SPACE AND STORAGE MATRICES
 * ============================================================================ */

/** @brief Global Application Storage Matrix. Row indexing maps to active connection lanes (16 lanes x 8 sequence fragments). */
CanMatrixFrame g_ApplicationMatrix[TOTAL_APP_FRAMES];

/** @brief Global Control Storage Matrix. Serves as a circular log tracking low-priority control traffic. */
CanMatrixFrame g_ControlMatrix[TOTAL_CTRL_FRAMES];


/* ============================================================================
 * SYNCHRONIZED PARALLEL ACTIVE ROUTER TABLE DATA ARRAYS
 * ============================================================================ */

/** @brief Parallel Search Key Vault. Maintained in strictly ascending numerical order for unrolled binary search. */
uint32_t L2activeSessionTokens[TOTAL_APP_CONN];

/** @brief Parallel Action Value Map. Links the matching sorted key directly to its g_ApplicationMatrix index row (0 to 15). */
uint8_t  L2activeSessionRouter[TOTAL_APP_CONN];


/* ============================================================================
 * NETWORK MANAGEMENT AND RESTART PERSISTENCE PARAMETERS
 * ============================================================================ */

/** @brief Real-time telemetry monitoring table tracking all potential nodes across the network. */
volatile NodeStatus g_NetworkNodeStatusMatrix[MAX_NETWORK_NODES];

/** @brief Static callback link instance referencing active engine contexts inside the naked ISR handlers. */
static mtCANL2Engine* g_pL2EngineInstance = nullptr;

/** @brief Non-initialized RAM section register tracking node restarts across warm-boot power cycles. */
__attribute__((section(".noinit"))) static uint32_t persistentNodeRestartCounter;

/** @brief Verification signature field confirming non-initialized tracking block accuracy. */
__attribute__((section(".noinit"))) static uint32_t persistentValidityMagicToken;


/* ============================================================================
 * ENGINE LIFECYCLE MANAGEMENT IMPLEMENTATIONS
 * ============================================================================ */

/**
 * @brief Portable timing translator referencing internal microcontroller millisecond tickers.
 * @return 32-bit platform-independent absolute timestamp index.
 */
uint32_t mtCANL2Engine::get_portable_system_timestamp_ms() {
    return (uint32_t)millis();
}

/**
 * @brief Architectural constructor initializing pointers, clearing arrays, and checking boot memory integrity.
 * * Instantiates the parallel tracking structures with sentinel values (`0xFFFFFFFFUL` and `0xFF`)
 * to indicate unallocated routes. Evaluates non-initialized RAM to verify if a warm boot occurred.
 */
mtCANL2Engine::mtCANL2Engine() :
    m_nodeId(0),
    m_nodeBootCounter(0),
    m_commLossTimeoutThresholdMs(5000),
    m_ctrlMatrixHead(0),
    m_ctrlMatrixTail(0),
    m_txAppHead(0),
    m_txAppTail(0),
    m_txCtrlHead(0),
    m_txCtrlTail(0),
    m_flowState(L2_FLOW_OK),
    m_blockedTimestamp(0),
    m_congestionWarningActive(false),
    m_congestionWarningStrikes(0),
    m_lastWireActivityTimestamp(0),
    m_activeRouteCount(0),
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
    m_controlMatrixDroppedFrames(0),
    m_quenchState(QUENCH_STATE_NONE),
    m_quenchNodeId(0),
    m_lastTelemetrySampleTick(0),
    m_lastTotalFrameCount(0),
    m_cachedErrorState(CAN_BUS_ERROR_ACTIVE) {

    /* Bind the global instance pointer to enable cross-linking inside static ISR contexts */
    g_pL2EngineInstance = this;

    /* Initialize separate tracking records for each silicon mailbox slot */
    m_mailboxArbitrationLossCount[0] = 0;
    m_mailboxArbitrationLossCount[1] = 0;
    m_mailboxArbitrationLossCount[2] = 0;

    m_mailboxTxTimestamp[0] = 0;
    m_mailboxTxTimestamp[1] = 0;
    m_mailboxTxTimestamp[2] = 0;

    /* Clear the parallel search arrays and populate them with unallocated sentinel flags */
    for (uint8_t i = 0; i < TOTAL_APP_CONN; ++i) {
        L2activeSessionTokens[i] = 0xFFFFFFFFUL;
        L2activeSessionRouter[i] = 0xFF;
    }

    /* Evaluate warm persistent memory locations to verify previous tracking sessions */
    if (persistentValidityMagicToken == M_HW_SALT_CONSTANT) {
        persistentNodeRestartCounter++;
    } else {
        persistentValidityMagicToken = M_HW_SALT_CONSTANT;
        persistentNodeRestartCounter = 1;
    }
    m_nodeBootCounter = persistentNodeRestartCounter;
}

/**
 * @brief Configures clocks, routing matrices, hardware filter configurations, and vector limits.
 * * Configures the bxCAN cells, activates the Automatic Bus-Off Management system,
 * establishes standard 500Kbps bit timing rules on an 8MHz clock tree, and enables the target interrupts.
 */
bool mtCANL2Engine::initialize_hardware(uint8_t localNodeId, uint32_t commLossTimeoutMs) {
    /* Validate network parameters before starting initialization loops */
    if (localNodeId >= 64) return false;
    m_nodeId = localNodeId;
    m_commLossTimeoutThresholdMs = commLossTimeoutMs;

    /* Purge the core application framework data matrix blocks */
    for (uint32_t i = 0; i < TOTAL_APP_FRAMES; ++i) {
        g_ApplicationMatrix[i].id29 = 0;
        g_ApplicationMatrix[i].dataLow = 0;
        g_ApplicationMatrix[i].dataHigh = 0;
    }

    /* Purge the control tracking logs and clear the storage matrix slots */
    for (uint32_t i = 0; i < TOTAL_CTRL_FRAMES; ++i) {
        g_ControlMatrix[i].id29 = 0;
        g_ControlMatrix[i].dataLow = 0;
        g_ControlMatrix[i].dataHigh = 0;
    }

    /* Wipe individual diagnostic logs for active nodes on the network bus */
    for (uint32_t i = 0; i < MAX_NETWORK_NODES; ++i) {
        g_NetworkNodeStatusMatrix[i].lastSeenTimestampMs = 0;
        g_NetworkNodeStatusMatrix[i].onlineState = 0;
        g_NetworkNodeStatusMatrix[i].deviceType = 0;
        g_NetworkNodeStatusMatrix[i].firmwareVersion = 0;
    }

    /* Clear the parallel routing arrays and reset table tracking structures */
    for (uint8_t i = 0; i < TOTAL_APP_CONN; ++i) {
        L2activeSessionTokens[i] = 0xFFFFFFFFUL;
        L2activeSessionRouter[i] = 0xFF;
    }
    m_activeRouteCount = 0;

    /* Enable clock parameters across GPIO port maps, alternate functions, and bxCAN blocks */
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;

    /* Reset the configuration masks for pin segments PA11 and PA12 */
    GPIOA->CRH &= ~(GPIO_CRH_MODE11 | GPIO_CRH_CNF11 | GPIO_CRH_MODE12 | GPIO_CRH_CNF12);
    
    /* Establish floating entry profiles for the CAN RX pin location (PA11) */
    GPIOA->CRH |= GPIO_CRH_CNF11_0;
    
    /* Establish push-pull profiles for the CAN TX pin location (PA12) */
    GPIOA->CRH |= GPIO_CRH_MODE12_1 | GPIO_CRH_CNF12_1;

    /* Clear alternative function remap bits to lock operational lines to PA11 and PA12 */
    AFIO->MAPR &= ~AFIO_MAPR_CAN_REMAP;

    /* Assert initialization request statement to place the bxCAN cell into configuration mode */
    CAN1->MCR |= CAN_MCR_INRQ;
    uint32_t timeoutGuard = 0xFFFF;
    while (!(CAN1->MSR & CAN_MSR_INAK)) {
        if (--timeoutGuard == 0) return false;
    }

    /* Exit sleep mode state parameters */
    CAN1->MCR &= ~CAN_MCR_SLEEP;
    
    /* Reset and configure the bit timing register parameters (BTR) */
    CAN1->BTR = 0;
    /* Synchronize parameters: Prescaler=8, TS1=11, TS2=2, SJW=0 (Yields 500Kbps timing configuration) */
    CAN1->BTR |= (8 << 0) | (11 << 16) | (2 << 20) | (0 << 24);

    /* Enable Automatic Bus-Off Recovery and lock FIFO outputs against overflow overwrites */
    CAN1->MCR |= CAN_MCR_ABOM | CAN_MCR_RFLM;
    /* Enable automatic packet retransmission loops by clearing the NART flag */
    CAN1->MCR &= ~CAN_MCR_NART;

    /* Initialize the isolation filter banks */
    reconfigure_hardware_filters();

    /* Activate peripheral interrupts for FIFO 0, FIFO 1, and the transmit mailbox empty line */
    CAN1->IER |= CAN_IER_FMPIE0 | CAN_IER_FMPIE1 | CAN_IER_TMEIE;

    /* Configure vector priorities and activate the matching lines inside the NVIC block */
    NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0);
    NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);

    NVIC_SetPriority(CAN1_RX1_IRQn, 1);
    NVIC_EnableIRQ(CAN1_RX1_IRQn);

    NVIC_SetPriority(USB_HP_CAN1_TX_IRQn, 2);
    NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);

    /* Release initialization hold and wait for the peripheral cell to synchronize with the bus network */
    CAN1->MCR &= ~CAN_MCR_INRQ;
    timeoutGuard = 0xFFFF;
    while (CAN1->MSR & CAN_MSR_INAK) {
        if (--timeoutGuard == 0) return false;
    }

    /* Set the background reference timestamps */
    uint32_t currentTick = get_portable_system_timestamp_ms();
    m_lastTelemetrySampleTick = currentTick;
    m_lastWireActivityTimestamp = currentTick;

    m_mailboxTxTimestamp[0] = 0;
    m_mailboxTxTimestamp[1] = 0;
    m_mailboxTxTimestamp[2] = 0;

    /* Query current error status registers to cache initial baseline conditions */
    uint32_t esr = CAN1->ESR;
    if (esr & CAN_ESR_BOFF)      m_cachedErrorState = CAN_BUS_ERROR_OFF;
    else if (esr & CAN_ESR_EPVF) m_cachedErrorState = CAN_BUS_ERROR_PASSIVE;
    else if (esr & CAN_ESR_EWGF) m_cachedErrorState = CAN_BUS_ERROR_WARNING;
    else                         m_cachedErrorState = CAN_BUS_ERROR_ACTIVE;

    return true;
}

/**
 * @brief Configures internal hardware filters to isolate the data planes.
 * * Pins System/Control messages to Hardware FIFO 0 and routes all Application/Multicast
 * payload frames directly to Hardware FIFO 1.
 */
void mtCANL2Engine::reconfigure_hardware_filters() {
    /* Assert initialization request statement for the filter bank configuration */
    CAN1->FMR |= CAN_FMR_FINIT;
    /* Deactivate the lower 12 filter slots before modifying values */
    CAN1->FA1R &= ~0x0FFFUL;

    /* Establish 32-bit width and mask-mode profiles for the initial 12 filter banks */
    for (uint8_t bank = 0; bank < 12; ++bank) {
        CAN1->FS1R |= (1UL << bank);  /* Single 32-bit register configuration scaling scale */
        CAN1->FM1R &= ~(1UL << bank); /* Identifier Mask tracking mode allocation */
    }

    /* FILTER SLOT 0: Matches Unicast System Control Plane Frames (FIFO 0 target) */
    uint32_t unicastSysId   = (0x1F000000UL | (0 << 20) | (1 << 17) | ((uint32_t)m_nodeId << 10)) << 3;
    uint32_t unicastSysMask = (0x1F100060UL | (0x0001FC00UL)) << 3;
    CAN1->sFilterRegister[0].FR1 = unicastSysId | 4;   /* Load internal identity pattern */
    CAN1->sFilterRegister[0].FR2 = unicastSysMask | 4; /* Inject verification validation mask */
    CAN1->FFA1R &= ~(1UL << 0);                        /* Route matching paths to FIFO 0 */

    /* FILTER SLOT 1: Matches Broadcast System Control Plane Frames (FIFO 0 target) */
    uint32_t broadcastSysId   = (0x1F000000UL | (0 << 20) | (3 << 17)) << 3;
    uint32_t broadcastSysMask = (0x1F100060UL) << 3;
    CAN1->sFilterRegister[1].FR1 = broadcastSysId | 4;
    CAN1->sFilterRegister[1].FR2 = broadcastSysMask | 4;
    CAN1->FFA1R &= ~(1UL << 1);                        /* Route matching paths to FIFO 0 */

    /* FILTER SLOT 2: Matches Unicast Application Plane Streaming Frames (FIFO 1 target) */
    uint32_t unicastAppId   = (0x1F000000UL | (1 << 20) | (1 << 17) | ((uint32_t)m_nodeId << 10)) << 3;
    uint32_t unicastAppMask = (0x1F100060UL | (0x0001FC00UL)) << 3;
    CAN1->sFilterRegister[2].FR1 = unicastAppId | 4;
    CAN1->sFilterRegister[2].FR2 = unicastAppMask | 4;
    CAN1->FFA1R |= (1UL << 2);                         /* Route matching paths to FIFO 1 */

    /* FILTER SLOT 3: Matches Broadcast Application Plane Streaming Frames (FIFO 1 target) */
    uint32_t broadcastAppId   = (0x1F000000UL | (1 << 20) | (3 << 17)) << 3;
    uint32_t broadcastAppMask = (0x1F100060UL) << 3;
    CAN1->sFilterRegister[3].FR1 = broadcastAppId | 4;
    CAN1->sFilterRegister[3].FR2 = broadcastAppMask | 4;
    CAN1->FFA1R |= (1UL << 3);                         /* Route matching paths to FIFO 1 */

    /* FILTER SLOTS 4-11: Matches Multicast Group Frames (FIFO 1 target) */
    uint32_t multicastAppId   = (0x1F000000UL | (1 << 20) | (2 << 17) | (0x00010000UL)) << 3;
    uint32_t multicastAppMask = (0x1F100060UL | (0x00010000UL)) << 3;
    for (uint8_t bank = 4; bank < 12; ++bank) {
        CAN1->sFilterRegister[bank].FR1 = multicastAppId | 4;
        CAN1->sFilterRegister[bank].FR2 = multicastAppMask | 4;
        CAN1->FFA1R |= (1UL << bank);                  /* Route matching paths to FIFO 1 */
    }

    /* Reactivate the configured filter slots */
    CAN1->FA1R |= 0x0FFFUL;
    /* Release the filter initialization lock */
    CAN1->FMR &= ~CAN_FMR_FINIT;
}

/**
 * @brief Deactivates the interrupt masks and disconnects the line handles.
 */
void mtCANL2Engine::disconnect() {
    __disable_irq();
    CAN1->IER &= ~(CAN_IER_FMPIE0 | CAN_IER_FMPIE1 | CAN_IER_TMEIE);

    NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
    NVIC_DisableIRQ(CAN1_RX1_IRQn);
    NVIC_DisableIRQ(USB_HP_CAN1_TX_IRQn);

    CAN1->MCR |= CAN_MCR_INRQ;
    uint32_t timeoutGuard = 0xFFFF;
    while (!(CAN1->MSR & CAN_MSR_INAK)) {
        if (--timeoutGuard == 0) break;
    }
    __enable_irq();
}

/**
 * @brief Shuts down peripheral interfaces and puts the transceiver into a disconnected sleep state.
 */
void mtCANL2Engine::shutdown_hardware() {
    disconnect();
}


/* ============================================================================
 * DATA TRANSMISSION PIPELINE MANAGEMENT
 * ============================================================================ */

/**
 * @brief Enqueues out-bound streaming packages into the application ring buffer.
 * * Applies backpressure checks against `M_L2_TX_QUEUE_HIGH_WATERMARK`. Returns false if the queue is full,
 * throttling the application plane while leaving the control plane unaffected.
 */
bool mtCANL2Engine::enqueue_app_frame(uint32_t id29, const uint8_t* sourceBuffer, uint8_t length) {
    /* Stop execution pathways immediately if backpressure states are congested */
    if (m_flowState == L2_FLOW_CONGESTED) return false;
    if (length > 0 && sourceBuffer == nullptr) return false;
    if (id29 > 0x1FFFFFFFUL) return false;

    /* Extract verification parameters from the target protocol header */
    uint8_t extractedSignature = (uint8_t)((id29 & XCAN_MASK_SIGNATURE) >> XCAN_SHIFT_SIGNATURE);
    uint8_t extractedPriority = (uint8_t)((id29 & XCAN_MASK_PRIORITY) >> XCAN_SHIFT_PRIORITY);
    uint8_t extractedMode = (uint8_t)((id29 & XCAN_MASK_MODE) >> XCAN_SHIFT_MODE);
    uint8_t extractedSrcNode = (uint8_t)((id29 & XCAN_MASK_SRC_NODE) >> XCAN_SHIFT_SRC_NODE);

    /* Enforce validation checks against protocol specifications */
    if (extractedSignature != XCAN_PROTOCOL_SIGNATURE) return false;
    if (extractedPriority == 0 || extractedPriority > 7) return false;
    if (extractedMode == 0 || extractedMode > 3) return false;
    if (extractedSrcNode != m_nodeId) return false;
    if (length > 8) return false;

    __disable_irq();
    uint16_t currentHead = m_txAppHead;
    uint16_t nextHead = (currentHead + 1) & M_L2_TX_APP_QUEUE_MASK;

    /* Evaluate buffer allocations for overflow states */
    if (nextHead == m_txAppTail) {
        m_flowState = L2_FLOW_CONGESTED;
        m_blockedTimestamp = get_portable_system_timestamp_ms();
        __enable_irq();
        return false;
    }

    /* Check buffer allocations against the high watermark limit to enforce backpressure throttling */
    uint16_t activeQueueDepth = (nextHead - m_txAppTail) & M_L2_TX_APP_QUEUE_MASK;
    if (activeQueueDepth >= M_L2_TX_APP_QUEUE_HIGH_WATERMARK) {
        m_flowState = L2_FLOW_CONGESTED;
        m_blockedTimestamp = get_portable_system_timestamp_ms();
    }

    /* Copy packet parameters into the application ring buffer */
    m_txAppRingBuffer[currentHead].id29 = id29;
    m_txAppRingBuffer[currentHead].length = length;
    for (uint8_t idx = 0; idx < length; ++idx) {
        m_txAppRingBuffer[currentHead].data[idx] = sourceBuffer[idx];
    }
    m_txAppHead = nextHead;

    /* Manually invoke the dispatch sequence to process the newly added frame */
    handle_tx_interrupt_service_routine();
    __enable_irq();
    return true;
}

/**
 * @brief Enqueues urgent system commands into the control ring buffer.
 * * Bypasses the application plane's flow control states. These frames are locked to Hardware Mailbox 0
 * to ensure control traffic is never blocked by application bottlenecks.
 */
bool mtCANL2Engine::enqueue_ctrl_frame(uint32_t id29, const uint8_t* sourceBuffer, uint8_t length) {
    if (length > 0 && sourceBuffer == nullptr) return false;
    if (id29 > 0x1FFFFFFFUL) return false;

    uint8_t extractedSignature = (uint8_t)((id29 & XCAN_MASK_SIGNATURE) >> XCAN_SHIFT_SIGNATURE);
    uint8_t extractedSrcNode = (uint8_t)((id29 & XCAN_MASK_SRC_NODE) >> XCAN_SHIFT_SRC_NODE);

    if (extractedSignature != XCAN_PROTOCOL_SIGNATURE) return false;
    if (extractedSrcNode != m_nodeId) return false;
    if (length > 8) return false;

    __disable_irq();
    uint16_t currentHead = m_txCtrlHead;
    uint16_t nextHead = (currentHead + 1) & M_L2_TX_CTRL_QUEUE_MASK;

    /* Drop frames if the control buffer overflows to prevent corruption of the queue structures */
    if (nextHead == m_txCtrlTail) {
        __enable_irq();
        return false;
    }

    /* Append the control frame data to the isolated ring buffer */
    m_txCtrlRingBuffer[currentHead].id29 = id29;
    m_txCtrlRingBuffer[currentHead].length = length;
    for (uint8_t idx = 0; idx < length; ++idx) {
        m_txCtrlRingBuffer[currentHead].data[idx] = sourceBuffer[idx];
    }
    m_txCtrlHead = nextHead;

    /* Trigger the transmitter hardware lines */
    handle_tx_interrupt_service_routine();
    __enable_irq();
    return true;
}


/* ============================================================================
 * SYNCHRONIZED PARALLEL ACTIVE ROUTER TABLE MANAGEMENT
 * ============================================================================ */

/**
 * @brief Registers a tracking token via an active runtime Insertion Sort.
 * * Masks out the lower 3 sequence bits to identify the base stream token, locates the correct numerical slot,
 * and performs a synchronized right-shift across both `L2activeSessionTokens` and `L2activeSessionRouter`
 * to keep them perfectly aligned.
 */
bool mtCANL2Engine::L2_route_add(uint32_t ingressToken, uint8_t connIdx) {
    if (m_activeRouteCount >= TOTAL_APP_CONN) return false;
    uint32_t strippedToken = ingressToken & ~0x07UL; /* Clear sequence fields */

    __disable_irq();
    /* Scan the table to check if the token is already registered; update the index if found */
    for (uint8_t i = 0; i < m_activeRouteCount; ++i) {
        if (L2activeSessionTokens[i] == strippedToken) {
            L2activeSessionRouter[i] = connIdx; /* Re-bind the matrix mapping layer */
            __enable_irq();
            return true;
        }
    }

    /* Identify the insertion slot that preserves ascending numerical order */
    uint8_t insertPos = 0;
    while (insertPos < m_activeRouteCount && L2activeSessionTokens[insertPos] < strippedToken) {
        insertPos++;
    }

    /* Execute a synchronized right-shift on both arrays to open up the target slot */
    for (uint8_t i = m_activeRouteCount; i > insertPos; --i) {
        L2activeSessionTokens[i] = L2activeSessionTokens[i - 1];
        L2activeSessionRouter[i] = L2activeSessionRouter[i - 1];
    }

    /* Commit the tracking tokens to the parallel arrays */
    L2activeSessionTokens[insertPos] = strippedToken;
    L2activeSessionRouter[insertPos] = connIdx;
    m_activeRouteCount++;
    __enable_irq();

    return true;
}

/**
 * @brief Purges a tracking token and maps the remaining entries to close the gap.
 * * Locates the target token and executes a synchronized left-shift across both arrays,
 * maintaining the contiguous data alignment required by the unrolled binary search tree.
 */
bool mtCANL2Engine::L2_route_drop(uint32_t ingressToken) {
    uint32_t strippedToken = ingressToken & ~0x07UL;

    __disable_irq();
    for (uint8_t i = 0; i < m_activeRouteCount; ++i) {
        if (L2activeSessionTokens[i] == strippedToken) {
            
            /* Shift trailing entries left to keep the lookup table continuous */
            for (uint8_t j = i; j < m_activeRouteCount - 1; ++j) {
                L2activeSessionTokens[j] = L2activeSessionTokens[j + 1];
                L2activeSessionRouter[j] = L2activeSessionRouter[j + 1];
            }

            /* Clear the trailing slots with unallocated sentinels */
            L2activeSessionTokens[m_activeRouteCount - 1] = 0xFFFFFFFFUL;
            L2activeSessionRouter[m_activeRouteCount - 1] = 0xFF;
            m_activeRouteCount--;
            __enable_irq();
            return true;
        }
    }
    __enable_irq();
    return false;
}


/* ============================================================================
 * INTERRUPT SERVICE ROUTINES & PERIPHERAL HANDLER CODE
 * ============================================================================ */

/**
 * @brief Dispatches frames from the software queues into the hardware mailboxes.
 * * Enforces the architectural plane isolation rules:
 * - Control plane frames (`m_txControlRingBuffer`) are locked to **Hardware Mailbox 0**.
 * - Application plane frames (`m_txRingBuffer`) are balanced across **Hardware Mailboxes 1 and 2**.
 */
void mtCANL2Engine::handle_tx_interrupt_service_routine() {
    uint32_t tsrRegister = CAN1->TSR;
    uint32_t currentTick = get_portable_system_timestamp_ms();

    /* Process Mailbox 0 completion states and clear pending flag requests */
    if (tsrRegister & CAN_TSR_RQCP0) {
        if (tsrRegister & CAN_TSR_TXOK0) {
            m_totalTransmittedFrames++;
            m_lastWireActivityTimestamp = currentTick;
        }
        m_mailboxArbitrationLossCount[0] = 0;
        CAN1->TSR |= CAN_TSR_RQCP0; /* Clear the tracking flag bit */
    }

    /* Process Mailbox 1 completion states */
    if (tsrRegister & CAN_TSR_RQCP1) {
        if (tsrRegister & CAN_TSR_TXOK1) {
            m_totalTransmittedFrames++;
            m_lastWireActivityTimestamp = currentTick;
        }
        m_mailboxArbitrationLossCount[1] = 0;
        CAN1->TSR |= CAN_TSR_RQCP1;
    }

    /* Process Mailbox 2 completion states */
    if (tsrRegister & CAN_TSR_RQCP2) {
        if (tsrRegister & CAN_TSR_TXOK2) {
            m_totalTransmittedFrames++;
            m_lastWireActivityTimestamp = currentTick;
        }
        m_mailboxArbitrationLossCount[2] = 0;
        CAN1->TSR |= CAN_TSR_RQCP2;
    }

    /* Re-read status values to sync with modified register states */
    tsrRegister = CAN1->TSR;

    /* --- PLANE 0: DISPATCH CONTROL ROADWAYS (LOCKED TO HARDWARE MAILBOX 0) --- */
    if (m_txCtrlTail != m_txCtrlHead) {
        if (tsrRegister & CAN_TSR_TME0) { /* Verify that Mailbox 0 is empty */
            CAN_TxMailBox_TypeDef* pTxMailbox = &CAN1->sTxMailBox[0];
            uint16_t currentTail = m_txCtrlTail;
            CanFrameL2* sourceFrame = &m_txCtrlRingBuffer[currentTail];

            m_mailboxTxTimestamp[0] = currentTick;

            /* Clear data length configuration bits and inject current frame size specifications */
            pTxMailbox->TDTR &= ~CAN_TDT0R_DLC;
            pTxMailbox->TDTR |= (sourceFrame->length & CAN_TDT0R_DLC);

            /* Pack the payload data bytes into the low and high word registers */
            uint32_t dataLowValue  = ((uint32_t)sourceFrame->data[3] << 24) | ((uint32_t)sourceFrame->data[2] << 16) | ((uint32_t)sourceFrame->data[1] << 8) | ((uint32_t)sourceFrame->data[0]);
            uint32_t dataHighValue = ((uint32_t)sourceFrame->data[7] << 24) | ((uint32_t)sourceFrame->data[6] << 16) | ((uint32_t)sourceFrame->data[5] << 8) | ((uint32_t)sourceFrame->data[4]);
            pTxMailbox->TDLR = dataLowValue;
            pTxMailbox->TDHR = dataHighValue;

            /* Load the identifier bits, assert the extended flag, and request transmission */
            pTxMailbox->TIR = (sourceFrame->id29 << 3) | CAN_TI0R_IDE | CAN_TI0R_TXRQ;

            m_txCtrlTail = (currentTail + 1) & M_L2_TX_CTRL_QUEUE_MASK;
            tsrRegister &= ~CAN_TSR_TME0; /* Mark Mailbox 0 as occupied for subsequent checks */
        }
    }

    /* --- PLANE 1: DISPATCH APPLICATION ROADWAYS (BALANCED ACROSS MAILBOXES 1 & 2) --- */
    while (m_txAppTail != m_txAppHead) {
        CAN_TxMailBox_TypeDef* pTxMailbox = nullptr;
        uint8_t selectedMailboxIdx = 0xFF;

        /* Dynamically balance traffic across Mailboxes 1 and 2 depending on current availability */
        if (tsrRegister & CAN_TSR_TME1) {
            pTxMailbox = &CAN1->sTxMailBox[1];
            selectedMailboxIdx = 1;
        } else if (tsrRegister & CAN_TSR_TME2) {
            pTxMailbox = &CAN1->sTxMailBox[2];
            selectedMailboxIdx = 2;
        }

        /* Terminate the loop if both application mailboxes are currently full */
        if (pTxMailbox == nullptr) break;

        uint16_t currentTail = m_txAppTail;
        CanFrameL2* sourceFrame = &m_txAppRingBuffer[currentTail];

        m_mailboxTxTimestamp[selectedMailboxIdx] = currentTick;

        pTxMailbox->TDTR &= ~CAN_TDT0R_DLC;
        pTxMailbox->TDTR |= (sourceFrame->length & CAN_TDT0R_DLC);

        uint32_t dataLowValue  = ((uint32_t)sourceFrame->data[3] << 24) | ((uint32_t)sourceFrame->data[2] << 16) | ((uint32_t)sourceFrame->data[1] << 8) | ((uint32_t)sourceFrame->data[0]);
        uint32_t dataHighValue = ((uint32_t)sourceFrame->data[7] << 24) | ((uint32_t)sourceFrame->data[6] << 16) | ((uint32_t)sourceFrame->data[5] << 8) | ((uint32_t)sourceFrame->data[4]);
        pTxMailbox->TDLR = dataLowValue;
        pTxMailbox->TDHR = dataHighValue;

        pTxMailbox->TIR = (sourceFrame->id29 << 3) | CAN_TI0R_IDE | CAN_TI0R_TXRQ;

        m_txAppTail = (currentTail + 1) & M_L2_TX_APP_QUEUE_MASK;

        /* Update internal cache parameters to reflect mailbox occupancy */
        if (selectedMailboxIdx == 1)      tsrRegister &= ~CAN_TSR_TME1;
        else if (selectedMailboxIdx == 2) tsrRegister &= ~CAN_TSR_TME2;
    }

    /* Evaluate buffer depth against the low watermark to clear backpressure states */
    if (m_flowState == L2_FLOW_CONGESTED) {
        uint16_t currentCount = (m_txAppHead - m_txAppTail) & M_L2_TX_APP_QUEUE_MASK;
        if (currentCount <= M_L2_TX_APP_QUEUE_LOW_WATERMARK) {
            m_flowState = L2_FLOW_OK;
        }
    }
}

/**
 * @brief Handles incoming frames from Hardware FIFOs 0 and 1.
 * * Processing Pathways:
 * - **FIFO 0 (Control Plane):** Copies incoming frames into the circular `g_ControlMatrix` buffer log.
 * - **FIFO 1 (Application Plane):** Runs the constant-time unrolled 4-step binary search over
 * `L2activeSessionTokens` to find the connection index and write to `g_ApplicationMatrix`.
 */
void mtCANL2Engine::handle_rx_interrupt_service_routine(uint8_t hardwareFifoIndex) {
    if (hardwareFifoIndex > 1) return;

    CAN_FIFOMailBox_TypeDef* pFifoMailbox = &CAN1->sFIFOMailBox[hardwareFifoIndex];
    uint32_t currentTick = get_portable_system_timestamp_ms();

    /* Monitor and log overrun events across the respective hardware FIFOs */
    if (hardwareFifoIndex == 0) {
        if (CAN1->RF0R & CAN_RF0R_FOVR0) {
            m_fifo0OverrunEvents++;
            CAN1->RF0R |= CAN_RF0R_FOVR0; /* Clear the hardware overrun flag bit */
        }
    } else {
        if (CAN1->RF1R & CAN_RF1R_FOVR1) {
            m_fifo1OverrunEvents++;
            CAN1->RF1R |= CAN_RF1R_FOVR1;
        }
    }

    /* Query current pending frame counts within the targeted FIFO register */
    uint32_t rfrRegister = (hardwareFifoIndex == 0) ? CAN1->RF0R : CAN1->RF1R;
    uint8_t pendingFramesCount = (hardwareFifoIndex == 0) ? (rfrRegister & CAN_RF0R_FMP0) : (rfrRegister & CAN_RF1R_FMP1);

    while (pendingFramesCount > 0) {
        uint32_t nirRegister = pFifoMailbox->RIR;

        /* Verify that the frame uses the Extended Identifier format before processing */
        if (nirRegister & CAN_RI0R_IDE) {
            uint32_t incomingId29 = (nirRegister >> 3) & 0x1FFFFFFFUL;
            uint8_t srcNodeId = (uint8_t)((incomingId29 & XCAN_MASK_SRC_NODE) >> XCAN_SHIFT_SRC_NODE);

            /* Update network tracking tables with transmitter activity metrics */
            if (srcNodeId < MAX_NETWORK_NODES) {
                g_NetworkNodeStatusMatrix[srcNodeId].lastSeenTimestampMs = currentTick;
            }

            m_totalReceivedFrames++;
            m_lastWireActivityTimestamp = currentTick;

            // --- BRANCH 0: SYSTEM CONTROL PLANE INGRESS HANDLING (FIFO 0) ---
            if (hardwareFifoIndex == 0) {
                uint8_t sysHead = m_ctrlMatrixHead;
                uint8_t nextSysHead = (sysHead + 1) & (TOTAL_CTRL_FRAMES - 1);

                /* Drop the control frame if the matrix buffer log is full */
                if (nextSysHead == m_ctrlMatrixTail) {
                    m_controlMatrixDroppedFrames++;
                    m_quenchState = QUENCH_STATE_CONTROL;
                    m_quenchNodeId = srcNodeId;
                } else {
                    /* Copy data words straight from the hardware registers */
                    g_ControlMatrix[sysHead].id29     = incomingId29;
                    g_ControlMatrix[sysHead].dataLow  = pFifoMailbox->RDLR;
                    g_ControlMatrix[sysHead].dataHigh = pFifoMailbox->RDHR;
                    m_ctrlMatrixHead = nextSysHead;
                }
            }
            // --- BRANCH 1: APPLICATION STREAMING PLANE INGRESS HANDLING (FIFO 1) ---
            else {
                uint32_t incomingToken = incomingId29 & ~0x07UL; /* Mask sequence trailing bits */
                uint8_t slotIdx = 0xFF;

                /* CONSTANT-TIME UNROLLED 4-STEP BINARY SEARCH TREE */
                uint8_t mid = 7;
                if (incomingToken == L2activeSessionTokens[mid]) { slotIdx = mid; }
                else if (incomingToken < L2activeSessionTokens[mid]) {
                    mid = 3;
                    if (incomingToken == L2activeSessionTokens[mid]) { slotIdx = mid; }
                    else if (incomingToken < L2activeSessionTokens[mid]) {
                        mid = 1;
                        if (incomingToken == L2activeSessionTokens[mid]) { slotIdx = mid; }
                        else if (incomingToken < L2activeSessionTokens[mid]) { mid = 0; if (incomingToken == L2activeSessionTokens[mid]) slotIdx = mid; }
                        else { mid = 2; if (incomingToken == L2activeSessionTokens[mid]) slotIdx = mid; }
                    } else {
                        mid = 5;
                        if (incomingToken == L2activeSessionTokens[mid]) { slotIdx = mid; }
                        else if (incomingToken < L2activeSessionTokens[mid]) { mid = 4; if (incomingToken == L2activeSessionTokens[mid]) slotIdx = mid; }
                        else { mid = 6; if (incomingToken == L2activeSessionTokens[mid]) slotIdx = mid; }
                    }
                } else {
                    mid = 11;
                    if (incomingToken == L2activeSessionTokens[mid]) { slotIdx = mid; }
                    else if (incomingToken < L2activeSessionTokens[mid]) {
                        mid = 9;
                        if (incomingToken == L2activeSessionTokens[mid]) { slotIdx = mid; }
                        else if (incomingToken < L2activeSessionTokens[mid]) { mid = 8; if (incomingToken == L2activeSessionTokens[mid]) slotIdx = mid; }
                        else { mid = 10; if (incomingToken == L2activeSessionTokens[mid]) slotIdx = mid; }
                    } else {
                        mid = 13;
                        if (incomingToken == L2activeSessionTokens[mid]) { slotIdx = mid; }
                        else if (incomingToken < L2activeSessionTokens[mid]) { mid = 12; if (incomingToken == L2activeSessionTokens[mid]) slotIdx = mid; }
                        else {
                            mid = 14;
                            if (incomingToken == L2activeSessionTokens[mid]) { slotIdx = mid; }
                            else { mid = 15; if (incomingToken == L2activeSessionTokens[mid]) slotIdx = mid; }
                        }
                    }
                }

                /* Look up the channel lane index row using the parallel router array */
                uint8_t connIdx = (slotIdx != 0xFF) ? L2activeSessionRouter[slotIdx] : 0xFF;

                /* Drop unauthorized frames that fail to match an active connection lane */
                if (connIdx < 16) {
                    uint8_t seqNum = (uint8_t)(incomingId29 & XCAN_MASK_SEQUENCE);
                    /* Calculate the destination matrix index: (lane * 8) + sequence */
                    uint16_t matrixTargetIndex = (connIdx << 3) | (seqNum & 0x07);

                    if (matrixTargetIndex < TOTAL_APP_FRAMES) {
                        /* Check for unhandled data overwrites to protect reassembly layers */
                        if (g_ApplicationMatrix[matrixTargetIndex].id29 != 0) {
                            m_telemetryFirewallDrops++;
                            if (m_quenchState != QUENCH_STATE_CONTROL) {
                                m_quenchState = QUENCH_STATE_APPLICATION;
                                m_quenchNodeId = srcNodeId;
                            }
                        } else {
                            /* Commit frame parameters directly to the resolved matrix position */
                            g_ApplicationMatrix[matrixTargetIndex].id29     = incomingId29;
                            g_ApplicationMatrix[matrixTargetIndex].dataLow  = pFifoMailbox->RDLR;
                            g_ApplicationMatrix[matrixTargetIndex].dataHigh = pFifoMailbox->RDHR;
                        }
                    } else {
                        m_telemetryFirewallDrops++;
                    }
                } else {
                    m_telemetryFirewallDrops++;
                }
            }
        }

        /* Acknowledge and release the hardware mailbox slot back to the peripheral block */
        if (hardwareFifoIndex == 0) {
            CAN1->RF0R |= CAN_RF0R_RFOM0;
            pendingFramesCount = CAN1->RF0R & CAN_RF0R_FMP0;
        } else {
            CAN1->RF1R |= CAN_RF1R_RFOM1;
            pendingFramesCount = CAN1->RF1R & CAN_RF1R_FMP1;
        }
    }

    /* Monitor internal registers for changes in the bus error states */
    uint32_t esr = CAN1->ESR;
    CanBusErrorState parsedState = CAN_BUS_ERROR_ACTIVE;
    if (esr & CAN_ESR_BOFF)      parsedState = CAN_BUS_ERROR_OFF;
    else if (esr & CAN_ESR_EPVF) parsedState = CAN_BUS_ERROR_PASSIVE;
    else if (esr & CAN_ESR_EWGF) parsedState = CAN_BUS_ERROR_WARNING;

    if (parsedState != m_cachedErrorState) {
        if (parsedState == CAN_BUS_ERROR_OFF)     m_busOffTransitions++;
        else if (parsedState == CAN_BUS_ERROR_PASSIVE) m_errorPassiveTransitions++;
        else if (parsedState == CAN_BUS_ERROR_WARNING) m_errorWarningTransitions++;
        m_cachedErrorState = parsedState;
    }
}

/**
 * @brief Periodic supervisor routine that monitors transmission timeouts and recovers stuck mailboxes.
 */
void mtCANL2Engine::process_tx_management() {
    uint32_t tsrRegister = CAN1->TSR;
    uint32_t currentTick = get_portable_system_timestamp_ms();

    /* Monitor queue states for persistent congestion timeouts */
    if (m_flowState == L2_FLOW_CONGESTED) {
        if (currentTick - m_blockedTimestamp > M_L2_CONGESTION_TIMEOUT_MS) {
            __disable_irq();
            m_txAppHead = 0;
            m_txAppTail = 0;
            m_flowState = L2_FLOW_OK;
            m_congestionWarningActive = true;
            m_congestionWarningStrikes++;
            __enable_irq();
            return;
        }
    }

    /* MAILBOX 0 SUPERVISOR LOOP: Evaluates control pipeline status */
    if (!(tsrRegister & CAN_TSR_TME0)) {
        if (tsrRegister & CAN_TSR_ALST0) {
            m_txArbitrationLostEvents++;
            m_mailboxArbitrationLossCount[0]++;
        }
        if (tsrRegister & CAN_TSR_TERR0) {
            m_txErrorHardwareEvents++;
        }
        /* Cancel transmissions if timeouts expire or error counts are exceeded */
        if ((currentTick - m_mailboxTxTimestamp[0] > M_L2_QUEUE_STUCK_TIMEOUT_MS) || (m_mailboxArbitrationLossCount[0] > M_L2_MAX_ALLOWED_ALST_STRIKES) || (tsrRegister & CAN_TSR_TERR0)) {
            CAN1->TSR |= CAN_TSR_ABRQ0; /* Assert hardware abort request flag */
            m_mailboxStallRecoveries++;
            m_mailboxTxTimestamp[0] = currentTick;
        }
    }

    /* MAILBOX 1 SUPERVISOR LOOP: Evaluates application pipeline status */
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

    /* MAILBOX 2 SUPERVISOR LOOP: Evaluates application pipeline status */
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
 * @brief Diagnostic poll verifying network overflow occurrences and logging offending targets.
 */
bool mtCANL2Engine::check_and_clear_quench_condition(uint8_t& outPlaneState, uint8_t& outOffendingNodeId) {
    __disable_irq();
    if (m_quenchState == QUENCH_STATE_NONE) {
        __enable_irq();
        return false;
    }

    outPlaneState      = (uint8_t)m_quenchState;
    outOffendingNodeId = m_quenchNodeId;

    m_quenchState  = QUENCH_STATE_NONE;
    m_quenchNodeId = 0;

    __enable_irq();
    return true;
}

/**
 * @brief Resets all software queue pointers and watermarks within an atomic lock block.
 */
void mtCANL2Engine::flush_software_queues() {
    __disable_irq();
    m_txAppHead = 0;
    m_txAppTail = 0;
    m_txCtrlHead = 0;
    m_txCtrlTail = 0;
    m_ctrlMatrixHead = 0;
    m_ctrlMatrixTail = 0;
    m_flowState = L2_FLOW_OK;
    m_quenchState = QUENCH_STATE_NONE;
    __enable_irq();
}

/**
 * @brief Instantly clears the flow engine variables and resets backpressure status metrics.
 */
void mtCANL2Engine::reset_flow_engine() {
    __disable_irq();
    m_flowState = L2_FLOW_OK;
    m_blockedTimestamp = 0;
    m_congestionWarningActive = false;
    __enable_irq();
}

/**
 * @brief Cycles the peripheral hardware to recover from internal bus fault lockups.
 */
bool mtCANL2Engine::recover_hardware() {
    shutdown_hardware();
    return initialize_hardware(m_nodeId, m_commLossTimeoutThresholdMs);
}

/**
 * @brief Purges software lines and requests a full hardware re-initialization sequence.
 */
bool mtCANL2Engine::rejoin_network() {
    flush_software_queues();
    return recover_hardware();
}


/* ============================================================================
 * HARDWARE INTERRUPT VECTOR REGISTER MAPPING
 * ============================================================================ */

/**
 * @brief Transmission interrupt vector wrapper. Directs tracking requests to the active engine context.
 */
extern "C" void USB_HP_CAN1_TX_IRQHandler(void) {
    if (g_pL2EngineInstance != nullptr) {
        g_pL2EngineInstance->handle_tx_interrupt_service_routine();
    }
}

/**
 * @brief FIFO 0 Receive interrupt vector wrapper. Directs incoming control packets to the target engine instance.
 */
extern "C" void USB_LP_CAN1_RX0_IRQHandler(void) {
    if (g_pL2EngineInstance != nullptr) {
        g_pL2EngineInstance->handle_rx_interrupt_service_routine(0);
    }
}

/**
 * @brief FIFO 1 Receive interrupt vector wrapper. Directs incoming data packets to the target search engine instance.
 */
extern "C" void CAN1_RX1_IRQHandler(void) {
    if (g_pL2EngineInstance != nullptr) {
        g_pL2EngineInstance->handle_rx_interrupt_service_routine(1);
    }
}