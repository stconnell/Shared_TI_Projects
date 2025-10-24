/*
 * Copyright (c) 2018-2019 Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== EMACF2838X.c ========
 *  Implementation of the F2838x EMAC driver
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

/* Use local (modified) copy of Low Level Driver (LLD) Ethernet module: */
#include "ethernet.h"

#include <xdc/runtime/System.h>
#include <ti/sysbios/hal/Hwi.h>

/* NDK pre-processor guard to avoid a POSIX dependency */
#define NDK_NOUSERAPIS 1

#include <ti/ndk/drivers/f2838x/EMACF2838X.h>

#include <ti/ndk/inc/stkmain.h>

/* Ensure the length of the Ethernet IF name is legal */
#define ASSERT_CONCAT_(a, b) a##b
#define ASSERT_CONCAT(a, b) ASSERT_CONCAT_(a, b)
#define STATIC_ASSERT(e,m) \
        ;enum { ASSERT_CONCAT(assert_line_, __LINE__) = 1/(int)(!!(e)) }

/* The below assertion fails if the interface name string is too long */
STATIC_ASSERT((sizeof(ETHERNET_NAME) <= MAX_INTERFACE_NAME_LEN), \
        "Error: Interface name string is too long");

/* The size of the CRC stored at the end of the received frames */
#define CRC_SIZE_BYTES 4

#define RX_BUFFER_SIZE (ETH_MAX_PAYLOAD + CRC_SIZE_BYTES)

/*
 * The struct is used to store the private data for the EMACF2838X controller.
 */
typedef struct EMACF2838X_Object {
    STKEVENT_Handle  hEvent;
    PBMQ             PBMQ_rx;
    uint32_t         rxCount;
    uint32_t         rxDrops;
    uint32_t         epdAllocErrorsRx;
    uint32_t         pbmAllocErrorsRx;
    uint32_t         txCount;
    uint32_t         txDrops;
    uint32_t         epdAllocErrorsTx;
    uint32_t         linkUp;

    EMACF2838XLLD_Device *emacDevice;
    Hwi_Handle       hwiTxCh0;
    Hwi_Handle       hwiRxCh0;
    Hwi_Handle       hwiGeneric;
} EMACF2838X_Object;

/*
 * This Ethernet module stat will be updated in this file, as its
 * associated callback is defined here, and also called from within this
 * module.
 */
extern uint32_t EMACF2838XLLD_numGetPacketBufferCallback;

/* *** Only supporting a single Ethernet interface *** */
static EMACF2838XLLD_InitInterfaceConfig emacInitCfg;
static EMACF2838X_Object emacData;

/* Funtion prototypes */
static void EMACF2838X_processTransmitted(EMACF2838XLLD_Handle h,
        EMACF2838XLLD_Pkt_Desc *ethFrame);
static EMACF2838XLLD_Pkt_Desc *EMACF2838X_getPacketBuffer(void);
static EMACF2838XLLD_Pkt_Desc *EMACF2838X_handleRx(EMACF2838XLLD_Handle h,
        EMACF2838XLLD_Pkt_Desc *ethFrame);
static int EMACF2838X_emacStart(struct NETIF_DEVICE* ptr_net_device);
static int EMACF2838X_emacStop(struct NETIF_DEVICE* ptr_net_device);
static bool isLinkUp(void);

/*
 *  ======== EMACF2838X_processTransmitted ========
 *  Free the TX Ethernet Packet Descriptor (EPD) and PBM buffer.
 *
 *  This function is called when the DMA has completed the transfer of the TX
 *  frame to the EMAC (and is hence done with the buffer)
 */
static void EMACF2838X_processTransmitted(EMACF2838XLLD_Handle h,
        EMACF2838XLLD_Pkt_Desc *ethPktDesc)
{
    /* If DMA has completed, free the PBM buffer and packet descriptor */
    if (ethPktDesc->flags & ETHERNET_INTERRUPT_FLAG_TRANSMIT) {
        /* Free the NDK PBM buffer and the Ethernet packet */
        PBM_free(ethPktDesc->pbuf);
        mmFree(ethPktDesc);
    }
}

/*
 *  ======== EMACF2838X_getPacketBuffer ========
 *  Get a new EPD + PBM to hold RX data
 */
static EMACF2838XLLD_Pkt_Desc *EMACF2838X_getPacketBuffer(void)
{
    EMACF2838XLLD_Pkt_Desc *ethPktDesc = NULL;
    PBM_Handle hPkt = NULL;

    EMACF2838XLLD_numGetPacketBufferCallback++;

    hPkt = PBM_alloc(RX_BUFFER_SIZE);
    if (!hPkt) {
        emacData.pbmAllocErrorsRx++;
        return (NULL);
    }

    ethPktDesc = mmAlloc(sizeof(EMACF2838XLLD_Pkt_Desc));
    if (!ethPktDesc) {
        emacData.epdAllocErrorsRx++;
        PBM_free(hPkt);
        return (NULL);
    }

    mmZeroInit(ethPktDesc, sizeof(EMACF2838XLLD_Pkt_Desc));

    /*
     * Couple the PBM and the EPD
     *
     * Set the EPD's buffer to point to the PBM's buffer (we want the DMA to
     * write directly into the PBM's data buffer)
     */
    ethPktDesc->dataBuffer = PBM_getDataBuffer(hPkt);

    ethPktDesc->dataOffset = 0;
    ethPktDesc->bufferLength = PBM_getBufferLen(hPkt);

    /*
     * Store a back reference to the PBM in the EPD. This will be used to
     * decouple the EPD and PBM later, so that the PBM can be passed up the
     * stack, and the EPD can be freed upon RX of the packet (frame)
     */
    ethPktDesc->pbuf = (void *)hPkt;

    /* 
     * Return the new EPD, where it will be enqueued for later consumption
     * by the RX DMA
     */
    return (ethPktDesc);
}

/*
 *  ======== EMACF2838X_handleRx ========
 *  Receive an Ethernet frame from the LLD.
 *
 *  - Get the PBM packet out of the EPD and pass it up the stack (the stack will
 *    ultimately free the PBM).
 *  - Allocate a new EPD and return to LLD to be enqueued for later use.
 */
static EMACF2838XLLD_Pkt_Desc *EMACF2838X_handleRx(EMACF2838XLLD_Handle h,
        EMACF2838XLLD_Pkt_Desc *ethPktDesc)
{
    PBM_Handle hPkt;

    /* Get the PBM associated with this EPD */
    hPkt = (PBM_Handle)ethPktDesc->pbuf;

    if (ethPktDesc->flags & ETHERNET_PKT_FLAG_CSE) {
        /* RX Checksum Offload Engine reported bad checksum. Drop it. */
        PBM_free(hPkt);
        emacData.rxDrops++;
    }
    else {
        emacData.rxCount++;

        /*
         * Update the PBM's buffer offsets with those set by DMA
         *
         *     1. Remove the CRC
         *     2. Set the valid length
         */
        PBM_setValidLen(hPkt, ethPktDesc->pktLength - CRC_SIZE_BYTES);
        PBM_setDataOffset(hPkt, ethPktDesc->dataOffset);
    
        /*
         *  Place the packet onto the receive queue to be handled in the
         *  EMACF2838X_pkt_service function (which is called by the
         *  NDK stack).
         */
        PBMQ_enq(&emacData.PBMQ_rx, hPkt);
    
        /*
         *  Notify NDK stack of pending Rx Ethernet packet and
         *  that it was triggered by an external event.
         */
        STKEVENT_signal(emacData.hEvent, STKEVENT_ETHERNET, 1);
    }

    /*
     * We are done with this EPD, free it (the PBM is freed by the stack,
     * or above in the case of Checksum error).
     */
    mmFree(ethPktDesc);

    /* Allocate a new EPD + PBM to replenish the RX Q (in Ethernet module) */
    return (EMACF2838X_getPacketBuffer());
}

/*
 *  ======== EMACF2838X_emacStart ========
 *  The function is used to initialize and start the EMAC
 *  controller and device.
 */
static int EMACF2838X_emacStart(struct NETIF_DEVICE* ptr_net_device)
{
    Hwi_Params hwiParams;

    /* Create the hardware interrupts */
    Hwi_Params_init(&hwiParams);
    hwiParams.priority = (~0);

    /* Create the RX channel 0 Hwi */
    emacData.hwiRxCh0 = Hwi_create(ETHERNET_RX_INTR_CH0,
            (Hwi_FuncPtr)EMACF2838XLLD_receiveISR, &hwiParams, NULL);

    if (emacData.hwiRxCh0 == NULL) {
        EMACF2838X_emacStop(ptr_net_device);
        return (-1);
    }

    /* Create the TX channel 0 Hwi */
    emacData.hwiTxCh0 = Hwi_create(ETHERNET_TX_INTR_CH0,
            (Hwi_FuncPtr)EMACF2838XLLD_transmitISR, &hwiParams, NULL);

    if (emacData.hwiTxCh0 == NULL) {
        EMACF2838X_emacStop(ptr_net_device);
        return (-1);
    }

    /* Create the Generic Interrupt Hwi */
    emacData.hwiGeneric = Hwi_create(ETHERNET_GENERIC_INTERRUPT,
            (Hwi_FuncPtr)EMACF2838XLLD_genericISR, &hwiParams, NULL);

    if (emacData.hwiGeneric == NULL) {
        EMACF2838X_emacStop(ptr_net_device);
        return (-1);
    }

    return (0);
}

/*
 *  ======== EMACF2838X_emacStop ========
 *  The function is used to de-initialize and stop the EMAC
 *  controller and device.
 */
static int EMACF2838X_emacStop(struct NETIF_DEVICE* ptr_net_device)
{
    PBM_Handle hPkt = NULL;

    if (emacData.hwiRxCh0 != NULL) {
        Hwi_delete(&emacData.hwiRxCh0);
    }
    
    if (emacData.hwiTxCh0 != NULL) {
        Hwi_delete(&emacData.hwiTxCh0);
    }
    
    if (emacData.hwiGeneric != NULL) {
        Hwi_delete(&emacData.hwiGeneric);
    }

    /*
     * TODO: Below call currently does nothing. Need to ensure all EPDs and
     * PBMs that were allocated during init calls are freed. May also need to
     * disable the MAC here, if below doesn't handle it.
     */
    /* Call the LLD clean up function to free EPDs in each channel's queue */
    EMACF2838XLLD_shutdownInterface();

    /* Dequeue and free all packets from the driver receive queue */
    while (PBMQ_count(&emacData.PBMQ_rx)) {
        hPkt = PBMQ_deq(&emacData.PBMQ_rx);
        PBM_free(hPkt);
    }

    return (0);
}

/*
 *  ======== EMACF2838X_emacPoll ========
 *  The function is used to poll the EMACF2838X controller to check
 *  if there has been any activity
 */
static void EMACF2838X_emacPoll(struct NETIF_DEVICE* ptr_net_device,
        uint32_t timer_tick)
{
    /* Check/update the link status */
    isLinkUp();
}

/*
 *  ======== EMACF2838X_emacSend ========
 *  The function is the interface routine invoked by the NDK stack to
 *  pass packets to the driver. Couple the TX PBM from the stack with an EPD
 *  and then pass the EPD down to the LLD.
 */
static int EMACF2838X_emacSend(struct NETIF_DEVICE* ptr_net_device,
        PBM_Handle hPkt)
{
    uint32_t status;
    EMACF2838XLLD_Pkt_Desc *ethPktDesc = NULL;

    ethPktDesc = mmAlloc(sizeof(EMACF2838XLLD_Pkt_Desc));
    if (!ethPktDesc) {
        emacData.epdAllocErrorsTx++;
        emacData.txDrops++;
        PBM_free(hPkt);
        return (-1);
    }

    mmZeroInit(ethPktDesc, sizeof(EMACF2838XLLD_Pkt_Desc));

    /* Copy the relevant data of the PBM object into the EPD */
    ethPktDesc->dataOffset = PBM_getDataOffset(hPkt);
    ethPktDesc->dataBuffer = PBM_getDataBuffer(hPkt);

    ethPktDesc->pktChannel = ETHERNET_DMA_CHANNEL_NUM_0;
    ethPktDesc->pktLength = PBM_getValidLen(hPkt);
    ethPktDesc->bufferLength = PBM_getBufferLen(hPkt);
    ethPktDesc->validLength = PBM_getValidLen(hPkt);
    ethPktDesc->timeStampHigh = 0; /* Note: PTP not supported */
    ethPktDesc->timeStampLow = 0; /* Note: PTP not supported */

    /* This only handles a single Ethernet frame at a time: */
    ethPktDesc->numPktFrags = 1;
    ethPktDesc->nextPacketDesc = 0;
    ethPktDesc->flags = ETHERNET_PKT_FLAG_SOP | ETHERNET_PKT_FLAG_EOP |
            ETHERNET_PKT_FLAG_CIC; /* Enable RX Checksum Offload in hardware */

    /* We'll use this back reference to the PBM to free it once DMA's done */
    ethPktDesc->pbuf = (void *)hPkt;

    status = EMACF2838XLLD_sendPacket(emacData.emacDevice, ethPktDesc);
    if (status != ETHERNET_RET_SUCCESS) {
        mmFree(ethPktDesc);
        PBM_free(hPkt);
        emacData.txDrops++;
        return (-1);
    }
    else {
       emacData.txCount++;
    }

    return (0);
}

/*
 *  ======== EMACF2838X_emacioctl ========
 *  The function is called by the NDK core stack to configure the driver
 */
static int EMACF2838X_emacioctl(struct NETIF_DEVICE* ptr_net_device,
        uint32_t cmd, void* pbuf, uint32_t size)
{
    int retval;

    switch (cmd) {
        case NIMU_ADD_MULTICAST_ADDRESS:
        case NIMU_DEL_MULTICAST_ADDRESS:
            /*
             * Must return success here, even though this isn't supported. The
             * stack treats this driver multicast filtering feature as a hard
             * requirement, when it should really be treated as an
             * optimization. Refer to NDK-399 for further details).
             */
            retval = 0;
            break;

        case NIMU_GET_DEVICE_ISLINKUP:
            if (size >= sizeof(uint32_t)) {
                if (isLinkUp()) {
                    *(uint32_t *)pbuf = 1;
                }
                else {
                    *(uint32_t *)pbuf = 0;
                }
                retval = 0;
            }
            else {
                /* user-provided buffer is too small */
                retval = -(NDK_EINVAL);
            }

            break;

        default:
            retval = -(NDK_EINVAL);
            break;
    }

    return (retval);
}

/*
 *  ======== EMAC_deletePackets ========
 *  Called to free any packets held in the RX queue that the NDK has not yet
 *  been able to consume.
 *  This happens when the target is halted and results in these packets never
 *  being freed due to:
 *
 *  1. The RBU error occuring and in result halting DMA operation.
 *  2. The NDK thread being continuously starved due to the generic ISR running
 *     over and over
 *
 *  Freeing these packets allows the next, new batch of incoming packets to be
 *  allocated and
 *  properly handled (dequeued and processed/free by the upper layers of the
 *  stack)
 */
static void EMAC_deletePackets(void)
{
    PBM_Handle hPkt;

    /* Dequeue and free all packets from the driver receive queue */
    while (PBMQ_count(&emacData.PBMQ_rx)) {
        hPkt = PBMQ_deq(&emacData.PBMQ_rx);
        PBM_free(hPkt);
        emacData.rxDrops++;
    }

    /* Work has been completed; the receive queue is empty. */
    return;
}

/*
 *  ======== EMACF2838X_pkt_service ========
 *  The function is called by the NDK core stack to receive any packets
 *  from the driver.
 */
static void EMACF2838X_pkt_service(NETIF_DEVICE *ptr_net_device)
{
    PBM_Handle hPkt;

    /* Give all queued packets to the stack */
    while (PBMQ_count(&emacData.PBMQ_rx)) {

        /* Dequeue a packet from the driver receive queue. */
        hPkt = PBMQ_deq(&emacData.PBMQ_rx);

        /*
         *  Prepare the packet so that it can be passed up the networking stack.
         *  If this 'step' is not done the fields in the packet are not correct
         *  and the packet will eventually be dropped.
         */
        PBM_setIFRx(hPkt, ptr_net_device);

        /* Pass the packet to the NDK Core stack. */
        NIMUReceivePacket(hPkt);
    }

    /* Work has been completed; the receive queue is empty. */
    return;
}

/*
 *  ======== EMACF2838X_NIMUInit ========
 *  The function is used to initialize and register the EMACF2838X
 *  with the Network Interface Management Unit (NIMU)
 */
int EMACF2838X_NIMUInit(STKEVENT_Handle hEvent)
{
    uint32_t status;
    uint32_t mac_lower;
    uint32_t mac_higher;

    NETIF_DEVICE *device;
    EMACF2838XLLD_InitConfig *pInitCfg = NULL;

    /* Initialize the global structures */
    memset(&emacData, 0, sizeof(EMACF2838X_Object));

    /*
     * Allocate memory for the EMAC. Memory freed in the NDK stack shutdown
     * (this will happen independently of the success/failure of this function)
     */
    device = mmAlloc(sizeof(NETIF_DEVICE));
    if (device == NULL) {
        return (-1);
    }

    /* Initialize the allocated memory block. */
    mmZeroInit(device, sizeof(NETIF_DEVICE));

    /* Initialize the Packet Device Information struct */
    PBMQ_init(&emacData.PBMQ_rx);
    emacData.hEvent = hEvent;

    emacData.rxCount = 0;
    emacData.rxDrops = 0;
    emacData.epdAllocErrorsRx = 0;
    emacData.pbmAllocErrorsRx = 0;
    emacData.txCount = 0;
    emacData.txDrops = 0;
    emacData.epdAllocErrorsTx = 0;
    emacData.linkUp = false;

    /* Initialize the device */
    memset(&emacInitCfg, 0, sizeof(emacInitCfg));
    emacInitCfg.ssbase = EMAC_SS_BASE;
    emacInitCfg.eqos_base = EMAC_BASE;
    emacInitCfg.phyMode = ETHERNET_SS_PHY_INTF_SEL_MII;
    pInitCfg = EMACF2838XLLD_initInterface(emacInitCfg);
    if (!pInitCfg) {
        goto EMACF2838X_NIMUInitFail;
    }

    /* Fill in the init config struct with known good values */
    EMACF2838XLLD_getInitConfig(pInitCfg);

    /* Override the default callbacks set by EMACF2838XLLD_getInitConfig() */
    pInitCfg->pfcbFreePacket = &EMACF2838X_processTransmitted;
    pInitCfg->pfcbGetPacket = &EMACF2838X_getPacketBuffer;
    pInitCfg->pfcbRxPacket = &EMACF2838X_handleRx;
    pInitCfg->pfcbDeletePackets = &EMAC_deletePackets;

    /* Prepare device for send and receive */
    status = EMACF2838XLLD_getHandle((EMACF2838XLLD_Handle)device, pInitCfg,
            (EMACF2838XLLD_Handle *)&(emacData.emacDevice));
    if (status != ETHERNET_RET_SUCCESS) {
        goto EMACF2838X_NIMUInitFail;
    }

    /*
     * Program the MAC address into the Ethernet controller. NOTE: must be done
     * after call to EMACF2838XLLD_getHandle()
     */

    mac_lower = EMACF2838X_macAddrLow;
    mac_higher = EMACF2838X_macAddrHigh;

    EMACF2838XLLD_setMACAddr(EMAC_BASE, 0, mac_higher, mac_lower,
            ETHERNET_CHANNEL_0);

    /* set the MAC address in the NIMU device */
    device->mac_address[0] = ((mac_lower >>  0) & 0xff);
    device->mac_address[1] = ((mac_lower >>  8) & 0xff);
    device->mac_address[2] = ((mac_lower >> 16) & 0xff);
    device->mac_address[3] = ((mac_lower >> 24) & 0xff);
    device->mac_address[4] = ((mac_higher >> 0) & 0xff);
    device->mac_address[5] = ((mac_higher >> 8) & 0xff);

    /* Populate the Network Interface Object. */
    
    /*
     * Copy the interface name string. Its size (including \0 terminator) must
     * fit into the NIMU device name, which is an array of
     * size MAX_INTERFACE_NAME_LEN bytes
     */
    strcpy(device->name, ETHERNET_NAME);

    device->mtu = ETH_MAX_PAYLOAD - ETHHDR_SIZE;
    device->pvt_data = (void *)&emacData;

    /* Inform NDK stack that this driver enables Checksum Offloading */
    device->flags = NIMU_DEVICE_ENABLE_HW_CHKSM_TX_ALL |
                    NIMU_DEVICE_ENABLE_HW_CHKSM_RX_ALL;

    /* Populate the Driver Interface Functions. */
    device->start       = EMACF2838X_emacStart;
    device->stop        = EMACF2838X_emacStop;
    device->poll        = EMACF2838X_emacPoll;
    device->send        = EMACF2838X_emacSend;
    device->pkt_service = EMACF2838X_pkt_service;
    device->ioctl       = EMACF2838X_emacioctl;
    device->add_header  = NIMUAddEthernetHeader;

    /* Register the device with NIMU */
    if (NIMURegister(device) < 0) {
        goto EMACF2838X_NIMUInitFail;
    }

    return (0);

EMACF2838X_NIMUInitFail:
    mmFree(device);
    return (-1);
}

/*
 *  ======== isLinkUp ========
 */
static bool isLinkUp(void)
{
    uint32_t newLinkStatus;

    /* Get the current link status */
    newLinkStatus = (EMACF2838XLLD_readPHYRegister(EMAC_BASE,
        ETHERNET_PHY_BMSR_ADDR) & ETHERNET_PHY_BMSR_LINKSTS) >>
        ETHERNET_PHY_BMSR_LINKSTS_S;

    /* Check if the link status changed */
    if (newLinkStatus != emacData.linkUp) {
        /* Set the new link status */
        emacData.linkUp = newLinkStatus;

        /* Signal the stack with the new link status */
        STKEVENT_signal(emacData.hEvent,
                emacData.linkUp ? STKEVENT_LINKUP : STKEVENT_LINKDOWN, 0);
    }

    return (emacData.linkUp);
}
