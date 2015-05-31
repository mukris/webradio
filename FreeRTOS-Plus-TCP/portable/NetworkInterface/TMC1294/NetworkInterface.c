/*
 * FreeRTOS+TCP Labs Build 141019 (C) 2014 Real Time Engineers ltd.
 * Authors include Hein Tibosch and Richard Barry
 *
 *******************************************************************************
 ***** NOTE ******* NOTE ******* NOTE ******* NOTE ******* NOTE ******* NOTE ***
 ***                                                                         ***
 ***                                                                         ***
 ***   FREERTOS+TCP IS STILL IN THE LAB:                                     ***
 ***                                                                         ***
 ***   This product is functional and is already being used in commercial    ***
 ***   products.  Be aware however that we are still refining its design,    ***
 ***   the source code does not yet fully conform to the strict coding and   ***
 ***   style standards mandated by Real Time Engineers ltd., and the         ***
 ***   documentation and testing is not necessarily complete.                ***
 ***                                                                         ***
 ***   PLEASE REPORT EXPERIENCES USING THE SUPPORT RESOURCES FOUND ON THE    ***
 ***   URL: http://www.FreeRTOS.org/contact  Active early adopters may, at   ***
 ***   the sole discretion of Real Time Engineers Ltd., be offered versions  ***
 ***   under a license other than that described below.                      ***
 ***                                                                         ***
 ***                                                                         ***
 ***** NOTE ******* NOTE ******* NOTE ******* NOTE ******* NOTE ******* NOTE ***
 *******************************************************************************
 *
 * - Open source licensing -
 * While FreeRTOS+TCP is in the lab it is provided only under version two of the
 * GNU General Public License (GPL) (which is different to the standard FreeRTOS
 * license).  FreeRTOS+TCP is free to download, use and distribute under the
 * terms of that license provided the copyright notice and this text are not
 * altered or removed from the source files.  The GPL V2 text is available on
 * the gnu.org web site, and on the following
 * URL: http://www.FreeRTOS.org/gpl-2.0.txt.  Active early adopters may, and
 * solely at the discretion of Real Time Engineers Ltd., be offered versions
 * under a license other then the GPL.
 *
 * FreeRTOS+TCP is distributed in the hope that it will be useful.  You cannot
 * use FreeRTOS+TCP unless you agree that you use the software 'as is'.
 * FreeRTOS+TCP is provided WITHOUT ANY WARRANTY; without even the implied
 * warranties of NON-INFRINGEMENT, MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE. Real Time Engineers Ltd. disclaims all conditions and terms, be they
 * implied, expressed, or statutory.
 *
 * 1 tab == 4 spaces!
 *
 * http://www.FreeRTOS.org
 * http://www.FreeRTOS.org/udp
 *
 */

/* Standard includes. */
#include <stdint.h>
#include <stdbool.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* FreeRTOS+TCP includes. */
#include "FreeRTOS_IP.h"
#include "FreeRTOS_IP_Private.h"
#include "NetworkBufferManagement.h"
#include "FreeRTOS_Stream_Buffer.h"

/* Driver includes. */
#include "driverlib/emac.h"
#include "driverlib/flash.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_nvic.h"
#include "inc/hw_emac.h"
#include "inc/hw_memmap.h"

#include "utils/uartstdio.h"

/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

static tEMACDMADescriptor rxDmaDescriptors[configNUM_RX_ETHERNET_DMA_DESCRIPTORS];
static tEMACDMADescriptor txDmaDescriptors[configNUM_TX_ETHERNET_DMA_DESCRIPTORS];
static uint32_t rxDmaDescriptorsIndex;
static uint32_t txDmaDescriptorsIndex;

/* First statically allocate the buffers, ensuring an additional ipBUFFER_PADDING
 bytes are allocated to each buffer. This example makes no effort to align
 the start of the buffers, but most hardware will have an alignment requirement.
 If an alignment is required then the size of each buffer must be adjusted to
 ensure it also ends on an alignment boundary.  Below shows an example assuming
 the buffers must also end on an 8-byte boundary. */
#define BUFFER_SIZE ( ipTOTAL_ETHERNET_FRAME_SIZE + ipBUFFER_PADDING )
#define BUFFER_SIZE_ROUNDED_UP ( ( BUFFER_SIZE + portBYTE_ALIGNMENT_MASK ) & ~portBYTE_ALIGNMENT_MASK )
static uint8_t ucBuffers[ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS][BUFFER_SIZE_ROUNDED_UP] __attribute__((aligned(8)));

/* The semaphore used to wake the deferred interrupt handler task when an Rx
 interrupt is received. */
static xSemaphoreHandle xEMACRxEventSemaphore = NULL;

/*
 * A deferred interrupt handler task that processes received frames.
 */
static void prvEMACDeferredInterruptHandlerTask(void *pvParameters);

static void initDescriptors(uint32_t ui32Base);

inline static tEMACDMADescriptor * getCurrentTxDmaDescriptor(void)
{
	return &txDmaDescriptors[txDmaDescriptorsIndex];
}
static tEMACDMADescriptor * getNextTxDmaDescriptor(void)
{
	txDmaDescriptorsIndex++;
	if (txDmaDescriptorsIndex == configNUM_TX_ETHERNET_DMA_DESCRIPTORS)
	{
		txDmaDescriptorsIndex = 0;
	}
	return &txDmaDescriptors[txDmaDescriptorsIndex];
}

inline static tEMACDMADescriptor * getCurrentRxDmaDescriptor(void)
{
	return &rxDmaDescriptors[rxDmaDescriptorsIndex];
}

static tEMACDMADescriptor * getNextRxDmaDescriptor(void)
{
	rxDmaDescriptorsIndex++;
	if (rxDmaDescriptorsIndex == configNUM_RX_ETHERNET_DMA_DESCRIPTORS)
	{
		rxDmaDescriptorsIndex = 0;
	}
	return &rxDmaDescriptors[rxDmaDescriptorsIndex];
}

#define HANDLED_BY_DMA(dmaDescriptor) (dmaDescriptor->ui32CtrlStatus & DES0_TX_CTRL_OWN)

#define logRX_EVENT_LOST() UARTprintf("***Rx event lost\n")

#if ipconfigETHERNET_DRIVER_FILTERS_FRAME_TYPES != 1
#define ipCONSIDER_FRAME_FOR_PROCESSING( pucEthernetBuffer ) eProcessBuffer
#else
#define ipCONSIDER_FRAME_FOR_PROCESSING( pucEthernetBuffer ) eConsiderFrameForProcessing( ( pucEthernetBuffer ) )
#endif

/*-----------------------------------------------------------*/

BaseType_t xNetworkInterfaceInitialise(void)
{
	uint32_t i, ui32User0, ui32User1;
	uint8_t ui8PHYAddr;
	uint8_t pui8MACAddr[6];

	//
	// Read the MAC address from the user registers.
	//
	FlashUserGet(&ui32User0, &ui32User1);
	if ((ui32User0 == 0xffffffff) || (ui32User1 == 0xffffffff))
	{
		//
		// We should never get here. This is an error if the MAC address has
		// not been programmed into the device. Exit the program.
		//
		while (1)
		{
		}
	}

	//
	// Convert the 24/24 split MAC address from NV ram into a 32/16 split MAC
	// address needed to program the hardware registers, then program the MAC
	// address into the Ethernet Controller registers.
	//
	pui8MACAddr[0] = ((ui32User0 >> 0) & 0xff);
	pui8MACAddr[1] = ((ui32User0 >> 8) & 0xff);
	pui8MACAddr[2] = ((ui32User0 >> 16) & 0xff);
	pui8MACAddr[3] = ((ui32User1 >> 0) & 0xff);
	pui8MACAddr[4] = ((ui32User1 >> 8) & 0xff);
	pui8MACAddr[5] = ((ui32User1 >> 16) & 0xff);

	// Enable and reset the MAC.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_EMAC0);
	SysCtlPeripheralReset(SYSCTL_PERIPH_EMAC0);

	// Enable and reset the internal PHY.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_EPHY0);
	SysCtlPeripheralReset(SYSCTL_PERIPH_EPHY0);

	// Ensure the MAC is completed its reset.
	while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_EMAC0))
	{
	}

	// Set the PHY type and configuration options.
	ui8PHYAddr = 0;
	EMACPHYConfigSet(EMAC0_BASE, EMAC_PHY_TYPE_INTERNAL | EMAC_PHY_INT_MDIX_EN | EMAC_PHY_AN_100B_T_FULL_DUPLEX);

	//
	// Reset the MAC to latch the PHY configuration.
	//
	EMACReset(EMAC0_BASE);

	// Initialize and configure the MAC.
	EMACInit(EMAC0_BASE, configCPU_CLOCK_HZ, EMAC_BCONFIG_MIXED_BURST | EMAC_BCONFIG_PRIORITY_FIXED, 4, 4, 0);

	EMACConfigSet(
			EMAC0_BASE,
			EMAC_CONFIG_USE_MACADDR0 | EMAC_CONFIG_IF_GAP_96BITS | EMAC_CONFIG_7BYTE_PREAMBLE
					| EMAC_CONFIG_BO_LIMIT_1024 | EMAC_CONFIG_SA_FROM_DESCRIPTOR | EMAC_CONFIG_FULL_DUPLEX,
			EMAC_MODE_TX_THRESHOLD_64_BYTES | EMAC_MODE_RX_THRESHOLD_64_BYTES, 0);

	initDescriptors(EMAC0_BASE);

	//
	// Program the hardware with its MAC address (for filtering).
	//
	EMACAddrSet(EMAC0_BASE, 0, pui8MACAddr);

	//
	// Wait for the link to become active.
	//
	while ((EMACPHYRead(EMAC0_BASE, ui8PHYAddr, EPHY_BMSR) & EPHY_BMSR_LINKSTAT) == 0)
	{
	}

	//
	// Set MAC filtering options. We receive all broadcast and multicast
	// packets along with those addressed specifically for us.
	//
	EMACFrameFilterSet(EMAC0_BASE,
						(EMAC_FRMFILTER_SADDR | EMAC_FRMFILTER_PASS_MULTICAST | EMAC_FRMFILTER_PASS_NO_CTRL));

	//
	// Clear any pending interrupts.
	//
	EMACIntClear(EMAC0_BASE, EMACIntStatus(EMAC0_BASE, false));

	//
	// Mark the receive descriptors as available to the DMA to start
	// the receive processing.
	//
	for (i = 0; i < configNUM_RX_ETHERNET_DMA_DESCRIPTORS; i++)
	{
		rxDmaDescriptors[i].ui32CtrlStatus |= DES0_RX_CTRL_OWN;
	}

	/* Create the event semaphore if it has not already been created. */
	if (xEMACRxEventSemaphore == NULL)
	{
		//vSemaphoreCreateBinary(xEMACRxEventSemaphore);
		xEMACRxEventSemaphore = xSemaphoreCreateCounting(configNUM_RX_ETHERNET_DMA_DESCRIPTORS, 0);
#if ipconfigINCLUDE_EXAMPLE_FREERTOS_PLUS_TRACE_CALLS == 1
		{
			/* If the trace recorder code is included name the semaphore for
			 viewing in FreeRTOS+Trace. */
			vTraceSetQueueName( xEMACRxEventSemaphore, "MAC_RX" );
		}
#endif /*  ipconfigINCLUDE_EXAMPLE_FREERTOS_PLUS_TRACE_CALLS == 1 */
	}

	configASSERT(xEMACRxEventSemaphore);

	/* The Rx deferred interrupt handler task is created at the highest
	 possible priority to ensure the interrupt handler can return directly to
	 it no matter which task was running when the interrupt occurred. */
	xTaskCreate(prvEMACDeferredInterruptHandlerTask, /* The function that implements the task. */
				"MACTsk", configMINIMAL_STACK_SIZE, /* Stack allocated to the task (defined in words, not bytes). */
				NULL, /* The task parameter is not used. */
				configMAX_PRIORITIES - 1, /* The priority assigned to the task. */
				NULL); /* The handle is not required, so NULL is passed. */

	//
	// Enable the Ethernet MAC transmitter and receiver.
	//
	EMACTxEnable(EMAC0_BASE);
	EMACRxEnable(EMAC0_BASE);

	IntPrioritySet(INT_EMAC0, configMAC_INTERRUPT_PRIORITY);
	//
	// Enable the Ethernet interrupt.
	//
	IntEnable(INT_EMAC0);

	//
	// Enable the Ethernet RX Packet interrupt source.
	//
	EMACIntEnable(EMAC0_BASE, EMAC_INT_RECEIVE);

	return pdPASS;
}
/*-----------------------------------------------------------*/

BaseType_t xNetworkInterfaceOutput(xNetworkBufferDescriptor_t * const pxNetworkBuffer, BaseType_t bReleaseAfterSend)
{
	tEMACDMADescriptor * txDmaDescriptor;
	configASSERT(xIsCallingFromIPTask() == pdTRUE);iptraceNETWORK_INTERFACE_TRANSMIT();

	txDmaDescriptor = getNextTxDmaDescriptor();

	// Wait for the transmit descriptor to free up.
	while (HANDLED_BY_DMA(txDmaDescriptor))
		;

	/* Further, this example assumes the DMADescriptor_t type has a member
	 called pucEthernetBuffer that points to the buffer the DMA will transmit, and
	 a member called xDataLength that holds the length of the data the DMA will
	 transmit. If BufferAllocation_2.c is being used then the DMA descriptor may
	 still be pointing to the buffer it last transmitted.  If this is the case
	 then the old buffer must be released (returned to the TCP/IP stack) before
	 descriptor is updated to point to the new data waiting to be transmitted. */
	if (txDmaDescriptor->pvBuffer1 != NULL)
	{
		/* Note this is releasing just an Ethernet buffer, not a network buffer
		 descriptor as the descriptor has already been released. */
		//vNetworkBufferRelease(txDmaDescriptor->pvBuffer1);
	}

	// Fill in the packet size and pointer, and tell the transmitter to start
	// work.
	txDmaDescriptor->ui32Count = (DES1_TX_CTRL_BUFF1_SIZE_M
			& (pxNetworkBuffer->xDataLength << DES1_TX_CTRL_BUFF1_SIZE_S));
	txDmaDescriptor->pvBuffer1 = pxNetworkBuffer->pucEthernetBuffer;
	txDmaDescriptor->ui32CtrlStatus = (DES0_TX_CTRL_LAST_SEG | DES0_TX_CTRL_FIRST_SEG |
	DES0_TX_CTRL_INTERRUPT | DES0_TX_CTRL_IP_ALL_CKHSUMS |
	DES0_TX_CTRL_CHAINED | DES0_TX_CTRL_OWN);

	//
	// Tell the DMA to reacquire the descriptor now that we’ve filled it in.
	// This call is benign if the transmitter hasn’t stalled and checking
	// the state takes longer than just issuing a poll demand so we do this
	// for all packets.
	//
	EMACTxDMAPollDemand(EMAC0_BASE);

	/* The network buffer descriptor must now be returned to the TCP/IP stack, but
	 the Ethernet buffer referenced by the network buffer descriptor is still in
	 use by the DMA.  Remove the reference to the Ethernet buffer from the network
	 buffer descriptor so releasing the network buffer descriptor does not result
	 in the Ethernet buffer also being released. */
	if (bReleaseAfterSend != pdFALSE)
	{
		//pxNetworkBuffer->pucEthernetBuffer = NULL;
		vReleaseNetworkBufferAndDescriptor(pxNetworkBuffer);
	}

	return pdPASS;
}
/*-----------------------------------------------------------*/

void ETH_IRQHandler(void)
{
	uint32_t ulInterruptCause;

	//UARTprintf("ETH interrupt\n");

	ulInterruptCause = EMACIntStatus(EMAC0_BASE, true);
	EMACIntClear(EMAC0_BASE, ulInterruptCause);

	/* Unblock the deferred interrupt handler task if the event was an Rx. */
	if ((ulInterruptCause & EMAC_INT_RECEIVE) != 0UL)
	{
		xSemaphoreGiveFromISR(xEMACRxEventSemaphore, NULL);
	}

	/* ulInterruptCause is used for convenience here.  A context switch is
	 wanted, but coding portEND_SWITCHING_ISR( 1 ) would likely result in a
	 compiler warning. */
	portEND_SWITCHING_ISR(ulInterruptCause);
}
/*-----------------------------------------------------------*/

static void swapReceiveBuffers(tEMACDMADescriptor * dmaDescriptor, xNetworkBufferDescriptor_t *pxNetworkBuffer)
{
	uint8_t *pucTemp;
	int32_t i32FrameLen;

	/*
	 * Swap the buffer in the network buffer with the buffer used by the DMA.
	 * This allows the data to be passed out without having to perform any copies.
	 */
	pucTemp = (uint8_t *) dmaDescriptor->pvBuffer1;
	dmaDescriptor->pvBuffer1 = pxNetworkBuffer->pucEthernetBuffer;
	pxNetworkBuffer->pucEthernetBuffer = pucTemp;

	i32FrameLen = ((dmaDescriptor->ui32CtrlStatus & DES0_RX_STAT_FRAME_LENGTH_M) >> DES0_RX_STAT_FRAME_LENGTH_S);

	/*
	 * Only supports frames coming in single buffers. If this frame is split
	 * across multiple buffers then reject it (and if the frame is needed increase
	 * the ipconfigNETWORK_MTU setting).
	 */
	if ((dmaDescriptor->ui32CtrlStatus & DES0_RX_STAT_LAST_DESC) != DES0_RX_STAT_LAST_DESC)
	{
		pxNetworkBuffer->xDataLength = 0;
	}
	else
	{
		pxNetworkBuffer->xDataLength = (size_t) i32FrameLen - ( ipSIZE_OF_ETH_CRC_BYTES - 1U);
	}
}
/*-----------------------------------------------------------*/

static void prvEMACDeferredInterruptHandlerTask(void *pvParameters)
{
	tEMACDMADescriptor * rxDmaDescriptor;
	xNetworkBufferDescriptor_t *pxNetworkBuffer;
	xIPStackEvent_t xRxEvent;
	uint32_t i;

	(void) pvParameters;
	configASSERT(xEMACRxEventSemaphore);

	for (;;)
	{
		// Wait for an incoming packet
		xSemaphoreTake(xEMACRxEventSemaphore, portMAX_DELAY);

		// Get the DMA descriptor of the received packet
		rxDmaDescriptor = getCurrentRxDmaDescriptor();

		// Ensure that the DMA finished working with the buffer
		if (HANDLED_BY_DMA(rxDmaDescriptor))
		{
			continue;
		}

		// The buffer filled by the DMA is going to be passed into the IP stack.
		// Allocate another buffer for the DMA descriptor.
		pxNetworkBuffer = pxGetNetworkBufferWithDescriptor( ipTOTAL_ETHERNET_FRAME_SIZE, (TickType_t) 0);

		// We cannot do anything without a buffer...
		if (pxNetworkBuffer == NULL)
		{
			logRX_EVENT_LOST();
			continue;
		}

		// Swap the buffer just allocated and referenced from the
		// pxNetworkBuffer with the buffer that has already been filled by
		// the DMA. pxNetworkBuffer will then hold a reference to the
		// buffer that already contains the data without any data having
		// been copied between buffers.
		swapReceiveBuffers(rxDmaDescriptor, pxNetworkBuffer);

		// Give the descriptor back to the DMA with the new empty buffer
		rxDmaDescriptor->ui32CtrlStatus = DES0_RX_CTRL_OWN;

		// Store a pointer to the network buffer structure in the
		// padding space that was left in front of the Ethernet frame.
		// The pointer is needed to ensure the network buffer structure
		// can be located when it is time for it to be freed if the
		// Ethernet frame gets used as a zero copy buffer.
		*((xNetworkBufferDescriptor_t **) ((pxNetworkBuffer->pucEthernetBuffer - ipBUFFER_PADDING))) = pxNetworkBuffer;

		// If the frame would not be processed by the IP stack then
		// don't even bother sending it to the IP stack.
		if (eConsiderFrameForProcessing(pxNetworkBuffer->pucEthernetBuffer) != eProcessBuffer)
		{
			pxNetworkBuffer->xDataLength = 0;
		}

		if (pxNetworkBuffer->xDataLength > 0)
		{
			xRxEvent.eEventType = eNetworkRxEvent;
			xRxEvent.pvData = (void *) pxNetworkBuffer;

			// Data was received and stored. Send it to the IP task for processing.
			if (xSendEventStructToIPTask(&xRxEvent, 0) == pdFALSE)
			{
				// The buffer could not be sent to the IP task so the
				// buffer must be released.
				vReleaseNetworkBufferAndDescriptor(pxNetworkBuffer);
				logRX_EVENT_LOST();
			}
		}
		else
		{
			// The buffer does not contain any data so there is no
			// point sending it to the IP task. Just release it.
			vReleaseNetworkBufferAndDescriptor(pxNetworkBuffer);
		}

		getNextRxDmaDescriptor();
	}
}
/*-----------------------------------------------------------*/

//*****************************************************************************
//
// Initialize the transmit and receive DMA descriptors.
//
//*****************************************************************************
static void initDescriptors(uint32_t ui32Base)
{
	uint32_t i;

	/*
	 * Initialize each of the transmit descriptors. Note that we leave the
	 * buffer pointer and size empty and the OWN bit clear here since we have
	 * not set up any transmissions yet.
	 */
	for (i = 0; i < configNUM_TX_ETHERNET_DMA_DESCRIPTORS; i++)
	{
		txDmaDescriptors[i].ui32CtrlStatus = (DES0_TX_CTRL_LAST_SEG | DES0_TX_CTRL_FIRST_SEG |
		DES0_TX_CTRL_INTERRUPT | DES0_TX_CTRL_CHAINED |
		DES0_TX_CTRL_IP_ALL_CKHSUMS);
		txDmaDescriptors[i].ui32Count = DES1_TX_CTRL_SADDR_INSERT;
		txDmaDescriptors[i].DES3.pLink =
				(i == (configNUM_TX_ETHERNET_DMA_DESCRIPTORS - 1)) ? txDmaDescriptors : &txDmaDescriptors[i + 1];
	}
	/*
	 * Initialize each of the receive descriptors. We clear the OWN bit here
	 * to make sure that the receiver doesn’t start writing anything
	 * immediately.
	 */
	for (i = 0; i < configNUM_RX_ETHERNET_DMA_DESCRIPTORS; i++)
	{
		rxDmaDescriptors[i].ui32CtrlStatus = 0;
		rxDmaDescriptors[i].ui32Count = (DES1_RX_CTRL_CHAINED
				| (ipTOTAL_ETHERNET_FRAME_SIZE << DES1_RX_CTRL_BUFF1_SIZE_S));
		rxDmaDescriptors[i].DES3.pLink =
				(i == (configNUM_RX_ETHERNET_DMA_DESCRIPTORS - 1)) ? rxDmaDescriptors : &rxDmaDescriptors[i + 1];
		rxDmaDescriptors[i].pvBuffer1 =
				pxGetNetworkBufferWithDescriptor(ipTOTAL_ETHERNET_FRAME_SIZE, (TickType_t) 0)->pucEthernetBuffer;
	}

	/* Set the descriptor pointers in the hardware. */
	EMACRxDMADescriptorListSet(ui32Base, rxDmaDescriptors);
	EMACTxDMADescriptorListSet(ui32Base, txDmaDescriptors);

	/*
	 * Start from the beginning of both descriptor chains. We actually set
	 * the transmit descriptor index to the last descriptor in the chain
	 * since it will be incremented before use and this means the first
	 * transmission we perform will use the correct descriptor.
	 */
	rxDmaDescriptorsIndex = 0;
	txDmaDescriptorsIndex = configNUM_TX_ETHERNET_DMA_DESCRIPTORS - 1;
}
/*-----------------------------------------------------------*/

/*
 * Next provide the vNetworkInterfaceAllocateRAMToBuffers() function, which
 * simply fills in the pucEthernetBuffer member of each descriptor.
 */
void vNetworkInterfaceAllocateRAMToBuffers(
		xNetworkBufferDescriptor_t pxNetworkBuffers[ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS])
{
	BaseType_t i;

	for (i = 0; i < ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS; i++)
	{
		/*
		 * pucEthernetBuffer is set to point ipBUFFER_PADDING bytes in from the
		 * beginning of the allocated buffer.
		 */
		pxNetworkBuffers[i].pucEthernetBuffer = &(ucBuffers[i][ ipBUFFER_PADDING]);

		/*
		 * The following line is also required, but will not be required in
		 * future versions.
		 */
		*((uint32_t *) &ucBuffers[i][0]) = (uint32_t) &(pxNetworkBuffers[i]);
	}
}
/*-----------------------------------------------------------*/

