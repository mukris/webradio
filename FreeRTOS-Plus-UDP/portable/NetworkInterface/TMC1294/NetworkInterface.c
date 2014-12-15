/*
 * FreeRTOS+UDP V1.0.4 (C) 2014 Real Time Engineers ltd.
 * All rights reserved
 *
 * This file is part of the FreeRTOS+UDP distribution.  The FreeRTOS+UDP license
 * terms are different to the FreeRTOS license terms.
 *
 * FreeRTOS+UDP uses a dual license model that allows the software to be used
 * under a pure GPL open source license (as opposed to the modified GPL license
 * under which FreeRTOS is distributed) or a commercial license.  Details of
 * both license options follow:
 *
 * - Open source licensing -
 * FreeRTOS+UDP is a free download and may be used, modified, evaluated and
 * distributed without charge provided the user adheres to version two of the
 * GNU General Public License (GPL) and does not remove the copyright notice or
 * this text.  The GPL V2 text is available on the gnu.org web site, and on the
 * following URL: http://www.FreeRTOS.org/gpl-2.0.txt.
 *
 * - Commercial licensing -
 * Businesses and individuals that for commercial or other reasons cannot comply
 * with the terms of the GPL V2 license must obtain a commercial license before
 * incorporating FreeRTOS+UDP into proprietary software for distribution in any
 * form.  Commercial licenses can be purchased from http://shop.freertos.org/udp
 * and do not require any source files to be changed.
 *
 * FreeRTOS+UDP is distributed in the hope that it will be useful.  You cannot
 * use FreeRTOS+UDP unless you agree that you use the software 'as is'.
 * FreeRTOS+UDP is provided WITHOUT ANY WARRANTY; without even the implied
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
#include "queue.h"
#include "semphr.h"

/* FreeRTOS+UDP includes. */
#include "FreeRTOS_UDP_IP.h"
#include "FreeRTOS_IP_Private.h"
#include "FreeRTOS_Sockets.h"
#include "NetworkBufferManagement.h"

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

/* Demo includes. */
#include "NetworkInterface.h"

#ifndef configNUM_RX_ETHERNET_DMA_DESCRIPTORS
#error configNUM_RX_ETHERNET_DMA_DESCRIPTORS must be defined in FreeRTOSConfig.h to set the number of RX DMA descriptors
#endif

#ifndef configNUM_TX_ETHERNET_DMA_DESCRIPTORS
#error configNUM_TX_ETHERNET_DMA_DESCRIPTORS must be defined in FreeRTOSConfig.h to set the number of TX DMA descriptors
#endif

/* If a packet cannot be sent immediately then the task performing the send
 operation will be held in the Blocked state (so other tasks can execute) for
 niTX_BUFFER_FREE_WAIT ticks.  It will do this a maximum of niMAX_TX_ATTEMPTS
 before giving up. */
#define niTX_BUFFER_FREE_WAIT	( ( TickType_t ) 2UL / portTICK_RATE_MS )
#define niMAX_TX_ATTEMPTS		( 5 )

static tEMACDMADescriptor g_psRxDescriptor[configNUM_RX_ETHERNET_DMA_DESCRIPTORS];
static tEMACDMADescriptor g_psTxDescriptor[configNUM_TX_ETHERNET_DMA_DESCRIPTORS];
static uint32_t g_ui32RxDescIndex;
static uint32_t g_ui32TxDescIndex;

#define RX_BUFFER_SIZE 1536
static uint8_t g_ppui8RxBuffer[configNUM_RX_ETHERNET_DMA_DESCRIPTORS][RX_BUFFER_SIZE];

/*-----------------------------------------------------------*/

/*
 * A deferred interrupt handler task that processes received frames.
 */
static void prvEMACDeferredInterruptHandlerTask(void *pvParameters);

/*-----------------------------------------------------------*/

/* The queue used to communicate Ethernet events to the IP task. */
extern xQueueHandle xNetworkEventQueue;

/* The semaphore used to wake the deferred interrupt handler task when an Rx
 interrupt is received. */
static xSemaphoreHandle xEMACRxEventSemaphore = NULL;

/*-----------------------------------------------------------*/

//*****************************************************************************
//
// Initialize the transmit and receive DMA descriptors.
//
//*****************************************************************************
void InitDescriptors(uint32_t ui32Base)
{
	uint32_t ui32Loop;

	/*
	 * Initialize each of the transmit descriptors. Note that we leave the
	 * buffer pointer and size empty and the OWN bit clear here since we have
	 * not set up any transmissions yet.
	 */
	for (ui32Loop = 0; ui32Loop < configNUM_TX_ETHERNET_DMA_DESCRIPTORS; ui32Loop++)
	{
		g_psTxDescriptor[ui32Loop].ui32Count = DES1_TX_CTRL_SADDR_INSERT;
		g_psTxDescriptor[ui32Loop].DES3.pLink =
				(ui32Loop == (configNUM_TX_ETHERNET_DMA_DESCRIPTORS - 1)) ?
						g_psTxDescriptor : &g_psTxDescriptor[ui32Loop + 1];
		g_psTxDescriptor[ui32Loop].ui32CtrlStatus = (DES0_TX_CTRL_LAST_SEG | DES0_TX_CTRL_FIRST_SEG |
		DES0_TX_CTRL_INTERRUPT | DES0_TX_CTRL_CHAINED |
		DES0_TX_CTRL_IP_ALL_CKHSUMS);
	}
	/*
	 * Initialize each of the receive descriptors. We clear the OWN bit here
	 * to make sure that the receiver doesn’t start writing anything
	 * immediately.
	 */
	for (ui32Loop = 0; ui32Loop < configNUM_RX_ETHERNET_DMA_DESCRIPTORS; ui32Loop++)
	{
		g_psRxDescriptor[ui32Loop].ui32CtrlStatus = 0;
		g_psRxDescriptor[ui32Loop].ui32Count = (DES1_RX_CTRL_CHAINED | (RX_BUFFER_SIZE << DES1_RX_CTRL_BUFF1_SIZE_S));
		g_psRxDescriptor[ui32Loop].pvBuffer1 = g_ppui8RxBuffer[ui32Loop];
		g_psRxDescriptor[ui32Loop].DES3.pLink =
				(ui32Loop == (configNUM_RX_ETHERNET_DMA_DESCRIPTORS - 1)) ?
						g_psRxDescriptor : &g_psRxDescriptor[ui32Loop + 1];
	}

	/* Set the descriptor pointers in the hardware. */
	EMACRxDMADescriptorListSet(ui32Base, g_psRxDescriptor);
	EMACTxDMADescriptorListSet(ui32Base, g_psTxDescriptor);

	/*
	 * Start from the beginning of both descriptor chains. We actually set
	 * the transmit descriptor index to the last descriptor in the chain
	 * since it will be incremented before use and this means the first
	 * transmission we perform will use the correct descriptor.
	 */
	g_ui32RxDescIndex = 0;
	g_ui32TxDescIndex = configNUM_TX_ETHERNET_DMA_DESCRIPTORS - 1;
}

/*-----------------------------------------------------------*/

BaseType_t xNetworkInterfaceInitialise(void)
{
	uint32_t ui32User0, ui32User1, ui32Loop;
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

	//
	// Initialize the Ethernet DMA descriptors.
	//
	InitDescriptors(EMAC0_BASE);

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
	for (ui32Loop = 0; ui32Loop < configNUM_RX_ETHERNET_DMA_DESCRIPTORS; ui32Loop++)
	{
		g_psRxDescriptor[ui32Loop].ui32CtrlStatus |= DES0_RX_CTRL_OWN;
	}

	/* Create the event semaphore if it has not already been created. */
	if (xEMACRxEventSemaphore == NULL)
	{
		vSemaphoreCreateBinary(xEMACRxEventSemaphore);
#if ipconfigINCLUDE_EXAMPLE_FREERTOS_PLUS_TRACE_CALLS == 1
		{
			/* If the trace recorder code is included name the semaphore for
			 viewing in FreeRTOS+Trace. */
			vTraceSetQueueName( xEMACRxEventSemaphore, "MAC_RX" );
		}
#endif /*  ipconfigINCLUDE_EXAMPLE_FREERTOS_PLUS_TRACE_CALLS == 1 */
	}

	configASSERT( xEMACRxEventSemaphore );

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

	IntPrioritySet(INT_EMAC0_TM4C129, configMAC_INTERRUPT_PRIORITY);
	//
	// Enable the Ethernet interrupt.
	//
	IntEnable(INT_EMAC0_TM4C129);

	//
	// Enable the Ethernet RX Packet interrupt source.
	//
	EMACIntEnable(EMAC0_BASE, EMAC_INT_RECEIVE);

	return pdPASS;

}
/*-----------------------------------------------------------*/

BaseType_t xNetworkInterfaceOutput(xNetworkBufferDescriptor_t * const pxNetworkBuffer)
{
	BaseType_t xReturn = pdFAIL;
	int32_t x;

	/* Attempt to obtain access to a Tx descriptor. */
	for (x = 0; x < niMAX_TX_ATTEMPTS; x++)
	{
		if ((g_psTxDescriptor[g_ui32TxDescIndex].ui32CtrlStatus & DES0_TX_CTRL_OWN) != 0)
		{

			/* Move to the next descriptor. */
			g_ui32TxDescIndex++;
			if (g_ui32TxDescIndex == configNUM_TX_ETHERNET_DMA_DESCRIPTORS)
			{
				g_ui32TxDescIndex = 0;
			}

			/* Fill in the packet size and pointer, and tell the transmitter to start work. */
			g_psTxDescriptor[g_ui32TxDescIndex].ui32Count = (uint32_t) pxNetworkBuffer->xDataLength;
			g_psTxDescriptor[g_ui32TxDescIndex].pvBuffer1 = pxNetworkBuffer->pucEthernetBuffer;
			g_psTxDescriptor[g_ui32TxDescIndex].ui32CtrlStatus = (DES0_TX_CTRL_LAST_SEG | DES0_TX_CTRL_FIRST_SEG |
			DES0_TX_CTRL_INTERRUPT | DES0_TX_CTRL_IP_ALL_CKHSUMS |
			DES0_TX_CTRL_CHAINED | DES0_TX_CTRL_OWN);

			/*
			 * Tell the DMA to reacquire the descriptor now that we’ve filled it in.
			 * This call is benign if the transmitter hasn’t stalled and checking
			 * the state takes longer than just issuing a poll demand so we do this
			 * for all packets.
			 */
			EMACTxDMAPollDemand(EMAC0_BASE);

			/* The Tx has been initiated. */
			return pdPASS;
		}
		else
		{
			iptraceWAITING_FOR_TX_DMA_DESCRIPTOR();
			vTaskDelay( niTX_BUFFER_FREE_WAIT);
		}
	}

	/* Finished with the network buffer. */
	vNetworkBufferRelease(pxNetworkBuffer);

	return xReturn;
}
/*-----------------------------------------------------------*/

void ETH_IRQHandler(void)
{
	uint32_t ulInterruptCause;

	UARTprintf("ETH interrupt\n");

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
void EMAC_NextPacketToRead( xNetworkBufferDescriptor_t *pxNetworkBuffer )
{
uint8_t *pucTemp;

	UARTprintf("EMAC_NextPacketToRead\n");

	/* Swap the buffer in the network buffer with the buffer used by the DMA.
	This allows the data to be passed out without having to perform any copies. */
	pucTemp = ( uint8_t * ) g_psRxDescriptor[ g_ui32RxDescIndex ].pvBuffer1;
	g_psRxDescriptor[ g_ui32RxDescIndex ].pvBuffer1 = pxNetworkBuffer->pucEthernetBuffer;
	pxNetworkBuffer->pucEthernetBuffer = pucTemp;

	/* Only supports frames coming in single buffers.  If this frame is split
	across multiple buffers then reject it (and if the frame is needed increase
	the ipconfigNETWORK_MTU setting). */
	/*if( ( g_psRxDescriptor[ g_psRxDescriptor ].ui32CtrlStatus & emacEXPECTED_RX_STATUS_MASK ) != emacEXPECTED_RX_STATUS_MASK )
	{
		pxNetworkBuffer->xDataLength = 0;
	}
	else
	{
		pxNetworkBuffer->xDataLength = ( size_t ) EMAC_GetReceiveDataSize() - ( ipETHERNET_CRC_BYTES - 1U );;
	}*/
}
/*-----------------------------------------------------------*/

static void prvEMACDeferredInterruptHandlerTask(void *pvParameters)
{
	xNetworkBufferDescriptor_t *pxNetworkBuffer;
	xIPStackEvent_t xRxEvent =
	{ eEthernetRxEvent, NULL };

	(void) pvParameters;
	configASSERT( xEMACRxEventSemaphore );

	for (;;)
	{
		/* Wait for the EMAC interrupt to indicate that another packet has been
		 received.  The while() loop is only needed if INCLUDE_vTaskSuspend is
		 set to 0 in FreeRTOSConfig.h.  If INCLUDE_vTaskSuspend is set to 1
		 then portMAX_DELAY would be an indefinite block time and
		 xSemaphoreTake() would only return when the semaphore was actually
		 obtained. */
		while ( xSemaphoreTake(xEMACRxEventSemaphore, portMAX_DELAY) == pdFALSE)
			;

		/* At least one packet has been received. */
		while (!(g_psRxDescriptor[g_ui32RxDescIndex].ui32CtrlStatus & DES0_RX_CTRL_OWN))
		{
			/* The buffer filled by the DMA is going to be passed into the IP
			 stack.  Allocate another buffer for the DMA descriptor. */
			pxNetworkBuffer = pxNetworkBufferGet( ipTOTAL_ETHERNET_FRAME_SIZE, (TickType_t) 0);

			if (pxNetworkBuffer != NULL)
			{
				/* Swap the buffer just allocated and referenced from the
				 pxNetworkBuffer with the buffer that has already been filled by
				 the DMA.  pxNetworkBuffer will then hold a reference to the
				 buffer that already contains the data without any data having
				 been copied between buffers. */
				EMAC_NextPacketToRead(pxNetworkBuffer);

#if ipconfigETHERNET_DRIVER_FILTERS_FRAME_TYPES == 1
				{
					if (pxNetworkBuffer->xDataLength > 0)
					{
						/* If the frame would not be processed by the IP stack then
						 don't even bother sending it to the IP stack. */
						if (eConsiderFrameForProcessing(pxNetworkBuffer->pucEthernetBuffer) != eProcessBuffer)
						{
							pxNetworkBuffer->xDataLength = 0;
						}
					}
				}
#endif

				if (pxNetworkBuffer->xDataLength > 0)
				{
					/* Store a pointer to the network buffer structure in the
					 padding	space that was left in front of the Ethernet frame.
					 The pointer	is needed to ensure the network buffer structure
					 can be located when it is time for it to be freed if the
					 Ethernet frame gets	used as a zero copy buffer. */
					*((xNetworkBufferDescriptor_t **) ((pxNetworkBuffer->pucEthernetBuffer - ipBUFFER_PADDING))) =
							pxNetworkBuffer;

					/* Data was received and stored.  Send it to the IP task
					 for processing. */
					xRxEvent.pvData = (void *) pxNetworkBuffer;
					if ( xQueueSendToBack(xNetworkEventQueue, &xRxEvent, (TickType_t ) 0) == pdFALSE)
					{
						/* The buffer could not be sent to the IP task so the
						 buffer must be released. */
						vNetworkBufferRelease(pxNetworkBuffer);
						iptraceETHERNET_RX_EVENT_LOST();
					}
					else
					{
						iptraceNETWORK_INTERFACE_RECEIVE();
					}
				}
				else
				{
					/* The buffer does not contain any data so there is no
					 point sending it to the IP task.  Just release it. */
					vNetworkBufferRelease(pxNetworkBuffer);
					iptraceETHERNET_RX_EVENT_LOST();
				}
			}
			else
			{
				iptraceETHERNET_RX_EVENT_LOST();
			}

			/*
			 * Now that we are finished dealing with this descriptor, hand
			 * it back to the hardware. Note that we assume
			 * ApplicationProcessFrame() is finished with the buffer at this point
			 * so it is safe to reuse.
			 */
			g_psRxDescriptor[g_ui32RxDescIndex].ui32CtrlStatus = DES0_RX_CTRL_OWN;

			/* Move on to the next descriptor in the chain. */
			g_ui32RxDescIndex++;

			if (g_ui32RxDescIndex == configNUM_RX_ETHERNET_DMA_DESCRIPTORS)
			{
				g_ui32RxDescIndex = 0;
			}
		}
	}
}
/*-----------------------------------------------------------*/

