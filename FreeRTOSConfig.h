/*
    FreeRTOS V7.0.2 - Copyright (C) 2011 Real Time Engineers Ltd.


    ***************************************************************************
     *                                                                       *
     *    FreeRTOS tutorial books are available in pdf and paperback.        *
     *    Complete, revised, and edited pdf reference manuals are also       *
     *    available.                                                         *
     *                                                                       *
     *    Purchasing FreeRTOS documentation will not only help you, by       *
     *    ensuring you get running as quickly as possible and with an        *
     *    in-depth knowledge of how to use FreeRTOS, it will also help       *
     *    the FreeRTOS project to continue with its mission of providing     *
     *    professional grade, cross platform, de facto standard solutions    *
     *    for microcontrollers - completely free of charge!                  *
     *                                                                       *
     *    >>> See http://www.FreeRTOS.org/Documentation for details. <<<     *
     *                                                                       *
     *    Thank you for using FreeRTOS, and thank you for your support!      *
     *                                                                       *
    ***************************************************************************


    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    >>>NOTE<<< The modification to the GPL is included to allow you to
    distribute a combined work that includes FreeRTOS without being obliged to
    provide the source code for proprietary components outside of the FreeRTOS
    kernel.  FreeRTOS is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
    or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public
    License and the FreeRTOS license exception along with FreeRTOS; if not it
    can be viewed here: http://www.freertos.org/a00114.html and also obtained
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!

    http://www.FreeRTOS.org - Documentation, latest information, license and
    contact details.

    http://www.SafeRTOS.com - A version that is certified for use in safety
    critical systems.

    http://www.OpenRTOS.com - Commercial support, development, porting,
    licensing and training services.
*/

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
 *
 * See http://www.freertos.org/a00110.html.
 *----------------------------------------------------------*/

#define configUSE_PREEMPTION                1
#define configUSE_IDLE_HOOK                 0
#define configUSE_TICK_HOOK                 0
#define configCPU_CLOCK_HZ                  ( ( unsigned long ) 50000000 )
#define configTICK_RATE_HZ                  ( ( portTickType ) 1000 )
#define configMINIMAL_STACK_SIZE            ( ( unsigned short ) 200 )
#define configTOTAL_HEAP_SIZE               ( ( size_t ) ( 30000 ) )
#define configMAX_TASK_NAME_LEN             ( 12 )
#define configUSE_TRACE_FACILITY            1
#define configUSE_16_BIT_TICKS              0
#define configIDLE_SHOULD_YIELD             0
#define configUSE_CO_ROUTINES               0
#define configUSE_MUTEXES                   1
#define configUSE_RECURSIVE_MUTEXES         1
#define configCHECK_FOR_STACK_OVERFLOW      2

#define configMAX_PRIORITIES                ( 16 )
#define configMAX_CO_ROUTINE_PRIORITIES     ( 2 )
#define configQUEUE_REGISTRY_SIZE           10

/* Software timer related definitions. */
#define configUSE_TIMERS				1
#define configTIMER_TASK_PRIORITY		( configMAX_PRIORITIES - 1 )
#define configTIMER_QUEUE_LENGTH		5
#define configTIMER_TASK_STACK_DEPTH	configMINIMAL_STACK_SIZE

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */

#define INCLUDE_vTaskPrioritySet            1
#define INCLUDE_uxTaskPriorityGet           1
#define INCLUDE_vTaskDelete                 1
#define INCLUDE_vTaskCleanUpResources       0
#define INCLUDE_vTaskSuspend                1
#define INCLUDE_vTaskDelayUntil             1
#define INCLUDE_vTaskDelay                  1
#define INCLUDE_uxTaskGetStackHighWaterMark 1

/* Be ENORMOUSLY careful if you want to modify these two values and make sure
 * you read http://www.freertos.org/a00110.html#kernel_priority first!
 */
#define configKERNEL_INTERRUPT_PRIORITY         ( 7 << 5 )    /* Priority 7, or 0xE0 as only the top three bits are implemented.  This is the lowest priority. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    ( 5 << 5 )  /* Priority 5, or 0xA0 as only the top three bits are implemented. */
#define configMAC_INTERRUPT_PRIORITY            ( 4 << 5 )

/* The LPC1830 Ethernet peripheral uses a DMA to transmit and receive packets.
The DMA uses a chain of descriptors to reference Ethernet buffers, and provide
information on the state of each buffer (full/empty/error/etc.).
configNUM_RX_ETHERNET_DMA_DESCRIPTORS defines the total number of receive
descriptors (descriptors that point to buffers into which the DMA will write
packets received from the network).  An Ethernet buffer is assigned to each
descriptor.  Having too few descriptors will impact reliability because the DMA
will have to drop packets that are received when there are no receive
descriptors free.  It is however only necessary to have a couple of free
descriptors at a time, and having more wastes the RAM used by the Ethernet
buffers that are surplus to requirements. */
#define configNUM_RX_ETHERNET_DMA_DESCRIPTORS	4

/* The LPC1830 Ethernet peripheral uses a DMA to transmit and receive packets.
The DMA uses a chain of descriptors to reference Ethernet buffers that are
waiting to be sent onto the network.  configNUM_TX_ETHERNET_DMA_DESCRIPTORS
defines the total number of transmit descriptors.  An Ethernet buffer is
not assigned to a transmit descriptor until data is actually sent, but will
remain assigned to the descriptor until the descriptor is re-used.  It is not
necessary to have many transmit descriptors as the IP stack task will be held
in the Blocked state (so other tasks can run) until a descriptor becomes
available if it attempts to transmit when all the descriptors are in use.  See
the iptraceWAITING_FOR_TX_DMA_DESCRIPTOR() IP trace macro. */
#define configNUM_TX_ETHERNET_DMA_DESCRIPTORS	1

/* The address of an echo server that will be used by the two demo echo client
tasks.
http://FreeRTOS.org/FreeRTOS-Plus/FreeRTOS_Plus_UDP/Embedded_Ethernet_Examples/Common_Echo_Clients.shtml */
#define configECHO_SERVER_ADDR0	172
#define configECHO_SERVER_ADDR1 25
#define configECHO_SERVER_ADDR2 218
#define configECHO_SERVER_ADDR3 103

/* MAC address configuration.  In a deployed production system this would
probably be read from an EEPROM.  In the demo it is just hard coded.  Make sure
each node on the network has a unique MAC address. */
#define configMAC_ADDR0	0x00
#define configMAC_ADDR1	0x1A
#define configMAC_ADDR2	0xB6
#define configMAC_ADDR3	0x02
#define configMAC_ADDR4	0xA9
#define configMAC_ADDR5	0x10

/* Default IP address configuration.  Used in ipconfigUSE_DNS is set to 0, or
ipconfigUSE_DNS is set to 1 but a DNS server cannot be contacted. */
#define configIP_ADDR0		172
#define configIP_ADDR1		25
#define configIP_ADDR2		218
#define configIP_ADDR3		200

/* Default gateway IP address configuration.  Used in ipconfigUSE_DNS is set to
0, or ipconfigUSE_DNS is set to 1 but a DNS server cannot be contacted. */
#define configGATEWAY_ADDR0	172
#define configGATEWAY_ADDR1	25
#define configGATEWAY_ADDR2	218
#define configGATEWAY_ADDR3	1

/* Default DNS server configuration.  OpenDNS addresses are 208.67.222.222 and
208.67.220.220.  Used in ipconfigUSE_DNS is set to 0, or ipconfigUSE_DNS is set
to 1 but a DNS server cannot be contacted.*/
#define configDNS_SERVER_ADDR0 	208
#define configDNS_SERVER_ADDR1 	67
#define configDNS_SERVER_ADDR2 	222
#define configDNS_SERVER_ADDR3 	222

/* Defalt netmask configuration.  Used in ipconfigUSE_DNS is set to 0, or
ipconfigUSE_DNS is set to 1 but a DNS server cannot be contacted. */
#define configNET_MASK0		255
#define configNET_MASK1		255
#define configNET_MASK2		255
#define configNET_MASK3		0

#endif /* FREERTOS_CONFIG_H */
