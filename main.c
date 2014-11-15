#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "FreeRTOS.h"
#include "FreeRTOS_UDP_IP.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// System Clock rate in Hertz.
uint32_t g_ui32SysClock;

/* The default IP and MAC address used by the demo.  The address configuration
 defined here will be used if ipconfigUSE_DHCP is 0, or if ipconfigUSE_DHCP is
 1 but a DHCP server could not be contacted.  See the online documentation for
 more information. */
static const uint8_t ucIPAddress[4] =
{ configIP_ADDR0, configIP_ADDR1, configIP_ADDR2, configIP_ADDR3 };

static const uint8_t ucNetMask[4] =
{ configNET_MASK0, configNET_MASK1, configNET_MASK2, configNET_MASK3 };

static const uint8_t ucGatewayAddress[4] =
{ configGATEWAY_ADDR0, configGATEWAY_ADDR1, configGATEWAY_ADDR2, configGATEWAY_ADDR3 };

static const uint8_t ucDNSServerAddress[4] =
{ configDNS_SERVER_ADDR0, configDNS_SERVER_ADDR1, configDNS_SERVER_ADDR2, configDNS_SERVER_ADDR3 };

/* The MAC address used by the demo.  In production units the MAC address would
 probably be read from flash memory or an EEPROM.  Here it is just hard coded. */
const uint8_t ucMACAddress[6] =
{ configMAC_ADDR0, configMAC_ADDR1, configMAC_ADDR2, configMAC_ADDR3, configMAC_ADDR4, configMAC_ADDR5 };

void PinoutSet(void)
{
	// Enable all the GPIO peripherals.
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);

	//
	// PA0-1 are used for UART0.
	//
	ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
	ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
	ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	//
	// this app wants to configure for ethernet LED function.
	//
	ROM_GPIOPinConfigure(GPIO_PF0_EN0LED0);
	ROM_GPIOPinConfigure(GPIO_PF4_EN0LED1);

	GPIOPinTypeEthernetLED(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);

	//
	// PJ0 and J1 are used for user buttons
	//
	ROM_GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	ROM_GPIOPinWrite(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1, 0);

	//
	// PN0 and PN1 are used for USER LEDs.
	//
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	MAP_GPIOPadConfigSet(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1,
	GPIO_STRENGTH_12MA,
							GPIO_PIN_TYPE_STD);

	//
	// Default the LEDs to OFF.
	//
	ROM_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1, 0);
}

int main(void)
{
	//
	// Make sure the main oscillator is enabled because this is required by
	// the PHY.  The system must have a 25MHz crystal attached to the OSC
	// pins.  The SYSCTL_MOSC_HIGHFREQ parameter is used when the crystal
	// frequency is 10MHz or higher.
	//
	SysCtlMOSCConfigSet(SYSCTL_MOSC_HIGHFREQ);

	// Set the clocking to run at 120 MHz from the PLL.
	g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),
											120000000);

	PinoutSet();

	UARTStdioConfig(0, 115200, g_ui32SysClock);

	FreeRTOS_IPInit(ucIPAddress, ucNetMask, ucGatewayAddress, ucDNSServerAddress, ucMACAddress);

	// Configure SysTick for a periodic interrupt at 10ms.
	SysTickPeriodSet((g_ui32SysClock / 1000) * 10);
	SysTickEnable();
	SysTickIntEnable();

	IntMasterEnable();

	IntPriorityGroupingSet(4);
	IntPrioritySet(INT_EMAC0_TM4C129, configMAC_INTERRUPT_PRIORITY);

	UARTprintf("--- Starting OS ---\n\n");

	vTaskStartScheduler();

	for (;;)
		;
}

//*****************************************************************************
//
// This hook is called by FreeRTOS when an stack overflow error is detected.
//
//*****************************************************************************
void vApplicationStackOverflowHook(xTaskHandle *pxTask, char *pcTaskName)
{
	(void) pcTaskName;
	(void) pxTask;

	/* Run time stack overflow checking is performed if
	 configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	 function is called if a stack overflow is detected. */
	UARTprintf("ERROR: ApplicationStackOverflow\n");
	taskDISABLE_INTERRUPTS();

	for (;;)
		;
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook(void)
{
	/* vApplicationMallocFailedHook() will only be called if
	 configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	 function that will get called if a call to pvPortMalloc() fails.
	 pvPortMalloc() is called internally by the kernel whenever a task, queue,
	 timer or semaphore is created.  It is also called by various parts of the
	 demo application.  If heap_1.c, heap_2.c or heap_4.c are used, then the
	 size of the heap available to pvPortMalloc() is defined by
	 configTOTAL_HEAP_SIZE in FreeRTOSConfig.h, and the xPortGetFreeHeapSize()
	 API function can be used to query the size of free heap space that remains
	 (although it does not provide information on how the remaining heap might
	 be fragmented). */
	UARTprintf("ERROR: ApplicationMallocFailed\n");
	taskDISABLE_INTERRUPTS();

	for (;;)
		;
}
/*-----------------------------------------------------------*/

/* Called by FreeRTOS+UDP when the network connects. */
void vApplicationIPNetworkEventHook(eIPCallbackEvent_t eNetworkEvent)
{
	static BaseType_t xTaskAlreadyCreated = pdFALSE;

	if (eNetworkEvent == eNetworkUp)
	{
		/* Create the tasks that transmit to and receive from a standard
		 echo server (see the web documentation for this port) in both
		 standard and zero copy mode. */
		if (xTaskAlreadyCreated == pdFALSE)
		{
			//vStartEchoClientTasks( mainECHO_CLIENT_TASK_STACK_SIZE, mainECHO_CLIENT_TASK_PRIORITY );
			xTaskAlreadyCreated = pdTRUE;
		}
	}
}
/*-----------------------------------------------------------*/

/* Called by FreeRTOS+UDP when a reply is received to an outgoing ping request. */
void vApplicationPingReplyHook(ePingReplyStatus_t eStatus, uint16_t usIdentifier)
{
	/*static const char *pcSuccess = "\r\n\r\nPing reply received - ";
	 static const char *pcInvalidChecksum = "\r\n\r\nPing reply received with invalid checksum - ";
	 static const char *pcInvalidData = "\r\n\r\nPing reply received with invalid data - ";
	 static char cMessage[ 50 ];
	 void vOutputString( const char * const pcMessage );

	 switch( eStatus )
	 {
	 case eSuccess	:
	 vOutputString( pcSuccess );
	 break;

	 case eInvalidChecksum :
	 vOutputString( pcInvalidChecksum );
	 break;

	 case eInvalidData :
	 vOutputString( pcInvalidData );
	 break;

	 default :
	 // It is not possible to get here as all enums have their own case.
	 break;
	 }

	 sprintf( cMessage, "identifier %d\r\n\r\n", ( int ) usIdentifier );
	 vOutputString( cMessage);
	 */
}
