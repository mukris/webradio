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
#include "FreeRTOS_IP.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "shoutcast.h"

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

/* The size of the stack and the priority used by the two echo client tasks. */
#define mainECHO_CLIENT_TASK_STACK_SIZE 	( configMINIMAL_STACK_SIZE * 2 )
#define mainECHO_CLIENT_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )

void PinoutSet(void)
{
	// Enable all the GPIO peripherals.
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);

	//
	// PA0-1 are used for UART0.
	//
	MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
	MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
	MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	//
	// PD0,1,3 are used for SPI.
	//
	MAP_GPIOPinConfigure(GPIO_PD0_SSI2XDAT1);
	MAP_GPIOPinConfigure(GPIO_PD1_SSI2XDAT0);
	MAP_GPIOPinConfigure(GPIO_PD3_SSI2CLK);
	GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_INT_PIN_3);
	GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_6);

	//
	// this app wants to configure for ethernet LED function.
	//
	MAP_GPIOPinConfigure(GPIO_PF0_EN0LED0);
	MAP_GPIOPinConfigure(GPIO_PF4_EN0LED1);

	GPIOPinTypeEthernetLED(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);

	//
	// PJ0 and J1 are used for user buttons
	//
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	MAP_GPIOPinWrite(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1, 0);

	//
	// PN0 and PN1 are used for USER LEDs.
	//
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	MAP_GPIOPadConfigSet(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1,
	GPIO_STRENGTH_4MA,
							GPIO_PIN_TYPE_STD);

	//
	// Default the LEDs to OFF.
	//
	MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1, 0);
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

	//IntPriorityGroupingSet(4);
	//IntPrioritySet(INT_EMAC0_TM4C129, configMAC_INTERRUPT_PRIORITY);
	IntPrioritySet(INT_UART0, configUART_INTERRUPT_PRIORITY);

	vStartRfidTask();

	UARTprintf("--- Starting OS ---\n\n");

	vTaskStartScheduler();

	for (;;)
		;
}

/*
 * The following function should be provided by the user and return true if it
 * matches the domain name.
 */
BaseType_t xApplicationDNSQueryHook(const char *pcName)
{
	if (memcmp(pcName, "tivac", 5))
	{
		return pdTRUE;
	}
	else
	{
		return pdFALSE;
	}
}

/*
 * With this option, the hostname can be registered as well which makes
 * it easier to lookup a device in a router's list of DHCP clients.
 */
const char *pcApplicationHostnameHook(void)
{
	return "ConnectedLaunchpad";
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
	//char radio[] = "http://185.33.23.5:80";
	//char radio[] = "http://50.30.37.166:42000";
	//char radio[] = "http://204.62.13.214:80/musicone/mp3/128k";
	char radio[] = "http://listen.radionomy.com/PARTYVIBERADIO-Reggae-Roots-Dancehall-Dub";

	if (eNetworkEvent == eNetworkUp)
	{
		if (xTaskAlreadyCreated == pdFALSE)
		{
			UARTprintf("Starting Shoutcast task\n");
			vStartShoutcastReceiver();

			xQueueSendToBack(radioChannelQueue, radio, 0);

			xTaskAlreadyCreated = pdTRUE;
		}
	}
}
/*-----------------------------------------------------------*/

#define vOutputString(msg) UARTprintf(msg)
/* Called by FreeRTOS+UDP when a reply is received to an outgoing ping request. */
void vApplicationPingReplyHook(ePingReplyStatus_t eStatus, uint16_t usIdentifier)
{
	static const char *pcSuccess = "\r\n\r\nPing reply received - ";
	static const char *pcInvalidChecksum = "\r\n\r\nPing reply received with invalid checksum - ";
	static const char *pcInvalidData = "\r\n\r\nPing reply received with invalid data - ";
	static char cMessage[50];

	switch (eStatus)
	{
	case eSuccess:
		vOutputString(pcSuccess);
		break;

	case eInvalidChecksum:
		vOutputString(pcInvalidChecksum);
		break;

	case eInvalidData:
		vOutputString(pcInvalidData);
		break;

	default:
		// It is not possible to get here as all enums have their own case.
		break;
	}

	sprintf(cMessage, "identifier %d\r\n\r\n", (int) usIdentifier);
	vOutputString(cMessage);
}
