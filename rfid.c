/* Standard includes. */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "driverlib/gpio.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "inc/hw_memmap.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "utils/MFRC552.h"
#include "utils/uartstdio.h"
#include "rfid.h"

#define STACK_SIZE ( configMINIMAL_STACK_SIZE * 3 )
#define delay(ms) vTaskDelay((TickType_t) (ms) / portTICK_RATE_MS)

extern Uid uid;

static void prvSetupSPI(void);
static void prvRfidTask(void * parameters);

void setChipSelectLow(void)
{
	GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0);
}
void setChipSelectHigh(void)
{
	GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_PIN_6);
}

uint8_t spiTransfer(uint8_t data)
{
	uint32_t ret;
	MAP_SSIDataPut(SSI2_BASE, data);
	MAP_SSIDataGet(SSI2_BASE, &ret);
	return (uint8_t) (ret & 0xFF);
}

void vStartRfidTask(void)
{
	prvSetupSPI();

	rfidEventQueue = xQueueCreate(10, sizeof(RfidMessage));

	xTaskCreate(prvRfidTask, /* The function that implements the task. */
				"RFID", /* Just a text name for the task to aid debugging. */
				STACK_SIZE, /* The stack size is defined in FreeRTOSIPConfig.h. */
				NULL, /* The task parameter, not used in this case. */
				tskIDLE_PRIORITY + 2, /* The priority assigned to the task is defined in FreeRTOSConfig.h. */
				NULL); /* The task handle is not used. */
}

static void prvRfidTask(void * parameters)
{
	RfidMessage rfidMessage;
	MFRC522_PCD_Init();

	for (;;)
	{
		// Look for new cards, and select one if present
		if (MFRC522_PICC_IsNewCardPresent() && MFRC522_PICC_ReadCardSerial())
		{
			uint32_t i;
			rfidMessage.size = uid.size;

			for(i = 0; i < 10; i++) {
				rfidMessage.uidByte[i] = uid.uidByte[i];
			}

			xQueueSendToBack(rfidEventQueue, &rfidMessage, 10);

			for (i = 0; i < uid.size; i++)
			{
				UARTprintf("%x", uid.uidByte[i]);
			}
			UARTprintf("\n");

			MFRC522_PICC_HaltA();
		}

		delay(50);
	}
}

static void prvSetupSPI(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
	// set the SSI clock to the PIOSC clock
	SSIClockSourceSet(SSI2_BASE, SSI_CLOCK_SYSTEM);
	SSIConfigSetExpClk(SSI2_BASE, MAP_SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 1000000, 8);
	MAP_SSIEnable(SSI2_BASE);
}

