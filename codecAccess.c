#if 0
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "inc/hw_memmap.h"
#include "driverlib/ssi.h"
#include "driverlib/udma.h"
#include "driverlib/gpio.h"

#define STACK_SIZE ( configMINIMAL_STACK_SIZE * 3 )
#define delay(ms) vTaskDelay((TickType_t) (ms) / portTICK_RATE_MS)
#define MP3_SAMPLE_RATE 44100 //TODO

#define NEW_PACK_INST 0
#define VOL_SET_INST 1

void SSIInit(){
	SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
			SSI_MODE_MASTER, 4000000, 8);
	SSIDMAEnable(SSI0_BASE, SSI_DMA_TX);
	SSIEnable(SSI0_BASE);
}

void DMAInit(){
	/*uDMAEnable();
	uDMAControlBaseSet();
	uDMAChannelAttributeEnable();
	uDMAChannelControlSet();
	uDMAChannelTransferSet();
	uDMAChannelEnable();
	uDMAChannelRequest();*/

	SSIDisable(SSI0_BASE);

	uint8_t pui8DMAControlTable[1024];
	uint8_t pui8SourceBuffer[256];
	uint8_t pui8DestBuffer[256];
	uDMAEnable();
	uDMAControlBaseSet(&pui8DMAControlTable[0]);
//uDMAChannelAttributeDisable(UDMA_CHANNEL_SW, UDMA_CONFIG_ALL);
	uDMAChannelControlSet(UDMA_CHANNEL_SW | UDMA_PRI_SELECT,
	UDMA_SIZE_8 | UDMA_SRC_INC_8 |
	UDMA_DST_INC_8 | UDMA_ARB_8);
	uDMAChannelTransferSet(UDMA_CHANNEL_SW | UDMA_PRI_SELECT,
	UDMA_MODE_AUTO, pui8SourceBuffer, pui8DestBuffer,
	sizeof(pui8DestBuffer));
	uDMAChannelEnable(UDMA_CHANNEL_SW);
	uDMAChannelRequest(UDMA_CHANNEL_SW);

	SSIEnable(SSI0_BASE);
}

void codecInstSend(uint8_t addr,uint16_t data){
	//ui32Port is the base address of the GPIO port.
	//ui8Pins is the bit-packed representation of the pin(s).
	while(GPIOPinRead(GPIO_PORTK_BASE,GPIO_PIN_2)==0){
		delay(10);
	}
	GPIOPinWrite(GPIO_PORTK_BASE,GPIO_PIN_3,0);
	SSISend((uint8_t)2);
	SSISend(addr);
	SSISend((uint8_t)(data>>8));
	while(GPIOPinRead(GPIO_PORTK_BASE,GPIO_PIN_2)!=0){
			delay(10);
		}
	while(GPIOPinRead(GPIO_PORTK_BASE,GPIO_PIN_2)==0){
			delay(10);
	SSISend((uint8_t)data);
	while(GPIOPinRead(GPIO_PORTK_BASE,GPIO_PIN_2)!=0){
		delay(10);
	}
	GPIOPinWrite(GPIO_PORTK_BASE,GPIO_PIN_3,0xFF);
	while(GPIOPinRead(GPIO_PORTK_BASE,GPIO_PIN_2)==0){
		delay(10);
	}
}

void codecInit(){
	GPIODirModeSet(GPIO_PORTK_BASE,GPIO_PIN_3,GPIO_DIR_MODE_OUT);
	codecInstSend(addr,data);//TODO
	codecInstSend(addr,data);
	codecInstSend(addr,data);
}

void volumeSet(int newVolume){

}

struct codecInst
{
	char instType;
	int data;
} codecInstBuffer;

//
// codecAccess task
//
void codecAccess(){
	DMAInit();
	int i = 0;
	QueueHandle_t codecQueue = xQueueCreate(10, sizeof(struct codecInst *));
	struct codecInst *codecInstBuffer;
	while(1){;
		for(i=uxQueueMessagesWaiting(codecQueue);i>0;i--){
			xQueueReceive(codecQueue,&codecInstBuffer,0);
			switch(codecInstBuffer->instType){
			case NEW_PACK_INST:
				//TODO
				break;
			case VOL_SET_INST:
				//TODO
			}
		}
		delay(50);
	}
}

void vStartCodecAccessTask(void)
{
	SSIInit();
	codecInit();
	//QueueHandle_t codecQueue = xQueueCreate(10, sizeof(struct codecInst *));
	xTaskCreate(codecAccess, /* The function that implements the task. */
				"codecAccess", /* Just a text name for the task to aid debugging. */
				STACK_SIZE, /* The stack size is defined in FreeRTOSIPConfig.h. */
				NULL, /* The task parameter, not used in this case. */
				tskIDLE_PRIORITY + 2, /* The priority assigned to the task is defined in FreeRTOSConfig.h. */
				NULL); /* The task handle is not used. */
}
#endif



