#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "inc/hw_memmap.h"
#include "driverlib/ssi.h"
#include "inc/hw_ssi.h"
#include "driverlib/udma.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "inc/hw_ints.h"

#define STACK_SIZE ( configMINIMAL_STACK_SIZE * 3 )
#define delay(ms) vTaskDelay((TickType_t) (ms) / portTICK_RATE_MS)
//#define NEW_PACK_INST 0
#define VOL_SET_INST 1
#define BITRATE_SET_INST 2

QueueHandle_t codecQueue;
QueueHandle_t codecInstQueue;
volatile uint8_t transState;//jó helyen?

void SSIInit(){
	SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
			SSI_MODE_MASTER, 4000000, 8);
	SSIEnable(SSI0_BASE);
}

void TransferInit(void* point0, uint32_t size0,void* point1, uint32_t size1){
	/*uDMAEnable();
	uDMAControlBaseSet();
	uDMAChannelAttributeEnable();
	uDMAChannelControlSet();
	uDMAChannelTransferSet();
	uDMAChannelEnable();
	uDMAChannelRequest();*/

	SSIDisable(SSI0_BASE);

	uint8_t pui8DMAControlTable[1024];
	uDMAEnable();
	uDMAControlBaseSet(&pui8DMAControlTable[0]);
	//setting primary
	uDMAChannelControlSet(UDMA_CHANNEL_SSI0TX | UDMA_PRI_SELECT,
	UDMA_SIZE_8 | UDMA_SRC_INC_8 |
	UDMA_DST_INC_8 | UDMA_ARB_4);
	uDMAChannelTransferSet(UDMA_CHANNEL_SSI0TX | UDMA_PRI_SELECT,
	UDMA_MODE_PINGPONG, point0, (SSI0_BASE + SSI_O_DR),
	size0);
	//setting alternative
	uDMAChannelControlSet(UDMA_CHANNEL_SSI0TX | UDMA_ALT_SELECT,
	UDMA_SIZE_8 | UDMA_SRC_INC_8 |
	UDMA_DST_INC_8 | UDMA_ARB_4);
	uDMAChannelTransferSet(UDMA_CHANNEL_SSI0TX | UDMA_ALT_SELECT,
	UDMA_MODE_PINGPONG, point1, (SSI0_BASE + SSI_O_DR),
	size1);
	uDMAChannelEnable(UDMA_CHANNEL_SSI0TX);

	SSIDMAEnable(SSI0_BASE, SSI_DMA_TX);
	SSIEnable(SSI0_BASE);

	//SSIIntEnable(SSI0_BASE,SSI_RXTO | SSI_RXOR);
	IntEnable(INT_SSI0);
}

//WARNINIG Spinlock
void codecInstSend(uint8_t addr,uint16_t data){
	while(GPIOPinRead(GPIO_PORTK_BASE,GPIO_PIN_2)==0){
		//delay(10);
	}
	GPIOPinWrite(GPIO_PORTK_BASE,GPIO_PIN_3,0);
	SSIDataPut((uint8_t)2);
	SSIDataPut(addr);
	SSIDataPut((uint8_t)(data>>8));
	while(GPIOPinRead(GPIO_PORTK_BASE,GPIO_PIN_2)!=0){
			//delay(10);
		}
	while(GPIOPinRead(GPIO_PORTK_BASE,GPIO_PIN_2)==0){
			//delay(10);
	}
	SSIDataPut((uint8_t)data);
	while(GPIOPinRead(GPIO_PORTK_BASE,GPIO_PIN_2)!=0){
		//delay(10);
	}
	GPIOPinWrite(GPIO_PORTK_BASE,GPIO_PIN_3,0xFF);
	while(GPIOPinRead(GPIO_PORTK_BASE,GPIO_PIN_2)==0){
		//delay(10);
	}
}

void codecInit(){
	GPIODirModeSet(GPIO_PORTK_BASE,GPIO_PIN_3,GPIO_DIR_MODE_OUT);
	codecInstSend(0x0,0x4840); //MICP (or LINE1)?
	codecInstSend(0x5,0xAC45); //44100 Hz stereo
	codecInstSend(0x8,0b11010000000000); //44100Hz 320kbit/s
	codecInstSend(0x9,0b00011 | (2047<<5)); //noCRC MP3 MPG2.5
	codecInstSend(0xB,0x8080); //volume - about half of max
}

struct codecInst
{
	char instType;
	uint32_t data;
} codecInstBuffer;

struct codecTranfer
{
	void* point;
	uint32_t size;
} codecTransferBuffer;

//
// The Interrupt
//
 void DMAInterrupt (void)
{
	 static char sel=0;
	 if(uxQueueMessagesWaitingFromISR(codecQueue)){
		 xQueueReceiveFromISR(codecQueue,&codecTransferBuffer,0);
		 if(!sel){
			 uDMAChannelTransferSet(UDMA_CHANNEL_SSI0TX | UDMA_PRI_SELECT,
			 UDMA_MODE_PINGPONG, codecTransferBuffer.point, (SSI0_BASE + SSI_O_DR),
			 codecTransferBuffer.size);
			 sel=1;
		 }else{
			 uDMAChannelTransferSet(UDMA_CHANNEL_SSI0TX | UDMA_ALT_SELECT,
			 UDMA_MODE_PINGPONG, codecTransferBuffer.point, (SSI0_BASE + SSI_O_DR),
			 codecTransferBuffer.size);
			 sel=0;
		 }
	 }else{
		 SSIDisable(SSI0_BASE);
		 transState = 1;
	 }
	 SSIIntClear(SSI0_BASE, SSI_RXTO | SSI_RXOR);//IRQ clear?
}

//
// codecAccess task
//
void codecAccess(){
	uint8_t i = 0;
	transState = 0;
	struct codecInst *codecInstBuffer;
	/*while(uxQueueMessagesWaiting(codecQueue)<2){
		delay(100);
	}*/
	{
		void* p0;
		uint32_t s0;
		xQueueReceive(codecQueue,&codecTransferBuffer,10000);
		p0 = codecTransferBuffer.point;
		s0 = codecTransferBuffer.size;
		xQueueReceive(codecQueue,&codecTransferBuffer,10000);
		TransferInit(p0,s0,codecTransferBuffer.point,codecTransferBuffer.size);
	}
	while(1){
		for(i=uxQueueMessagesWaiting(codecInstQueue);i>0;i--){
			xQueueReceive(codecInstQueue,&codecInstBuffer,0);
			switch(codecInstBuffer->instType){
			case VOL_SET_INST:
				uDMAChannelDisable(UDMA_CHANNEL_SSI0TX);
				codecInstSend(0xB,codecInstBuffer->data);
				uDMAChannelEnable(UDMA_CHANNEL_SSI0TX);
			break;
			case BITRATE_SET_INST:
				//TODO
			}
		}
		if(transState!=0 && uxQueueMessagesWaiting(codecQueue)>0){
			SSIEnable(SSI0_BASE);
			IntTrigger(INT_SSI0);
		}
		delay(100);
	}
}

void vStartCodecAccessTask(void)
{
	SSIInit();
	codecInit();
	QueueHandle_t codecInstQueue = xQueueCreate(10, sizeof(struct codecInst *));
	QueueHandle_t codecQueue = xQueueCreate(10, sizeof(char*));
	xTaskCreate(codecAccess, /* The function that implements the task. */
				"codecAccess", /* Just a text name for the task to aid debugging. */
				STACK_SIZE, /* The stack size is defined in FreeRTOSIPConfig.h. */
				NULL, /* The task parameter, not used in this case. */
				tskIDLE_PRIORITY + 2, /* The priority assigned to the task is defined in FreeRTOSConfig.h. */
				NULL); /* The task handle is not used. */
}




