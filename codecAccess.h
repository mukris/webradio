#ifndef CODEC_ACCESS_H
#define CODEC_ACCESS_H

typedef enum
{
	BASSTREBLE_SET = 0x2, //
	VOL_SET = 0xB,
} CodecInstruction;

void vStartCodecAccessTask(void);

void DMAInterrupt(void);

void SendCodecInstruct(CodecInstruction instType, uint16_t data);
void SendCodecTransfer(void* point, uint32_t size);

//QueueHandle_t codecInstQueue; //hidden
//QueueHandle_t codecQueue; //hidden

#endif /* CODEC_ACCESS_H */
