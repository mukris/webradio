#ifndef CODEC_ACCESS_H
#define CODEC_ACCESS_H

#define VOL_SET_INST 1
#define BASSTREBLE_SET_INST 2

void vStartCodecAccessTask(void);

void DMAInterrupt (void);

void SendCodecInstruct(char instType, uint16_t data);
void SendCodecTransfer(void* point, uint32_t size);

//QueueHandle_t codecInstQueue; //hidden
//QueueHandle_t codecQueue; //hidden

#endif /* CODEC_ACCESS_H */
