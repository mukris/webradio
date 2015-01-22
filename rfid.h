#ifndef RFID_H_
#define RFID_H_

#include "queue.h"


typedef struct {
	uint8_t size;
	uint8_t uidByte[10];
} RfidMessage;

QueueHandle_t rfidEventQueue;

void vStartRfidTask(void);

#endif /* RFID_H_ */
