/*
 * lcd.h
 *
 *  Created on: Apr 22, 2015
 *      Author: Tamas
 */

#ifndef LCD_H_
#define LCD_H_

#include "FreeRTOS.h"
#include "queue.h"

xQueueHandle lcdQueue;

//LCD task
void vStartLCDTask(void);

// Initialize LCD
// Inputs: none
// Outputs: none
void LCD_Init(void);

void LCDCommand(unsigned char command);

void LCDWrite(unsigned char inputData);

void LCDWriteText(char* inputText,unsigned char row, unsigned char col);

#endif /* LCD_H_ */
