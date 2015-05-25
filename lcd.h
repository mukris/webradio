/*
 * lcd.h
 *
 *  Created on: Apr 22, 2015
 *      Author: Tamas
 */

#ifndef LCD_H_
#define LCD_H_

//LCD task
void vStartLCDTask(void);
void prvLCDTask(void);

// Initialize LCD
// Inputs: none
// Outputs: none
void LCD_Init(void);

void LCDCommand(unsigned char command);

void LCDWrite(unsigned char inputData);

void LCDWriteText(char* inputText,unsigned char row, unsigned char col);

void PulseE(int ms);

#endif /* LCD_H_ */
