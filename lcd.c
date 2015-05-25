/*
 * lcd.c
 *
 *  Created on: Apr 22, 2015
 *      Author: Tamas
 */
#include "stdbool.h"
#include "stdint.h"
#include "stdio.h"
#include "inc/tm4c1294ncpdt.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"
#include "include/FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "lcd.h"

//Port and Pins
#define PortL 			GPIO_PORTL_BASE
#define PORT_ENABLE 	SYSCTL_PERIPH_GPIOL
#define D4 				GPIO_PIN_0
#define D5 				GPIO_PIN_1
#define D6				GPIO_PIN_2
#define D7 				GPIO_PIN_3
#define RS 		 		GPIO_PIN_4
#define E 				GPIO_PIN_5
#define STACK_SIZE ( configMINIMAL_STACK_SIZE * 10 )

static const TickType_t xTaskDelay = 1 / portTICK_RATE_MS;

static void prvLCDTask(void *pvParameters){
	LCD_Init();
	while(true);
}
//LCD task
void vStartLCDTask(void){
	lcdQueue = xQueueCreate(10, 32);
	xTaskCreate(prvLCDTask, "LCD", STACK_SIZE, NULL, configMAX_PRIORITIES - 4, NULL);
}


// Initialize LCD
// Inputs: none
// Outputs: none
void LCD_Init(void){
	SysCtlPeripheralEnable(PORT_ENABLE);	//enable port
	GPIOPinTypeGPIOOutput(PortL,0xff);		//port out

	vTaskDelay(40*xTaskDelay);				//wait 15 ms

	GPIOPinWrite(PortL, RS,  0x00 );

	GPIOPinWrite(PortL, D7 | D6 | D5 | D4,  D4 | D5 ); // Function Set 3h
	GPIOPinWrite(PortL, E, E);	//enable
	vTaskDelay(1*xTaskDelay);	//wait a bit
	GPIOPinWrite(PortL, E, 0x00);	//enable off
	vTaskDelay(5*xTaskDelay);	//wait 4.1 ms

	GPIOPinWrite(PortL, D7 | D6 | D5 | D4,  D4 | D5 );	// Function Set 3h
	GPIOPinWrite(PortL, E, E); //enable
	vTaskDelay(1*xTaskDelay); //wait a bit
	GPIOPinWrite(PortL, E, 0x00); //enable off
	vTaskDelay(5*xTaskDelay); //wait 100 us

	GPIOPinWrite(PortL, D7 | D6 | D5 | D4,  D4 | D5 );	// Function Set 3h
	GPIOPinWrite(PortL, E, E); //enable
	vTaskDelay(1*xTaskDelay); //wait a bit
	GPIOPinWrite(PortL, E, 0x00); //enable off
	vTaskDelay(5*xTaskDelay); //wait 100 us

	GPIOPinWrite(PortL, D7 | D6 | D5 | D4,  D5 ); // Function Set 2h
	GPIOPinWrite(PortL, E, E); //enable
	vTaskDelay(1*xTaskDelay); //wait a bit
	GPIOPinWrite(PortL, E, 0x00); //enable off
	vTaskDelay(5*xTaskDelay); //wait 100 us

	// Function set
	GPIOPinWrite(PortL, D7 | D6 | D5 | D4,  D5 ); // Function Set 2h
	GPIOPinWrite(PortL, E, E); //enable
	vTaskDelay(1*xTaskDelay); //wait a bit
	GPIOPinWrite(PortL, D7 | D6 | D5 | D4,  D7 ); // Function Set 8h
	GPIOPinWrite(PortL, E, 0x00); //enable off
	vTaskDelay(5*xTaskDelay); //wait 100 us

	// Display ON/OFF (OFF)
	GPIOPinWrite(PortL, D7 | D6 | D5 | D4,  0x00 ); // Function Set 0h
	GPIOPinWrite(PortL, E, E); //enable
	vTaskDelay(1*xTaskDelay); //wait a bit
	GPIOPinWrite(PortL, D7 | D6 | D5 | D4,  D7 ); // Function Set 8h
	GPIOPinWrite(PortL, E, 0x00); //enable off
	vTaskDelay(5*xTaskDelay); //wait 100 us

	//Clear display
	GPIOPinWrite(PortL, D7 | D6 | D5 | D4,  0x00 ); // Function Set 0h
	GPIOPinWrite(PortL, E, E); //enable
	vTaskDelay(1*xTaskDelay); //wait a bit
	GPIOPinWrite(PortL, D7 | D6 | D5 | D4,  D4 ); // Function Set 1h
	GPIOPinWrite(PortL, E, 0x00); //enable off
	vTaskDelay(5*xTaskDelay); //wait 100 us

	//Entry mode
	GPIOPinWrite(PortL, D7 | D6 | D5 | D4,  0x00 ); // Function Set 0h
	GPIOPinWrite(PortL, E, E); //enable
	vTaskDelay(1*xTaskDelay); //wait a bit
	GPIOPinWrite(PortL, D7 | D6 | D5 | D4,  D5 | D6 ); // Function Set 6h
	GPIOPinWrite(PortL, E, 0x00); //enable off
	vTaskDelay(5*xTaskDelay); //wait 100 us

	// Display ON/OFF
	GPIOPinWrite(PortL, D7 | D6 | D5 | D4,  0x00 ); // Function Set 0h
	GPIOPinWrite(PortL, E, E); //enable
	vTaskDelay(1*xTaskDelay); //wait a bit
	GPIOPinWrite(PortL, D7 | D6 | D5 | D4,  D7 | D6  ); // Function Set 8h
	GPIOPinWrite(PortL, E, 0x00); //enable off
	vTaskDelay(5*xTaskDelay); //wait 100 us*/

	LCDWriteText("", 1, 16);
}

void LCDCommand(unsigned char command) {

	GPIOPinWrite(PortL, D7 | D6 | D5 | D4, (command & 0xf0) ); //
	GPIOPinWrite(PortL, RS, 0x00);
	GPIOPinWrite(PortL, E, E);
	vTaskDelay(1*xTaskDelay);
	GPIOPinWrite(PortL, E, 0x00);
	vTaskDelay(1*xTaskDelay);

	GPIOPinWrite(PortL, D7 | D6 | D5 | D4, (command & 0x0f) << 4 );
	GPIOPinWrite(PortL, RS, 0x00);
	GPIOPinWrite(PortL, E, E);
	vTaskDelay(1*xTaskDelay);
	GPIOPinWrite(PortL, E, 0x00);

	vTaskDelay(1*xTaskDelay);

}

void LCDWrite(unsigned char inputData) {

	GPIOPinWrite(PortL, D7 | D6 | D5 | D4, (inputData & 0xf0) );
	GPIOPinWrite(PortL, RS, RS);
	GPIOPinWrite(PortL, E, E);
	vTaskDelay(1*xTaskDelay);
	GPIOPinWrite(PortL, E, 0x00);

	vTaskDelay(1*xTaskDelay);

	GPIOPinWrite(PortL, D7 | D6 | D5 | D4, (inputData & 0x0f) << 4 );
	GPIOPinWrite(PortL, RS, RS);
	GPIOPinWrite(PortL, E, E);
	vTaskDelay(2*xTaskDelay);
	GPIOPinWrite(PortL, E, 0);

	vTaskDelay(5*xTaskDelay);

}

void LCDWriteText(char* inputText,unsigned char row, unsigned char col) {
	unsigned char address_d = 0;		// address of the data in the screen.
	switch(row)
	{
	case 0: address_d = 0x80 + col;		// at zeroth row
	break;
	case 1: address_d = 0xC0 + col;		// at first row
	break;
	case 2: address_d = 0x94 + col;		// at second row
	break;
	case 3: address_d = 0xD4 + col;		// at third row
	break;
	default: address_d = 0x80 + col;	// returns to first row if invalid row number is detected
	break;
	}

	LCDCommand(address_d);

	while(*inputText)					// Place a string, letter by letter.
		LCDWrite(*inputText++);
}
