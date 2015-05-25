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

//Port and Pins
#define PortK 			GPIO_PORTK_BASE
#define PORT_ENABLE 	SYSCTL_PERIPH_GPIOK
#define D4 				GPIO_PIN_4
#define D5 				GPIO_PIN_5
#define D6				GPIO_PIN_6
#define D7 				GPIO_PIN_7
#define RS 		 		GPIO_PIN_1
#define E 				GPIO_PIN_0
#define STACK_SIZE ( configMINIMAL_STACK_SIZE * 10 )

static const TickType_t xTaskDelay = 0.001 / portTICK_RATE_MS;

void prvLCDTask(void){
	LCD_Init();
	while(true);
}
//LCD task
void vStartLCDTask(void){
	xTaskCreate(prvLCDTask, "LCD", STACK_SIZE, NULL, configMAX_PRIORITIES - 4, NULL);
}


// Initialize LCD
// Inputs: none
// Outputs: none
void LCD_Init(void){
	SysCtlPeripheralEnable(PORT_ENABLE);	//enable port
	GPIOPinTypeGPIOOutput(PortK, D7 | D6 | D5 | D4 | RS | E);		//port out

	vTaskDelay(20000*xTaskDelay);				//wait 15 ms

	//NULL decl
	GPIOPinWrite(PortK, RS,  0x00 );
	GPIOPinWrite(PortK, E,  0x00 );
	GPIOPinWrite(PortK, D7 | D6 | D5 | D4,  0x00 );

	//First
	GPIOPinWrite(PortK, D7 | D6 | D5 | D4,  D4 | D5 ); // Function Set 3h
	PulseE(5000);

	//Second
	GPIOPinWrite(PortK, D7 | D6 | D5 | D4,  D4 | D5 );	// Function Set 3h
	PulseE(500);

	//Third
	GPIOPinWrite(PortK, D7 | D6 | D5 | D4,  D4 | D5 );	// Function Set 3h
	PulseE(500);

	GPIOPinWrite(PortK, D7 | D6 | D5 | D4,  D5 ); // Function Set 2h
	PulseE(500);

	// Function set
	GPIOPinWrite(PortK, D7 | D6 | D5 | D4,  D5 ); // Function Set 2h
	PulseE(500);

	GPIOPinWrite(PortK, D7 | D6 | D5 | D4,  D7 ); // Function Set 8h
	PulseE(500);

	// Display ON/OFF (OFF)
	GPIOPinWrite(PortK, D7 | D6 | D5 | D4,  0x00 ); // Function Set 0h
	PulseE(500);

	GPIOPinWrite(PortK, D7 | D6 | D5 | D4,  D7 ); // Function Set 8h
	PulseE(500);

	//Clear display
	GPIOPinWrite(PortK, D7 | D6 | D5 | D4,  0x00 ); // Function Set 0h
	PulseE(500);

	GPIOPinWrite(PortK, D7 | D6 | D5 | D4,  D4 ); // Function Set 1h
	PulseE(500);

	//Entry mode
	GPIOPinWrite(PortK, D7 | D6 | D5 | D4,  0x00 ); // Function Set 0h
	PulseE(500);

	GPIOPinWrite(PortK, D7 | D6 | D5 | D4,  D5 | D6 ); // Function Set 6h
	PulseE(500);

	// Display ON/OFF
	GPIOPinWrite(PortK, D7 | D6 | D5 | D4,  0x00 ); // Function Set 0h
	PulseE(500);

	GPIOPinWrite(PortK, D7 | D6 | D5 | D4,  D7 | D6  ); // Function Set 8h
	PulseE(500);

	LCDWriteText("Hello World!", 0, 0);
	LCDWriteText("Hello World!", 0, 0);
	LCDWriteText("Hello World!", 0, 0);
	LCDWriteText("Hello World!", 0, 0);

}

void LCDCommand(unsigned char command) {

	GPIOPinWrite(PortK, D7 | D6 | D5 | D4, (command & 0xf0) );
	GPIOPinWrite(PortK, RS, 0x00);
	PulseE(500);

	GPIOPinWrite(PortK, D7 | D6 | D5 | D4, (command & 0x0f) << 4 );
	GPIOPinWrite(PortK, RS, 0x00);
	PulseE(500);

	vTaskDelay(1*xTaskDelay);

}

void LCDWrite(unsigned char inputData) {

	GPIOPinWrite(PortK, D7 | D6 | D5 | D4, (inputData & 0xf0) );
	GPIOPinWrite(PortK, RS, RS);
	PulseE(500);

	GPIOPinWrite(PortK, D7 | D6 | D5 | D4, (inputData & 0x0f) << 4 );
	GPIOPinWrite(PortK, RS, RS);
	PulseE(500);

}

void LCDWriteText(char* inputText,unsigned char row, unsigned char col) {
	unsigned char address_d = 0x80;		// address of the data in the screen.
	switch(row)
	{
	case 0: address_d = 0x80 + col;		// at zeroth row
	break;
	case 1: address_d = 0xC0 + col;		// at first row
	break;
	default: address_d = 0x80 + col;
	break;
	}

	//LCDCommand(address_d);

	while(*inputText)					// Place a string, letter by letter.
		LCDWrite(*inputText++);
}

void PulseE(double us){
	GPIOPinWrite(PortK, E, E);	//enable
	vTaskDelay(100*xTaskDelay);	//wait a bit
	GPIOPinWrite(PortK, E, 0x00);	//enable off
	vTaskDelay(us*xTaskDelay);	//wait 4.1 ms
}
