/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"

#define STACK_SIZE ( configMINIMAL_STACK_SIZE * 2 )

#define BROADCAST_PORT 8765
#define MAGIC_STRING "### Asteroid game magic string ###"

static void prvBroadcasterTask(void *pvParameters);

static TickType_t xRepeatRate = 500 / portTICK_RATE_MS;

void vStartBroadcasterTask(UBaseType_t uxTaskPriority)
{
	/* Create the echo client task that does not use the zero copy interface. */
	xTaskCreate(prvBroadcasterTask, /* The function that implements the task. */
				"Broadcast", /* Just a text name for the task to aid debugging. */
				STACK_SIZE, /* The stack size is defined in FreeRTOSIPConfig.h. */
				NULL, /* The task parameter, not used in this case. */
				uxTaskPriority, /* The priority assigned to the task is defined in FreeRTOSConfig.h. */
				NULL); /* The task handle is not used. */
}

static void prvBroadcasterTask(void *pvParameters)
{
	xSocket_t xSocket;
	struct freertos_sockaddr xBroadcastAddress;

	uint32_t localIP, broadcastIP, netmask, gateway, dns;

	/* Remove compiler warning about unused parameters. */
	(void) pvParameters;

	/* Get the current IP and netmask of the device */
	FreeRTOS_GetAddressConfiguration(&localIP, &netmask, &gateway, &dns);

	/* Calculate broadcast IP */
	broadcastIP = (localIP & netmask) | ~netmask;

	xBroadcastAddress.sin_port = FreeRTOS_htons(BROADCAST_PORT);
	xBroadcastAddress.sin_addr = broadcastIP;

	broadcastIP = FreeRTOS_htonl(broadcastIP);
	FreeRTOS_debug_printf(("Broadcast address is %lxip\n", broadcastIP));
	FreeRTOS_debug_printf(("Starting to send broadcast UDP to %lxip:%d\n", broadcastIP, BROADCAST_PORT));

	for (;;)
	{
		/* Create a socket. */
		xSocket = FreeRTOS_socket( FREERTOS_AF_INET, FREERTOS_SOCK_DGRAM, FREERTOS_IPPROTO_UDP);
		configASSERT( xSocket != FREERTOS_INVALID_SOCKET );

		/*
		 * Send the string to the socket.  ulFlags is set to 0, so the zero
		 * copy interface is not used.  That means the data from cTxString is
		 * copied into a network buffer inside FreeRTOS_sendto(), and cTxString
		 * can be reused as soon as FreeRTOS_sendto() has returned. 1 is added
		 * to ensure the NULL string terminator is sent as part of the message.
		 */
		FreeRTOS_sendto(xSocket, /* The socket being sent to. */
						MAGIC_STRING, /* The data being sent. */
						strlen(MAGIC_STRING) + 1,/* The length of the data being sent. */
						0, /* ulFlags with the FREERTOS_ZERO_COPY bit clear. */
						&xBroadcastAddress, /* The destination address. */
						sizeof(xBroadcastAddress));

		/* Pause for a short while to ensure the network is not too congested. */
		vTaskDelay(xRepeatRate);
		/* Close this socket before looping back to create another. */
		FreeRTOS_closesocket(xSocket);
	}
}
