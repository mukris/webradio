/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

#include "FreeRTOS_IP.h"
#include "FreeRTOS_IP_Private.h"
#include "FreeRTOS_Sockets.h"
#include "NetworkBufferManagement.h"

#define STACK_SIZE ( configMINIMAL_STACK_SIZE * 3 )
static const TickType_t xTaskDelay = 250 / portTICK_RATE_MS;

static void prvTcpTask(void *pvParameters);
static void prvCreateTCPServerSocket(void *pvParameters);

void vStartTcpServer(void)
{
	xTaskCreate(prvCreateTCPServerSocket, /* The function that implements the task. */
				"TcpServer", /* Just a text name for the task to aid debugging. */
				STACK_SIZE, /* The stack size is defined in FreeRTOSIPConfig.h. */
				NULL, /* The task parameter, pointer to the connected socket */
				tskIDLE_PRIORITY + 1, NULL); /* The task handle is not used. */
}

static void prvCreateTCPServerSocket(void *pvParameters)
{
	struct freertos_sockaddr xClient, xBindAddress;
	xSocket_t xListeningSocket, xConnectedSocket;
	socklen_t xSize = sizeof(xClient);
	static const TickType_t xReceiveTimeOut = portMAX_DELAY;
	const BaseType_t xBacklog = 20;

	/* Attempt to open the socket. */
	xListeningSocket = FreeRTOS_socket( FREERTOS_AF_INET, FREERTOS_SOCK_STREAM, FREERTOS_IPPROTO_TCP);

	/* Check the socket was created. */
	configASSERT( xListeningSocket != FREERTOS_INVALID_SOCKET );

	/* If FREERTOS_SO_RCVBUF or FREERTOS_SO_SNDBUF are to be used with
	 FreeRTOS_setsockopt() to change the buffer sizes from their default then do
	 it here!.  (see the FreeRTOS_setsockopt() documentation. */

	/* If ipconfigUSE_TCP_WIN is set to 1 and FREERTOS_SO_WIN_PROPERTIES is to
	 be used with FreeRTOS_setsockopt() to change the sliding window size from
	 its default then do it here! (see the FreeRTOS_setsockopt()
	 documentation. */

	/* Set a time out so accept() will just wait for a connection. */
	FreeRTOS_setsockopt(xListeningSocket, 0, FREERTOS_SO_RCVTIMEO, &xReceiveTimeOut, sizeof(xReceiveTimeOut));

	/* Set the listening port to 8766. */
	xBindAddress.sin_port = (uint16_t) 8766;
	xBindAddress.sin_port = FreeRTOS_htons(xBindAddress.sin_port);

	/* Bind the socket to the port that the client RTOS task will send to. */
	FreeRTOS_bind(xListeningSocket, &xBindAddress, sizeof(xBindAddress));

	/* Set the socket into a listening state so it can accept connections.
	 The maximum number of simultaneous connections is limited to 20. */
	FreeRTOS_listen(xListeningSocket, xBacklog);

	FreeRTOS_debug_printf(("TCP socket listening on port %d\n", FreeRTOS_htons(xBindAddress.sin_port)));

	for (;;)
	{
		/* Wait for incoming connections. */
		xConnectedSocket = FreeRTOS_accept(xListeningSocket, &xClient, &xSize);
		configASSERT( xConnectedSocket != FREERTOS_INVALID_SOCKET );

		FreeRTOS_debug_printf(("TCP connection accepted\n"));
		/* Spawn a RTOS task to handle the connection. */
		xTaskCreate(prvTcpTask, /* The function that implements the task. */
					"TcpClient", /* Just a text name for the task to aid debugging. */
					STACK_SIZE, /* The stack size is defined in FreeRTOSIPConfig.h. */
					xConnectedSocket, /* The task parameter, pointer to the connected socket */
					tskIDLE_PRIORITY + 1, NULL); /* The task handle is not used. */
	}
}

static void prvTcpTask(void *pvParameters)
{
	xNetworkBufferDescriptor_t * networBufferDesc;
	uint8_t * buff;
	size_t size = strlen("Hello World");
	struct freertos_sockaddr xAddress;
	xSocket_t xSocket = (xSocket_t) pvParameters;
	xFreeRTOS_Socket_t *pxSocket = ( xFreeRTOS_Socket_t * ) xSocket;
	FreeRTOS_getremoteaddress( xSocket, &xAddress);

	FreeRTOS_debug_printf(("Sending Hello World to %lxip:%d\n", FreeRTOS_htonl(xAddress.sin_addr), FreeRTOS_htons(xAddress.sin_port)));

	FreeRTOS_send(xSocket, /* The socket being sent to. */
					"Hello World", /* The data being sent. */
					size, /* The remaining length of data to send. */
					0); /* ulFlags. */

	/* Initiate graceful shutdown. */
	FreeRTOS_shutdown(xSocket, FREERTOS_SHUT_RDWR);

	networBufferDesc = pxGetNetworkBufferWithDescriptor(size, 0);
	buff = networBufferDesc->pucEthernetBuffer;

	/* Wait for the socket to disconnect gracefully (indicated by FreeRTOS_recv()
	 returning a FREERTOS_EINVAL error) before closing the socket. */
	while (FreeRTOS_recv(xSocket, buff, size, 0) >= 0)
	{
		/* Wait for shutdown to complete.  If a receive block time is used then
		 this delay will not be necessary as FreeRTOS_recv() will place the RTOS task
		 into the Blocked state anyway. */
		vTaskDelay(xTaskDelay);

		/* Note - real applications should implement a timeout here, not just
		 loop forever. */
	}

	vReleaseNetworkBufferAndDescriptor(networBufferDesc);

	/* The socket has shut down and is safe to close. */
	FreeRTOS_closesocket(xSocket);

	vTaskDelete( NULL );
}
