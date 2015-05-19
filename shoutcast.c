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
#include "FreeRTOS_Stream_Buffer.h"
#include "NetworkBufferManagement.h"
#include "utils/uartstdio.h"

#define USER_NAME "WEBRADIO"
#define DIR_MAX_LEN 128
#define HOST_NAME_MAX_LEN 128
#define REQUEST_HEADER_MAX_LEN 200

#define REC_BUFF_SIZE 64
#define RECEIVED_BUFF_SIZE 4096
#define STACK_SIZE ( configMINIMAL_STACK_SIZE * 80 )

static const TickType_t xTaskDelay = 250 / portTICK_RATE_MS;
static const TickType_t xSendTimeOut = 1000 / portTICK_RATE_MS;
static const TickType_t xReceiveTimeOut = 1000 / portTICK_RATE_MS;

typedef struct
{
	uint32_t bitrate;
	uint32_t metaInt;
} IcyData;

static void prvShoutcastTask(void *pvParameters);
void parseHeader(xStreamBuffer * streamBuff, IcyData * icyData);
void parseURL(char *URL, char *request_header, char *host_name, uint16_t *port);

void vStartShoutcastReceiver(void)
{
	xTaskCreate(prvShoutcastTask, /* The function that implements the task. */
				"Shoutcast", /* Just a text name for the task to aid debugging. */
				STACK_SIZE, /* The stack size is defined in FreeRTOSIPConfig.h. */
				NULL, /* The task parameter, pointer to the connected socket */
				configMAX_PRIORITIES - 3, NULL); /* The task handle is not used. */
}

static void prvShoutcastTask(void *pvParameters)
{
	xSocket_t xSocket;
	struct freertos_sockaddr xServer, xBindAddr;
	uint16_t port;
	char url[] = "http://185.33.23.5:80";
	//char url[] = "http://50.30.37.166:42000";
	char hostname[HOST_NAME_MAX_LEN];
	char request_header[REQUEST_HEADER_MAX_LEN];
	uint8_t recBuff[REC_BUFF_SIZE];

	BaseType_t ret;
	uint8_t header_received = 0;
	IcyData icyData;
	char * tempPtr;

	xStreamBuffer * streamBuff;
	streamBuff = (xStreamBuffer *) pvPortMallocLarge(
			sizeof( *streamBuff ) - sizeof( streamBuff->ucArray ) + RECEIVED_BUFF_SIZE + 1);

	streamBuff->LENGTH = RECEIVED_BUFF_SIZE;
	vStreamBufferClear(streamBuff);

	xBindAddr.sin_addr = 0;
	xBindAddr.sin_port = FreeRTOS_htons(1234);

	/* Attempt to open the socket. */
	xSocket = FreeRTOS_socket( FREERTOS_AF_INET, FREERTOS_SOCK_STREAM, FREERTOS_IPPROTO_TCP);

	/* Check the socket was created. */
	configASSERT(xSocket != FREERTOS_INVALID_SOCKET);

	FreeRTOS_bind(xSocket, &xBindAddr, sizeof(struct freertos_sockaddr));

	parseURL(url, request_header, hostname, &port);

	xServer.sin_addr = FreeRTOS_inet_addr(&url[7]);
	//xServer.sin_addr = FreeRTOS_gethostbyname("www.google.com");
	xServer.sin_port = FreeRTOS_htons(port);

	/* If FREERTOS_SO_RCVBUF or FREERTOS_SO_SNDBUF are to be used with
	 FreeRTOS_setsockopt() to change the buffer sizes from their default then do
	 it here!.  (see the FreeRTOS_setsockopt() documentation. */

	/* If ipconfigUSE_TCP_WIN is set to 1 and FREERTOS_SO_WIN_PROPERTIES is to
	 be used with FreeRTOS_setsockopt() to change the sliding window size from
	 its default then do it here! (see the FreeRTOS_setsockopt()
	 documentation. */

	/* Set a time out so accept() will just wait for a connection. */
	FreeRTOS_setsockopt(xSocket, 0, FREERTOS_SO_RCVTIMEO, &xReceiveTimeOut, sizeof(xReceiveTimeOut));
	FreeRTOS_setsockopt(xSocket, 0, FREERTOS_SO_SNDTIMEO, &xSendTimeOut, sizeof(xSendTimeOut));

	ret = FreeRTOS_connect(xSocket, &xServer, sizeof(struct freertos_sockaddr));

	if (ret == 0)
	{
		FreeRTOS_send(xSocket, request_header, strlen(request_header), 0);

		for (;;)
		{
			ret = FreeRTOS_recv(xSocket, recBuff, REC_BUFF_SIZE - 1, 0);

			if (ret > 0)
			{
				recBuff[ret] = 0;
				if (!header_received && (tempPtr = strstr((char *) recBuff, "\r\n\r\n")) != NULL)
				{
					header_received = 1;
					UARTprintf("ICY Header received\r\n");

					*((int *) tempPtr) = 0;
					lStreamBufferAdd(streamBuff, 0, recBuff, ret);
					parseHeader(streamBuff, &icyData);
					UARTprintf("ICY bitrate: %d kbps\r\n", icyData.bitrate);
					UARTprintf("ICY metaInt: %d\r\n", icyData.metaInt);
				}
				else
				{
					lStreamBufferAdd(streamBuff, 0, recBuff, ret);
				}

				UARTprintf(".");
			}
			else if (ret == 0)
			{
				UARTprintf("Received 0 bytes\n");
			}
			else
			{
				UARTprintf("Receive ERROR %d... shutting down...\n", ret);
				/* Error (maybe the connected socket already shut down the socket?).
				 Attempt graceful shutdown. */
				FreeRTOS_shutdown(xSocket, FREERTOS_SHUT_RDWR);
				break;
			}
		}

		/* The RTOS task will get here if an error is received on a read.  Ensure the
		 socket has shut down (indicated by FreeRTOS_recv() returning a FREERTOS_EINVAL
		 error before closing the socket). */

		while (FreeRTOS_recv(xSocket, recBuff, REC_BUFF_SIZE, 0) >= 0)
		{
			/* Wait for shutdown to complete.  If a receive block time is used then
			 this delay will not be necessary as FreeRTOS_recv() will place the RTOS task
			 into the Blocked state anyway. */
			vTaskDelay(xTaskDelay);

			/* Note - real applications should implement a timeout here, not just
			 loop forever. */
		}

		/* Shutdown is complete and the socket can be safely closed. */
		FreeRTOS_closesocket(xSocket);

	}

	UARTprintf("Dying\r\n\r\n");
	while (1)
	{

	}
}

void parseHeader(xStreamBuffer * streamBuff, IcyData * icyData)
{
	uint8_t * buff;
	char * tempPtr;
	char * tmp;
	lStreamBufferGetPtr(streamBuff, &buff);
	if ((tempPtr = strstr((char *) buff, "icy-metaint:")) != NULL)
	{
		tempPtr += strlen("icy-metaint:");
		icyData->metaInt = strtol(tempPtr, &tmp, 10) << 4;
	}
	if ((tempPtr = strstr((char *) buff, "icy-br:")) != NULL)
	{
		tempPtr += strlen("icy-br:");
		icyData->bitrate = strtol(tempPtr, &tmp, 10);
	}
}

void parseURL(char *URL, char *request_header, char *host_name, uint16_t *port)
{
	char *url_start, *url_end;
	char directory[DIR_MAX_LEN];
	uint8_t cnt;

	/* find host name */
	url_start = strstr(URL, "http");		// http://urladdr.com
	url_start += 7;
	url_end = strstr(url_start, ":");

	cnt = 0;
	do
	{
		host_name[cnt] = *url_start;
		cnt++;
	} while ((++url_start != url_end) && (cnt < HOST_NAME_MAX_LEN - 1));
	host_name[cnt] = '\0';

	/* find directory path */
	url_start = strstr(url_end, "/");
	if (url_start == NULL)
	{
		directory[0] = '/';
		directory[1] = 0;
	}
	else
	{
		cnt = 0;
		do
		{
			directory[cnt] = *url_start;
			cnt++;
		} while ((++url_start != NULL) && (cnt < DIR_MAX_LEN - 1));
		directory[cnt] = '\0';
	}

	*(url_end) = 0;

	/* find port number */
	url_end++;		// pointer to port number
	*port = atoi(url_end);

	/* create request header */
	strcpy(request_header, "GET ");
	strcat(request_header, directory);
	strcat(request_header, " HTTP/1.0\r\nHost: ");
	strcat(request_header, host_name);
	strcat(request_header, "\r\n");
	strcat(request_header, "User-Agent: ");
	strcat(request_header, USER_NAME);
	strcat(request_header, "\r\nAccept: */*\r\nIcy-MetaData:1\r\nConnection: close\r\n\r\n");
}

