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

#define USER_NAME "WEBRADIO"
#define DIR_MAX_LEN 128
#define HOST_NAME_MAX_LEN 128
#define REQUEST_HEADER_MAX_LEN 200

#define REC_BUFF_SIZE 4096
#define STACK_SIZE ( configMINIMAL_STACK_SIZE * 80 )

static const TickType_t xTaskDelay = 250 / portTICK_RATE_MS;
static const TickType_t xReceiveTimeOut = 30000 / portTICK_RATE_MS;

static void prvShoutcastTask(void *pvParameters);
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
	char hostname[HOST_NAME_MAX_LEN];
	char request_header[REQUEST_HEADER_MAX_LEN];
	char recBuff[REC_BUFF_SIZE];
	BaseType_t ret;

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

	ret = FreeRTOS_connect(xSocket, &xServer, sizeof(struct freertos_sockaddr));

	if (ret == 0)
	{
		FreeRTOS_send(xSocket, request_header, strlen(request_header), 0);

		for (;;)
		{
			ret = FreeRTOS_recv(xSocket, recBuff, REC_BUFF_SIZE, 0);

			if (ret > 0)
			{
				//process data
				UARTprintf(".");
			}
			else
			{
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

	while (1)
	{

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

