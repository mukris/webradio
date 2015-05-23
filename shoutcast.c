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

#define REC_BUFF_SIZE 256
#define STREAM_BUFF_SIZE 4096
#define META_BUFF_SIZE 512
#define STACK_SIZE ( configMINIMAL_STACK_SIZE * 80 )

static const TickType_t xTaskDelay = 250 / portTICK_RATE_MS;
static const TickType_t xSendTimeOut = 1000 / portTICK_RATE_MS;
static const TickType_t xReceiveTimeOut = 1000 / portTICK_RATE_MS;

typedef struct
{
	char name[128]; // name of the channel
	char title[128]; // title of the currently playing song
	uint32_t bitrate;
	uint32_t streamBlockSize; // how many bytes of stream data are between two meta data blocks
	uint32_t metaDataBlockSize; // how many bytes of meta data are between two stream data blocks
	uint32_t streamBytesUntilMetadata; // how many bytes left from the current stream block
	uint32_t metaDataBytesUntilStreamData; // how many bytes left from the current meta data block
} IcyData;

static void prvShoutcastTask(void *pvParameters);
uint32_t processReceivedData(uint8_t * data, uint32_t len, xStreamBuffer * streamBuff, xStreamBuffer * metaBuff,
								IcyData * icyData);
void parseMetaInfo(xStreamBuffer * metaBuff, IcyData * icyData);
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
	uint8_t * pRecBuff;
	int processedBytes;

	BaseType_t ret;
	IcyData icyData;

	xStreamBuffer * streamBuff;
	streamBuff = (xStreamBuffer *) pvPortMallocLarge(
			sizeof( *streamBuff ) - sizeof( streamBuff->ucArray ) + STREAM_BUFF_SIZE + 1);

	streamBuff->LENGTH = STREAM_BUFF_SIZE;
	vStreamBufferClear(streamBuff);

	xStreamBuffer * metaBuff;
	metaBuff = (xStreamBuffer *) pvPortMallocLarge(
			sizeof( *metaBuff ) - sizeof( metaBuff->ucArray ) + META_BUFF_SIZE + 1);

	metaBuff->LENGTH = META_BUFF_SIZE;
	vStreamBufferClear(metaBuff);

	memset(&icyData, 0, sizeof(IcyData));

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
				pRecBuff = recBuff;
				do
				{
					processedBytes = processReceivedData(pRecBuff, ret, streamBuff, metaBuff, &icyData);
					ret -= processedBytes;
					pRecBuff += processedBytes;
				} while (ret > 0);

				//UARTprintf(".");
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

typedef enum
{
	STATE_HEADER, //
	STATE_STREAM_START, //
	STATE_STREAM, //
	STATE_META_DATA_START, //
	STATE_META_DATA
} IcyParserState;

uint32_t processReceivedData(uint8_t * data, uint32_t len, xStreamBuffer * streamBuff, xStreamBuffer * metaBuff,
								IcyData * icyData)
{
	static IcyParserState state = STATE_HEADER;
	char * tempPtr;
	uint32_t partLen;

	data[len] = '\0';

	switch (state)
	{
	case STATE_HEADER:
		if ((tempPtr = strstr((char *) data, "\r\n\r\n")) != NULL)
		{
			UARTprintf("ICY Header received\r\n");

			// close the header with '\0' for the parsing to work correctly
			*((int *) tempPtr) = '\0';

			lStreamBufferAdd(streamBuff, 0, data, (uint32_t) tempPtr + 4 - (uint32_t) data);

			parseHeader(streamBuff, icyData);

			UARTprintf("ICY name: %s\r\n", icyData->name);
			UARTprintf("ICY bitrate: %d kbps\r\n", icyData->bitrate);
			UARTprintf("ICY metaDataGap: %d\r\n", icyData->streamBlockSize);

			state = STATE_STREAM_START;
			return (uint32_t) tempPtr + 4 - (uint32_t) data;
		}
		else
		{
			lStreamBufferAdd(streamBuff, 0, data, len);
			return len;
		}

	case STATE_STREAM_START:
		icyData->streamBytesUntilMetadata = icyData->streamBlockSize;
		state = STATE_STREAM;
		//no break

	case STATE_STREAM:
		partLen = FreeRTOS_min_uint32(len, icyData->streamBytesUntilMetadata);

		lStreamBufferAdd(streamBuff, 0, data, partLen);

		icyData->streamBytesUntilMetadata -= partLen;

		if (icyData->streamBytesUntilMetadata == 0)
		{
			state = STATE_META_DATA_START;
		}
		return partLen;

	case STATE_META_DATA_START:
		if (data[0] != 0)
		{
			icyData->metaDataBlockSize = data[0] << 4;
			icyData->metaDataBytesUntilStreamData = data[0] << 4;
			vStreamBufferClear(metaBuff);
			state = STATE_META_DATA;
		}
		else
		{
			state = STATE_STREAM_START;
		}
		return 1;

	case STATE_META_DATA:
		partLen = FreeRTOS_min_uint32(len, icyData->metaDataBytesUntilStreamData);
		lStreamBufferAdd(metaBuff, 0, data, partLen);

		icyData->metaDataBytesUntilStreamData -= partLen;

		if (icyData->metaDataBytesUntilStreamData == 0)
		{
			parseMetaInfo(metaBuff, icyData);
			state = STATE_STREAM_START;
		}
		return partLen;
	}
	return len;
}

void parseHeader(xStreamBuffer * streamBuff, IcyData * icyData)
{
	uint8_t * buff;
	char * tempPtr;
	char * tmp;
	lStreamBufferGetPtr(streamBuff, &buff);
	if ((tempPtr = strstr((char *) buff, "icy-name:")) != NULL)
	{
		int i = 0;
		tempPtr += strlen("icy-name:");

		for (i = 0; i < sizeof(icyData->name); i++)
		{
			if (tempPtr[i] == '\r' || tempPtr[i] == '\n')
			{
				icyData->name[i] = '\0';
				break;
			}
			else
			{
				icyData->name[i] = tempPtr[i];
			}
		}
	}
	if ((tempPtr = strstr((char *) buff, "icy-metaint:")) != NULL)
	{
		tempPtr += strlen("icy-metaint:");
		icyData->streamBlockSize = strtol(tempPtr, &tmp, 10);
	}
	if ((tempPtr = strstr((char *) buff, "icy-br:")) != NULL)
	{
		tempPtr += strlen("icy-br:");
		icyData->bitrate = strtol(tempPtr, &tmp, 10);
	}
}

void parseMetaInfo(xStreamBuffer * metaBuff, IcyData * icyData)
{
	uint8_t * buff;
	char * tempPtr;
	lStreamBufferGetPtr(metaBuff, &buff);
	if ((tempPtr = strstr((char *) buff, "StreamTitle='")) != NULL)
	{
		int i = 0;
		tempPtr += strlen("StreamTitle='");

		for (i = 0; i < sizeof(icyData->title); i++)
		{
			if (tempPtr[i] == '\'')
			{
				icyData->title[i] = '\0';
				break;
			}
			else
			{
				icyData->title[i] = tempPtr[i];
			}
		}
	}
	UARTprintf("Title: %s\n", icyData->title);
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

