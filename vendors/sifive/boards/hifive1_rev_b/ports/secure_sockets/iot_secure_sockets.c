/*
 * Amazon FreeRTOS Secure Sockets for Infineon XMC4800 IoT Connectivity Kit V1.0.1
 * Copyright (C) 2018 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Copyright (c) 2018, Infineon Technologies AG
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,are permitted provided that the
 * following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 * disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holders nor the names of its contributors may be used to endorse or promote
 * products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE  FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY,OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * To improve the quality of the software, users are encouraged to share modifications, enhancements or bug fixes with
 * Infineon Technologies AG dave@infineon.com).
 */

/**
 * @file iot_secure_sockets.c
 * @brief WiFi and Secure Socket interface implementation.
 */

/* Define _SECURE_SOCKETS_WRAPPER_NOT_REDEFINE to prevent secure sockets functions
 * from redefining in iot_secure_sockets_wrapper_metrics.h */
#define _SECURE_SOCKETS_WRAPPER_NOT_REDEFINE

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* TLS includes. */
#include "iot_tls.h"
#include "iot_pkcs11.h"
#include "iot_pkcs11_config.h"

/* Socket and WiFi interface includes. */
#include "iot_wifi.h"
#include "iot_secure_sockets.h"
#include "esp/esp_netconn.h"

/* WiFi configuration includes. */
#include "aws_wifi_config.h"
#include "aws_clientcredential.h"
#include "aws_clientcredential_keys.h"
#include "iot_default_root_certificates.h"

extern BaseType_t xDisconnectAlert;
extern BaseType_t xIsWiFiInitialized;
extern WIFINetworkParams_t pxLastNetworkParams;

#undef _SECURE_SOCKETS_WRAPPER_NOT_REDEFINE

/**
 * @brief A Flag to indicate whether or not a socket is
 * secure i.e. it uses TLS or not.
 */
#define securesocketsSOCKET_SECURE_FLAG                 ( 1UL << 0 )

/**
 * @brief A flag to indicate whether or not a socket is closed
 * for receive.
 */
#define securesocketsSOCKET_READ_CLOSED_FLAG            ( 1UL << 1 )

/**
 * @brief A flag to indicate whether or not a socket is closed
 * for send.
 */
#define securesocketsSOCKET_WRITE_CLOSED_FLAG           ( 1UL << 2 )

/**
 * @brief A flag to indicate whether or not a non-default server
 * certificate has been bound to the socket.
 */
#define securesocketsSOCKET_TRUSTED_SERVER_CERT_FLAG    ( 1UL << 3 )

/**
 * @brief A flag to indicate whether or not the socket is connected.
 *
 */
#define securesocketsSOCKET_IS_CONNECTED                ( 1UL << 4 )

/**
 * @brief Represents a secure socket.
 */
typedef struct SSocketContext_t
{
	esp_netconn_type_t xSocketType;
	esp_netconn_p xSocket;
	uint32_t ulFlags;                   /**< Various properties of the socket (secured etc.). */
	char * pcDestination;               /**< Destination URL. Set using SOCKETS_SO_SERVER_NAME_INDICATION option in SOCKETS_SetSockOpt function. */
	void * pvTLSContext;                /**< The TLS Context. */
	char * pcServerCertificate;         /**< Server certificate. Set using SOCKETS_SO_TRUSTED_SERVER_CERTIFICATE option in SOCKETS_SetSockOpt function. */
	uint32_t ulServerCertificateLength; /**< Length of the server certificate. */
	char ** ppcAlpnProtocols;
	uint32_t ulAlpnProtocolsCount;
	uint8_t ucInUse;                    /**< Tracks whether the socket is in use or not. */
	esp_pbuf_p pbuf;
	BaseType_t available;
	size_t offset;
	SemaphoreHandle_t xUcInUse;
	SemaphoreHandle_t xRecieveUcInUse;
} SSocketContext_t;

/**
 * @brief Secure socket objects.
 *
 * An index in this array is returned to the user from SOCKETS_Socket
 * function.
 */
static SSocketContext_t xSockets[ESP_CFG_MAX_CONNS];

static const TickType_t xMaxSemaphoreBlockTime = pdMS_TO_TICKS( 60000UL );

static BaseType_t rootCAwritten = 0;

static uint32_t prvGetFreeSocket( void )
{
	uint32_t ulSocketNumber;

	taskENTER_CRITICAL();
	/* Iterate over xSockets array to see if any free socket
	* is available. */
	for (ulSocketNumber = 0 ; ulSocketNumber < ( uint32_t ) ESP_CFG_MAX_CONNS ; ulSocketNumber++)
	{
		if (xSockets[ ulSocketNumber ].ucInUse == 0U)
		{
			/* Mark the socket as "in-use". */
			xSockets[ ulSocketNumber ].ucInUse = 1;

			/* We have found a free socket, so stop. */
			break;
		}
	}
	taskEXIT_CRITICAL();

	/* Did we find a free socket? */
	if (ulSocketNumber == (uint32_t) ESP_CFG_MAX_CONNS)
	{
		/* Return SOCKETS_INVALID_SOCKET if we fail to
		* find a free socket. */
		ulSocketNumber = (uint32_t) SOCKETS_INVALID_SOCKET;
	}

	return ulSocketNumber;
}
/*-----------------------------------------------------------*/

static BaseType_t prvReturnSocket( uint32_t ulSocketNumber )
{
	BaseType_t xResult = pdFAIL;

	/* Since multiple tasks can be accessing this simultaneously,
	* this has to be in critical section. */
	taskENTER_CRITICAL();

	/* Mark the socket as free. */
	xSockets[ ulSocketNumber ].ucInUse = 0;

	xResult = pdTRUE;

	taskEXIT_CRITICAL();

	return xResult;
}

/*-----------------------------------------------------------*/

static BaseType_t prvIsValidSocket( uint32_t ulSocketNumber )
{
	BaseType_t xValid = pdFALSE;

	/* Check that the provided socket number is within the valid index range. */
	if (ulSocketNumber < ( uint32_t ) ESP_CFG_MAX_CONNS )
	{
		taskENTER_CRITICAL();

		/* Check that this socket is in use. */
		if (xSockets[ ulSocketNumber ].ucInUse == 1U )
		{
			/* This is a valid socket number. */
			xValid = pdTRUE;
		}

		taskEXIT_CRITICAL();
	}

	return xValid;
}
/*-----------------------------------------------------------*/

static BaseType_t prvNetworkSend( void * pvContext,
                                  const unsigned char * pucData,
                                  size_t xDataLength )
{
	BaseType_t lRetVal = 0;
	SSocketContext_t* pxContext = NULL;
	uint32_t ulSocketNumber = ( uint32_t ) pvContext; 		/*lint !e923 cast is necessary for port. */

	pxContext = &( xSockets[ ulSocketNumber ] );

	if ((pucData != NULL) && (pxContext != NULL))
	{
		espr_t res = esp_netconn_write(pxContext->xSocket, pucData, xDataLength);

		if (res == espOK)
		{
			res = esp_netconn_flush(pxContext->xSocket);    /* Flush data to output */
			if (res == espOK)					            /* Were data sent? */
			{
				lRetVal = xDataLength;
#ifdef configESP32_DATA_EXCHANGE_LOG
					configPRINT_STRING(("\033[0;32m"));
					configPRINTF(("Send %d \r\n", lRetVal));
					configPRINT_STRING(("\033[0m"));
#endif
			}
		}
	}

	vTaskDelay(1);

	return lRetVal;
}

/*-----------------------------------------------------------*/

static BaseType_t prvNetworkRecv( void * pvContext,
                                  unsigned char * pucReceiveBuffer,
                                  size_t xReceiveLength )
{
	BaseType_t lRetVal = 0;
	SSocketContext_t* pxContext = NULL;
	uint32_t ulSocketNumber = ( uint32_t ) pvContext; 		/*lint !e923 cast is necessary for port. */

	pxContext = &( xSockets[ ulSocketNumber ] );

	if ((pucReceiveBuffer != NULL) && (pxContext != NULL))
	{
		if (pxContext->available == 0)
		{
			espr_t res = esp_netconn_receive(pxContext->xSocket, &(pxContext->pbuf));
			if ((res == espCLOSED) || (res == espTIMEOUT))
			{
				if (pxContext->pbuf != NULL)
				{
					esp_pbuf_free(pxContext->pbuf);    /* Free the memory after usage */
					pxContext->pbuf = NULL;
				}
			}
			else if ((res == espOK) && (pxContext->pbuf != NULL))
			{
				pxContext->available = esp_pbuf_length(pxContext->pbuf, 1);

#ifdef configESP32_DATA_EXCHANGE_LOG
					configPRINT_STRING(("\033[0;32m"));
					configPRINTF(("Recv %d \r\n", pxContext->available));
					configPRINT_STRING(("\033[0m"));
#endif
				pxContext->offset = 0;
			}
		}

		if (pxContext->available > 0)
		{
			lRetVal = esp_pbuf_copy(pxContext->pbuf, pucReceiveBuffer, xReceiveLength, pxContext->offset);
			pxContext->offset += lRetVal;
			pxContext->available -= lRetVal;
			if (pxContext->available == 0)
			{
				esp_pbuf_free(pxContext->pbuf);    /* Free the memory after usage */
				pxContext->pbuf = NULL;
			}
		}
	}

	vTaskDelay(1);

	return lRetVal;
}
/*-----------------------------------------------------------*/

Socket_t SOCKETS_Socket( int32_t lDomain,
                         int32_t lType,
                         int32_t lProtocol )
{
	uint32_t ulSocketNumber;

	/* Ensure that only supported values are supplied. */
	configASSERT( lDomain == SOCKETS_AF_INET );
	configASSERT( lType == SOCKETS_SOCK_STREAM );
	configASSERT( lProtocol == SOCKETS_IPPROTO_TCP );

	ulSocketNumber = prvGetFreeSocket();

	/* If we get a free socket, set its attributes. */
	if(ulSocketNumber != (uint32_t)SOCKETS_INVALID_SOCKET)
	{
		/* Obtain the socketInUse mutex. */
		if (xSemaphoreTake( xSockets[ ulSocketNumber ].xUcInUse, xMaxSemaphoreBlockTime) == pdTRUE)
		{
			if((xDisconnectAlert == pdTRUE) && (xIsWiFiInitialized == pdTRUE))
			{
				xDisconnectAlert = pdFALSE;
				WIFI_ConnectAP(&pxLastNetworkParams);
			}

			xSockets[ ulSocketNumber ].xSocketType = ESP_NETCONN_TYPE_TCP;
			xSockets[ ulSocketNumber ].ulFlags = 0;
			xSockets[ ulSocketNumber ].pcDestination = NULL;
			xSockets[ ulSocketNumber ].pvTLSContext = NULL;
			xSockets[ ulSocketNumber ].pcServerCertificate = NULL;
			xSockets[ ulSocketNumber ].ulServerCertificateLength = 0;
			xSockets[ ulSocketNumber ].ppcAlpnProtocols = NULL;
			xSockets[ ulSocketNumber ].ulAlpnProtocolsCount = 0;
			xSockets[ ulSocketNumber ].pbuf = NULL;
			xSockets[ ulSocketNumber ].available = 0;
			xSockets[ ulSocketNumber ].offset = 0;

			xSockets[ ulSocketNumber ].xSocket = esp_netconn_new(xSockets[ ulSocketNumber ].xSocketType);

			if (xSockets[ ulSocketNumber ].xSocket != NULL)
			{
				esp_netconn_set_receive_timeout(xSockets[ ulSocketNumber ].xSocket, 0);
			}
			else
			{
				ulSocketNumber = (uint32_t)SOCKETS_INVALID_SOCKET;
			}
			/* Give back the socketInUse mutex. */
			xSemaphoreGive(xSockets[ ulSocketNumber ].xUcInUse);
		}
		else
		{
			ulSocketNumber = (uint32_t)SOCKETS_INVALID_SOCKET;
		}
	}

	return (Socket_t)ulSocketNumber;
}
/*-----------------------------------------------------------*/

Socket_t SOCKETS_Accept( Socket_t xSocket,
                         SocketsSockaddr_t * pxAddress,
                         Socklen_t * pxAddressLength )
{
  return SOCKETS_INVALID_SOCKET;
}
/*-----------------------------------------------------------*/


int32_t SOCKETS_Connect( Socket_t xSocket,
                         SocketsSockaddr_t * pxAddress,
                         Socklen_t xAddressLength )
{
    uint32_t ulSocketNumber = ( uint32_t ) xSocket; /*lint !e923 cast required for portability. */
    SSocketContext_t * pxSecureSocket;
    int32_t lRetVal = SOCKETS_ERROR_NONE;

    /* Ensure that a valid socket was passed. */
    if(( prvIsValidSocket( ulSocketNumber ) == pdTRUE )  && (pxAddress != NULL) && (pxAddress->usPort != 0))
    {
		/* Shortcut for easy access. */
		pxSecureSocket = &( xSockets[ ulSocketNumber ] );

		/* Obtain the socketInUse mutex. */
		if (xSemaphoreTake( pxSecureSocket->xUcInUse, xMaxSemaphoreBlockTime) == pdTRUE)
		{
			if((xDisconnectAlert == pdTRUE) && (xIsWiFiInitialized == pdTRUE))
			{
				xDisconnectAlert = pdFALSE;
				WIFI_ConnectAP(&pxLastNetworkParams);
			}
			/* Check that the socket is not already connected. */
			if( ( pxSecureSocket->ulFlags & securesocketsSOCKET_IS_CONNECTED ) != 0UL )
			{
				/* Connect attempted on an already connected socket. */
				lRetVal = SOCKETS_EISCONN;
			}

			if( lRetVal == SOCKETS_ERROR_NONE )
			{
				char host[20];
				/* Store server certificate if we are using offload SSL
				 * and the socket is a secure socket. */
#ifdef USE_OFFLOAD_SSL
				if( ( pxSecureSocket->ulFlags & securesocketsSOCKET_SECURE_FLAG ) != 0UL )
				{
					if (pxSecureSocket->pcDestination != NULL)
					{
						if(0 == memcmp(pxSecureSocket->pcDestination, clientcredentialMQTT_BROKER_ENDPOINT, sizeof(clientcredentialMQTT_BROKER_ENDPOINT)))
						{
							lRetVal = SOCKETS_ERROR_NONE;
						}
						else
						{
							lRetVal = SOCKETS_SOCKET_ERROR;
						}
					}
					if (lRetVal == SOCKETS_ERROR_NONE)
					{
						/* Store the default certificate. */
						uint8_t * pemBuffer = NULL;
						uint32_t pemLength = 0;
						if (pxSecureSocket->pcServerCertificate == NULL)
						{
							pemBuffer = (uint8_t *)allocPkiBinImg((unsigned char*)tlsSTARFIELD_ROOT_CERTIFICATE_PEM, tlsSTARFIELD_ROOT_CERTIFICATE_LENGTH, PKI_TYPE_CA, &pemLength);
							rootCAwritten++;
						}
						else
						{
							pemBuffer = (uint8_t *)allocPkiBinImg((unsigned char*)pxSecureSocket->pcServerCertificate, pxSecureSocket->ulServerCertificateLength, PKI_TYPE_CA, &pemLength);
							rootCAwritten = 0;
						}

						if (rootCAwritten < 2)
						{
							if( esp_StoreCert(pemBuffer, PKI_TYPE_CA, pemLength) == espOK )
							{
								/* Certificate stored successfully. */
								lRetVal = SOCKETS_ERROR_NONE;
							}
							else
							{
								/* Failed to store certificate. */
								lRetVal = SOCKETS_SOCKET_ERROR;
							}
						}

						vPortFree( pemBuffer );

						pxSecureSocket->xSocketType = ESP_NETCONN_TYPE_SSL;

						if (pxSecureSocket->xSocket)
						{
							esp_set_type(pxSecureSocket->xSocket, pxSecureSocket->xSocketType);
						}
						else
						{
							lRetVal = SOCKETS_SOCKET_ERROR;
						}
					}
				}
#endif /* USE_OFFLOAD_SSL */
				if( lRetVal == SOCKETS_ERROR_NONE )
				{
#ifdef USE_OFFLOAD_SSL
					if( ( pxSecureSocket->ulFlags & securesocketsSOCKET_SECURE_FLAG ) != 0UL )
					{
						esp_datetime_t dt = { 0 };
						while(dt.year <= 1970)
						{
							esp_sntp_gettime(&dt, NULL, NULL, 1);
							vTaskDelay(500);
						}
					}
#endif /* USE_OFFLOAD_SSL */

					SOCKETS_inet_ntoa(pxAddress->ulAddress, host);

					if (esp_sta_has_ip())
					{
						espr_t conn_status = espOK;
#ifdef USE_OFFLOAD_SSL
						if( ( pxSecureSocket->ulFlags & securesocketsSOCKET_SECURE_FLAG ) != 0UL )
						{
							conn_status = esp_conn_set_ssl_conf((ESP_CFG_MAX_CONNS - ulSocketNumber - 1), 3, 0, 0, 1);
						}
	#endif /* USE_OFFLOAD_SSL */
						if (conn_status == espOK)
						{
							conn_status = esp_netconn_connect((esp_netconn_p)(pxSecureSocket->xSocket), host, SOCKETS_ntohs(pxAddress->usPort));
						}

						if (conn_status == espOK)
						{
							pxSecureSocket->ulFlags |= securesocketsSOCKET_IS_CONNECTED;
							lRetVal = SOCKETS_ERROR_NONE;
						}
						else
						{
							/* Connection failed. */
							lRetVal = SOCKETS_ENOTCONN;
						}
					}
					else
					{
						lRetVal = SOCKETS_ENOTCONN;
					}
				}
			}
			else
			{
				/* Could not acquire semaphore. */
				lRetVal = SOCKETS_EISCONN;
			}
			/* Give back the socketInUse mutex. */
			xSemaphoreGive(pxSecureSocket->xUcInUse);
		}
    }
    else
    {
        /* Invalid socket handle was passed. */
        lRetVal = SOCKETS_EINVAL;
    }

    /* TLS initialization is needed only if we are not using offload SSL. */
#ifndef USE_OFFLOAD_SSL
    /* Negotiate TLS if requested. */
    if ((lRetVal == SOCKETS_ERROR_NONE) && ((pxSecureSocket->ulFlags & securesocketsSOCKET_SECURE_FLAG) != 0))
    {
		TLSParams_t xTLSParams = { 0 };
		xTLSParams.ulSize = sizeof( xTLSParams );
		xTLSParams.pcDestination = pxSecureSocket->pcDestination;
		xTLSParams.pcServerCertificate = pxSecureSocket->pcServerCertificate;
		xTLSParams.ulServerCertificateLength = pxSecureSocket->ulServerCertificateLength;
		xTLSParams.ppcAlpnProtocols = ( const char ** ) pxSecureSocket->ppcAlpnProtocols;
		xTLSParams.ulAlpnProtocolsCount = pxSecureSocket->ulAlpnProtocolsCount;
		xTLSParams.pvCallerContext = pxSecureSocket;
		xTLSParams.pxNetworkRecv = prvNetworkRecv;
		xTLSParams.pxNetworkSend = prvNetworkSend;

		/* Initialize TLS. */
		if (TLS_Init(&(pxSecureSocket->pvTLSContext), &(xTLSParams)) == pdFREERTOS_ERRNO_NONE)
		{
			if (xSemaphoreTake(xTLSConnect, xMaxSemaphoreBlockTime) == pdTRUE)
			{
				/* Initiate TLS handshake. */
				if (TLS_Connect(pxSecureSocket->pvTLSContext) != pdFREERTOS_ERRNO_NONE)
				{
					/* TLS handshake failed. */
					esp_netconn_close(pxSecureSocket->xSocket);
					pxSecureSocket->ulFlags &= ~securesocketsSOCKET_IS_CONNECTED;
					TLS_Cleanup(pxSecureSocket->pvTLSContext);
					pxSecureSocket->pvTLSContext = NULL;
					lRetVal = SOCKETS_TLS_HANDSHAKE_ERROR;
				}
				/* Release semaphore */
				xSemaphoreGive(xTLSConnect);
			}
		}
		else
		{
			/* TLS Initialization failed. */
			lRetVal = SOCKETS_TLS_INIT_ERROR;
		}
    }
#endif /* USE_OFFLOAD_SSL*/

    return lRetVal;
}
/*-----------------------------------------------------------*/

int32_t SOCKETS_Recv( Socket_t xSocket,
                      void * pvBuffer,
                      size_t xBufferLength,
                      uint32_t ulFlags )
{
	uint32_t ulSocketNumber = (uint32_t)xSocket;
	SSocketContext_t * pxContext;
    int32_t lReceivedBytes = 0;

    /* Remove warning about unused parameters. */
    ( void ) ulFlags;

	/* Ensure that a valid socket was passed and the
	* passed buffer is not NULL. */
	if ((prvIsValidSocket(ulSocketNumber) == pdTRUE) && (pvBuffer != NULL))
	{
		pxContext = &( xSockets[ulSocketNumber] );

		/* Obtain the socketInUse mutex. */
		if (xSemaphoreTake( pxContext->xRecieveUcInUse, xMaxSemaphoreBlockTime) == pdTRUE)
		{
			if ((pxContext->ulFlags & securesocketsSOCKET_IS_CONNECTED) != 0UL)
			{
				/* Check that receive is allowed on the socket. */
				if ((pxContext->ulFlags & securesocketsSOCKET_READ_CLOSED_FLAG ) == 0UL)
				{
	#ifndef USE_OFFLOAD_SSL
					if ((pxContext->ulFlags & securesocketsSOCKET_SECURE_FLAG) != 0UL)
					{
						/* Receive through TLS pipe, if negotiated. */
						lReceivedBytes = TLS_Recv(pxContext->pvTLSContext, pvBuffer, xBufferLength);

						/* Convert the error code. */
						if (lReceivedBytes < 0)
						{
							/* TLS_Recv failed. */
							lReceivedBytes = SOCKETS_TLS_RECV_ERROR;
						}
					}
					else
					{
						/* Receive un-encrypted. */
						lReceivedBytes = prvNetworkRecv(pxContext, pvBuffer, xBufferLength);
					}
	#else /* USE_OFFLOAD_SSL */
					/* Always receive using prvNetworkRecv if using offload SSL. */
					lReceivedBytes = prvNetworkRecv( xSocket, pvBuffer, xBufferLength );
	#endif /* USE_OFFLOAD_SSL */
				}
				else
				{
					/* The socket has been closed for read. */
					lReceivedBytes = SOCKETS_ECLOSED;
				}
			}
			else
			{
				/* The socket has been closed for read. */
				lReceivedBytes = SOCKETS_ECLOSED;
			}
			/* Give back the socketInUse mutex. */
			xSemaphoreGive(pxContext->xRecieveUcInUse);
		}
	}
	else
	{
		lReceivedBytes = SOCKETS_EINVAL;
	}

	return lReceivedBytes;
}
/*-----------------------------------------------------------*/

int32_t SOCKETS_Send( Socket_t xSocket,
                      const void * pvBuffer,
                      size_t xDataLength,
                      uint32_t ulFlags )
{
	uint32_t ulSocketNumber = (uint32_t)xSocket;
	SSocketContext_t * pxContext;
    int32_t lSentBytes = 0;

	/* Remove warning about unused parameters. */
	(void)ulFlags;

	if ((prvIsValidSocket(ulSocketNumber) == pdTRUE) && (pvBuffer != NULL))
	{
		pxContext = &( xSockets[ulSocketNumber] );

		/* Obtain the socketInUse mutex. */
		if (xSemaphoreTake( pxContext->xUcInUse, xMaxSemaphoreBlockTime) == pdTRUE)
		{
			if ((pxContext->ulFlags & securesocketsSOCKET_IS_CONNECTED) != 0UL)
			{
				/* Check that send is allowed on the socket. */
				if ((pxContext->ulFlags & securesocketsSOCKET_WRITE_CLOSED_FLAG ) == 0UL)
				{
	#ifndef USE_OFFLOAD_SSL
					if ((pxContext->ulFlags & securesocketsSOCKET_SECURE_FLAG) != 0UL)
					{
						/* Send through TLS pipe, if negotiated. */
						lSentBytes = TLS_Send(pxContext->pvTLSContext, pvBuffer, xDataLength);

						/* Convert the error code. */
						if (lSentBytes < 0)
						{
							/* TLS_Recv failed. */
							lSentBytes = SOCKETS_TLS_SEND_ERROR;
						}
					}
					else
					{
						/* Send unencrypted. */
						lSentBytes = prvNetworkSend(pxContext, pvBuffer, xDataLength);
					}
	#else /* USE_OFFLOAD_SSL */
					/* Always send using prvNetworkSend if using offload SSL. */
					lSentBytes = prvNetworkSend( xSocket, pvBuffer, xDataLength );
	#endif /* USE_OFFLOAD_SSL */
				}
				else
				{
					/* The socket has been closed for write. */
					lSentBytes = SOCKETS_ECLOSED;
				}
			}
			else
			{
			/* The supplied socket is closed. */
				lSentBytes =  SOCKETS_ECLOSED;
			}
			/* Give back the socketInUse mutex. */
			xSemaphoreGive(pxContext->xUcInUse);
		}
	}
	else
	{
		lSentBytes =  SOCKETS_EINVAL;
	}

	return lSentBytes;
}
/*-----------------------------------------------------------*/

int32_t SOCKETS_Shutdown( Socket_t xSocket,
                          uint32_t ulHow )
{
	uint32_t ulSocketNumber = ( uint32_t ) xSocket;
	SSocketContext_t * pxContext;
    int32_t lRetVal = SOCKETS_SOCKET_ERROR;

	/* Ensure that a valid socket was passed. */
	if (prvIsValidSocket(ulSocketNumber) == pdTRUE)
	{
		pxContext = &( xSockets[ulSocketNumber] );

		/* Obtain the socketInUse mutex. */
		if (xSemaphoreTake( pxContext->xUcInUse, xMaxSemaphoreBlockTime) == pdTRUE)
		{
			switch( ulHow )
			{
				case SOCKETS_SHUT_RD:
					/* Further receive calls on this socket should return error. */
					pxContext->ulFlags |= securesocketsSOCKET_READ_CLOSED_FLAG;

					/* Return success to the user. */
					lRetVal = SOCKETS_ERROR_NONE;
					break;

				case SOCKETS_SHUT_WR:
					/* Further send calls on this socket should return error. */
					pxContext->ulFlags |= securesocketsSOCKET_WRITE_CLOSED_FLAG;

					/* Return success to the user. */
					lRetVal = SOCKETS_ERROR_NONE;
					break;

				case SOCKETS_SHUT_RDWR:
					/* Further send or receive calls on this socket should return error. */
					pxContext->ulFlags |= securesocketsSOCKET_READ_CLOSED_FLAG;
					pxContext->ulFlags |= securesocketsSOCKET_WRITE_CLOSED_FLAG;

					/* Return success to the user. */
					lRetVal = SOCKETS_ERROR_NONE;
					break;

				default:
					/* An invalid value was passed for ulHow. */
					lRetVal = SOCKETS_EINVAL;
					break;
			}
			/* Give back the socketInUse mutex. */
			xSemaphoreGive(pxContext->xUcInUse);
		}
	}
	else
	{
		lRetVal = SOCKETS_EINVAL;
	}

	return lRetVal;
}
/*-----------------------------------------------------------*/

int32_t SOCKETS_Close( Socket_t xSocket )
{
	int32_t lRetVal = SOCKETS_SOCKET_ERROR;
	uint32_t ulSocketNumber = (uint32_t)xSocket;
	SSocketContext_t * pxContext;

	/* Ensure that a valid socket was passed. */
	if (prvIsValidSocket(ulSocketNumber) == pdTRUE)
	{
		pxContext = &( xSockets[ulSocketNumber] );

		/* Obtain the socketInUse mutex. */
		if (xSemaphoreTake( pxContext->xUcInUse, xMaxSemaphoreBlockTime) == pdTRUE)
		{
			pxContext->ulFlags |= securesocketsSOCKET_READ_CLOSED_FLAG;
			pxContext->ulFlags |= securesocketsSOCKET_WRITE_CLOSED_FLAG;

			/* Clean-up destination string. */
			if (pxContext->pcDestination != NULL)
			{
				vPortFree(pxContext->pcDestination);
			}

			/* Clean-up server certificate. */
			if (pxContext->pcServerCertificate != NULL)
			{
				vPortFree(pxContext->pcServerCertificate);
			}

			/* Clean-up application protocol array. */
			if (pxContext->ppcAlpnProtocols != NULL)
			{
				for (uint32_t ulProtocol = 0; ulProtocol < pxContext->ulAlpnProtocolsCount; ulProtocol++)
				{
					if (pxContext->ppcAlpnProtocols[ ulProtocol ] != NULL)
					{
						vPortFree( pxContext->ppcAlpnProtocols[ ulProtocol ] );
					}
				}

				vPortFree(pxContext->ppcAlpnProtocols);
			}

			#ifndef USE_OFFLOAD_SSL
			/* Clean-up TLS context. */
			if ((pxContext->ulFlags & securesocketsSOCKET_SECURE_FLAG) != 0UL)
			{
				TLS_Cleanup(pxContext->pvTLSContext);
			}
			#endif /* USE_OFFLOAD_SSL */

			if ((pxContext->ulFlags & securesocketsSOCKET_IS_CONNECTED) != 0UL)
			{
				pxContext->ulFlags &= ~securesocketsSOCKET_IS_CONNECTED;
				lRetVal = SOCKETS_ERROR_NONE;
			}
			else
			{
				lRetVal = SOCKETS_ERROR_NONE;
			}

			esp_netconn_close(pxContext->xSocket);
			esp_netconn_delete(pxContext->xSocket);

			/* Return the socket back to the free socket pool. */
			if (prvReturnSocket(ulSocketNumber) != pdTRUE)
			{
				lRetVal = SOCKETS_SOCKET_ERROR;
			}
			/* Give back the socketInUse mutex. */
			xSemaphoreGive(pxContext->xUcInUse);
		}
	}
	else
	{
		lRetVal = SOCKETS_EINVAL;
	}

	return lRetVal;
}
/*-----------------------------------------------------------*/

int32_t SOCKETS_SetSockOpt( Socket_t xSocket,
                            int32_t lLevel,
                            int32_t lOptionName,
                            const void * pvOptionValue,
                            size_t xOptionLength )
{
	int32_t lRetCode = SOCKETS_ERROR_NONE;
	uint32_t ulSocketNumber = ( uint32_t ) xSocket;
	TickType_t xTimeout;
	SSocketContext_t * pxContext;

	/* Remove compiler warnings about unused parameters. */
	(void)lLevel;

	/* Ensure that a valid socket was passed. */
	if (prvIsValidSocket(ulSocketNumber) == pdTRUE)
	{
		pxContext = &( xSockets[ulSocketNumber] );

		/* Obtain the socketInUse mutex. */
		if (xSemaphoreTake( pxContext->xUcInUse, xMaxSemaphoreBlockTime) == pdTRUE)
		{
			switch( lOptionName )
			{
				case SOCKETS_SO_SERVER_NAME_INDICATION:
					if ((pxContext->ulFlags & securesocketsSOCKET_IS_CONNECTED) == 0)
					{
						/* Non-NULL destination string indicates that SNI extension should
						* be used during TLS negotiation. */
						pxContext->pcDestination = ( char * ) pvPortMalloc( 1U + xOptionLength );

						if (pxContext->pcDestination == NULL)
						{
							lRetCode = SOCKETS_ENOMEM;
						}
						else
						{
							if (pvOptionValue != NULL)
							{
								memcpy(pxContext->pcDestination, pvOptionValue, xOptionLength );
								pxContext->pcDestination[ xOptionLength ] = '\0';
							}
							else
							{
								lRetCode = SOCKETS_EINVAL;
							}
						}
					}
					else
					{
						lRetCode = SOCKETS_EISCONN;
					}

				break;

				case SOCKETS_SO_TRUSTED_SERVER_CERTIFICATE:
					if ((pxContext->ulFlags & securesocketsSOCKET_IS_CONNECTED) == 0)
					{
						/* Non-NULL server certificate field indicates that the default trust
						* list should not be used. */
						pxContext->pcServerCertificate = ( char * ) pvPortMalloc( xOptionLength );

						if (pxContext->pcServerCertificate == NULL)
						{
							lRetCode = SOCKETS_ENOMEM;
						}
						else
						{
							if (pvOptionValue != NULL)
							{
								memcpy( pxContext->pcServerCertificate, pvOptionValue, xOptionLength );
								pxContext->ulServerCertificateLength = xOptionLength;
							}
							else
							{
								lRetCode = SOCKETS_EINVAL;
							}
						}
					}
					else
					{
						lRetCode = SOCKETS_EISCONN;
					}

				break;

				case SOCKETS_SO_REQUIRE_TLS:
					if ((pxContext->ulFlags & securesocketsSOCKET_IS_CONNECTED) == 0)
					{
						pxContext->ulFlags |= securesocketsSOCKET_SECURE_FLAG;
	#ifdef USE_OFFLOAD_SSL
						/* Set the socket type to SSL to use offload SSL. */
						pxContext->xSocketType = ESP_NETCONN_TYPE_SSL;
						if (pxContext->xSocket)
						{
							esp_set_type(pxContext->xSocket, pxContext->xSocketType);
						}
						else
						{
							lRetCode = SOCKETS_SOCKET_ERROR;
						}
	#endif /* USE_OFFLOAD_SSL */
					}
					else
					{
						/* Do not set the ALPN option if the socket is already connected. */
						lRetCode = SOCKETS_EISCONN;
					}
				break;

				case SOCKETS_SO_ALPN_PROTOCOLS:
					if ((pxContext->ulFlags & securesocketsSOCKET_IS_CONNECTED) == 0)
					{
						/* Allocate a sufficiently long array of pointers. */
						pxContext->ulAlpnProtocolsCount = 1 + xOptionLength;

						if (NULL == (pxContext->ppcAlpnProtocols = (char **)pvPortMalloc(pxContext->ulAlpnProtocolsCount * sizeof(char *))))
						{
							lRetCode = SOCKETS_ENOMEM;
						}
						else
						{
							pxContext->ppcAlpnProtocols[pxContext->ulAlpnProtocolsCount - 1 ] = NULL;
						}

						/* Copy each protocol string. */
						for (uint32_t ulProtocol = 0; (ulProtocol < pxContext->ulAlpnProtocolsCount - 1) && (lRetCode == pdFREERTOS_ERRNO_NONE); ulProtocol++)
						{
							char ** ppcAlpnIn = ( char ** ) pvOptionValue;
							size_t xLength = strlen(ppcAlpnIn[ ulProtocol ]);

							if ((pxContext->ppcAlpnProtocols[ ulProtocol ] = (char *)pvPortMalloc(1 + xLength)) == NULL)
							{
								lRetCode = SOCKETS_ENOMEM;
							}
							else
							{
								memcpy( pxContext->ppcAlpnProtocols[ ulProtocol ], ppcAlpnIn[ ulProtocol ], xLength );
								pxContext->ppcAlpnProtocols[ ulProtocol ][ xLength ] = '\0';
							}
						}
					}
					else
					{
						/* Do not set the ALPN option if the socket is already connected. */
						lRetCode = SOCKETS_EISCONN;
					}

				break;

				case SOCKETS_SO_SNDTIMEO:
				break;

				case SOCKETS_SO_RCVTIMEO:
					if (pvOptionValue != NULL)
					{
						xTimeout = *( ( const TickType_t * ) pvOptionValue ); /*lint !e9087 pvOptionValue passed should be of TickType_t. */
						esp_netconn_set_receive_timeout(pxContext->xSocket, xTimeout);
					}
					else
					{
						lRetCode = SOCKETS_EINVAL;
					}

				break;

				case SOCKETS_SO_NONBLOCK:
					if ((pxContext->ulFlags & securesocketsSOCKET_IS_CONNECTED ) != 0)
					{
						/* Set the timeouts to the smallest value possible.
						* This isn't true nonblocking, but as close as we can get. */
						esp_netconn_set_receive_timeout(pxContext->xSocket, 1);
					}
					else
					{
						lRetCode = SOCKETS_EISCONN;
					}
				break;

				default:
					lRetCode = SOCKETS_EINVAL;
				break;
			}
			/* Give back the socketInUse mutex. */
			xSemaphoreGive(pxContext->xUcInUse);
		}
	}
	else
	{
		lRetCode = SOCKETS_EINVAL;
	}

	return lRetCode;
}
/*-----------------------------------------------------------*/

uint32_t SOCKETS_GetHostByName(const char * pcHostName)
{
	if (pcHostName != NULL)
	{
		uint32_t ulIPAddres;
		if (WIFI_GetHostIP((char * )pcHostName, ( uint8_t *)&(ulIPAddres)) == eWiFiSuccess)
		{
			return ulIPAddres;
		}
	}

	return 0;
}
/*-----------------------------------------------------------*/

BaseType_t SOCKETS_Init( void )
{
	uint32_t ulIndex;

	/* Mark all the sockets as free */
	for (ulIndex = 0 ; ulIndex < (uint32_t)ESP_CFG_MAX_CONNS ; ulIndex++)
	{
		xSockets[ ulIndex ].ucInUse = 0;
	}

//	/* Create the global mutex which is used to ensure
//	* that only one socket is accessing the ucInUse bits in
//	* the socket array. */
	for (ulIndex = 0; ulIndex <  (uint32_t)ESP_CFG_MAX_CONNS; ++ulIndex)
	{
		xSockets[ ulIndex ].xUcInUse = xSemaphoreCreateMutex();
		xSockets[ ulIndex ].xRecieveUcInUse = xSemaphoreCreateMutex();
		if (xSockets[ ulIndex ].xUcInUse == NULL || xSockets[ ulIndex ].xRecieveUcInUse == NULL)
		{
			return pdFAIL;
		}
	}

	return pdPASS;
}
/*-----------------------------------------------------------*/

static CK_SESSION_HANDLE xPkcs11Session = 0;
static CK_FUNCTION_LIST_PTR pxPkcs11FunctionList = NULL;

uint32_t ulRand( void )
{
    CK_RV xResult = 0;
    CK_C_GetFunctionList pxCkGetFunctionList = NULL;
    CK_ULONG ulCount = 1;
    uint32_t ulRandomValue = 0;
    CK_SLOT_ID xSlotId = 0;

    portENTER_CRITICAL();

    if( 0 == xPkcs11Session )
    {
        /* One-time initialization. */

        /* Ensure that the PKCS#11 module is initialized. */
        if( 0 == xResult )
        {
            pxCkGetFunctionList = C_GetFunctionList;
            xResult = pxCkGetFunctionList( &pxPkcs11FunctionList );
        }

        if( 0 == xResult )
        {
            xResult = pxPkcs11FunctionList->C_Initialize( NULL );
        }

        /* Get the default slot ID. */
        if( 0 == xResult )
        {
            xResult = pxPkcs11FunctionList->C_GetSlotList(
                CK_TRUE,
                &xSlotId,
                &ulCount );
        }

        /* Start a session with the PKCS#11 module. */
        if( 0 == xResult )
        {
            xResult = pxPkcs11FunctionList->C_OpenSession(
                xSlotId,
                CKF_SERIAL_SESSION,
                NULL,
                NULL,
                &xPkcs11Session );
        }
    }

    if( 0 == xResult )
    {
        /* Request a sequence of cryptographically random byte values using
         * PKCS#11. */
        xResult = pxPkcs11FunctionList->C_GenerateRandom( xPkcs11Session,
                                                          ( CK_BYTE_PTR ) &ulRandomValue,
                                                          sizeof( ulRandomValue ) );
    }

    portEXIT_CRITICAL();

    /* Check if any of the API calls failed. */
    if( 0 != xResult )
    {
        ulRandomValue = 0;
    }

    return ulRandomValue;
}
/*-----------------------------------------------------------*/
