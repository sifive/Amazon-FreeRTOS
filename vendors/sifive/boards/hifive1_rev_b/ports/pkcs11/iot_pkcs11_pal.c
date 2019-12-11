/*
 * Amazon FreeRTOS PKCS #11 PAL for SiFive Learn Inventor V1.0.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Copyright (c) 2019, SiFive, Inc.
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification,are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holders nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://aws.amazon.com/freertos
 * http://www.FreeRTOS.org
 */


/**
 * @file iot_pkcs11_pal.c
 * @brief Amazon FreeRTOS device specific helper functions for
 * PKCS#11 implementation based on mbedTLS.  This
 * file deviates from the FreeRTOS style standard for some function names and
 * data types in order to maintain compliance with the PKCS#11 standard.
 */

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "FreeRTOSIPConfig.h"
#include "task.h"
#include "iot_pkcs11.h"
#include "iot_pkcs11_config.h"
#include "aws_clientcredential_keys.h"
#include "iot_default_root_certificates.h"

#include "hal_flash.h"

/* C runtime includes. */
#include <stdio.h>
#include <string.h>

/* WiFi includes. */
#ifdef USE_OFFLOAD_SSL
    #include "iot_wifi.h"
/* mbedTLS includes. */
    #include "mbedtls/pk.h"
    #include "mbedtls/base64.h"
    #include "mbedtls/platform.h"

/* The maximum lengths (in bytes) that the Inventek
 * module has for credential storage. */
    #define pkcs11ROOT_CA_MAX_LENGTH            2048
    #define pkcs11CLIENT_CERT_MAX_LENGTH        2048
    #define pkcs11CLIENT_PRIV_KEY_MAX_LENGTH    2048
#endif /* ifdef USE_OFFLOAD_SSL */

#define pkcs11OBJECT_MAX_SIZE                   4096
#define pkcs11OBJECT_PRESENT_MAGIC              ( 0xABCD0000uL )
#define pkcs11OBJECT_LENGTH_MASK                ( 0x0000FFFFuL )
#define pkcs11OBJECT_PRESENT_MASK               ( 0xFFFF0000uL )

enum eObjectHandles
{
    eInvalidHandle = 0, /* From PKCS #11 spec: 0 is never a valid object handle.*/
    eAwsDevicePrivateKey = 1,
    eAwsDevicePublicKey,
    eAwsDeviceCertificate,
    eAwsCodeSigningKey,
	eAwsJITPCertificate
};

/**
 * @brief Structure for certificates/key storage.
 */
typedef struct
{
    CK_CHAR cDeviceCertificate[ pkcs11OBJECT_MAX_SIZE ];
    CK_CHAR cDeviceKey[ pkcs11OBJECT_MAX_SIZE ];
    CK_CHAR cCodeSignKey[ pkcs11OBJECT_MAX_SIZE ];
    CK_CHAR cDevicePubKey[ pkcs11OBJECT_MAX_SIZE ];
    uint32_t ulDeviceCertificateMark;
    uint32_t ulDeviceKeyMark;
    uint32_t ulCodeSignKeyMark;
    uint32_t uDevicePubKeyMark;
} P11KeyConfig_t;


/**
 * @brief Certificates/key storage in flash.
 */
P11KeyConfig_t P11KeyConfig __attribute__( ( section( "UNINIT_FIXED_LOC" ) ) );
const uint32_t sectionSize = 0x5000;	// from linker script
/*-----------------------------------------------------------*/

extern const struct hal_flash fe310_flash_dev;

static uint8_t tempBuf[32] = {0};
int FLASH_update(uint32_t dst_addr, const void *data, uint32_t size)
{
	uint32_t addr = dst_addr - fe310_flash_dev.hf_base_addr;

	//If last sector clear only once
	if (addr >= (fe310_flash_dev.hf_size - fe310_flash_dev.hf_sector_size))
	{
		uint32_t addrLastSector = fe310_flash_dev.hf_size - fe310_flash_dev.hf_sector_size;
		memset(tempBuf, 0, sizeof(tempBuf));

		//read last sector
		fe310_flash_read(addrLastSector, tempBuf, sizeof(tempBuf));

		//erase last sector
		fe310_flash_erase_sector(addrLastSector);

		//change data
		memcpy(&tempBuf[addr - addrLastSector], data, size);

		//write data
		fe310_flash_write(addrLastSector, tempBuf, sizeof(tempBuf));
	}
	else
	{
		fe310_flash_erase_sector(addr);
		fe310_flash_write(addr, data, size);
	}

	return size;
}

#ifdef USE_OFFLOAD_SSL
	#define PKI_MAGIC_KEYVAL 0xF1F1

	#pragma pack(1)
	typedef struct
	{
		uint16_t magicKey;
		uint16_t listSz;
		uint32_t len;
	} PkiHeader;

	typedef struct
	{
		uint8_t type;
		uint8_t id;
		uint16_t len;
		//data
		//4B align
	} PkiItemHeader;
	#pragma pack()

unsigned char* pemShiftBeg(void* pem)
{
	uint8_t* p = (uint8_t*)pem;

	//Shifting pointer to the usefull data
	if (p)
	{
		while(*p != '\n')
			++p;
		return ++p;
	}

	return NULL;
}

uint32_t pemGetLen(unsigned char* pemUsefull)
{
	uint32_t len = 0;

	if (pemUsefull)
	{
		while(*pemUsefull != '-')
		{
			++pemUsefull;
			++len;
		}

		while(*pemUsefull != '\n')
		{
			++pemUsefull;
			++len;
		}
	}

	return len;
}

void* allocPkiBinImg(unsigned char* pdata, uint32_t dataSize, uint8_t type, uint32_t* PRetLen)
{
	unsigned char* pdataRaw = pemShiftBeg(pdata);
	uint32_t len = pemGetLen(pdataRaw) + (pdataRaw - pdata), len2Aligned;

	if (len > dataSize)
	{
		len = dataSize;
	}

	len2Aligned = sizeof(PkiItemHeader) + len;
	//Adding 4B align
	if (len2Aligned & 3)
		len2Aligned += sizeof(uint32_t) - (len2Aligned & 3);

	PkiHeader* ph = (PkiHeader*)pvPortMalloc(len2Aligned + sizeof(PkiHeader));
	if (PRetLen)
		*PRetLen = len2Aligned + sizeof(PkiHeader);

	if (ph)
	{
		ph->magicKey = PKI_MAGIC_KEYVAL;
		ph->listSz = 1;
		ph->len = 0;

		PkiItemHeader* ph2 = (PkiItemHeader*)(ph + 1);
		ph2->type = type;
		ph2->id = 0;

		ph2->len = len;
		ph->len += len2Aligned;

		uint8_t* pdata2 = (uint8_t*)(ph2 + 1);
		memcpy(pdata2, pdata, len);
	}

	return ph;
}
#endif /* ifdef USE_OFFLOAD_SSL */
/*-----------------------------------------------------------*/

/**
 * @brief Saves an object in non-volatile storage.
 *
 * Port-specific file write for cryptographic information.
 *
 * @param[in] pxLabel       The label of the object to be stored.
 * @param[in] pucData       The object data to be saved
 * @param[in] pulDataSize   Size (in bytes) of object data.
 *
 * @return The object handle if successful.
 * eInvalidHandle = 0 if unsuccessful.
 */
CK_OBJECT_HANDLE PKCS11_PAL_SaveObject( CK_ATTRIBUTE_PTR pxLabel,
                                        uint8_t * pucData,
                                        uint32_t ulDataSize )

{
    CK_OBJECT_HANDLE xHandle = eInvalidHandle;
    CK_RV xBytesWritten = 0;
    CK_RV xReturn = CKR_OK;
    uint32_t ulFlashMark = ( pkcs11OBJECT_PRESENT_MAGIC | ( ulDataSize ) );
    uint8_t * pemBuffer = NULL;
    uint32_t pemLength = 0;
    CK_BBOOL xIsDestroy = CK_TRUE;
    int temp;

    #ifdef USE_OFFLOAD_SSL
        for( int i = 0; i < ulDataSize; i++ )
        {
            if( pucData[ i ] != 0 )
            {
                xIsDestroy = CK_FALSE;
                break;
            }
        }
    #endif

    if( ulDataSize <= pkcs11OBJECT_MAX_SIZE )
    {
        /*
         * write client certificate.
         */
        if( strcmp( pxLabel->pValue,
                    pkcs11configLABEL_DEVICE_CERTIFICATE_FOR_TLS ) == 0 )
        {
            xBytesWritten = FLASH_update( ( uint32_t ) P11KeyConfig.cDeviceCertificate,
                                          pucData,
                                          ( ulDataSize ) );

            if( xBytesWritten == ( ulDataSize ) )
            {
                xHandle = eAwsDeviceCertificate;

                /*change flash written mark'*/
                FLASH_update( ( uint32_t ) &P11KeyConfig.ulDeviceCertificateMark,
                              &ulFlashMark,
                              sizeof( uint32_t ) );
            }

            #ifdef USE_OFFLOAD_SSL

                /* If we are using offload SSL, write the certificate to the
                 * WiFi module as well. */
                if( xIsDestroy == CK_FALSE )
                {
			    	pemBuffer = allocPkiBinImg((unsigned char*)keyCLIENT_CERTIFICATE_PEM, sizeof(keyCLIENT_CERTIFICATE_PEM), PKI_TYPE_CERT, &pemLength);
                }
                else
                {
                    pemBuffer = pvPortMalloc( pkcs11CLIENT_CERT_MAX_LENGTH );

                    if( pemBuffer == NULL )
                    {
                        xReturn = CKR_HOST_MEMORY;
                    }
                    else
                    {
                        memset( pemBuffer, 0x00, pkcs11CLIENT_CERT_MAX_LENGTH );
                        pemLength = pkcs11CLIENT_CERT_MAX_LENGTH;
                    }
                }

                if( xReturn != CKR_OK )
                {
                    xHandle = eInvalidHandle;
                }

                if( xHandle == eAwsDeviceCertificate )
                {
                	if( ( temp = esp_StoreCert( pemBuffer, PKI_TYPE_CERT, pemLength ) ) != eWiFiSuccess )
                    {
                        xHandle = eInvalidHandle;
                    }
                }

                if( xReturn == CKR_OK )
                {
                    vPortFree( pemBuffer );
                }
            #endif /* USE_OFFLOAD_SSL */
        }

        /*
         * write client key.
         */

        else if( strcmp( pxLabel->pValue,
                         pkcs11configLABEL_DEVICE_PRIVATE_KEY_FOR_TLS ) == 0 )
        {
            xBytesWritten = FLASH_update( ( uint32_t ) P11KeyConfig.cDeviceKey,
                                          pucData,
                                          ulDataSize );

            if( xBytesWritten == ( ulDataSize ) )
            {
                xHandle = eAwsDevicePrivateKey;
                /*change flash written mark'*/
                FLASH_update( ( uint32_t ) &P11KeyConfig.ulDeviceKeyMark,
                              &ulFlashMark,
                              sizeof( uint32_t ) );
            }

            #ifdef USE_OFFLOAD_SSL

                /* If we are using offload SSL, write the private key to the
                 * WiFi module as well. */
                if( xIsDestroy == CK_FALSE )
                {
                	pemBuffer = allocPkiBinImg((unsigned char*)keyCLIENT_PRIVATE_KEY_PEM, sizeof(keyCLIENT_PRIVATE_KEY_PEM), PKI_TYPE_KEY, &pemLength);
                }
                else
                {
                    pemBuffer = pvPortMalloc( pkcs11CLIENT_PRIV_KEY_MAX_LENGTH );

                    if( pemBuffer == NULL )
                    {
                        xReturn = CKR_HOST_MEMORY;
                    }
                    else
                    {
                        memset( pemBuffer, 0x00, pkcs11CLIENT_PRIV_KEY_MAX_LENGTH );
                        pemLength = pkcs11CLIENT_PRIV_KEY_MAX_LENGTH;
                    }
                }

                if( xReturn != CKR_OK )
                {
                    xHandle = eInvalidHandle;
                }

                if( xHandle == eAwsDevicePrivateKey )
                {
					if( ( temp = esp_StoreCert( pemBuffer, PKI_TYPE_KEY, pemLength ) ) != eWiFiSuccess )
                    {
                        xHandle = eInvalidHandle;
                    }
                }

                if( xReturn == CKR_OK )
                {
                    vPortFree( pemBuffer );
                }
            #endif /* USE_OFFLOAD_SSL */
        }

        else if( strcmp( pxLabel->pValue,
                         pkcs11configLABEL_CODE_VERIFICATION_KEY ) == 0 )
        {
            xBytesWritten = FLASH_update( ( uint32_t ) P11KeyConfig.cCodeSignKey,
                                          pucData,
                                          ulDataSize );

            if( xBytesWritten == ( ulDataSize ) )
            {
                xHandle = eAwsCodeSigningKey;

                /*change flash written mark'*/
                FLASH_update( ( uint32_t ) &P11KeyConfig.ulCodeSignKeyMark,
                              &ulFlashMark,
                              sizeof( uint32_t ) );
            }
        }

        else if( strcmp( pxLabel->pValue,
        		pkcs11configLABEL_DEVICE_PUBLIC_KEY_FOR_TLS ) == 0 )
		{
		   xBytesWritten = FLASH_update( ( uint32_t ) P11KeyConfig.cDevicePubKey,
										 pucData,
										 ulDataSize );

		   if( xBytesWritten == ( ulDataSize ) )
		   {
			   xHandle = eAwsDevicePublicKey;

			   /*change flash written mark'*/
			   FLASH_update( ( uint32_t ) &P11KeyConfig.uDevicePubKeyMark,
							 &ulFlashMark,
							 sizeof( uint32_t ) );
		   }
		}
    }

    return xHandle;
}



/*-----------------------------------------------------------*/

/**
 * @brief Translates a PKCS #11 label into an object handle.
 *
 * Port-specific object handle retrieval.
 *
 *
 * @param[in] pLabel         Pointer to the label of the object
 *                           who's handle should be found.
 * @param[in] usLength       The length of the label, in bytes.
 *
 * @return The object handle if operation was successful.
 * Returns eInvalidHandle if unsuccessful.
 */

CK_OBJECT_HANDLE PKCS11_PAL_FindObject( uint8_t * pLabel,
                                        uint8_t usLength )
{
    CK_OBJECT_HANDLE xHandle = eInvalidHandle;

    if( ( 0 == memcmp( pLabel, pkcs11configLABEL_DEVICE_CERTIFICATE_FOR_TLS, usLength ) ) &&
        ( ( P11KeyConfig.ulDeviceCertificateMark & pkcs11OBJECT_PRESENT_MASK ) == pkcs11OBJECT_PRESENT_MAGIC ) )
    {
        xHandle = eAwsDeviceCertificate;
    }
    else if( ( 0 == memcmp( pLabel, pkcs11configLABEL_DEVICE_PRIVATE_KEY_FOR_TLS, usLength ) ) &&
             ( ( P11KeyConfig.ulDeviceKeyMark & pkcs11OBJECT_PRESENT_MASK ) == pkcs11OBJECT_PRESENT_MAGIC ) )
    {
        xHandle = eAwsDevicePrivateKey;
    }
    else if( ( 0 == memcmp( pLabel, pkcs11configLABEL_CODE_VERIFICATION_KEY, usLength ) ) &&
             ( ( P11KeyConfig.ulCodeSignKeyMark & pkcs11OBJECT_PRESENT_MASK ) == pkcs11OBJECT_PRESENT_MAGIC ) )
    {
        xHandle = eAwsCodeSigningKey;
    }
    else if( ( 0 == memcmp( pLabel, pkcs11configLABEL_DEVICE_PUBLIC_KEY_FOR_TLS, usLength ) ) &&
			 ( ( P11KeyConfig.uDevicePubKeyMark & pkcs11OBJECT_PRESENT_MASK ) == pkcs11OBJECT_PRESENT_MAGIC ) )
	{
		xHandle = eAwsDevicePublicKey;
	}

    return xHandle;
}

/**
 * @brief Gets the value of an object in storage, by handle.
 *
 * Port-specific file access for cryptographic information.
 *
 * This call dynamically allocates the buffer which object value
 * data is copied into.  PKCS11_PAL_GetObjectValueCleanup()
 * should be called after each use to free the dynamically allocated
 * buffer.
 *
 * @sa PKCS11_PAL_GetObjectValueCleanup
 *
 * @param[in] pcFileName    The name of the file to be read.
 * @param[out] ppucData     Pointer to buffer for file data.
 * @param[out] pulDataSize  Size (in bytes) of data located in file.
 * @param[out] pIsPrivate   Boolean indicating if value is private (CK_TRUE)
 *                          or exportable (CK_FALSE)
 *
 * @return CKR_OK if operation was successful.  CKR_KEY_HANDLE_INVALID if
 * no such object handle was found, CKR_DEVICE_MEMORY if memory for
 * buffer could not be allocated, CKR_FUNCTION_FAILED for device driver
 * error.
 */

CK_RV PKCS11_PAL_GetObjectValue( CK_OBJECT_HANDLE xHandle,
                                 uint8_t ** ppucData,
                                 uint32_t * pulDataSize,
                                 CK_BBOOL * pIsPrivate )

{
    CK_RV ulReturn = CKR_OBJECT_HANDLE_INVALID;

    /*
     * Read client certificate.
     */

    if( xHandle == eAwsDeviceCertificate )
    {
        /*
         * return reference and size only if certificates are present in flash
         */
        if( ( P11KeyConfig.ulDeviceCertificateMark & pkcs11OBJECT_PRESENT_MASK ) == pkcs11OBJECT_PRESENT_MAGIC )
        {
            *ppucData = P11KeyConfig.cDeviceCertificate;
            *pulDataSize = ( uint32_t ) P11KeyConfig.ulDeviceCertificateMark & pkcs11OBJECT_LENGTH_MASK;
            *pIsPrivate = CK_FALSE;
            ulReturn = CKR_OK;
        }
    }

    /*
     * Read client key.
     */

    else if( xHandle == eAwsDevicePrivateKey )
    {
        /*
         * return reference and size only if certificates are present in flash
         */
        if( ( P11KeyConfig.ulDeviceKeyMark & pkcs11OBJECT_PRESENT_MASK ) == pkcs11OBJECT_PRESENT_MAGIC )
        {
            *ppucData = P11KeyConfig.cDeviceKey;
            *pulDataSize = ( uint32_t ) ( P11KeyConfig.ulDeviceKeyMark & pkcs11OBJECT_LENGTH_MASK );
            *pIsPrivate = CK_TRUE;
            ulReturn = CKR_OK;
        }
    }

    else if( xHandle == eAwsDevicePublicKey )
    {
        /*
         * return reference and size only if certificates are present in flash
         */
        if( ( P11KeyConfig.uDevicePubKeyMark & pkcs11OBJECT_PRESENT_MASK ) == pkcs11OBJECT_PRESENT_MAGIC )
        {
            *ppucData = P11KeyConfig.cDevicePubKey;
            *pulDataSize = ( uint32_t ) P11KeyConfig.uDevicePubKeyMark & pkcs11OBJECT_LENGTH_MASK;
            *pIsPrivate = CK_FALSE;
            ulReturn = CKR_OK;
        }
    }

    else if( xHandle == eAwsCodeSigningKey )
    {
        if( ( P11KeyConfig.ulCodeSignKeyMark & pkcs11OBJECT_PRESENT_MASK ) == pkcs11OBJECT_PRESENT_MAGIC )
        {
            *ppucData = P11KeyConfig.cCodeSignKey;
            *pulDataSize = ( uint32_t ) P11KeyConfig.ulCodeSignKeyMark & pkcs11OBJECT_LENGTH_MASK;
            *pIsPrivate = CK_FALSE;
            ulReturn = CKR_OK;
        }
    }

    return ulReturn;
}

/*-----------------------------------------------------------*/

/**
 * @brief Cleanup after PKCS11_GetObjectValue().
 *
 * @param[in] pucData       The buffer to free.
 *                          (*ppucData from PKCS11_PAL_GetObjectValue())
 * @param[in] ulDataSize    The length of the buffer to free.
 *                          (*pulDataSize from PKCS11_PAL_GetObjectValue())
 */
void PKCS11_PAL_GetObjectValueCleanup( uint8_t * pucData,
                                       uint32_t ulDataSize )
{
    /* Unused parameters. */
    ( void ) pucData;
    ( void ) ulDataSize;

    /* Since no buffer was allocated on heap, there is no cleanup
     * to be done. */
}
/*-----------------------------------------------------------*/

/**
 * \brief           Entropy poll callback for a hardware source
 *
 * \note            This must accept NULL as its first argument.
 * \note            Notice: For best security practice, it is recommended to utilize
 *          a random number generation solution that is truly randomized and conforms
 *          to the guidelines provided in the Amazon FreeRTOS Qualification Guide
 *          ( https://docs.aws.amazon.com/freertos/latest/qualificationguide/afq-checklist.html).
 *          The random number generator method presented in this file by the silicon
 *          vendor is not truly random in nature. Please contact the silicon vendor for
 *          details regarding the method implemented.
 */
int mbedtls_hardware_poll( void * data,
                           unsigned char * output,
                           size_t len,
                           size_t * olen )
{
	static uint32_t ulSeedValid=0;
	uint32_t ulSeed;

	/* random number is generated using rand() function. Initial seed
		     * is network latency.
		     */
	if (!ulSeedValid)
	{
		asm volatile ("csrr %0, mcycle" : "=r"(ulSeed));

		ulSeedValid = 1;
		ulSeed = ulSeed ^ xTaskGetTickCount();
		srand(ulSeed);
	}

	*olen = sizeof(int);
	if ( (*olen) > len )
	{
		*olen = len;
	}
	int rand_num = rand();
	unsigned char *pucTemp = ( unsigned char * )&rand_num;
	memcpy( ( void* ) output, (void*) pucTemp, *olen);

    return 0;
}
