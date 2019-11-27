/*
 * Amazon FreeRTOS V201910.00
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
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

#ifndef AWS_CLIENT_CREDENTIAL_KEYS_H
#define AWS_CLIENT_CREDENTIAL_KEYS_H

/*
 * @brief PEM-encoded client certificate.
 *
 * @todo If you are running one of the Amazon FreeRTOS demo projects, set this
 * to the certificate that will be used for TLS client authentication.
 *
 * @note Must include the PEM header and footer:
 * "-----BEGIN CERTIFICATE-----\n"\
 * "...base64 data...\n"\
 * "-----END CERTIFICATE-----\n"
 */
#define keyCLIENT_CERTIFICATE_PEM    ""

//#define keyCLIENT_CERTIFICATE_PEM    "-----BEGIN CERTIFICATE-----\n"\
//"MIIDWjCCAkKgAwIBAgIVAOg+VWILFTo5wSyuv1Oakupv93caMA0GCSqGSIb3DQEB\n"\
//"CwUAME0xSzBJBgNVBAsMQkFtYXpvbiBXZWIgU2VydmljZXMgTz1BbWF6b24uY29t\n"\
//"IEluYy4gTD1TZWF0dGxlIFNUPVdhc2hpbmd0b24gQz1VUzAeFw0xOTA0MTYxMjU5\n"\
//"MDJaFw00OTEyMzEyMzU5NTlaMB4xHDAaBgNVBAMME0FXUyBJb1QgQ2VydGlmaWNh\n"\
//"dGUwggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEKAoIBAQChHiSpvE5PparVA/0C\n"\
//"P+VvGyl/MMcPma0K8OUpa2+K+nMGr1v2ejPVCn1E4yQJHWPKFnc/5foljHEceah9\n"\
//"hj7BlR/aUiJCEwy6UpV2zhTkxPNsXXrBLTZIhRw66htpQqUeCVVPjUOr02s+XN3p\n"\
//"I7070iKodWcJhq8MzlifXsB55oSCqQtWhexDzZNDtrlAHhesVLqNl5ZwgDn7MYjq\n"\
//"vRFosK4z2hbRDDFP6DM7SF647kQYUkkHkNnmCDLudRKHPQx96Fg5aQELQLW+LR+R\n"\
//"lzeExd8N4VXkUIQea4SQf90pdsY5vcPlk6/YW0o8sDy8Niu+jfzvS4cAP1MgJaS0\n"\
//"gaNXAgMBAAGjYDBeMB8GA1UdIwQYMBaAFFgvITVUr8MAowMaf003w3LZ94tiMB0G\n"\
//"A1UdDgQWBBTpE5H6GJ7LvMJ9qU7Bm4+I/8PkSDAMBgNVHRMBAf8EAjAAMA4GA1Ud\n"\
//"DwEB/wQEAwIHgDANBgkqhkiG9w0BAQsFAAOCAQEAlAXJHjjMh8XLHGV5fZsx4MK1\n"\
//"a6YGXe0x0KGxEGrJHjupkFimIcci0P0TIWGfm6rTTvFKe34laTNjt4cbvPLPCgNa\n"\
//"RoRHSnz+yuE/QALrxL36QNn+klVifFK6iOh4HsQ/ZrJqmQJUjLoJTedYe82xccde\n"\
//"JkeslKTIqlXgvyR5uqzjwMoiUDg9kS/1O8gCS8y9PiwmDAeWRyP5yyoW8nFUPH1k\n"\
//"33KoJPcEJUsiXTKVV+KOU331tbKwlvVuYNOMFOBIFA896aEFqMeuu2nTj0uwVbOw\n"\
//"km72KWR/A1dfIkhh86Bq3aJqlgCAR4btWZsKTfDJaPYMHTTtsvws1btE29v03Q==\n"\
//"-----END CERTIFICATE-----\n"

/*
 * @brief PEM-encoded issuer certificate for AWS IoT Just In Time Registration (JITR).
 *
 * @todo If you are using AWS IoT Just in Time Registration (JITR), set this to
 * the issuer (Certificate Authority) certificate of the client certificate above.
 *
 * @note This setting is required by JITR because the issuer is used by the AWS
 * IoT gateway for routing the device's initial request. (The device client
 * certificate must always be sent as well.) For more information about JITR, see:
 *  https://docs.aws.amazon.com/iot/latest/developerguide/jit-provisioning.html,
 *  https://aws.amazon.com/blogs/iot/just-in-time-registration-of-device-certificates-on-aws-iot/.
 *
 * If you're not using JITR, set below to NULL.
 *
 * Must include the PEM header and footer:
 * "-----BEGIN CERTIFICATE-----\n"\
 * "...base64 data...\n"\
 * "-----END CERTIFICATE-----\n"
 */
//#define keyJITR_DEVICE_CERTIFICATE_AUTHORITY_PEM    NULL
#define keyJITR_DEVICE_CERTIFICATE_AUTHORITY_PEM    ""

/*
 * @brief PEM-encoded client private key.
 *
 * @todo If you are running one of the Amazon FreeRTOS demo projects, set this
 * to the private key that will be used for TLS client authentication.
 *
 * @note Must include the PEM header and footer:
 * "-----BEGIN RSA PRIVATE KEY-----\n"\
 * "...base64 data...\n"\
 * "-----END RSA PRIVATE KEY-----\n"
 */
#define keyCLIENT_PRIVATE_KEY_PEM  ""

//#define keyCLIENT_PRIVATE_KEY_PEM  "-----BEGIN RSA PRIVATE KEY-----\n"\
//"MIIEowIBAAKCAQEAoR4kqbxOT6Wq1QP9Aj/lbxspfzDHD5mtCvDlKWtvivpzBq9b\n"\
//"9noz1Qp9ROMkCR1jyhZ3P+X6JYxxHHmofYY+wZUf2lIiQhMMulKVds4U5MTzbF16\n"\
//"wS02SIUcOuobaUKlHglVT41Dq9NrPlzd6SO9O9IiqHVnCYavDM5Yn17AeeaEgqkL\n"\
//"VoXsQ82TQ7a5QB4XrFS6jZeWcIA5+zGI6r0RaLCuM9oW0QwxT+gzO0heuO5EGFJJ\n"\
//"B5DZ5ggy7nUShz0MfehYOWkBC0C1vi0fkZc3hMXfDeFV5FCEHmuEkH/dKXbGOb3D\n"\
//"5ZOv2FtKPLA8vDYrvo3870uHAD9TICWktIGjVwIDAQABAoIBAQCOPA4hyhtYNOib\n"\
//"JOo3EjF55IC9yFHzc7dP8VpdNntCNddWD3kgHsGjUZ9FMUCey1NNFIgx4NT0UQeK\n"\
//"G9yi8+BzBTMZiWi07BxQYJeSSlwnmbdL8zprT36chuZlHEcWpb8exR6LHCqhhgMn\n"\
//"CtTc/LASH3B5RyuvPTdMFt4l04AYHAIuY8rvOlicLJJEGreyb6pQT3f+085EpB0L\n"\
//"5KVwvTkY/DBbOHUfbuJc0URUI9OS8WMtLEMp37+BRnfaA5E06081ZIo1WYanSYtQ\n"\
//"Sm42j4xBjSnMqkuIbeV1jNrQPIn4jhiZCTNNc10jlTSzIOL4bu/05NwlQtDiKsaM\n"\
//"csCj4MABAoGBANAiQuASe14DGDo2/bORHYJ9cKMGcUByOaA3OlB6vkSS4dqGE3M7\n"\
//"xNP05bJaUsUaCdMVfzhrogdvakKFdEIzpl2adAoV1x5Ku4C6kTBLXRfFd/zajKzU\n"\
//"X1YkK0Y51pdHVEYVofWHjJY8xw7/UYhFUphwZqildblt6t5CT0Bzuf+nAoGBAMYr\n"\
//"13kOx2fsS8wXSOYxF/W/6N6LWGbgUX7vAKFnoSnnsi8LVHFaf/5vvLFr03EqeyPZ\n"\
//"aTkBua7rQ1mckDKgEG9PZv9MeUQqE7V3Vf6P3YKA5BA+2jeQQR4+16zvem5tWw/T\n"\
//"iMcv+wSM6e3koyNceTO2jp3fOGUyEODMl7/ntjTRAoGAKxE711PbtTtcswcX/iUj\n"\
//"/xFn0kQOATDhnbZv2aFcZ6IhG++sbKbMdrNxKDOqduCJceU72vfiojk6uqMUyHIZ\n"\
//"lbrVEhUs2zbJj7yLOHwPulCBIJA0lbqoM5HuZmyna8w1sv7c8Ypn29qdsgvA2jrF\n"\
//"+Ycgpvr93i3RcMY10fdgYHcCgYBJXE9+9BajhRkmK3SEKqZpBispv0Hxi5b/u3Br\n"\
//"T9ackbYnrjULZWo+gZTDxVvX59cAFWrbfKUFePhMwyWqaBDNdAJ3/hJiRoVXRx8h\n"\
//"hpaEyjwjO3FC4ncIwFDNJzYWjQsJ19AplLyDGbs7osk1vPTtTOaM0Oaqp5cR8Qs4\n"\
//"MooEEQKBgF5RfJ3RniaGSuRd4u8rKTMp1M38Vy0ei2gU1gWdx3Xutbp7mnH4wscC\n"\
//"d0GPDLtivWEGi7AX4Q6FKTav1Vr0rB/Koj7oRdkUafKdo/jGWaWSN7Sq4zFOzCQA\n"\
//"dgaZkJRfylzpLkw+QxIEIL686IMwlqCOlPjB3XQfFHJ93xfj6zDC\n"\
//"-----END RSA PRIVATE KEY-----\n"

#endif /* AWS_CLIENT_CREDENTIAL_KEYS_H */
