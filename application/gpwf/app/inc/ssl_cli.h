/***************************************************************************
* ssl_cli.h
*
* Purpose: Header file for SSL client connection
*
* Author: Eugene Hsu
*
* Date: 2016/08/10
*
* Copyright Generalplus Corp. ALL RIGHTS RESERVED.
*
* Version : 
* History :
*
******************************************************************************/
#ifndef __SSL_CLI_H__
#define __SSL_CLI_H__

#define POLARSSL_ERR_NET_UNKNOWN_HOST                      -0x0056  /**< Failed to get an IP address for the given hostname. */
#define POLARSSL_ERR_NET_SOCKET_FAILED                     -0x0042  /**< Failed to open a socket. */
#define POLARSSL_ERR_NET_CONNECT_FAILED                    -0x0044  /**< The connection to the given server / port failed. */
#define POLARSSL_ERR_NET_BIND_FAILED                       -0x0046  /**< Binding of the socket failed. */
#define POLARSSL_ERR_NET_LISTEN_FAILED                     -0x0048  /**< Could not listen on the socket. */
#define POLARSSL_ERR_NET_ACCEPT_FAILED                     -0x004A  /**< Could not accept the incoming connection. */
#define POLARSSL_ERR_NET_RECV_FAILED                       -0x004C  /**< Reading information from the socket failed. */
#define POLARSSL_ERR_NET_SEND_FAILED                       -0x004E  /**< Sending information through the socket failed. */
#define POLARSSL_ERR_NET_CONN_RESET                        -0x0050  /**< Connection was reset by peer. */
#define POLARSSL_ERR_NET_WANT_READ                         -0x0052  /**< Connection requires a read call. */
#define POLARSSL_ERR_NET_WANT_WRITE                        -0x0054  /**< Connection requires a write call. */

/* SSL Error codes */
#define POLARSSL_ERR_SSL_FEATURE_UNAVAILABLE               -0x7080  /**< The requested feature is not available. */
#define POLARSSL_ERR_SSL_BAD_INPUT_DATA                    -0x7100  /**< Bad input parameters to function. */
#define POLARSSL_ERR_SSL_INVALID_MAC                       -0x7180  /**< Verification of the message MAC failed. */
#define POLARSSL_ERR_SSL_INVALID_RECORD                    -0x7200  /**< An invalid SSL record was received. */
#define POLARSSL_ERR_SSL_CONN_EOF                          -0x7280  /**< The connection indicated an EOF. */
#define POLARSSL_ERR_SSL_UNKNOWN_CIPHER                    -0x7300  /**< An unknown cipher was received. */
#define POLARSSL_ERR_SSL_NO_CIPHER_CHOSEN                  -0x7380  /**< The server has no ciphersuites in common with the client. */
#define POLARSSL_ERR_SSL_NO_RNG                            -0x7400  /**< No RNG was provided to the SSL module. */
#define POLARSSL_ERR_SSL_NO_CLIENT_CERTIFICATE             -0x7480  /**< No client certification received from the client, but required by the authentication mode. */
#define POLARSSL_ERR_SSL_CERTIFICATE_TOO_LARGE             -0x7500  /**< Our own certificate(s) is/are too large to send in an SSL message.*/
#define POLARSSL_ERR_SSL_CERTIFICATE_REQUIRED              -0x7580  /**< The own certificate is not set, but needed by the server. */
#define POLARSSL_ERR_SSL_PRIVATE_KEY_REQUIRED              -0x7600  /**< The own private key or pre-shared key is not set, but needed. */
#define POLARSSL_ERR_SSL_CA_CHAIN_REQUIRED                 -0x7680  /**< No CA Chain is set, but required to operate. */
#define POLARSSL_ERR_SSL_UNEXPECTED_MESSAGE                -0x7700  /**< An unexpected message was received from our peer. */
#define POLARSSL_ERR_SSL_FATAL_ALERT_MESSAGE               -0x7780  /**< A fatal alert message was received from our peer. */
#define POLARSSL_ERR_SSL_PEER_VERIFY_FAILED                -0x7800  /**< Verification of our peer failed. */
#define POLARSSL_ERR_SSL_PEER_CLOSE_NOTIFY                 -0x7880  /**< The peer notified us that the connection is going to be closed. */
#define POLARSSL_ERR_SSL_BAD_HS_CLIENT_HELLO               -0x7900  /**< Processing of the ClientHello handshake message failed. */
#define POLARSSL_ERR_SSL_BAD_HS_SERVER_HELLO               -0x7980  /**< Processing of the ServerHello handshake message failed. */
#define POLARSSL_ERR_SSL_BAD_HS_CERTIFICATE                -0x7A00  /**< Processing of the Certificate handshake message failed. */
#define POLARSSL_ERR_SSL_BAD_HS_CERTIFICATE_REQUEST        -0x7A80  /**< Processing of the CertificateRequest handshake message failed. */
#define POLARSSL_ERR_SSL_BAD_HS_SERVER_KEY_EXCHANGE        -0x7B00  /**< Processing of the ServerKeyExchange handshake message failed. */
#define POLARSSL_ERR_SSL_BAD_HS_SERVER_HELLO_DONE          -0x7B80  /**< Processing of the ServerHelloDone handshake message failed. */
#define POLARSSL_ERR_SSL_BAD_HS_CLIENT_KEY_EXCHANGE        -0x7C00  /**< Processing of the ClientKeyExchange handshake message failed. */
#define POLARSSL_ERR_SSL_BAD_HS_CLIENT_KEY_EXCHANGE_RP     -0x7C80  /**< Processing of the ClientKeyExchange handshake message failed in DHM / ECDH Read Public. */
#define POLARSSL_ERR_SSL_BAD_HS_CLIENT_KEY_EXCHANGE_CS     -0x7D00  /**< Processing of the ClientKeyExchange handshake message failed in DHM / ECDH Calculate Secret. */
#define POLARSSL_ERR_SSL_BAD_HS_CERTIFICATE_VERIFY         -0x7D80  /**< Processing of the CertificateVerify handshake message failed. */
#define POLARSSL_ERR_SSL_BAD_HS_CHANGE_CIPHER_SPEC         -0x7E00  /**< Processing of the ChangeCipherSpec handshake message failed. */
#define POLARSSL_ERR_SSL_BAD_HS_FINISHED                   -0x7E80  /**< Processing of the Finished handshake message failed. */
#define POLARSSL_ERR_SSL_MALLOC_FAILED                     -0x7F00  /**< Memory allocation failed */
#define POLARSSL_ERR_SSL_HW_ACCEL_FAILED                   -0x7F80  /**< Hardware acceleration function returned with error */
#define POLARSSL_ERR_SSL_HW_ACCEL_FALLTHROUGH              -0x6F80  /**< Hardware acceleration function skipped / left alone data */
#define POLARSSL_ERR_SSL_COMPRESSION_FAILED                -0x6F00  /**< Processing of the compression / decompression failed */
#define POLARSSL_ERR_SSL_BAD_HS_PROTOCOL_VERSION           -0x6E80  /**< Handshake protocol not within min/max boundaries */
#define POLARSSL_ERR_SSL_BAD_HS_NEW_SESSION_TICKET         -0x6E00  /**< Processing of the NewSessionTicket handshake message failed. */
#define POLARSSL_ERR_SSL_SESSION_TICKET_EXPIRED            -0x6D80  /**< Session ticket has expired. */
#define POLARSSL_ERR_SSL_PK_TYPE_MISMATCH                  -0x6D00  /**< Public key type mismatch (eg, asked for RSA key exchange and presented EC key) */
#define POLARSSL_ERR_SSL_UNKNOWN_IDENTITY                  -0x6C80  /**< Unknown identity received (eg, PSK identity) */
#define POLARSSL_ERR_SSL_INTERNAL_ERROR                    -0x6C00  /**< Internal error (eg, unexpected failure in lower-level module) */
#define POLARSSL_ERR_SSL_COUNTER_WRAPPING                  -0x6B80  /**< A counter would wrap (eg, too many messages exchanged). */

#define TEST_GET_REQUEST   "GET / HTTP/1.0\r\n\r\n"

#define SSL_1_ENABLE	BIT0
#define SSL_2_ENABLE	BIT1

#endif	//__SSL_CLI_H__