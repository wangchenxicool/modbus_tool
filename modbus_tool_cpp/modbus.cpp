/*
 * Copyright 漏 2001-2010 St茅phane Raimbault <stephane.raimbault@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser Public License for more details.
 *
 * You should have received a copy of the GNU Lesser Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
   The library is designed to send and receive data from a device that
   communicate via the Modbus protocol.

   The function names used are inspired by the Modicon Modbus Protocol
   Reference Guide which can be obtained from Schneider at
   www.schneiderautomation.com.

   Documentation:
   http://www.easysw.com/~mike/serial/serial.html
   http://copyleft.free.fr/wordpress/index.php/libmodbus/
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#ifdef HAVE_INTTYPES_H
#include <inttypes.h>
#endif
#ifdef HAVE_STDINT_H
#include <stdint.h>
#endif
#include <termios.h>
#include <sys/time.h>
#include <unistd.h>
#include <errno.h>
#include <limits.h>
#include <fcntl.h>

/* TCP */
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#if defined(__FreeBSD__ ) && __FreeBSD__ < 5
#include <netinet/in_systm.h>
#endif
#include <netinet/in.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>

#if !defined(UINT16_MAX)
#define UINT16_MAX 0xFFFF
#endif

#include "modbus.h"

#define UNKNOWN_ERROR_MSG "Not defined in modbus specification"

#define DEBUG

#ifdef DEBUG
#define wprintf(format, arg...)  \
    printf( format , ##arg)
#else
#define wprintf(format, arg...)
#endif


static const uint8_t NB_TAB_ERROR_MSG = 12;

static const char *TAB_ERROR_MSG[] =
{
    /* 0x00 */ UNKNOWN_ERROR_MSG,
    /* 0x01 */ "Illegal function code",
    /* 0x02 */ "Illegal data address",
    /* 0x03 */ "Illegal data value",
    /* 0x04 */ "Slave device or server failure",
    /* 0x05 */ "Acknowledge",
    /* 0x06 */ "Slave device or server busy",
    /* 0x07 */ "Negative acknowledge",
    /* 0x08 */ "Memory parity error",
    /* 0x09 */ UNKNOWN_ERROR_MSG,
    /* 0x0A */ "Gateway path unavailable",
    /* 0x0B */ "Target device failed to respond"
};

/* Table of CRC values for high-order byte */
static uint8_t table_crc_hi[] =
{
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};

/* Table of CRC values for low-order byte */
static uint8_t table_crc_lo[] =
{
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
    0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
    0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
    0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
    0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
    0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
    0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
    0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
    0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
    0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
    0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
    0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
    0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
    0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
    0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
    0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};

static const int TAB_HEADER_LENGTH[2] =
{
    HEADER_LENGTH_RTU,
    HEADER_LENGTH_TCP
};

static const int TAB_CHECKSUM_LENGTH[2] =
{
    CHECKSUM_LENGTH_RTU,
    CHECKSUM_LENGTH_TCP
};

static const int TAB_MAX_ADU_LENGTH[2] =
{
    MAX_ADU_LENGTH_RTU,
    MAX_ADU_LENGTH_TCP,
};

c_modbus::c_modbus (const char *device,
                    int baud, const char *parity, int data_bit,
                    int stop_bit, int slave)
{
    mb_param = new modbus_param_t;

    modbus_init_rtu (device, baud, parity, data_bit, stop_bit, slave);
}

c_modbus::~c_modbus ()
{
    delete mb_param;
}

void c_modbus::modbus_sleep (long int s, long int us)
{
    struct timeval tv;
    tv.tv_sec = s;
    tv.tv_usec = us;
    if (select (0, NULL, NULL, NULL, &tv) < 0)
    {
        perror ("[modbus_sleep: select error]");
    }
}

/* Treats errors and flush or close connection if necessary */
void c_modbus::error_treat (int code, const char *string)
{
    /*fprintf ( stderr, "\nerror_treat: ERROR %s (%0X)\n", string, -code );*/
    if (mb_param->debug)
    {
        wprintf ("\033[31;40;1m \nerror_treat: %s (%0X)\n\033[0m", string, -code);
    }

    if (mb_param->error_handling == FLUSH_OR_CONNECT_ON_ERROR)
    {
        switch (code)
        {
        case INVALID_DATA:
        case INVALID_CRC:
        case INVALID_EXCEPTION_CODE:
            modbus_flush ();
            break;
        case SELECT_FAILURE:
        case SOCKET_FAILURE:
        case CONNECTION_CLOSED:
            modbus_close ();
            modbus_connect ();
            break;
        default:
            /* NOP */
            break;
        } // end switch
    } // end if
}

void c_modbus::modbus_flush ()
{
    if (mb_param->type_com == RTU)
    {
        tcflush (mb_param->fd, TCIOFLUSH);
    }
    else
    {
        int ret;
        do
        {
            /* Extract the garbage from the socket */
            char devnull[MAX_ADU_LENGTH_TCP];
#if (!HAVE_DECL___CYGWIN__)
            ret = recv (mb_param->fd, devnull, MAX_ADU_LENGTH_TCP, MSG_DONTWAIT);
#else
            /* On Cygwin, it's a bit more complicated to not wait */
            fd_set rfds;
            struct timeval tv;

            tv.tv_sec = 0;
            tv.tv_usec = 0;
            FD_ZERO (&rfds);
            FD_SET (mb_param->fd, &rfds);
            ret = select (mb_param->fd + 1, &rfds, NULL, NULL, &tv);
            if (ret > 0)
            {
                ret = recv (mb_param->fd, devnull, MAX_ADU_LENGTH_TCP, 0);
            }
            else if (ret == -1)
            {
                /* error_treat() doesn't call modbus_flush() in
                this case (avoid infinite loop) */
                error_treat (SELECT_FAILURE, "Select failure");
            }
#endif
            if (mb_param->debug && ret > 0)
            {
                wprintf ("%d bytes flushed\n", ret);
            }
        }
        while (ret > 0);
    }
}

/* Computes the length of the expected response */
unsigned int c_modbus::compute_response_length (uint8_t *query , uint8_t data_type)
{
    int length;
    int offset;

    offset = TAB_HEADER_LENGTH[mb_param->type_com];

    switch (query[offset])
    {
    case FC_READ_COIL_STATUS:
    case FC_READ_INPUT_STATUS:
    {
        /* Header + nb values (code from force_multiple_coils) */
        int nb = (query[offset + 3] << 8) | query[offset + 4];
        length = 2 + (nb / 8) + ( (nb % 8) ? 1 : 0);
    }
    break;
    case FC_READ_HOLDING_REGISTERS:
    case FC_READ_INPUT_REGISTERS:
        switch (data_type)
        {
        case    0:      // 数据类型为8位整数
        case    1:      // 数据类型为8位无符号整数
            /* Header + 1 * nb values */
            length = 2 + (query[offset + 3] << 8 |
                          query[offset + 4]);
            break;
        case    2:      // 数据类型为16位整数
        case    3:      // 数据类型为16位无符号整数
            /* Header + 2 * nb values */
            length = 2 + 2 * (query[offset + 3] << 8 |
                              query[offset + 4]);
            break;
        case    4:      // 数据类型为32位整数
        case    5:      // 数据类型为32位无符号整数
            /* Header + 4 * nb values */
            length = 2 + 4 * (query[offset + 3] << 8 |
                              query[offset + 4]);
            break;
        case    6:      // 数据类型为64位整数
        case    7:      // 数据类型为64位无符号整数
            /* Header + 2 * nb values */
            length = 2 + 2 * (query[offset + 3] << 8 |
                              query[offset + 4]);
            break;
        case    8:      // 单精度浮点数
            /* Header + 2 * nb values */
            length = 2 + 2 * (query[offset + 3] << 8 |
                              query[offset + 4]);
            break;
        case    9:      // 双精度浮点数
            /* Header + 2 * nb values */
            length = 2 + 2 * (query[offset + 3] << 8 |
                              query[offset + 4]);
            break;
        default:
            /* Header + 2 * nb values */
            length = 2 + 2 * (query[offset + 3] << 8 |
                              query[offset + 4]);
            break;
        }
        break;
    case FC_READ_EXCEPTION_STATUS:
        length = 3;
        break;
    case FC_REPORT_SLAVE_ID:
        /* The response is device specific (the header provides the
           length) */
        return MSG_LENGTH_UNDEFINED;
    default:
        length = 5;
    }

    return length + offset + TAB_CHECKSUM_LENGTH[mb_param->type_com];
}

/* Builds a RTU query header */
int c_modbus::build_query_basis_rtu (int slave, int function, int start_addr, int nb, uint8_t *query)
{
    query[0] = slave;
    query[1] = function;
    query[2] = start_addr >> 8;
    query[3] = start_addr & 0x00ff;
    query[4] = nb >> 8;
    query[5] = nb & 0x00ff;

    return PRESET_QUERY_LENGTH_RTU;
}

/* Builds a TCP query header */
int c_modbus::build_query_basis_tcp (int slave, int function,
                                     int start_addr, int nb,
                                     uint8_t *query)
{

    /* Extract from MODBUS Messaging on TCP/IP Implementation
       Guide V1.0b (page 23/46):
       The transaction identifier is used to associate the future
       response with the request. So, at a time, on a TCP
       connection, this identifier must be unique.
    */
    static uint16_t t_id = 0;

    /* Transaction ID */
    if (t_id < UINT16_MAX)
        t_id++;
    else
        t_id = 0;
    query[0] = t_id >> 8;
    query[1] = t_id & 0x00ff;

    /* Protocol Modbus */
    query[2] = 0;
    query[3] = 0;

    /* Length will be defined later by set_query_length_tcp at offsets 4
     * and 5 */

    query[6] = slave;
    query[7] = function;
    query[8] = start_addr >> 8;
    query[9] = start_addr & 0x00ff;
    query[10] = nb >> 8;
    query[11] = nb & 0x00ff;

    return PRESET_QUERY_LENGTH_TCP;
}

int c_modbus::build_query_basis (int function, int start_addr,
                                 int nb, uint8_t *query)
{
    if (mb_param->type_com == RTU)
        return build_query_basis_rtu (mb_param->slave, function, start_addr, nb, query);
    else
        return build_query_basis_tcp (mb_param->slave, function, start_addr, nb, query);
}

/* Builds a RTU response header */
int c_modbus::build_response_basis_rtu (sft_t *sft, uint8_t *response)
{
    response[0] = sft->slave;
    response[1] = sft->function;

    return PRESET_RESPONSE_LENGTH_RTU;
}

/* Builds a TCP response header */
int c_modbus::build_response_basis_tcp (sft_t *sft, uint8_t *response)
{
    /* Extract from MODBUS Messaging on TCP/IP Implementation
       Guide V1.0b (page 23/46):
       The transaction identifier is used to associate the future
       response with the request. */
    response[0] = sft->t_id >> 8;
    response[1] = sft->t_id & 0x00ff;

    /* Protocol Modbus */
    response[2] = 0;
    response[3] = 0;

    /* Length to fix later with set_message_length_tcp (4 and 5) */

    response[6] = sft->slave;
    response[7] = sft->function;

    return PRESET_RESPONSE_LENGTH_TCP;
}

int c_modbus::build_response_basis (sft_t *sft, uint8_t *response)
{
    if (mb_param->type_com == RTU)
        return build_response_basis_rtu (sft, response);
    else
        return build_response_basis_tcp (sft, response);
}

/* Sets the length of TCP message in the message (query and response) */
void c_modbus::set_message_length_tcp (uint8_t *msg, int msg_length)
{
    /* Substract the header length to the message length */
    int mbap_length = msg_length - 6;

    msg[4] = mbap_length >> 8;
    msg[5] = mbap_length & 0x00FF;
}

/* Fast CRC */
uint16_t c_modbus::crc16 (uint8_t *buffer, uint16_t buffer_length)
{
    uint8_t crc_hi = 0xFF; /* high CRC byte initialized */
    uint8_t crc_lo = 0xFF; /* low CRC byte initialized */
    unsigned int i; /* will index into CRC lookup */

    /* pass through message buffer */
    while (buffer_length--)
    {
        i = crc_hi ^ *buffer++; /* calculate the CRC  */
        crc_hi = crc_lo ^ table_crc_hi[i];
        crc_lo = table_crc_lo[i];
    }

    return (crc_hi << 8 | crc_lo);
}

/* If CRC is correct returns msg_length else returns INVALID_CRC */
int c_modbus::check_crc16 (uint8_t *msg, const int msg_length)
{
    int ret;
    uint16_t crc_calc;
    uint16_t crc_received;

    crc_calc = crc16 (msg, msg_length - 2);
    crc_received = (msg[msg_length - 2] << 8) | msg[msg_length - 1];

    /* Check CRC of msg */
    if (crc_calc == crc_received)
    {
        ret = msg_length;
    }
    else
    {
        char s_error[128];
        sprintf (s_error, "!!!!!!!!!!!!!invalid crc received %0X - crc_calc %0X",
                 crc_received, crc_calc);
        ret = INVALID_CRC;
        error_treat (ret, s_error);
    }

    return ret;
}

/* Sends a query/response over a serial or a TCP communication */
int c_modbus::modbus_send (uint8_t *query, int query_length)
{
    int i;
    int ret;
    uint16_t s_crc;

    if (mb_param->type_com == RTU)
    {
        s_crc = crc16 (query, query_length);
        query[query_length++] = s_crc >> 8;
        query[query_length++] = s_crc & 0x00FF;
    }
    else
    {
        set_message_length_tcp (query, query_length);
    }

    if (mb_param->debug)
    {
        wprintf ("\033[34;40;1m \nsend:\033[0m");
        for (i = 0; i < query_length; i++)
            wprintf ("[%.2X]", query[i]);
        wprintf ("\n");
    }

    if (mb_param->type_com == RTU)
    {
        /*tcflush ( mb_param->fd, TCIOFLUSH ); */
        ret = write (mb_param->fd, query, query_length);
    }
    else
        ret = send (mb_param->fd, query, query_length, MSG_NOSIGNAL);

    /* Return the number of bytes written (0 to n)
    or SOCKET_FAILURE on error */
    if ( (ret == -1) || (ret != query_length))
    {
        ret = SOCKET_FAILURE;
        error_treat (ret, "modbus_send: Write socket failure");
    }

    return ret;
}

/* Sends a query/response over a serial or a TCP communication */
int c_modbus::serial_send (uint8_t *query, int query_length)
{
    int i;
    int ret;
    uint16_t s_crc;

    if (mb_param->type_com == RTU)
    {
        //s_crc = crc16 (query, query_length);
        //query[query_length++] = s_crc >> 8;
        //query[query_length++] = s_crc & 0x00FF;
    }
    else
    {
        set_message_length_tcp (query, query_length);
    }

    if (mb_param->debug)
    {
        wprintf ("\033[34;40;1m \nsend:\033[0m");
        for (i = 0; i < query_length; i++)
            wprintf ("[%.2X]", query[i]);
        wprintf ("\n");
    }

    if (mb_param->type_com == RTU)
    {
        /*tcflush ( mb_param->fd, TCIOFLUSH ); */
        ret = write (mb_param->fd, query, query_length);
    }
    else
    {
        ret = send (mb_param->fd, query, query_length, MSG_NOSIGNAL);
    }

    /* Return the number of bytes written (0 to n)
    or SOCKET_FAILURE on error */
    if ( (ret == -1) || (ret != query_length))
    {
        ret = SOCKET_FAILURE;
        error_treat (ret, "modbus_send: Write socket failure");
    }

    return ret;
}

/* Computes the length of the header following the function code */
uint8_t c_modbus::compute_query_length_header (int function)
{
    int length;

    if (function <= FC_FORCE_SINGLE_COIL ||
            function == FC_PRESET_SINGLE_REGISTER)
        /* Read and single write */
        length = 4;
    else if (function == FC_FORCE_MULTIPLE_COILS ||
             function == FC_PRESET_MULTIPLE_REGISTERS)
        /* Multiple write */
        length = 5;
    else if (function == FC_REPORT_SLAVE_ID)
        length = 1;
    else
        length = 0;

    return length;
}

/* Computes the length of the data to write in the query */
int c_modbus::compute_query_length_data (uint8_t *msg)
{
    int function = msg[TAB_HEADER_LENGTH[mb_param->type_com]];
    int length;

    if (function == FC_FORCE_MULTIPLE_COILS ||
            function == FC_PRESET_MULTIPLE_REGISTERS)
        length = msg[TAB_HEADER_LENGTH[mb_param->type_com] + 5];
    else if (function == FC_REPORT_SLAVE_ID)
        length = msg[TAB_HEADER_LENGTH[mb_param->type_com] + 1];
    else
        length = 0;

    length += TAB_CHECKSUM_LENGTH[mb_param->type_com];

    return length;
}

#define WAIT_DATA()                                                                \
{                                                                                  \
    while ((select_ret = select(mb_param->fd+1, &rfds, NULL, NULL, &tv)) == -1) {  \
            if (errno == EINTR) {                                                  \
                    fprintf(stderr, "A non blocked signal was caught\n");          \
                    /* Necessary after an error */                                 \
                    FD_ZERO(&rfds);                                                \
                    FD_SET(mb_param->fd, &rfds);                                   \
            } else {                                                               \
                    error_treat(SELECT_FAILURE, "Select failure");       \
                    return SELECT_FAILURE;                                         \
            }                                                                      \
    }                                                                              \
                                                                                   \
    if (select_ret == 0) {                                                         \
            /* Timeout */                                                          \
            if (msg_length == (TAB_HEADER_LENGTH[mb_param->type_com] + 2 +         \
                               TAB_CHECKSUM_LENGTH[mb_param->type_com])) {         \
                    /* Optimization allowed because exception response is          \
                       the smallest trame in modbus protocol (3) so always         \
                       raise a timeout error */                                    \
                    return MB_EXCEPTION;                                           \
            } else {                                                               \
                    /* Call to error_treat is done later to manage exceptions */   \
                    if ( mb_param->debug )                                         \
                        wprintf ( "\n" );                                          \
                    return SELECT_TIMEOUT;                                         \
            }                                                                      \
    }                                                                              \
}

/* Waits a reply from a modbus slave or a query from a modbus master.
   This function blocks if there is no replies (3 timeouts).

   In
   - msg_length_computed must be set to MSG_LENGTH_UNDEFINED if undefined

   Out
   - msg is an array of uint8_t to receive the message

   On success, return the number of received characters. On error, return
   a negative value.
*/
int c_modbus::receive_msg (int msg_length_computed, uint8_t *msg, int select_time)
{
    int select_ret;
    int read_ret;
    fd_set rfds;
    struct timeval tv;
    int length_to_read;
    uint8_t *p_msg;
    enum { FUNCTION, BYTE, COMPLETE };
    int state;

    int msg_length = 0;

    if (mb_param->debug)
    {
        if (msg_length_computed == MSG_LENGTH_UNDEFINED)
            wprintf ("\nWaiting for a message...\n");
        else
            wprintf ("\nWaiting for a message (%d bytes)...\n", msg_length_computed);
    }

    /* Add a file descriptor to the set */
    FD_ZERO (&rfds);
    FD_SET (mb_param->fd, &rfds);

    if (msg_length_computed == MSG_LENGTH_UNDEFINED)
    {
        /* Wait for a message */
        tv.tv_sec = 5;
        tv.tv_usec = 0;

        /* The message length is undefined (query receiving) so
        * we need to analyse the message step by step.
        * At the first step, we want to reach the function
        * code because all packets have that information. */
        state = FUNCTION;
        msg_length_computed = TAB_HEADER_LENGTH[mb_param->type_com] + 1;
    }
    else
    {
        tv.tv_sec = 0;
        /*tv.tv_usec = TIME_OUT_BEGIN_OF_TRAME;*/
        tv.tv_usec = select_time * 1000;
        state = COMPLETE;
    }

    length_to_read = msg_length_computed;

    select_ret = 0;
    WAIT_DATA();

    p_msg = msg;
    if (mb_param->debug)
    {
        wprintf ("\033[32;40;1m \nrcv:\033[0m");
    }
    while (select_ret)
    {
        if (mb_param->type_com == RTU)
            read_ret = read (mb_param->fd, p_msg, length_to_read);
        else
            read_ret = recv (mb_param->fd, p_msg, length_to_read, 0);

        if (read_ret == 0)
        {
            return CONNECTION_CLOSED;
        }
        else if (read_ret < 0)
        {
            /* The only negative possible value is -1 */
            error_treat (SOCKET_FAILURE, "receive_msg: Read socket failure");
            return SOCKET_FAILURE;
        }

        /* Sums bytes received */
        msg_length += read_ret;

        /* Display the hex code of each character received */
        if (mb_param->debug)
        {
            int i;
            for (i = 0; i < read_ret; i++)
                wprintf ("<%.2X>", p_msg[i]);
        }

        if (msg_length < msg_length_computed)
        {
            /* Message incomplete */
            length_to_read = msg_length_computed - msg_length;
        }
        else
        {
            switch (state)
            {
            case FUNCTION:
                /* Function code position */
                length_to_read = compute_query_length_header (msg[TAB_HEADER_LENGTH[mb_param->type_com]]);
                msg_length_computed += length_to_read;
                /* It's useless to check the value of
                msg_length_computed in this case (only
                defined values are used). */
                state = BYTE;
                break;
            case BYTE:
                length_to_read = compute_query_length_data (msg);
                msg_length_computed += length_to_read;
                if (msg_length_computed > TAB_MAX_ADU_LENGTH[mb_param->type_com])
                {
                    error_treat (INVALID_DATA, "receive: Too many data");
                    return INVALID_DATA;
                }
                state = COMPLETE;
                break;
            case COMPLETE:
                /*printf ( "\n" );*/
                length_to_read = 0;
                break;
            }
        } // end else

        /* Moves the pointer to receive other data */
        p_msg = & (p_msg[read_ret]);

        if (length_to_read > 0)
        {
            /* If no character at the buffer wait
            TIME_OUT_END_OF_TRAME before to generate an error. */
            tv.tv_sec = 0;
            tv.tv_usec = TIME_OUT_END_OF_TRAME;

            WAIT_DATA();
        }
        else
        {
            /* All chars are received */
            select_ret = FALSE;
        }
    } // end while

    if (mb_param->debug)
    {
        wprintf ("\n");
    }

    if (mb_param->type_com == RTU)
    {
        /* Returns msg_length on success and a negative value on
        failure */
        return check_crc16 (msg, msg_length);
    }
    else
    {
        /* OK */
        return msg_length;
    }
}

int c_modbus::rcv_msg (uint8_t *msg, int select_time, int wait_time)
{
    int select_ret;
    int read_ret;
    fd_set rfds;
    struct timeval tv;
    int msg_length;
    int length_to_read = 1024;
    uint8_t *p_msg;
    enum { FUNCTION, BYTE, COMPLETE };

    if (mb_param->debug)
    {
        wprintf ("\nWaiting for a message...\n");
    }

    /* Add a file descriptor to the set */
    FD_ZERO (&rfds);
    FD_SET (mb_param->fd, &rfds);

    tv.tv_sec = 0;
    tv.tv_usec = select_time * 1000;
    WAIT_DATA();
    modbus_sleep (0, wait_time * 1000);

    p_msg = msg;
    if (mb_param->debug)
    {
        wprintf ("\033[32;40;1m \nrcv:\033[0m");
    }
    if (mb_param->type_com == RTU)
    {
        read_ret = read (mb_param->fd, p_msg, length_to_read);
    }
    else
    {
        read_ret = recv (mb_param->fd, p_msg, length_to_read, 0);
    }
    if (read_ret == 0)
    {
        return CONNECTION_CLOSED;
    }
    else if (read_ret < 0)
    {
        /* The only negative possible value is -1 */
        error_treat (SOCKET_FAILURE, "receive_msg: Read socket failure");
        return SOCKET_FAILURE;
    }

    /* Display the hex code of each character received */
    if (mb_param->debug)
    {
        int i;
        for (i = 0; i < read_ret; i++)
        {
            wprintf ("<%.2X>", p_msg[i]);
        }
    }

    if (mb_param->debug)
    {
        wprintf ("\n");
    }

    if (mb_param->type_com == RTU)
    {
        /* Returns msg_length on success and a negative value on
        failure */
        return check_crc16 (msg, read_ret);
    }
    else
    {
        /* OK */
        return read_ret;
    }
}

/* Listens for any query from a modbus master in TCP, requires the socket file
   descriptor etablished with the master device in argument or -1 to use the
   internal one of modbus_param_t.

   Returns:
   - byte length of the message on success, or a negative error number if the
     request fails
   - query, message received
*/
int c_modbus::modbus_slave_receive (int sockfd, uint8_t *query,
                                    int select_time)
{
    if (sockfd != -1)
    {
        mb_param->fd = sockfd;
    }

    /* The length of the query to receive isn't known. */
    return receive_msg (MSG_LENGTH_UNDEFINED, query, select_time);
}

/* Receives the response and checks values (and checksum in RTU).

   Returns:
   - the number of values (bits or words) if success or the response
     length if no value is returned
   - less than 0 for exception errors

   Note: all functions used to send or receive data with modbus return
   these values. */
int c_modbus::modbus_receive (uint8_t *query, uint8_t *response,
                              uint8_t data_type, int select_time)
{
    int ret;
    int times = 0; // 记录超时次数
    int response_length_computed;
    int offset = TAB_HEADER_LENGTH[mb_param->type_com];

again:
    response_length_computed = compute_response_length (query, data_type);
    ret = receive_msg (response_length_computed, response, select_time);
    if (ret >= 0)
    {
        /* GOOD RESPONSE */
        int query_nb_value;
        int response_nb_value;

        /* The number of values is returned if it's corresponding
        * to the query */
        switch (response[offset])
        {
        case FC_READ_COIL_STATUS:
        case FC_READ_INPUT_STATUS:
            /* Read functions, 8 values in a byte (nb
            * of values in the query and byte count in
            * the response. */
            query_nb_value = (query[offset + 3] << 8) + query[offset + 4];
            query_nb_value = (query_nb_value / 8) + ( (query_nb_value % 8) ? 1 : 0);
            response_nb_value = response[offset + 1];
            break;
        case FC_READ_HOLDING_REGISTERS:
        case FC_READ_INPUT_REGISTERS:
            query_nb_value = (query[offset + 3] << 8) + query[offset + 4];
            switch (data_type)
            {
            case    0:      // 数据类型为8位整数
            case    1:      // 数据类型为8位无符号整数
                /* Read functions 1 value = 1 bytes */
                response_nb_value = response[offset + 1];
                /*ret = MB_EXCEPTION;*/
                break;
            case    2:      // 数据类型为16位整数
            case    3:      // 数据类型为16位无符号整数
                /* Read functions 1 value = 2 bytes */
                response_nb_value = (response[offset + 1] / 2);
                break;
            case    4:      // 数据类型为32位整数
            case    5:      // 数据类型为32位无符号整数
                /* Read functions 1 value = 4 bytes */
                response_nb_value = (response[offset + 1] / 4);
                break;
            case    6:      // 数据类型为64位整数
            case    7:      // 数据类型为64位无符号整数
                ret = MB_EXCEPTION;
                break;
            case    8:      // 单精度浮点数
                ret = MB_EXCEPTION;
                break;
            case    9:      // 双精度浮点数
                ret = MB_EXCEPTION;
                break;
            default:
                ret = MB_EXCEPTION;
                break;
            }
            break;
        case FC_FORCE_MULTIPLE_COILS:
        case FC_PRESET_MULTIPLE_REGISTERS:
            /* N Write functions */
            query_nb_value = (query[offset + 3] << 8) + query[offset + 4];
            response_nb_value = (response[offset + 3] << 8) | response[offset + 4];
            break;
        case FC_REPORT_SLAVE_ID:
            /* Report slave ID (bytes received) */
            query_nb_value = response_nb_value = ret;
            break;
        default:
            /* 1 Write functions & others */
            query_nb_value = response_nb_value = 1;
        } // end switch (response[offset])

        if (query_nb_value == response_nb_value)
        {
            ret = response_nb_value;
        }
        else
        {
            /*char *s_error = ( char* ) malloc ( 64 * sizeof ( char ) );*/
            char s_error[128];
            sprintf (s_error, "Quantity not corresponding to the query (%d != %d)", response_nb_value, query_nb_value);
            ret = INVALID_DATA;
            error_treat (ret, s_error);
            /*free ( s_error );*/
        }
    } // end if (ret >= 0)
    else if (ret == MB_EXCEPTION)
    {
        /* EXCEPTION CODE RECEIVED */

        /* CRC must be checked here (not done in receive_msg) */
        if (mb_param->type_com == RTU)
        {
            ret = check_crc16 (response, EXCEPTION_RESPONSE_LENGTH_RTU);
            if (ret < 0)
                return ret;
        }

        /* Check for exception response.
        0x80 + function is stored in the exception
        response. */
        if (0x80 + query[offset] == response[offset])
        {
            int exception_code = response[offset + 1];
            // FIXME check test
            if (exception_code < NB_TAB_ERROR_MSG)
            {
                error_treat (-exception_code, TAB_ERROR_MSG[response[offset + 1]]);
                /* RETURN THE EXCEPTION CODE */
                /* Modbus error code is negative */
                return -exception_code;
            }
            else
            {
                /* The chances are low to hit this
                case but it can avoid a vicious
                segfault */
                /*char *s_error = ( char* ) malloc ( 64 * sizeof ( char ) );*/
                char s_error[128];
                sprintf (s_error,
                         "Invalid exception code %d",
                         response[offset + 1]);
                error_treat (INVALID_EXCEPTION_CODE,
                             s_error);
                /*free ( s_error );*/
                return INVALID_EXCEPTION_CODE;
            }
        }
    }
    else if (ret == SELECT_TIMEOUT)
    {
        if (++times <= 1)
        {
            error_treat (ret, "modbus_receive");
            /*wprintf ( "\033[33;32;1m select_timeout!\n\033[0m" );*/
            /*goto again;*/
        }
        else /* Other errors */
            error_treat (ret, "modbus_receive");
    }

    return ret;
}

int c_modbus::response_io_status (int address, int nb,
                                  uint8_t *tab_io_status,
                                  uint8_t *response, int offset)
{
    int shift = 0;
    int byte = 0;
    int i;

    for (i = address; i < address + nb; i++)
    {
        byte |= tab_io_status[i] << shift;
        if (shift == 7)
        {
            /* Byte is full */
            response[offset++] = byte;
            byte = shift = 0;
        }
        else
        {
            shift++;
        }
    }

    if (shift != 0)
        response[offset++] = byte;

    return offset;
}

/* Build the exception response */
int c_modbus::response_exception (sft_t *sft, int exception_code, uint8_t *response)
{
    int response_length;

    sft->function = sft->function + 0x80;
    response_length = build_response_basis (sft, response);

    /* Positive exception code */
    response[response_length++] = -exception_code;

    return response_length;
}

/* Manages the received query.
   Analyses the query and constructs a response.

   If an error occurs, this function construct the response
   accordingly.
*/
void c_modbus::modbus_slave_manage (const uint8_t *query,
                                    int query_length, modbus_mapping_t *mb_mapping)
{
    int offset = TAB_HEADER_LENGTH[mb_param->type_com];
    int slave = query[offset - 1];
    int function = query[offset];
    uint16_t address = (query[offset + 1] << 8) + query[offset + 2];
    uint8_t response[MAX_MESSAGE_LENGTH];
    int resp_length = 0;
    sft_t sft;

    if (slave != mb_param->slave && slave != MODBUS_BROADCAST_ADDRESS)
    {
        // Ignores the query (not for me)
        if (mb_param->debug)
        {
            wprintf ("Request for slave %d ignored (not %d)\n",
                     slave, mb_param->slave);
        }
        return;
    }

    sft.slave = slave;
    sft.function = function;
    if (mb_param->type_com == TCP)
    {
        sft.t_id = (query[0] << 8) + query[1];
    }
    else
    {
        sft.t_id = 0;
        query_length -= CHECKSUM_LENGTH_RTU;
    }

    switch (function)
    {
    case FC_READ_COIL_STATUS:
    {
        int nb = (query[offset + 3] << 8) + query[offset + 4];

        if ( (address + nb) > mb_mapping->nb_coil_status)
        {
            wprintf ("Illegal data address %0X in read_coil_status\n",
                     address + nb);
            resp_length = response_exception (&sft, ILLEGAL_DATA_ADDRESS, response);
        }
        else
        {
            resp_length = build_response_basis (&sft, response);
            response[resp_length++] = (nb / 8) + ( (nb % 8) ? 1 : 0);
            resp_length = response_io_status (address, nb,
                                              mb_mapping->tab_coil_status,
                                              response, resp_length);
        }
    }
    break;

    case FC_READ_INPUT_STATUS:
    {
        /* Similar to coil status (but too much arguments to use a
         * function) */
        int nb = (query[offset + 3] << 8) + query[offset + 4];

        if ( (address + nb) > mb_mapping->nb_input_status)
        {
            wprintf ("Illegal data address %0X in read_input_status\n",
                     address + nb);
            resp_length = response_exception (&sft, ILLEGAL_DATA_ADDRESS, response);
        }
        else
        {
            resp_length = build_response_basis (&sft, response);
            response[resp_length++] = (nb / 8) + ( (nb % 8) ? 1 : 0);
            resp_length = response_io_status (address, nb,
                                              mb_mapping->tab_input_status,
                                              response, resp_length);
        }
    }
    break;

    case FC_READ_HOLDING_REGISTERS:
    {
        int nb = (query[offset + 3] << 8) + query[offset + 4];

        if ( (address + nb) > mb_mapping->nb_holding_registers)
        {
            wprintf ("Illegal data address %0X in read_holding_registers\n",
                     address + nb);
            resp_length = response_exception (&sft, ILLEGAL_DATA_ADDRESS, response);
        }
        else
        {
            int i;

            resp_length = build_response_basis (&sft, response);
            response[resp_length++] = nb << 1;
            for (i = address; i < address + nb; i++)
            {
                response[resp_length++] = mb_mapping->tab_holding_registers[i] >> 8;
                response[resp_length++] = mb_mapping->tab_holding_registers[i] & 0xFF;
            }
        }
    }
    break;

    case FC_READ_INPUT_REGISTERS:
    {
        /* Similar to holding registers (but too much arguments to use a
         * function) */
        int nb = (query[offset + 3] << 8) + query[offset + 4];

        if ( (address + nb) > mb_mapping->nb_input_registers)
        {
            wprintf ("Illegal data address %0X in read_input_registers\n",
                     address + nb);
            resp_length = response_exception (&sft, ILLEGAL_DATA_ADDRESS, response);
        }
        else
        {
            int i;

            resp_length = build_response_basis (&sft, response);
            response[resp_length++] = nb << 1;
            for (i = address; i < address + nb; i++)
            {
                response[resp_length++] = mb_mapping->tab_input_registers[i] >> 8;
                response[resp_length++] = mb_mapping->tab_input_registers[i] & 0xFF;
            }
        }
    }
    break;

    case FC_FORCE_SINGLE_COIL:
    {
        if (address >= mb_mapping->nb_coil_status)
        {
            wprintf ("Illegal data address %0X in force_singe_coil\n", address);
            resp_length = response_exception (&sft, ILLEGAL_DATA_ADDRESS, response);
        }
        else
        {
            int data = (query[offset + 3] << 8) + query[offset + 4];

            if (data == 0xFF00 || data == 0x0)
            {
                mb_mapping->tab_coil_status[address] = (data) ? ON : OFF;

                /* In RTU mode, the CRC is computed and added
                   to the query by modbus_send, the computed
                   CRC will be same and optimisation is
                   possible here (FIXME). */
                memcpy (response, query, query_length);
                resp_length = query_length;
            }
            else
            {
                wprintf ("Illegal data value %0X in force_single_coil request at address %0X\n",
                         data, address);
                resp_length = response_exception (&sft, ILLEGAL_DATA_VALUE, response);
            }
        }
    }
    break;

    case FC_PRESET_SINGLE_REGISTER:
        if (address >= mb_mapping->nb_holding_registers)
        {
            wprintf ("Illegal data address %0X in preset_holding_register\n", address);
            resp_length = response_exception (&sft, ILLEGAL_DATA_ADDRESS, response);
        }
        else
        {
            int data = (query[offset + 3] << 8) + query[offset + 4];

            mb_mapping->tab_holding_registers[address] = data;
            memcpy (response, query, query_length);
            resp_length = query_length;
        }
        break;
    case FC_FORCE_MULTIPLE_COILS:
    {
        int nb = (query[offset + 3] << 8) + query[offset + 4];

        if ( (address + nb) > mb_mapping->nb_coil_status)
        {
            wprintf ("Illegal data address %0X in force_multiple_coils\n",
                     address + nb);
            resp_length = response_exception (&sft, ILLEGAL_DATA_ADDRESS, response);
        }
        else
        {
            /* 6 = byte count */
            set_bits_from_bytes (mb_mapping->tab_coil_status, address, nb, &query[offset + 6]);

            resp_length = build_response_basis (&sft, response);
            /* 4 to copy the coil address (2) and the quantity of coils */
            memcpy (response + resp_length, query + resp_length, 4);
            resp_length += 4;
        }
    }
    break;
    case FC_PRESET_MULTIPLE_REGISTERS:
    {
        int nb = (query[offset + 3] << 8) + query[offset + 4];

        if ( (address + nb) > mb_mapping->nb_holding_registers)
        {
            wprintf ("Illegal data address %0X in preset_multiple_registers\n",
                     address + nb);
            resp_length = response_exception (&sft, ILLEGAL_DATA_ADDRESS, response);
        }
        else
        {
            int i, j;
            for (i = address, j = 6; i < address + nb; i++, j += 2)
            {
                /* 6 and 7 = first value */
                mb_mapping->tab_holding_registers[i] =
                    (query[offset + j] << 8) + query[offset + j + 1];
            }

            resp_length = build_response_basis (&sft, response);
            /* 4 to copy the address (2) and the no. of registers */
            memcpy (response + resp_length, query + resp_length, 4);
            resp_length += 4;
        }
    }
    break;
    case FC_READ_EXCEPTION_STATUS:
    case FC_REPORT_SLAVE_ID:
        wprintf ("Not implemented\n");
        break;
    }

    modbus_send (response, resp_length);
}

/* Reads IO status */
int c_modbus::read_io_status (int function,
                              int start_addr, int nb, uint8_t *data_dest, int select_time)
{
    int ret;
    int query_length;

    uint8_t query[MIN_QUERY_LENGTH];
    uint8_t response[MAX_MESSAGE_LENGTH];

    query_length = build_query_basis (function, start_addr, nb, query);

    ret = modbus_send (query, query_length);
    if (ret > 0)
    {
        int i, temp, bit;
        int pos = 0;
        int offset;
        int offset_end;

        ret = modbus_receive (query, response, UINT16, select_time);
        if (ret < 0)
        {
            return ret;
        }

        offset = TAB_HEADER_LENGTH[mb_param->type_com];
        offset_end = offset + ret;
        for (i = offset; i < offset_end; i++)
        {
            /* Shift reg hi_byte to temp */
            temp = response[i + 2];

            for (bit = 0x01; (bit & 0xff) && (pos < nb);)
            {
                data_dest[pos++] = (temp & bit) ? TRUE : FALSE;
                bit = bit << 1;
            }

        }
    }

    return ret;
}

/* Reads the boolean status of coils and sets the array elements
   in the destination to TRUE or FALSE. */
int c_modbus::read_coil_status (int start_addr,
                                int nb, uint8_t *data_dest, int select_time)
{
    int status;

    if (nb > MAX_STATUS)
    {
        fprintf (stderr,
                 "ERROR Too many coils status requested (%d > %d)\n",
                 nb, MAX_STATUS);
        return INVALID_DATA;
    }

    status = read_io_status (FC_READ_COIL_STATUS, start_addr, nb, data_dest, select_time);
    if (status > 0)
    {
        status = nb;
    }

    return status;
}

/* Same as read_coil_status but reads the slaves input table */
int c_modbus::read_input_status (int start_addr,
                                 int nb, uint8_t *data_dest, int select_time)
{
    int status;

    if (nb > MAX_STATUS)
    {
        fprintf (stderr,
                 "ERROR Too many input status requested (%d > %d)\n",
                 nb, MAX_STATUS);
        return INVALID_DATA;
    }

    status = read_io_status (FC_READ_INPUT_STATUS,
                             start_addr, nb, data_dest, select_time);

    if (status > 0)
    {
        status = nb;
    }

    return status;
}

/* Reads the data from a modbus slave and put that data into an array */
int c_modbus::read_registers (int function, int start_addr,
                              int nb, uint32_t *data_dest, uint8_t data_type, int select_time)
{
    int ret;
    int query_length;
    uint8_t query[MIN_QUERY_LENGTH];
    uint8_t response[MAX_MESSAGE_LENGTH * 4];

    if (nb > MAX_REGISTERS)
    {
        fprintf (stderr, "ERROR Too many holding registers requested (%d > %d)\n", nb, MAX_REGISTERS);
        return INVALID_DATA;
    }

    query_length = build_query_basis (function, start_addr, nb, query);

    ret = modbus_send (query, query_length);
    if (ret > 0)
    {
        int i;
        int offset;

        ret = modbus_receive (query, response, data_type, select_time);

        offset = TAB_HEADER_LENGTH[mb_param->type_com];

        /* If ret is negative, the loop is jumped ! */
        for (i = 0; i < ret; i++)
        {
            switch (data_type)
            {
            case    0:      // 数据类型为8位整数
                data_dest[i] = (int8_t) (response[offset + 2 + i]);
            case    1:      // 数据类型为8位无符号整数
                data_dest[i] = (uint8_t) (response[offset + 2 + i]);
                break;
            case    2:      // 数据类型为16位整数
                data_dest[i] = (int16_t) ( (response[offset + 2 + (i << 1) ] << 8)
                                           | response[offset + 3 + (i << 1)]);
            case    3:      // 数据类型为16位无符号整数
                data_dest[i] = (uint16_t) ( (response[offset + 2 + (i << 1) ] << 8)
                                            | response[offset + 3 + (i << 1)]);
                break;
            case    4:      // 数据类型为32位整数
                data_dest[i] = (int32_t) ( (response[offset + 2 + (i << 2) ] << 24)
                                           | (response[offset + 3 + (i << 2) ] << 16)
                                           | (response[offset + 4 + (i << 2) ] << 8)
                                           | response[offset + 5 + (i << 2) ]);
            case    5:      // 数据类型为32位无符号整数
                data_dest[i] = (uint32_t) ( (response[offset + 2 + (i << 2) ] << 24)
                                            | (response[offset + 3 + (i << 2) ] << 16)
                                            | (response[offset + 4 + (i << 2) ] << 8)
                                            | response[offset + 5 + (i << 2) ]);
                break;
            case    6:      // 数据类型为64位整数
            case    7:      // 数据类型为64位无符号整数
                //ret = MB_EXCEPTION;
                //return ret;
            case    8:      // 单精度浮点数
                //ret = MB_EXCEPTION;
                //break;
            case    9:      // 双精度浮点数
                //ret = MB_EXCEPTION;
                //return ret;
            default:
                data_dest[i] = (uint32_t) ( (response[offset + 2 + (i << 2) ] << 24)
                                            | (response[offset + 3 + (i << 2) ] << 16)
                                            | (response[offset + 4 + (i << 2) ] << 8)
                                            | response[offset + 5 + (i << 2) ]);
                //ret = MB_EXCEPTION;
                //return ret;
            }
        }
    } // end if

    return ret;
}

/* Reads the holding registers in a slave and put the data into an
   array */
int c_modbus::read_holding_registers (int start_addr, int nb,
                                      uint32_t *data_dest, uint8_t data_type, int select_time)
{
    int status;

    if (nb > MAX_REGISTERS)
    {
        fprintf (stderr,
                 "ERROR Too many holding registers requested (%d > %d)\n",
                 nb, MAX_REGISTERS);
        return INVALID_DATA;
    }

    status = read_registers (FC_READ_HOLDING_REGISTERS, start_addr, nb, data_dest, data_type, select_time);
    return status;
}

/* Reads the input registers in a slave and put the data into
   an array */
int c_modbus::read_input_registers (int start_addr, int nb,
                                    uint16_t *data_dest, int select_time)
{
    int status;

    if (nb > MAX_REGISTERS)
    {
        fprintf (stderr,
                 "ERROR Too many input registers requested (%d > %d)\n",
                 nb, MAX_REGISTERS);
        return INVALID_DATA;
    }

    status = read_registers (FC_READ_INPUT_REGISTERS,
                             start_addr, nb, (uint32_t*) data_dest, UINT16, select_time);

    return status;
}

/* Sends a value to a register in a slave.
   Used by force_single_coil and preset_single_register */
int c_modbus::set_single (int function, int addr, int value,
                          int select_time)
{
    int ret;
    int query_length;
    uint8_t query[MIN_QUERY_LENGTH];

    query_length = build_query_basis (function, addr, value, query);

    ret = modbus_send (query, query_length);
    if (ret > 0)
    {
        /* Used by force_single_coil and
        * preset_single_register */
        uint8_t response[MIN_QUERY_LENGTH];
        ret = modbus_receive (query, response, UINT16, select_time);
    }

    return ret;
}

/* Turns ON or OFF a single coil in the slave device */
int c_modbus::force_single_coil (int coil_addr, int state, int select_time)
{
    int status;

    printf ("force_single_coil ..\n");
    if (state)
    {
        state = 0xFF00;
    }

    mb_param->debug = 1;
    status = set_single (FC_FORCE_SINGLE_COIL, coil_addr, state, select_time);
    mb_param->debug = 0;

    return status;
}

/* Sets a value in one holding register in the slave device */
int c_modbus::preset_single_register (int reg_addr, int value, int select_time)
{
    int status;

    status = set_single (FC_PRESET_SINGLE_REGISTER, reg_addr, value, select_time);

    return status;
}

/* Sets/resets the coils in the slave from an array in argument */
int c_modbus::force_multiple_coils (int start_addr, int nb,
                                    const uint8_t *data_src, int select_time)
{
    int ret;
    int i;
    int byte_count;
    int query_length;
    int coil_check = 0;
    int pos = 0;

    uint8_t query[MAX_MESSAGE_LENGTH];

    if (nb > MAX_STATUS)
    {
        fprintf (stderr, "ERROR Writing to too many coils (%d > %d)\n",
                 nb, MAX_STATUS);
        return INVALID_DATA;
    }

    query_length = build_query_basis (FC_FORCE_MULTIPLE_COILS,
                                      start_addr, nb, query);
    byte_count = (nb / 8) + ( (nb % 8) ? 1 : 0);
    query[query_length++] = byte_count;

    for (i = 0; i < byte_count; i++)
    {
        int bit;

        bit = 0x01;
        query[query_length] = 0;

        while ( (bit & 0xFF) && (coil_check++ < nb))
        {
            if (data_src[pos++])
                query[query_length] |= bit;
            else
                query[query_length] &= ~ bit;

            bit = bit << 1;
        }
        query_length++;
    }

    ret = modbus_send (query, query_length);
    if (ret > 0)
    {
        uint8_t response[MAX_MESSAGE_LENGTH];
        ret = modbus_receive (query, response, UINT16, select_time);
    }


    return ret;
}

/* Copies the values in the slave from the array given in argument */
int c_modbus::preset_multiple_registers (int start_addr, int nb,
        const uint16_t *data_src, int select_time)
{
    int ret;
    int i;
    int query_length;
    int byte_count;

    uint8_t query[MAX_MESSAGE_LENGTH];

    if (nb > MAX_REGISTERS)
    {
        fprintf (stderr,
                 "ERROR Trying to write to too many registers (%d > %d)\n",
                 nb, MAX_REGISTERS);
        return INVALID_DATA;
    }

    query_length = build_query_basis (FC_PRESET_MULTIPLE_REGISTERS, start_addr, nb, query);
    byte_count = nb * 2;
    query[query_length++] = byte_count;

    for (i = 0; i < nb; i++)
    {
        query[query_length++] = data_src[i] >> 8;
        query[query_length++] = data_src[i] & 0x00FF;
    }

    ret = modbus_send (query, query_length);
    if (ret > 0)
    {
        uint8_t response[MAX_MESSAGE_LENGTH];
        ret = modbus_receive (query, response, UINT16, select_time);
    }

    return ret;
}

/* Returns the slave id! */
int c_modbus::report_slave_id (uint8_t *data_dest, int select_time)
{
    int ret;
    int query_length;
    uint8_t query[MIN_QUERY_LENGTH];

    query_length = build_query_basis (FC_REPORT_SLAVE_ID, 0, 0, query);

    /* HACKISH, start_addr and count are not used */
    query_length -= 4;

    ret = modbus_send (query, query_length);
    if (ret > 0)
    {
        int i;
        int offset;
        int offset_end;
        uint8_t response[MAX_MESSAGE_LENGTH];

        /* Byte count, slave id, run indicator status,
           additional data */
        ret = modbus_receive (query, response, UINT16, select_time);
        if (ret < 0)
            return ret;

        offset = TAB_HEADER_LENGTH[mb_param->type_com] - 1;
        offset_end = offset + ret;

        for (i = offset; i < offset_end; i++)
            data_dest[i] = response[i];
    }

    return ret;
}

/* Initializes the modbus_param_t structure for RTU
   - device: "/dev/ttyS0"
   - baud:   9600, 19200, 57600, 115200, etc
   - parity: "even", "odd" or "none"
   - data_bits: 5, 6, 7, 8
   - stop_bits: 1, 2
*/
void c_modbus::modbus_init_rtu (const char *device,
                                int baud, const char *parity, int data_bit,
                                int stop_bit, int slave)
{
    memset (mb_param, 0, sizeof (modbus_param_t));
    strcpy (mb_param->device, device);
    mb_param->baud = baud;
    strcpy (mb_param->parity, parity);
    mb_param->debug = FALSE;
    mb_param->data_bit = data_bit;
    mb_param->stop_bit = stop_bit;
    mb_param->type_com = RTU;
    mb_param->error_handling = FLUSH_OR_CONNECT_ON_ERROR;
    mb_param->slave = slave;


    //--->liblog

    /* Initialize the logger module. */
    /*logger_init (LOG_LEVEL_DEBUG);*/

    /* Log to stderr. */
    /*logger_add_fp_handler (stderr);*/

    /* Add a callback handler. */
    /*logger_add_callback_handler (log_callback, NULL);*/

    /* Add a rotating file handler. */
    /*logger_add_rotating_handler ("/var/log/wcx_modbus.log", 5000, 3);*/
}

/* Initializes the modbus_param_t structure for TCP.
   - ip : "192.168.0.5"
   - port : 1099

   Set the port to MODBUS_TCP_DEFAULT_PORT to use the default one
   (502). It's convenient to use a port number greater than or equal
   to 1024 because it's not necessary to be root to use this port
   number.
*/
void c_modbus::modbus_init_tcp (const char *ip, int port, int slave)
{
    memset (mb_param, 0, sizeof (modbus_param_t));
    strncpy (mb_param->ip, ip, sizeof (char) * 16);
    mb_param->port = port;
    mb_param->type_com = TCP;
    mb_param->error_handling = FLUSH_OR_CONNECT_ON_ERROR;
    mb_param->slave = slave;


    //--->liblog

    /* Initialize the logger module. */
    /*logger_init (LOG_LEVEL_DEBUG);*/

    /* Log to stderr. */
    /*logger_add_fp_handler (stderr);*/

    /* Add a callback handler. */
    /*logger_add_callback_handler (log_callback, NULL);*/

    /* Add a rotating file handler. */
    /*logger_add_rotating_handler ("/var/log/wcx_modbus.log", 5000, 3);*/
}

/* Define the slave number.
   The special value MODBUS_BROADCAST_ADDRESS can be used. */
void c_modbus::modbus_set_slave (int slave)
{
    mb_param->slave = slave;
}

/* By default, the error handling mode used is FLUSH_OR_CONNECT_ON_ERROR.

   With FLUSH_OR_CONNECT_ON_ERROR, the library will attempt an immediate
   reconnection which may hang for several seconds if the network to
   the remote target unit is down.

   With NOP_ON_ERROR, it is expected that the application will
   check for error returns and deal with them as necessary.
*/
void c_modbus::modbus_set_error_handling (error_handling_t error_handling)
{
    if (error_handling == FLUSH_OR_CONNECT_ON_ERROR ||
            error_handling == NOP_ON_ERROR)
    {
        mb_param->error_handling = error_handling;
    }
    else
    {
        fprintf (stderr,
                 "Invalid setting for error handling (not changed)\n");
    }
}


/* Sets up a serial port for RTU communications */
int c_modbus::modbus_connect_rtu ()
{
    struct termios tios;
    speed_t speed;

    if (mb_param->debug)
    {
        fprintf (stderr, "Opening %s at %d bauds (%s)\n", mb_param->device, mb_param->baud, mb_param->parity);
    }

    /* The O_NOCTTY flag tells UNIX that this program doesn't want
       to be the "controlling terminal" for that port. If you
       don't specify this then any input (such as keyboard abort
       signals and so forth) will affect your process

       Timeouts are ignored in canonical input mode or when the
       NDELAY option is set on the file via open or fcntl */
    mb_param->fd = open (mb_param->device, O_RDWR | O_NOCTTY | O_NDELAY | O_EXCL);
    if (mb_param->fd < 0)
    {
        fprintf (stderr, "ERROR Can't open the device %s (%s)\n", mb_param->device, strerror (errno));
        return -1;
    }

    /* Save */
    tcgetattr (mb_param->fd, & (mb_param->old_tios));

    memset (&tios, 0, sizeof (struct termios));

    /* C_ISPEED     Input baud (new interface)
       C_OSPEED     Output baud (new interface)
    */
    switch (mb_param->baud)
    {
    case 110:
        speed = B110;
        break;
    case 300:
        speed = B300;
        break;
    case 600:
        speed = B600;
        break;
    case 1200:
        speed = B1200;
        break;
    case 2400:
        speed = B2400;
        break;
    case 4800:
        speed = B4800;
        break;
    case 9600:
        speed = B9600;
        break;
    case 19200:
        speed = B19200;
        break;
    case 38400:
        speed = B38400;
        break;
    case 57600:
        speed = B57600;
        break;
    case 115200:
        speed = B115200;
        break;
    default:
        speed = B9600;
        fprintf (stderr, "WARNING Unknown baud rate %d for %s (B9600 used)\n", mb_param->baud, mb_param->device);
    }

    /* Set the baud rate */
    if ( (cfsetispeed (&tios, speed) < 0) || (cfsetospeed (&tios, speed) < 0))
    {
        perror ("cfsetispeed/cfsetospeed\n");
        return -1;
    }

    /* C_CFLAG      Control options
       CLOCAL       Local line - do not change "owner" of port
       CREAD        Enable receiver
    */
    tios.c_cflag |= (CREAD | CLOCAL);
    /* CSIZE, HUPCL, CRTSCTS (hardware flow control) */

    /* Set data bits (5, 6, 7, 8 bits)
       CSIZE        Bit mask for data bits
    */
    tios.c_cflag &= ~CSIZE;
    switch (mb_param->data_bit)
    {
    case 5:
        tios.c_cflag |= CS5;
        break;
    case 6:
        tios.c_cflag |= CS6;
        break;
    case 7:
        tios.c_cflag |= CS7;
        break;
    case 8:
    default:
        tios.c_cflag |= CS8;
        break;
    }

    /* Stop bit (1 or 2) */
    if (mb_param->stop_bit == 1)
        tios.c_cflag &= ~ CSTOPB;
    else /* 2 */
        tios.c_cflag |= CSTOPB;

    /* PARENB       Enable parity bit
       PARODD       Use odd parity instead of even */
    if (strncmp (mb_param->parity, "none", 4) == 0)
    {
        tios.c_cflag &= ~ PARENB;
    }
    else if (strncmp (mb_param->parity, "even", 4) == 0)
    {
        tios.c_cflag |= PARENB;
        tios.c_cflag &= ~ PARODD;
    }
    else
    {
        /* odd */
        tios.c_cflag |= PARENB;
        tios.c_cflag |= PARODD;
    }

    /* Read the man page of termios if you need more information. */

    /* This field isn't used on POSIX systems
       tios.c_line = 0;
    */

    /* C_LFLAG      Line options

       ISIG Enable SIGINTR, SIGSUSP, SIGDSUSP, and SIGQUIT signals
       ICANON       Enable canonical input (else raw)
       XCASE        Map uppercase \lowercase (obsolete)
       ECHO Enable echoing of input characters
       ECHOE        Echo erase character as BS-SP-BS
       ECHOK        Echo NL after kill character
       ECHONL       Echo NL
       NOFLSH       Disable flushing of input buffers after
       interrupt or quit characters
       IEXTEN       Enable extended functions
       ECHOCTL      Echo control characters as ^char and delete as ~?
       ECHOPRT      Echo erased character as character erased
       ECHOKE       BS-SP-BS entire line on line kill
       FLUSHO       Output being flushed
       PENDIN       Retype pending input at next read or input char
       TOSTOP       Send SIGTTOU for background output

       Canonical input is line-oriented. Input characters are put
       into a buffer which can be edited interactively by the user
       until a CR (carriage return) or LF (line feed) character is
       received.

       Raw input is unprocessed. Input characters are passed
       through exactly as they are received, when they are
       received. Generally you'll deselect the ICANON, ECHO,
       ECHOE, and ISIG options when using raw input
    */

    /* Raw input */
    tios.c_lflag &= ~ (ICANON | ECHO | ECHOE | ISIG);

    /* C_IFLAG      Input options

       Constant     Description
       INPCK        Enable parity check
       IGNPAR       Ignore parity errors
       PARMRK       Mark parity errors
       ISTRIP       Strip parity bits
       IXON Enable software flow control (outgoing)
       IXOFF        Enable software flow control (incoming)
       IXANY        Allow any character to start flow again
       IGNBRK       Ignore break condition
       BRKINT       Send a SIGINT when a break condition is detected
       INLCR        Map NL to CR
       IGNCR        Ignore CR
       ICRNL        Map CR to NL
       IUCLC        Map uppercase to lowercase
       IMAXBEL      Echo BEL on input line too long
    */
    if (strncmp (mb_param->parity, "none", 4) == 0)
    {
        tios.c_iflag &= ~INPCK;
    }
    else
    {
        tios.c_iflag |= INPCK;
    }

    /* Software flow control is disabled */
    tios.c_iflag &= ~ (IXON | IXOFF | IXANY);

    /* C_OFLAG      Output options
       OPOST        Postprocess output (not set = raw output)
       ONLCR        Map NL to CR-NL

       ONCLR ant others needs OPOST to be enabled
    */

    /* Raw ouput */
    tios.c_oflag &= ~ OPOST;

    /* C_CC         Control characters
       VMIN         Minimum number of characters to read
       VTIME        Time to wait for data (tenths of seconds)

       UNIX serial interface drivers provide the ability to
       specify character and packet timeouts. Two elements of the
       c_cc array are used for timeouts: VMIN and VTIME. Timeouts
       are ignored in canonical input mode or when the NDELAY
       option is set on the file via open or fcntl.

       VMIN specifies the minimum number of characters to read. If
       it is set to 0, then the VTIME value specifies the time to
       wait for every character read. Note that this does not mean
       that a read call for N bytes will wait for N characters to
       come in. Rather, the timeout will apply to the first
       character and the read call will return the number of
       characters immediately available (up to the number you
       request).

       If VMIN is non-zero, VTIME specifies the time to wait for
       the first character read. If a character is read within the
       time given, any read will block (wait) until all VMIN
       characters are read. That is, once the first character is
       read, the serial interface driver expects to receive an
       entire packet of characters (VMIN bytes total). If no
       character is read within the time allowed, then the call to
       read returns 0. This method allows you to tell the serial
       driver you need exactly N bytes and any read call will
       return 0 or N bytes. However, the timeout only applies to
       the first character read, so if for some reason the driver
       misses one character inside the N byte packet then the read
       call could block forever waiting for additional input
       characters.

       VTIME specifies the amount of time to wait for incoming
       characters in tenths of seconds. If VTIME is set to 0 (the
       default), reads will block (wait) indefinitely unless the
       NDELAY option is set on the port with open or fcntl.
    */
    /* Unused because we use open with the NDELAY option */
    tios.c_cc[VMIN] = 0;
    tios.c_cc[VTIME] = 0;

    /*处理未接收字符*/
    tcflush (mb_param->fd, TCIFLUSH);

    if (tcsetattr (mb_param->fd, TCSANOW, &tios) < 0)
    {
        perror ("tcsetattr\n");
        return -1;
    }



    return 0;
}

/* Establishes a modbus TCP connection with a modbus slave */
int c_modbus::modbus_connect_tcp ()
{
    int ret;
    int option;
    struct sockaddr_in addr;

    mb_param->fd = socket (PF_INET, SOCK_STREAM, 0);
    if (mb_param->fd < 0)
    {
        return mb_param->fd;
    }

    /* Set the TCP no delay flag */
    /* SOL_TCP = IPPROTO_TCP */
    option = 1;
    ret = setsockopt (mb_param->fd, IPPROTO_TCP, TCP_NODELAY, (const void *) &option, sizeof (int));
    if (ret < 0)
    {
        perror ("setsockopt TCP_NODELAY");
        close (mb_param->fd);
        return ret;
    }

#if (!HAVE_DECL___CYGWIN__)
    /**
     * Cygwin defines IPTOS_LOWDELAY but can't handle that flag so it's
     * necessary to workaround that problem.
     **/
    /* Set the IP low delay option */
    option = IPTOS_LOWDELAY;
    ret = setsockopt (mb_param->fd, IPPROTO_IP, IP_TOS, (const void *) &option, sizeof (int));
    if (ret < 0)
    {
        perror ("setsockopt IP_TOS");
        close (mb_param->fd);
        return ret;
    }
#endif

    if (mb_param->debug)
    {
        wprintf ("Connecting to %s\n", mb_param->ip);
    }

    addr.sin_family = AF_INET;
    addr.sin_port = htons (mb_param->port);
    addr.sin_addr.s_addr = inet_addr (mb_param->ip);
    ret = connect (mb_param->fd, (struct sockaddr *) &addr, sizeof (struct sockaddr_in));
    if (ret < 0)
    {
        perror ("connect");
        close (mb_param->fd);
        return ret;
    }

    return 0;
}

/* Establishes a modbus connexion.
   Returns 0 on success or -1 on failure. */
int c_modbus::modbus_connect ()
{
    int ret;

    if (mb_param->type_com == RTU)
        ret = modbus_connect_rtu ();
    else
        ret = modbus_connect_tcp ();

    return ret;
}

/* Closes the file descriptor in RTU mode */
void c_modbus::modbus_close_rtu ()
{
    if (tcsetattr (mb_param->fd, TCSANOW, & (mb_param->old_tios)) < 0)
        perror ("tcsetattr");

    close (mb_param->fd);
}

/* Closes the network connection and socket in TCP mode */
void c_modbus::modbus_close_tcp ()
{
    shutdown (mb_param->fd, SHUT_RDWR);
    close (mb_param->fd);
}

/* Closes a modbus connection */
void c_modbus::modbus_close ()
{
    if (mb_param->type_com == RTU)
        modbus_close_rtu ();
    else
        modbus_close_tcp ();
}

/* Activates the debug messages */
void c_modbus::modbus_set_debug (int boolean)
{
    mb_param->debug = boolean;

    /* Log to stderr. */
    /*logger_add_fp_handler (stderr);*/
}

/* Allocates 4 arrays to store coils, input status, input registers and
   holding registers. The pointers are stored in modbus_mapping structure.

   Returns 0 on success and -1 on failure.
*/
int c_modbus::modbus_mapping_new (modbus_mapping_t *mb_mapping,
                                  int nb_coil_status, int nb_input_status,
                                  int nb_holding_registers, int nb_input_registers)
{
    /* 0X */
    mb_mapping->nb_coil_status = nb_coil_status;
    mb_mapping->tab_coil_status =
        (uint8_t *) malloc (nb_coil_status * sizeof (uint8_t));
    memset (mb_mapping->tab_coil_status, 0,
            nb_coil_status * sizeof (uint8_t));
    if (mb_mapping->tab_coil_status == NULL)
        return -1;

    /* 1X */
    mb_mapping->nb_input_status = nb_input_status;
    mb_mapping->tab_input_status =
        (uint8_t *) malloc (nb_input_status * sizeof (uint8_t));
    memset (mb_mapping->tab_input_status, 0,
            nb_input_status * sizeof (uint8_t));
    if (mb_mapping->tab_input_status == NULL)
    {
        free (mb_mapping->tab_coil_status);
        return -1;
    }

    /* 4X */
    mb_mapping->nb_holding_registers = nb_holding_registers;
    mb_mapping->tab_holding_registers =
        (uint16_t *) malloc (nb_holding_registers * sizeof (uint16_t));
    memset (mb_mapping->tab_holding_registers, 0,
            nb_holding_registers * sizeof (uint16_t));
    if (mb_mapping->tab_holding_registers == NULL)
    {
        free (mb_mapping->tab_coil_status);
        free (mb_mapping->tab_input_status);
        return -1;
    }

    /* 3X */
    mb_mapping->nb_input_registers = nb_input_registers;
    mb_mapping->tab_input_registers =
        (uint16_t *) malloc (nb_input_registers * sizeof (uint16_t));
    memset (mb_mapping->tab_input_registers, 0,
            nb_input_registers * sizeof (uint16_t));
    if (mb_mapping->tab_input_registers == NULL)
    {
        free (mb_mapping->tab_coil_status);
        free (mb_mapping->tab_input_status);
        free (mb_mapping->tab_holding_registers);
        return -1;
    }

    return 0;
}

/* Frees the 4 arrays */
void c_modbus::modbus_mapping_free (modbus_mapping_t *mb_mapping)
{
    free (mb_mapping->tab_coil_status);
    free (mb_mapping->tab_input_status);
    free (mb_mapping->tab_holding_registers);
    free (mb_mapping->tab_input_registers);
}

/* Listens for any query from one or many modbus masters in TCP */
int c_modbus::modbus_slave_listen_tcp (int nb_connection)
{
    int new_socket;
    int yes;
    struct sockaddr_in addr;

    new_socket = socket (PF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (new_socket < 0)
    {
        perror ("socket");
        return -1;
    }

    yes = 1;
    if (setsockopt (new_socket, SOL_SOCKET, SO_REUSEADDR, (char *) &yes, sizeof (yes)) < 0)
    {
        perror ("setsockopt");
        close (new_socket);
        return -1;
    }

    memset (&addr, 0, sizeof (addr));
    addr.sin_family = AF_INET;
    /* If the modbus port is < to 1024, we need the setuid root. */
    addr.sin_port = htons (mb_param->port);
    addr.sin_addr.s_addr = INADDR_ANY;
    if (bind (new_socket, (struct sockaddr *) &addr, sizeof (addr)) < 0)
    {
        perror ("bind");
        close (new_socket);
        return -1;
    }

    if (listen (new_socket, nb_connection) < 0)
    {
        perror ("listen");
        close (new_socket);
        return -1;
    }

    return new_socket;
}

int c_modbus::modbus_slave_accept_tcp (int *socket)
{
    struct sockaddr_in addr;
    socklen_t addrlen;

    addrlen = sizeof (struct sockaddr_in);
again:
    mb_param->fd = accept (*socket, (struct sockaddr *) &addr, &addrlen);
    /*
    if (mb_param->fd < 0)
    {
            perror("accept");
            close(*socket);
            *socket = 0;
    }
    else
    {
            wprintf("The client %s is connected\n", inet_ntoa(addr.sin_addr));
    }
    */

    /* changeed by wcx */
    if (mb_param->fd < 0)
    {
        if ( (errno == ECONNABORTED) || (errno == EINTR))
        {
            wprintf ("Listen Again ...\n");
            goto again;
        }
        else
        {
            perror ("accept");
            close (*socket);
            *socket = 0;
        }
    }
    else
    {
        wprintf ("The client %s is connected\n", inet_ntoa (addr.sin_addr));
    }

    return mb_param->fd;
}

/* Closes a TCP socket */
void c_modbus::modbus_slave_close_tcp (int socket)
{
    shutdown (socket, SHUT_RDWR);
    close (socket);
}

/** Utils **/

/* Sets many input/coil status from a single byte value (all 8 bits of
   the byte value are set) */
void c_modbus::set_bits_from_byte (uint8_t *dest, int address, const uint8_t value)
{
    int i;

    for (i = 0; i < 8; i++)
    {
        dest[address + i] = (value & (1 << i)) ? ON : OFF;
    }
}

/* Sets many input/coil status from a table of bytes (only the bits
   between address and address + nb_bits are set) */
void c_modbus::set_bits_from_bytes (uint8_t *dest, int address, int nb_bits,
                                    const uint8_t tab_byte[])
{
    int i;
    int shift = 0;

    for (i = address; i < address + nb_bits; i++)
    {
        dest[i] = tab_byte[ (i - address) / 8] & (1 << shift) ? ON : OFF;
        /* gcc doesn't like: shift = (++shift) % 8; */
        shift++;
        shift %= 8;
    }
}

/* Gets the byte value from many input/coil status.
   To obtain a full byte, set nb_bits to 8. */
uint8_t c_modbus::get_byte_from_bits (const uint8_t *src, int address, int nb_bits)
{
    int i;
    uint8_t value = 0;

    if (nb_bits > 8)
    {
        fprintf (stderr, "ERROR nb_bits is too big\n");
        nb_bits = 8;
    }

    for (i = 0; i < nb_bits; i++)
    {
        value |= (src[address + i] << i);
    }

    return value;
}

/* Read a float from 4 bytes in Modbus format */
float c_modbus::modbus_read_float (const uint16_t *src)
{
    float r = 0.0f;
    uint32_t i;

    i = ( ( (uint32_t) src[1]) << 16) + src[0];
    memcpy (&r, &i, sizeof (r));

    return r;
}

/* Write a float to 4 bytes in Modbus format */
void c_modbus::modbus_write_float (float real, uint16_t *dest)
{
    uint32_t i = 0;

    memcpy (&i, &real, sizeof (i));
    dest[0] = (uint16_t) i;
    dest[1] = (uint16_t) (i >> 16);
}
