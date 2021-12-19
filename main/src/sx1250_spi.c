/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    Functions used to handle LoRa concentrator SX1250 radios.

License: Revised BSD License, see LICENSE.TXT file include in the project
*/


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>     /* C99 types */
#include <stdio.h>      /* printf fprintf */
#include <unistd.h>     /* lseek, close */
#include <fcntl.h>      /* open */
#include <string.h>     /* memset */


#include "loragw_spi.h"
#include "loragw_aux.h"
#include "sx1250_spi.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#if DEBUG_RAD == 1
    #define DEBUG_MSG(str)                fprintf(stdout, str)
    #define DEBUG_PRINTF(fmt, args...)    fprintf(stdout,"%s:%d: "fmt, __FUNCTION__, __LINE__, args)
    #define CHECK_NULL(a)                if(a==NULL){fprintf(stderr,"%s:%d: ERROR: NULL POINTER AS ARGUMENT\n", __FUNCTION__, __LINE__);return LGW_SPI_ERROR;}
#else
    #define DEBUG_MSG(str)
    #define DEBUG_PRINTF(fmt, args...)
    #define CHECK_NULL(a)                if(a==NULL){return LGW_SPI_ERROR;}
#endif

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define WAIT_BUSY_SX1250_MS  1

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */
int sx1250_spi_w(uint8_t spi_mux_target, sx1250_op_code_t op_code, uint8_t *data, uint16_t size, void *spi_)
{
    spi_device_handle_t *spi = (spi_device_handle_t*)spi_;

    int cmd_size = 2; /* header + op_code */
    uint8_t out_buf[cmd_size + size];
    uint8_t command_size;
    int i;

    /* wait BUSY */
    wait_ms(WAIT_BUSY_SX1250_MS);

    /* check input variables */
    CHECK_NULL(data);

    /* prepare frame to be sent */
    out_buf[0] = spi_mux_target;
    out_buf[1] = (uint8_t)op_code;
    for (i = 0; i < (int)size; i++)
    {
        out_buf[cmd_size + i] = data[i];
    }
    command_size = cmd_size + size;

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    t.length = command_size*8;
    t.tx_buffer = out_buf;
    esp_err_t ret = spi_device_transmit(*spi, &t);

    return (ESP_OK == ret) ? LGW_SPI_SUCCESS : LGW_SPI_ERROR;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1250_spi_r(uint8_t spi_mux_target, sx1250_op_code_t op_code, uint8_t *data, uint16_t size, void *spi_)
{
    spi_device_handle_t *spi = (spi_device_handle_t*)spi_;

    int cmd_size = 2; /* header + op_code + NOP */
    uint8_t out_buf[cmd_size + size];
    uint8_t command_size;
    uint8_t in_buf[ARRAY_SIZE(out_buf)];
    int i;

    /* wait BUSY */
    wait_ms(WAIT_BUSY_SX1250_MS);

    /* check input variables */
    CHECK_NULL(data);

    /* prepare frame to be sent */
    out_buf[0] = spi_mux_target;
    out_buf[1] = (uint8_t)op_code;
    for(i = 0; i < (int)size; i++)
    {
        out_buf[cmd_size + i] = data[i];
    }
    command_size = cmd_size + size;

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    t.length = command_size*8;
    t.tx_buffer = out_buf;
    t.rx_buffer = in_buf;
    #if 0
    esp_err_t ret = spi_device_polling_transmit(*spi, &t);
    #else
    esp_err_t ret = spi_device_transmit(*spi, &t);
    #endif

    if (ESP_OK == ret)
    {
        memcpy(data, in_buf + cmd_size, size);
        return LGW_SPI_SUCCESS;
    }
    else
    {
        printf("Error in r 0x%X\n", ret);
        return LGW_SPI_ERROR;
    }
}

/* --- EOF ------------------------------------------------------------------ */
