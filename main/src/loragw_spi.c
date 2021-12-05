/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    Host specific functions to address the LoRa concentrator registers through
    a SPI interface.
    Single-byte read/write and burst read/write.
    Could be used with multiple SPI ports in parallel (explicit file descriptor)

License: Revised BSD License, see LICENSE.TXT file include in the project
*/


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>     /* C99 types */
#include <stdio.h>      /* printf fprintf */
#include <stdlib.h>     /* malloc free */
#include <unistd.h>     /* lseek, close */
#include <fcntl.h>      /* open */
#include <string.h>     /* memset */

#include "loragw_spi.h"
#include "loragw_aux.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"
/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#if DEBUG_COM == 1
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

#define READ_ACCESS     0x00
#define WRITE_ACCESS    0x80

#define LGW_BURST_CHUNK (1024)

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* Simple write */
int lgw_spi_w(uint8_t spi_mux_target, uint16_t address, uint8_t data, spi_device_handle_t *spi)
{
    uint8_t out_buf[4];
    uint8_t command_size;

    /* prepare frame to be sent */
    out_buf[0] = spi_mux_target;
    out_buf[1] = WRITE_ACCESS | ((address >> 8) & 0x7F);
    out_buf[2] = ((address >> 0) & 0xFF);
    out_buf[3] = data;
    command_size = 4;

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    t.length = command_size*8;
    t.tx_buffer = out_buf;
    esp_err_t ret = spi_device_transmit(*spi, &t);

    return (ESP_OK == ret) ? LGW_SPI_SUCCESS : LGW_SPI_ERROR;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* Simple read */
int lgw_spi_r(uint8_t spi_mux_target, uint16_t address, uint8_t *data, spi_device_handle_t *spi)
{
    uint8_t out_buf[5];
    uint8_t command_size;
    uint8_t in_buf[ARRAY_SIZE(out_buf)];

    /* prepare frame to be sent */
    out_buf[0] = spi_mux_target;
    out_buf[1] = READ_ACCESS | ((address >> 8) & 0x7F);
    out_buf[2] = ((address >> 0) & 0xFF);
    out_buf[3] = 0x00;
    out_buf[4] = 0x00;
    command_size = 5;

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
        *data = in_buf[command_size - 1];
        return LGW_SPI_SUCCESS;
    }
    else
    {
        printf("Error in r 0x%X\n", ret);
        return LGW_SPI_ERROR;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* Single Byte Read-Modify-Write */
int lgw_spi_rmw(uint8_t spi_mux_target, uint16_t address, uint8_t offs, uint8_t leng, uint8_t data, spi_device_handle_t *spi)
{
    int spi_stat = LGW_SPI_SUCCESS;
    uint8_t buf[4] = {0};

    /* Read */
    spi_stat += lgw_spi_r(spi_mux_target, address, &buf[0], spi);

    /* Modify */
    buf[1] = ((1 << leng) - 1) << offs; /* bit mask */
    buf[2] = ((uint8_t)data) << offs; /* new data offsetted */
    buf[3] = (~buf[1] & buf[0]) | (buf[1] & buf[2]); /* mixing old & new data */

    /* Write */
    spi_stat += lgw_spi_w(spi_mux_target, address, buf[3], spi);

    return spi_stat;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* Burst (multiple-byte) write */
int lgw_spi_wb(uint8_t spi_mux_target, uint16_t address, uint8_t *data, uint16_t size, spi_device_handle_t *spi)
{
    uint8_t command[3];
    uint8_t command_buffer[3+LGW_BURST_CHUNK];
    uint8_t command_size;
    esp_err_t ret;

    /* check input parameters */
    CHECK_NULL(data);
    if (size == 0)
    {
        DEBUG_MSG("ERROR: BURST OF NULL LENGTH\n");
        return LGW_SPI_ERROR;
    }
    if (size > LGW_BURST_CHUNK)
    {
        printf("spi_wb size[%d] too big, asign to %d, may lose some datas\n", size, LGW_BURST_CHUNK);
        size = LGW_BURST_CHUNK;
    }

    memset(command_buffer, 0, sizeof(command_buffer));
    /* prepare command byte */
    command[0] = spi_mux_target;
    command[1] = WRITE_ACCESS | ((address >> 8) & 0x7F);
    command[2] = ((address >> 0) & 0xFF);
    command_size = 3;
    memcpy(command_buffer, command, command_size);

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    //cmd & data
    memcpy(&command_buffer[command_size], data, size);
    t.length = (command_size + size) * 8;
    t.tx_buffer = command_buffer;

    ret = spi_device_transmit(*spi, &t);
    if (ESP_OK != ret)
    {
        printf("Error in wb send data 0x%X\n", ret);
        return LGW_SPI_ERROR;
    }

    return LGW_SPI_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* Burst (multiple-byte) read */
int lgw_spi_rb(uint8_t spi_mux_target, uint16_t address, uint8_t *data, uint16_t size, spi_device_handle_t *spi)
{
    uint8_t command[4+LGW_BURST_CHUNK];
    uint8_t command_size;

    uint8_t in_buf[ARRAY_SIZE(command)];
    esp_err_t ret;

    /* check input parameters */
    CHECK_NULL(data);
    if (size == 0)
    {
        DEBUG_MSG("ERROR: BURST OF NULL LENGTH\n");
        return LGW_SPI_ERROR;
    }
    if (size > LGW_BURST_CHUNK)
    {
        printf("spi_rb size[%d] too big, asign to %d, may lose some datas\n", size, LGW_BURST_CHUNK);
        size = LGW_BURST_CHUNK;
    }

    /* prepare command byte */
    memset(command, 0, sizeof(command));
    command[0] = spi_mux_target;
    command[1] = READ_ACCESS | ((address >> 8) & 0x7F);
    command[2] = ((address >> 0) & 0xFF);
    command[3] = 0x00;
    command_size = 4 + size;

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

#if 0
    //cmd & rx data
    t.length = command_size * 8;
    t.tx_buffer = command;
    t.rx_buffer = in_buf;
    ret = spi_device_transmit(*spi, &t);
    if (ESP_OK != ret)
    {
        printf("Error in rb send cmd 0x%X\n", ret);
        return LGW_SPI_ERROR;
    }
    memcpy(data, &in_buf[4], size);
#else
    for (int idx = 0; idx < size; ++idx)
    {
        lgw_spi_r(spi_mux_target, address+idx, &data[idx], spi);
    }
#endif

    return LGW_SPI_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
uint16_t lgw_spi_chunk_size(void)
{
    return (uint16_t)LGW_BURST_CHUNK;
}

/* --- EOF ------------------------------------------------------------------ */
