#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include <string.h>
#include <loragw_com.h>
#include <loragw_aux.h>
#include <loragw_hal.h>

#define BUFF_SIZE_SPI   (1024)

#define SX1302_AGC_MCU_MEM  (0x0000)
#define SX1302_REG_COMMON   (0x5600)
#define SX1302_REG_AGC_MCU  (0x5780)

#define SX1250_PWR_EN   (33)
#define SX1302_RESET    (25)

#define SX1302_MISO (19)
#define SX1302_MOSI (23)
#define SX1302_CLK  (18)
#define SX1302_CS   (5)

static spi_device_handle_t spi_hdlr;

/* Buffers */
static uint8_t * test_buff = NULL;
static uint8_t * test_buff_backup = NULL;
static uint8_t * read_buff = NULL;

uint16_t max_buff_size;
uint8_t data = 0;
uint16_t size;
int cycle_number;

static void reset_sx1302(void)
{
    // Reset LoraGW
    gpio_set_direction(SX1302_RESET, GPIO_MODE_OUTPUT);
    gpio_set_direction(SX1250_PWR_EN, GPIO_MODE_OUTPUT);

    printf("SX1302 PWR ON\n");
    gpio_set_level(SX1250_PWR_EN, 1);
    vTaskDelay( 500 / portTICK_PERIOD_MS );

    printf("SX1302 Reset\n");
    gpio_set_level(SX1302_RESET, 1);
    vTaskDelay( 500 / portTICK_PERIOD_MS );
    gpio_set_level(SX1302_RESET, 0);
    vTaskDelay( 500 / portTICK_PERIOD_MS );
}

static void init_spi(void)
{
    esp_err_t ret;
    spi_bus_config_t buscfg;
    memset(&buscfg, 0, sizeof(spi_bus_config_t));
    buscfg.miso_io_num = SX1302_MISO;
    buscfg.mosi_io_num = SX1302_MOSI;
    buscfg.sclk_io_num = SX1302_CLK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 10000;
    
    spi_device_interface_config_t devcfg;
    memset(&devcfg, 0, sizeof(spi_device_interface_config_t));
    devcfg.clock_speed_hz = 2*1000*1000;
    devcfg.mode = 0;
    devcfg.spics_io_num = SX1302_CS;
    devcfg.queue_size = 7;

    //Initialize the SPI bus
    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI2_HOST);
    ESP_ERROR_CHECK(ret);
    //devcfg.flags |= SPI_DEVICE_BIT_LSBFIRST;
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi_hdlr);
    ESP_ERROR_CHECK(ret);
    printf("SPI Init\n");
}

void setup(void)
{
    int x;
    reset_sx1302();
    init_spi();

    printf("Beginning of test for loragw_com\n");
    x = lgw_com_open(&spi_hdlr);

    x = lgw_com_r(LGW_SPI_MUX_TARGET_SX1302, SX1302_REG_COMMON + 6, &data);
    if (x != 0)
    {
        printf("ERROR: failed to read register %d\n", __LINE__);
        while(1);
    }
    printf("SX1302 version: 0x%02X\n", data);

    x = lgw_com_r(LGW_SPI_MUX_TARGET_SX1302, SX1302_REG_AGC_MCU + 0, &data);
    if (x != 0)
    {
        printf("ERROR: failed to read register %d\n", __LINE__);
        while(1);
    }
    x = lgw_com_w(LGW_SPI_MUX_TARGET_SX1302, SX1302_REG_AGC_MCU + 0, 0x06); /* mcu_clear, host_prog */
    if (x != 0)
    {
        printf("ERROR: failed to write register %d\n", __LINE__);
        while(1);
    }

    /* Allocate buffers according to com type capabilities */
    max_buff_size = BUFF_SIZE_SPI;
    test_buff = (uint8_t*)malloc(max_buff_size * sizeof(uint8_t));
    if (test_buff == NULL)
    {
        printf("ERROR: failed to allocate memory for test_buff\n");
        while(1);
    }
    test_buff_backup = (uint8_t*)malloc(max_buff_size * sizeof(uint8_t));
    if (test_buff_backup == NULL)
    {
        printf("ERROR: failed to allocate memory for test_buff\n");
        while(1);
    }
    read_buff = (uint8_t*)malloc(max_buff_size * sizeof(uint8_t));
    if (read_buff == NULL)
    {
        printf("ERROR: failed to allocate memory for read_buff \n");
        while(1);
    }
}

void lgw_com_test(void)
{
    int x;

    /*************************************************
     *
     *      WRITE SINGLE BYTE TEST
     *
     * ***********************************************/
    /* Single byte r/w test */
    printf("Cycle> %d\n", cycle_number);

    test_buff[0] = random() % 255;
    /* Write single byte */
    x = lgw_com_w(LGW_SPI_MUX_TARGET_SX1302, SX1302_AGC_MCU_MEM, test_buff[0]);
    if (x != 0)
    {
        printf("ERROR: failed to write register %d\n", __LINE__);
        while(1);
    }

    /* Read back */
    x = lgw_com_r(LGW_SPI_MUX_TARGET_SX1302, SX1302_AGC_MCU_MEM, &read_buff[0]);
    if (x != 0)
    {
        printf("ERROR: failed to read register %d\n", __LINE__);
        while(1);
    }

    printf("Write[%02X], Read[%02X]\n", test_buff[0], read_buff[0]);
    /* Compare read / write bytes */
    if (test_buff[0] != read_buff[0])
    {
        printf("error during the byte comparison\n");
        /* exit */
        //while(1);
    }
    else
    {
        printf("did a 1-byte R/W on a data buffer with no error\n");
        ++cycle_number;
    }
    
    /*************************************************
     *
     *      WRITE BURST TEST
     *
     * ***********************************************/
    size = random() % max_buff_size;
    for (int i = 0; i < size; ++i)
    {
        test_buff[i] = random() % 255;
        test_buff_backup[i] = test_buff[i];
    }
    printf("Cycle> %d\n", cycle_number);

    /* Write burst with random data */
    x = lgw_com_wb(LGW_SPI_MUX_TARGET_SX1302, SX1302_AGC_MCU_MEM, test_buff, size);
    if (x != 0)
    {
        printf("ERROR: failed to write register %d\n", __LINE__);
        while(1);
    }

    /* Read back */
    memset(read_buff, 0, size);
    x = lgw_com_rb(LGW_SPI_MUX_TARGET_SX1302, SX1302_AGC_MCU_MEM, read_buff, size);
    if (x != 0)
    {
        printf("ERROR: failed to read register %d\n", __LINE__);
        while(1);
    }

    /* Compare read / write buffers */
    int cmpIdx = 0;
    for (; ((cmpIdx < size) && (test_buff_backup[cmpIdx] == read_buff[cmpIdx])); ++cmpIdx);
    if (cmpIdx != size)
    {
        printf("error during the buffer comparison\n");

        /* Print what has been written */
        printf("Written values[%d]:\n", size);
        for (int i=0; i<size; ++i)
        {
            printf("%02X ", test_buff_backup[i]);
            if (i%16 == 15)
            {
                printf("\n");
            }
        }
        printf("\n");

        /* Print what has been read back */
        printf("Read values[%d]:\n", size);
        for (int i=0; i<size; ++i)
        {
            printf("%02X ", read_buff[i]);
            if (i%16 == 15)
            {
                printf("\n");
            }
        }
        printf("\n");

        /* exit */
        //while(1);
    }
    else
    {
        printf("did a R/W on a data buffer[%dB] with no error\n", size);
        ++cycle_number;
    }
}
#include "esp_log.h"
void app_main(void)
{
    esp_log_level_set("spi_master", ESP_LOG_VERBOSE);
    setup();

    srand((int)time(NULL));
    while (1)
    {
        lgw_com_test();
        vTaskDelay( 1000 / portTICK_PERIOD_MS );
    }
}

