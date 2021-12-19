#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include <string.h>
#include <loragw_com.h>
#include <loragw_aux.h>
#include <loragw_hal.h>

#include <loragw_reg.h>
#include <loragw_sx1250.h>
#include <loragw_sx1302.h>

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
#define BUFF_SIZE 16
uint8_t test_buff[BUFF_SIZE];
uint8_t read_buff[BUFF_SIZE];

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

void com_1250test_setup(void)
{
    int i;
    int x;
    reset_sx1302();
    init_spi();

    printf("Beginning of test for loragw_com_sx1250\n");

    x = lgw_connect(&spi_hdlr);

    /* Reset radios */
    for (i = 0; i < LGW_RF_CHAIN_NB; i++) {
        sx1302_radio_reset(i, LGW_RADIO_TYPE_SX1250);
        sx1302_radio_set_mode(i, LGW_RADIO_TYPE_SX1250);
    }

    /* Select the radio which provides the clock to the sx1302 */
    sx1302_radio_clock_select(0);

    /* Ensure we can control the radio */
    lgw_reg_w(SX1302_REG_COMMON_CTRL0_HOST_RADIO_CTRL, 0x01);

    /* Ensure PA/LNA are disabled */
    lgw_reg_w(SX1302_REG_AGC_MCU_CTRL_FORCE_HOST_FE_CTRL, 1);
    lgw_reg_w(SX1302_REG_AGC_MCU_RF_EN_A_PA_EN, 0);
    lgw_reg_w(SX1302_REG_AGC_MCU_RF_EN_A_LNA_EN, 0);

    /* Set Radio in Standby mode */
    test_buff[0] = (uint8_t)STDBY_XOSC;
    x = sx1250_reg_w(SET_STANDBY, test_buff, 1, 0);
    if (x != LGW_REG_SUCCESS) {
        printf("ERROR(%d): Failed to configure sx1250_0\n", __LINE__);
        while(1);
    }
    x = sx1250_reg_w(SET_STANDBY, test_buff, 1, 1);
    if (x != LGW_REG_SUCCESS) {
        printf("ERROR(%d): Failed to configure sx1250_1\n", __LINE__);
        while(1);
    }
    wait_ms(10);

    test_buff[0] = 0x00;
    x = sx1250_reg_r(GET_SX1250_STATUS, test_buff, 1, 0);
    if (x != LGW_REG_SUCCESS) {
        printf("ERROR(%d): Failed to get sx1250_0 status\n", __LINE__);
        while(1);
    }
    printf("Radio0: get_status: 0x%02X\n", test_buff[0]);

    test_buff[0] = 0x00;
    x = sx1250_reg_r(GET_SX1250_STATUS, test_buff, 1, 1);
    if (x != LGW_REG_SUCCESS) {
        printf("ERROR(%d): Failed to get sx1250_1 status\n", __LINE__);
        while(1);
    }
    printf("Radio1: get_status: 0x%02X\n", test_buff[0]);
}

void lgw_com_1250test(void)
{
    uint32_t test_val, read_val;

    /* databuffer R/W stress test */
    test_buff[0] = rand() & 0x7F;
    test_buff[1] = rand() & 0xFF;
    test_buff[2] = rand() & 0xFF;
    test_buff[3] = rand() & 0xFF;
    test_val = (test_buff[0] << 24) | (test_buff[1] << 16) | (test_buff[2] << 8) | (test_buff[3] << 0);
    sx1250_reg_w(SET_RF_FREQUENCY, test_buff, 4, 0);

    read_buff[0] = 0x08;
    read_buff[1] = 0x8B;
    read_buff[2] = 0x00;
    read_buff[3] = 0x00;
    read_buff[4] = 0x00;
    read_buff[5] = 0x00;
    read_buff[6] = 0x00;
    sx1250_reg_r(READ_REGISTER, read_buff, 7, 0);
    read_val = (read_buff[3] << 24) | (read_buff[4] << 16) | (read_buff[5] << 8) | (read_buff[6] << 0);

    printf("Cycle %i > ", cycle_number);
    if (read_val != test_val)
    {
        printf("error during the buffer comparison\n");
        printf("Written value: %08X\n", test_val);
        printf("Read value:    %08X\n", read_val);
    }
    else
    {
        printf("did a %i-byte R/W on a register with no error\n", 4);
        ++cycle_number;
    }
}

#include "esp_log.h"
void app_main(void)
{
    esp_log_level_set("spi_master", ESP_LOG_VERBOSE);
    com_1250test_setup();

    srand((int)time(NULL));
    while (1)
    {
        lgw_com_1250test();
        vTaskDelay( 1000 / portTICK_PERIOD_MS );
    }
}

