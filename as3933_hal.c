/*
 * as3933_hal.c
 *
 *  Created on: 23 maj 2020
 *      Author: kurza
 */

/*********************
 *      INCLUDES
 *********************/
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include "esp_log.h"
#include "esp_err.h"

#include "spi_master_ext.h"
#include "as3933_hal.h"
#include "as3933_defs.h"

/*********************
 *      DEFINES
 *********************/
#define ESP_INTR_FLAG_DEFAULT 0

/**********************
 *      TYPEDEFS
 **********************/
typedef struct
{
	spi_device_handle_t spi_handle;
	as3933_cfg_t spi_config;
} as3933_dev_t;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void IRAM_ATTR as3933_w_up_gpio_isr_handler(void *arg);
static esp_err_t _gpio_init(gpio_num_t data, gpio_num_t w_up);

/**********************
 *  STATIC VARIABLES
 **********************/
static const char *TAG = "as3933_hal";

static spi_device_handle_t as3933_spi_handle;
static w_up_callback_t w_up_callback = NULL;

/**********************
 *      MACROS
 **********************/
#define CHECK(a, ret_val, str, ...)                                           \
    if (!(a))                                                                 \
    {                                                                         \
        ESP_LOGE(TAG, "%s(%u): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
        return (ret_val);                                                     \
    }

#define ASSERT(con)                                                                     \
    do                                                                                  \
    {                                                                                   \
        if (!(con))                                                                     \
        {                                                                               \
            ESP_LOGE(TAG, "assert errno:%d, errno_str: !(%s)", errno, strerror(errno)); \
            assert(0 && #con);                                                          \
        }                                                                               \
    } while (0)

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
as3933_handle_t as3933_create(const as3933_cfg_t *spi_cfg)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	as3933_dev_t *as3933_dev = (as3933_dev_t*) calloc(1, sizeof(as3933_dev_t));
	CHECK((as3933_dev != NULL), NULL, "AS3933 ALLOC FAIL");

	spi_bus_config_t buscfg =
		{
			.miso_io_num = spi_cfg->miso,
			.mosi_io_num = spi_cfg->mosi,
			.sclk_io_num = spi_cfg->sclk,
			.quadwp_io_num = -1,
			.quadhd_io_num = -1,
		};

	spi_device_interface_config_t devcfg =
		{
			.clock_speed_hz = spi_cfg->spi_clock_speed_hz, 				// SPI_MASTER_FREQ_10M / 2, //Clock out at 10 MHz
			.mode = 1,                                     				//SPI mode 1
			.spics_io_num = spi_cfg->cs,                   				//CS pin
			.flags = SPI_DEVICE_POSITIVE_CS,
			.queue_size = 1, 											//We want to be able to queue 7 transactions at a time
			.command_bits = 2,
			.address_bits = 6,
		};

	esp_err_t error = ESP_OK;
	if (spi_handle[spi_cfg->spi_host] == NULL)
	{
		//Initialize the SPI bus
		ESP_LOGD(TAG, "Initialization SPI%d", spi_cfg->spi_host + 1);
		error = spi_bus_initialize(spi_cfg->spi_host, &buscfg, 0);
		CHECK((error == ESP_OK), NULL, "SPI device %d initialize fail", spi_cfg->spi_host);
	}

	//Attach the Device to the SPI bus
	error = spi_bus_add_device(spi_cfg->spi_host, &devcfg, &as3933_spi_handle);
	CHECK((error == ESP_OK), NULL, "SPI device %d add fail", spi_cfg->spi_host);

	as3933_dev->spi_config = *spi_cfg;
	as3933_dev->spi_handle = as3933_spi_handle;

	if (spi_handle[spi_cfg->spi_host] == NULL)
	{
		spi_handle[spi_cfg->spi_host] = as3933_spi_handle;
	}

	error = _gpio_init(spi_cfg->data, spi_cfg->w_up);
	CHECK((error == ESP_OK), NULL, "GPIO INIT FAIL");

	return as3933_dev;
}

esp_err_t as3933_w_up_intr_init(gpio_num_t w_up_gpio, w_up_callback_t cb)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	CHECK((cb != NULL), ESP_ERR_INVALID_ARG, "CALLBACK FUNC NOT EXIST");
	CHECK(GPIO_IS_VALID_GPIO(w_up_gpio), ESP_ERR_INVALID_ARG, "WRONG GPIO NUM");

    gpio_config_t gpio_in_conf =
	{
		.mode = GPIO_MODE_INPUT,
		.intr_type = GPIO_INTR_ANYEDGE,
		.pin_bit_mask = (1ULL << w_up_gpio),
		.pull_down_en = GPIO_PULLDOWN_ENABLE,
		.pull_up_en = GPIO_PULLUP_DISABLE, };

    esp_err_t ret = gpio_config(&gpio_in_conf);
    CHECK(ret == ESP_OK, ret, "GPIO %d CONFIG FAIL", w_up_gpio);
    //install gpio isr service
    ret = gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //CHECK(ret == ESP_OK, ret, "GPIO %d ISR SERVICE FAIL", w_up_gpio);
    //hook isr handler for specific gpio pin
    ret = gpio_isr_handler_add(w_up_gpio, as3933_w_up_gpio_isr_handler, (void*) w_up_gpio);
    CHECK(ret == ESP_OK, ret, "GPIO %d ISR HANDLER ADD FAIL", w_up_gpio);
	w_up_callback = cb;

	return ESP_OK;
}

esp_err_t as3933_switch(as3933_handle_t handle)
{
	CHECK((handle != NULL), ESP_ERR_INVALID_STATE, "AS3933 interface uninitialized.");
	as3933_dev_t *dev = (as3933_dev_t*) handle;
	CHECK((dev->spi_handle != NULL), ESP_ERR_INVALID_STATE, "SPI interface uninitialized.");

	as3933_spi_handle = dev->spi_handle;
	return ESP_OK;
}

esp_err_t as3933_destroy(as3933_handle_t handle, bool del_bus)
{
	esp_err_t error = ESP_OK;
	CHECK((handle != NULL), ESP_ERR_INVALID_STATE, "SPI interface uninitialized.");
	as3933_dev_t *dev = (as3933_dev_t*) handle;
	error = spi_bus_remove_device(dev->spi_handle);
	CHECK((error == ESP_OK), ESP_ERR_INVALID_STATE, "SPI device %d. remove fail", dev->spi_config.spi_host);

	if (del_bus)
	{
		error = spi_bus_free(dev->spi_config.spi_host);
		CHECK((error == ESP_OK), ESP_ERR_INVALID_STATE, "SPI device %d. free fail", dev->spi_config.spi_host);
	}
	free(handle);
	handle = NULL;
	return error;
}

esp_err_t as3933_cmd(uint8_t cmd)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	CHECK((as3933_spi_handle != NULL), ESP_ERR_INVALID_STATE, "SPI interface uninitialized.");

	spi_transaction_t tx_trans =
			{
				.cmd = AS3933_DIRECT_COMMAND,
				.addr = cmd,
				.length = 0,
				.rx_buffer = NULL,
			};

	esp_err_t error = spi_device_polling_transmit(as3933_spi_handle, &tx_trans);
	return error;
}

esp_err_t as3933_read_byte(uint8_t addr, uint8_t *byte)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	CHECK((as3933_spi_handle != NULL), ESP_ERR_INVALID_STATE, "SPI interface uninitialized.");

	spi_transaction_t tx_trans =
			{
				.cmd = AS3933_READ_SINGLE,
				.addr = addr,
				.length = 8,
				.rx_buffer = byte,
			};

	esp_err_t error = spi_device_polling_transmit(as3933_spi_handle, &tx_trans);
	return error;
}

esp_err_t as3933_write_byte(uint8_t addr, const uint8_t byte)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	CHECK((as3933_spi_handle != NULL), ESP_ERR_INVALID_STATE, "SPI interface uninitialized.");

	spi_transaction_t tx_trans =
			{
				.cmd = AS3933_WRITE_SINGLE,
				.addr = addr,
				.length = 8,
				.tx_buffer = &byte,
			};

	esp_err_t error = spi_device_polling_transmit(as3933_spi_handle, &tx_trans);
	return error;
}

esp_err_t as3933_read_bytes(uint16_t addr, uint8_t *buffer, uint8_t length)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	CHECK((as3933_spi_handle != NULL), ESP_ERR_INVALID_STATE, "SPI interface uninitialized.");

	spi_transaction_t tx_trans =
			{
				.cmd = AS3933_READ_SINGLE,
				.addr = addr,
				.length = length * 8,
				.rx_buffer = buffer,
			};

	esp_err_t error = spi_device_polling_transmit(as3933_spi_handle, &tx_trans);
	return error;
}

esp_err_t as3933_write_bytes(uint16_t addr, uint8_t *buffer, uint8_t length)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	CHECK((as3933_spi_handle != NULL), ESP_ERR_INVALID_STATE, "SPI interface uninitialized.");

	spi_transaction_t tx_trans =
			{
				.cmd = AS3933_WRITE_SINGLE,
				.addr = addr,
				.length = length * 8,
				.tx_buffer = buffer,
			};

	esp_err_t error = spi_device_polling_transmit(as3933_spi_handle, &tx_trans);
	return error;
}

/**********************
 *   STATIC FUNCTIONS
 **********************/
static void IRAM_ATTR as3933_w_up_gpio_isr_handler(void *arg)
{
	gpio_num_t w_up_gpio = (gpio_num_t)arg;
	int gpio_lvl = gpio_get_level(w_up_gpio);
	if (w_up_callback != NULL){
		w_up_callback(gpio_lvl);
	}
}

static esp_err_t _gpio_init(gpio_num_t data, gpio_num_t w_up)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	CHECK(GPIO_IS_VALID_GPIO(w_up) || GPIO_IS_VALID_GPIO(data), ESP_ERR_INVALID_ARG, "WRONG GPIO NUM");

	gpio_config_t gpio_in_conf =
		{
			.mode = GPIO_MODE_INPUT,
			.intr_type = GPIO_INTR_DISABLE,
			.pin_bit_mask = (1ULL << w_up) | (1ULL << data),
			.pull_down_en = GPIO_PULLDOWN_ENABLE,
			.pull_up_en = GPIO_PULLUP_DISABLE,
		};

	    esp_err_t ret = gpio_config(&gpio_in_conf);
	    CHECK(ret == ESP_OK, ret, "GPIO %d or GPIO %d CONFIG FAIL", w_up, data);

	    return ESP_OK;
}
