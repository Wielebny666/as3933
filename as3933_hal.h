#ifndef AS3933_HAL_H
#define AS3933_HAL_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************
 *      INCLUDES
 *********************/
#include <driver/spi_master.h>
#include <driver/gpio.h>

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/
typedef struct
{
    spi_host_device_t spi_host;
    uint32_t spi_clock_speed_hz;
    gpio_num_t mosi;
    gpio_num_t miso;
    gpio_num_t sclk;
    gpio_num_t cs;
    gpio_num_t data;
    gpio_num_t w_up;
} __attribute__((packed)) as3933_cfg_t;

typedef void (*w_up_callback_t)(bool value);

typedef void *as3933_handle_t;

/**********************
 * GLOBAL PROTOTYPES
 **********************/
as3933_handle_t as3933_create(const as3933_cfg_t *spi_cfg);
esp_err_t as3933_switch(as3933_handle_t handle);
esp_err_t as3933_destroy(as3933_handle_t, bool);

esp_err_t as3933_w_up_intr_init(gpio_num_t w_up_gpio, w_up_callback_t cb);

esp_err_t as3933_cmd(uint8_t cmd);
esp_err_t as3933_read_byte(uint8_t addr, uint8_t *byte);
esp_err_t as3933_write_byte(uint8_t addr, const uint8_t byte);
esp_err_t as3933_read_bytes(uint16_t addr, uint8_t *buffer, uint8_t size);
esp_err_t as3933_write_bytes(uint16_t addr, uint8_t *buffer, uint8_t size);

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* AS3933_HAL_H */
