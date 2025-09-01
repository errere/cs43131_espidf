#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <inttypes.h>
#include <driver/i2c_master.h>

#define CS43131_PWR_XSP (1U << 7)
#define CS43131_PWR_ASP (1U << 6)
#define CS43131_PWR_DSDIF (1U << 5)
#define CS43131_PWR_HP (1U << 4)
#define CS43131_PWR_XTAL (1U << 3)
#define CS43131_PWR_PLL (1U << 2)
#define CS43131_PWR_CLKOUT (1U << 1)

void cs43131_gpio_rst();

void cs43131_print_device_id();

void cs43131_set_pwr_on(uint8_t obj);
void cs43131_set_pwr_off(uint8_t obj);

void cs43131_set_vol(uint8_t vol);

void cs43131_set_mute(uint8_t m);

void cs43131_set_sample_length(uint8_t bps);

void cs43131_set_sample_rate(uint32_t sr);

void cs43131_set_filter(uint8_t m);

void cs43131_print_flag();

void cs43131_init(SemaphoreHandle_t *iic_free,i2c_master_dev_handle_t *handle);

// eof