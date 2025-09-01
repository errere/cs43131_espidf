#include <driver/gpio.h>
#include <esp_log.h>

#include "cs43131.h"

#define CS43131_RST_PIN 19

static const char *TAG = "cs43131";

static SemaphoreHandle_t *pSemaphoreIICFree;
static i2c_master_dev_handle_t *i2c_handle;

void cs43131_gpio_rst()
{
    gpio_reset_pin(CS43131_RST_PIN);

    gpio_set_direction(CS43131_RST_PIN, GPIO_MODE_OUTPUT);

    gpio_set_level(CS43131_RST_PIN, 0);
    vTaskDelay(1);
    gpio_set_level(CS43131_RST_PIN, 1);
    vTaskDelay(10);
}

static esp_err_t cs43131_write_reg(uint32_t addr, uint8_t dat)
{

    if (*pSemaphoreIICFree != 0)
    {
        xSemaphoreTake(*pSemaphoreIICFree, portMAX_DELAY);
    }

    uint8_t addr_bit[5] = {0, 0, 0, 0x01,0x0};
    addr_bit[0] = (addr >> 16) & 0xff;
    addr_bit[1] = (addr >> 8) & 0xff;
    addr_bit[2] = (addr) & 0xff;
    addr_bit[4] = dat;

    esp_err_t ret = i2c_master_transmit(*i2c_handle, addr_bit, 5, -1);
    // i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // i2c_master_start(cmd);
    // i2c_master_write_byte(cmd, (CS43131_IIC_ADDRESS << 1) | I2C_MASTER_WRITE, 0x1);
    // i2c_master_write(cmd, addr_bit, 4, 0x1);
    // i2c_master_write_byte(cmd, dat, 0x1);
    // i2c_master_stop(cmd);
    // esp_err_t ret = i2c_master_cmd_begin(CS43131_IIC_DEVICE, cmd, portMAX_DELAY);
    // i2c_cmd_link_delete(cmd);

    if (*pSemaphoreIICFree != 0)
    {
        xSemaphoreGive(*pSemaphoreIICFree);
    }

    return ret;
}

static esp_err_t cs43131_read_reg(uint32_t addr, uint8_t *dst)
{

    if (*pSemaphoreIICFree != 0)
    {
        xSemaphoreTake(*pSemaphoreIICFree, portMAX_DELAY);
    }

    uint8_t addr_bit[4] = {0, 0, 0, 0x01};
    addr_bit[0] = (addr >> 16) & 0xff;
    addr_bit[1] = (addr >> 8) & 0xff;
    addr_bit[2] = (addr) & 0xff;

    esp_err_t ret = i2c_master_transmit_receive(*i2c_handle, addr_bit, 4, dst, 1, -1);

    // i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // i2c_master_start(cmd);
    // i2c_master_write_byte(cmd, (CS43131_IIC_ADDRESS << 1) | I2C_MASTER_WRITE, 0x1);
    // i2c_master_write(cmd, addr_bit, 4, 0x1);
    // // i2c_master_stop(cmd);
    // i2c_master_start(cmd);
    // i2c_master_write_byte(cmd, (CS43131_IIC_ADDRESS << 1) | I2C_MASTER_READ, 0x1);
    // i2c_master_read_byte(cmd, dst, I2C_MASTER_NACK);
    // i2c_master_stop(cmd);

    // esp_err_t ret = i2c_master_cmd_begin(CS43131_IIC_DEVICE, cmd, portMAX_DELAY);
    // i2c_cmd_link_delete(cmd);

    if (*pSemaphoreIICFree != 0)
    {
        xSemaphoreGive(*pSemaphoreIICFree);
    }

    return ret;
}

void cs43131_set_pwr_on(uint8_t obj)
{
    uint8_t tmp = 0xff;
    cs43131_read_reg(0x020000, &tmp); // get prev status
    tmp = tmp & ~(obj);
    cs43131_write_reg(0x020000, tmp); // set pwr up
    //ESP_LOGI(TAG,"por write %02x",tmp);
}

void cs43131_set_pwr_off(uint8_t obj)
{
    uint8_t tmp = 0xff;
    cs43131_read_reg(0x020000, &tmp); // get prev status
    tmp = tmp | (obj);
    tmp = tmp & 0b11111110;
    cs43131_write_reg(0x020000, tmp); // set pwr down
}

void cs43131_print_device_id()
{
    uint8_t tmp[5];
    cs43131_read_reg(0x010000, &tmp[0]);
    cs43131_read_reg(0x010001, &tmp[1]);
    cs43131_read_reg(0x010002, &tmp[2]);
    cs43131_read_reg(0x010004, &tmp[3]);
    cs43131_read_reg(0x010005, &tmp[4]);

    ESP_LOGI(TAG, "CS43131 id = %02x%02x%02x%02x%02x", tmp[0], tmp[1], tmp[2], tmp[3], tmp[4]);
}

/**
 * 
 * @param vol volume 0x0 = 0db , 0xff = mute
*/
void cs43131_set_vol(uint8_t vol)
{
    cs43131_write_reg(0x090001, vol); // PCM Volume
    cs43131_write_reg(0x090002, vol);
}

void cs43131_set_mute(uint8_t m)
{
    uint8_t tmp = 0;
    cs43131_read_reg(0x090003, &tmp);
    if (m)
    {
        tmp = tmp | 0b00000011; // set bit[1:0] to 0b11
    }
    else
    {
        tmp = tmp & ~(0b00000011); // set bit[1:0] to 0b00
    }
    cs43131_write_reg(0x090003, tmp);
}

void cs43131_set_sample_length(uint8_t bps)
{
    uint8_t reg = 0x00;
    switch (bps)
    {
    case 32:
        reg = 0;
        break;
    case 24:
        reg = 1;
        break;
    case 16:
        reg = 2;
        break;
    case 8:
        reg = 3;
        break;

    default:
        ESP_LOGE(TAG, "sample length %d error", bps);
        break;
    }
    cs43131_write_reg(0x01000c, reg);
}

void cs43131_set_sample_rate(uint32_t sr)
{
    uint8_t reg = 0x00;
    switch (sr)
    {
    case 32000:
        reg = 0;
        break;
    case 44100:
        reg = 1;
        break;
    case 48000:
        reg = 2;
        break;
    case 88200:
        reg = 3;
        break;
    case 96000:
        reg = 4;
        break;
    case 176400:
        reg = 5;
        break;
    case 192000:
        reg = 6;
        break;
    case 352800:
        reg = 7;
        break;
    case 384000:
        reg = 8;
        break;

    default:
        ESP_LOGE(TAG, "sample rate %ld error", sr);
        break;
    }
    cs43131_write_reg(0x01000b, reg);
}

void cs43131_set_filter(uint8_t m)
{
    cs43131_write_reg(0x090000, m);
}

void cs43131_print_flag()
{
    uint8_t x[5] = {0x12,0x34,0x56,0x78,0x9a};
    for (size_t i = 0; i < 5; i++)
    {
        cs43131_read_reg(0x0f0000 + i, x + i);
    }
    ESP_LOGI(TAG, "int flag = %02x %02x %02x %02x %02x", x[0], x[1], x[2], x[3], x[4]);
}

void cs43131_init(SemaphoreHandle_t *iic_free, i2c_master_dev_handle_t *handle)
{
    pSemaphoreIICFree = iic_free;
    i2c_handle = handle;
    cs43131_gpio_rst();
    vTaskDelay(1); // 10ms@100hz,need 1.5ms

    // Configure XTAL driver
    cs43131_write_reg(0x020052, 0x02); // Crystal Setting to 12.5ua
    cs43131_write_reg(0x010006, 0x06); // Internal MCLK is expected to be 22.5792 MHz | RCO Mode
    // cs43131_read_reg(xxx,&xxx);//clear any pending interrupts
    // cs43131_write_reg(xxx,xxx);Enable XTAL interrupts
    //cs43131_write_reg(0x020000, 0xf6); // Power up XTAL
    cs43131_set_pwr_on(CS43131_PWR_XTAL);

    /*
    To power up the PLL, use the following default sequence:
    1. Enable the PLL by clearing PDN_PLL.
    2. Configure PLL_REF_PREDIV.
    3. Configure PLL_OUT_DIV.
    4. Configure the three fractional factor registers, PLL_DIV_FRAC.
    5. Set the integer factor, PLL_DIV_INT, to the desired value.
    6. Configure PLL_MODE and PLL_CAL_RATIO.
    7. After properly unmasked (clearing PLL_READY_INT_MASK and PLL_ERROR_INT_MASK), PLL_READY_INT, and PLL_ERROR_INT are used to monitor if PLL has been successfully started.
    8. Turn on the PLL by setting PLL_START.
    */

    // Configure PLL
    //cs43131_write_reg(0x020000, 0xf2); // Power up XTAL | PLL
    cs43131_set_pwr_on(CS43131_PWR_PLL);
    cs43131_write_reg(0x040002, 0x03); // PLL_REF_PREDIV = 8
    cs43131_write_reg(0x030008, 0x0a); // PLL_OUT_DIV = 0x0a
    cs43131_write_reg(0x030002, 0x80); // PLL_DIV_FRAC = 0x797680
    cs43131_write_reg(0x030003, 0x76);
    cs43131_write_reg(0x030004, 0x79);
    cs43131_write_reg(0x030005, 0x45); // PLL_DIV_INT = 0x45
    cs43131_write_reg(0x03001b, 0x01); // PLL_MODE = 1
    cs43131_write_reg(0x03000a, 0x6f); // PLL_CAL_RATIO = 111
    // cs43131_write_reg(xxx,xxx);//enable int

    cs43131_write_reg(0x030001, 0x01); // PLL_START

    // clk out(debug use)
    //cs43131_write_reg(0x040004, 0x01); // source = pll | divide 2
    //cs43131_write_reg(0x020000, 0xf0); // Power up XTAL | PLL | CLKOUT

    // Configure mclk to use pll
    cs43131_write_reg(0x010006, 0x05); // Internal MCLK is expected to be 22.5792 MHz | PLL Mode

    // Configure ASP interface.
    cs43131_set_sample_rate(48000); // Serial Port Sample Rate = 48k
    cs43131_set_sample_length(16);  // Serial Port Sample Bit Size = 32bit

    cs43131_write_reg(0x01000d, 0x00); // When in Master Mode, serial port clocks are active.

    cs43131_write_reg(0x040018, 0x0c); // ASP Master Mode | ASP SCLK Inverted
    cs43131_write_reg(0x040019, 0x0a); // ASP LRCK fixed 50/50 duty cycle | ASP frame start delay = 1

    cs43131_write_reg(0x050000, 0x00); // ASP Rx channel n location. Start on SCLK 0
    cs43131_write_reg(0x050001, 0x00);

    cs43131_write_reg(0x05000A, 0x07); // 32 bits | Input channel data is propagated to the internal data path | L/R
    cs43131_write_reg(0x05000B, 0x0f);

    // Configure DSD

    // Configure PCM
    cs43131_set_filter(0x00);          // PCM Filter
    cs43131_set_vol(0x20);             // PCM Volume
    cs43131_write_reg(0x090003, 0x0f); // This mute and unmute is controlled by PCM_SZC |  Volume setting of both channels are controlled by PCM_VOLUME_A. PCM_VOLUME_B is ignored | PCM_SZC = Soft ramp
    cs43131_write_reg(0x090004, 0x00); // PCM Path Signal Control 2 invert polarity & copy data(all disable)

    //cs43131_write_reg(0x020000, 0xb0); // Power up ASP | XTAL | PLL | CLKOUT
    cs43131_set_pwr_on(CS43131_PWR_ASP);

    // Configure HP
    cs43131_write_reg(0x080000, 0x10); // HP Output Control 1  Output full scale voltage is at 1 V
    cs43131_write_reg(0x0b0000, 0x00); // High voltage mode enable | Fixed, Mode 0 (±VP_LDO)

    cs43131_write_reg(0x010010, 0x99); // pop free setting
    cs43131_write_reg(0x080032, 0x20);

    //cs43131_write_reg(0x020000, 0xA0); // Power up HP | ASP | XTAL | PLL | CLKOUT
    cs43131_set_pwr_on(CS43131_PWR_HP);

    vTaskDelay(2); // 2ms

    cs43131_write_reg(0x010010, 0x00); // pop free setting Restore
    cs43131_write_reg(0x080032, 0x00);

    // cs43131_write_reg(0x080000, 0x10); // HP Output Control 1
    // cs43131_write_reg(0x090000, 0x02); // PCM Filter
    cs43131_set_mute(0); // disable mute
}

// eof