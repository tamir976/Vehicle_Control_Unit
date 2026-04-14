#include "include/oled_display.h"

#include "Lpi2c_Ip_Cfg.h"
#include "Lpi2c_Ip_Sa_PBcfg.h"
#include "Lpi2c_Ip_Irq.h"
#include "Lpi2c_Ip.h"
#include "include/ascii_table.h"

#include <string.h>

static uint8_t g_oledBuf[OLED_BUF_SIZE];
static uint8_t g_oledInitialized;

static int i2c0_write_blocking(const uint8_t *data, uint32_t len)
{
    Lpi2c_Ip_StatusType st;

    st = Lpi2c_Ip_MasterSendDataBlocking(INST_I2C, (uint8 *)data, len, TRUE, 0xFFFFFFFFu);
    return (st == LPI2C_IP_SUCCESS_STATUS) ? 0 : -1;
}

static void ssd1306_write_cmd(uint8_t cmd)
{
    uint8_t pkt[2] = {0x00U, cmd};
    (void)i2c0_write_blocking(pkt, (uint32_t)sizeof(pkt));
}

static void ssd1306_write_data(const uint8_t *data, uint32_t len)
{
    uint8_t pkt[1u + 16u];
    pkt[0] = 0x40U;

    while (len > 0U)
    {
        uint32_t n = (len > 16U) ? 16U : len;

        for (uint32_t i = 0U; i < n; i++)
        {
            pkt[1u + i] = data[i];
        }

        (void)i2c0_write_blocking(pkt, 1U + n);

        data += n;
        len  -= n;
    }
}

static void draw_pixel(uint8_t x, uint8_t y, uint8_t on)
{
    uint16_t idx;
    uint8_t bit;

    if ((x >= OLED_W) || (y >= OLED_H))
    {
        return;
    }

    idx = (uint16_t)x + ((uint16_t)(y / 8U) * OLED_W);
    bit = (uint8_t)(1U << (y & 7U));

    if (on != 0U)
    {
        g_oledBuf[idx] |= bit;
    }
    else
    {
        g_oledBuf[idx] &= (uint8_t)~bit;
    }
}

static const uint8_t *glyph5(char c)
{
    if ((c < 32) || (c > 127))
    {
        c = '?';
    }

    return font5x7_ascii[(uint8_t)c - 32U];
}

static void draw_char5x7(uint8_t x, uint8_t y, char c)
{
    const uint8_t *g = glyph5(c);

    for (uint8_t col = 0U; col < 5U; col++)
    {
        uint8_t bits = g[col];

        for (uint8_t row = 0U; row < 7U; row++)
        {
            uint8_t on = (uint8_t)((bits >> row) & 1U);
            draw_pixel((uint8_t)(x + col), (uint8_t)(y + row), on);
        }
    }
}

static void draw_string5x7(uint8_t x, uint8_t y, const char *s)
{
    while ((s != NULL) && (*s != '\0'))
    {
        draw_char5x7(x, y, *s);
        s++;
        x = (uint8_t)(x + 6U);

        if (x > (OLED_W - 6U))
        {
            break;
        }
    }
}

void ssd1306_init(void)
{
    ssd1306_write_cmd(0xAE);

    ssd1306_write_cmd(0xD5);
    ssd1306_write_cmd(0x80);
    ssd1306_write_cmd(0xA8);
    ssd1306_write_cmd(0x1F);
    ssd1306_write_cmd(0xD3);
    ssd1306_write_cmd(0x00);
    ssd1306_write_cmd(0x40);

    ssd1306_write_cmd(0x8D);
    ssd1306_write_cmd(0x14);

    ssd1306_write_cmd(0x20);
    ssd1306_write_cmd(0x00);

    ssd1306_write_cmd(0xA1);
    ssd1306_write_cmd(0xC8);

    ssd1306_write_cmd(0xDA);
    ssd1306_write_cmd(0x02);

    ssd1306_write_cmd(0x81);
    ssd1306_write_cmd(0x7F);
    ssd1306_write_cmd(0xD9);
    ssd1306_write_cmd(0xF1);
    ssd1306_write_cmd(0xDB);
    ssd1306_write_cmd(0x40);

    ssd1306_write_cmd(0xA4);
    ssd1306_write_cmd(0xA6);

    g_oledInitialized = 1U;
}

void ssd1306_on(void)
{
    ssd1306_write_cmd(0xAF);
}

void ssd1306_update(void)
{
    ssd1306_write_cmd(0x21);
    ssd1306_write_cmd(0x00);
    ssd1306_write_cmd(0x7F);

    ssd1306_write_cmd(0x22);
    ssd1306_write_cmd(0x00);
    ssd1306_write_cmd(0x03);

    ssd1306_write_data(g_oledBuf, (uint32_t)sizeof(g_oledBuf));
}

void ssd1306_clear(void)
{
    (void)memset(g_oledBuf, 0, sizeof(g_oledBuf));
}

void ssd1306_draw_text(uint8_t x, uint8_t y, const char *s)
{
    if (g_oledInitialized == 0U)
    {
        ssd1306_init();
        ssd1306_on();
    }

    ssd1306_clear();
    draw_string5x7(x, y, s);
    ssd1306_update();
}
