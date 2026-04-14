#ifndef OLED_DISPLAY_H_
#define OLED_DISPLAY_H_

#include <stdint.h>

#define OLED_W             (128u)
#define OLED_H             (32u)
#define OLED_BUF_SIZE      (OLED_W * OLED_H / 8u)
#define INST_I2C           (0u)

void ssd1306_init(void);
void ssd1306_on(void);
void ssd1306_update(void);
void ssd1306_clear(void);
void ssd1306_draw_text(uint8_t x, uint8_t y, const char *s);

#endif /* OLED_DISPLAY_H_ */
