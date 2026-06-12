#ifndef __SSD1306_H__
#define __SSD1306_H__

#include "i2c.h"            // I2C haberleşmesi için gerekli
#include "ssd1306_fonts.h"  // Yazı tiplerine erişim için

/* OLED Donanım Ayarları */
#define SSD1306_I2C_ADDR        (0x3C << 1) // OLED I2C adresi (7-bit 0x3C kaydırılmış)
#define SSD1306_WIDTH           128         // Ekran genişliği (pixel)
#define SSD1306_HEIGHT          64          // Ekran yüksekliği (pixel)

/* Renk Tanımları: Hata aldığın 'Black' ve 'White' burada tanımlanıyor */
typedef enum {
    Black = 0x00, // Siyah (Pixel kapalı)
    White = 0x01  // Beyaz (Pixel açık)
} SSD1306_COLOR;

/* Fonksiyon Prototipleri: Derleyicinin fonksiyonları tanımasını sağlar */
void ssd1306_Init(void);                                          // Ekranı uyandırır
void ssd1306_Fill(SSD1306_COLOR color);                           // Ekranı tek renk boyar
void ssd1306_UpdateScreen(void);                                  // Hafızayı ekrana yansıtır
void ssd1306_SetCursor(uint8_t x, uint8_t y);                     // Yazı konumunu belirler
void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color); // Tek bir nokta çizer
void ssd1306_WriteString(char* str, FontDef Font, SSD1306_COLOR color); // Metin yazar

#endif
