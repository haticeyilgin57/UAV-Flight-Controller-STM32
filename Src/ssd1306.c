#include "ssd1306.h"
#include <string.h> // memset fonksiyonu için

/* Ekran Belleği (Buffer): Görüntü önce burada hazırlanır */
static uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];
static uint8_t CurrentX = 0, CurrentY = 0; // Güncel yazı konumu

/* OLED'e komut gönderme fonksiyonu */
void ssd1306_WriteCommand(uint8_t cmd) {
    HAL_I2C_Master_Transmit(&hi2c1, SSD1306_I2C_ADDR, &cmd, 1, 10);
}

/* Ekranı başlatma: SSD1306 çipine başlangıç ayarlarını gönderir */
void ssd1306_Init(void) {
    HAL_Delay(100);             // Ekranın elektriksel olarak uyanması için bekle
    ssd1306_WriteCommand(0xAE); // Ekranı kapat
    ssd1306_WriteCommand(0x20); // Bellek adresleme modu
    ssd1306_WriteCommand(0x10); // Yatay adresleme
    ssd1306_WriteCommand(0xAF); // Ekranı aç
    ssd1306_Fill(Black);        // Ekranı temizle
    ssd1306_UpdateScreen();     // Boş ekranı göster
}

/* Ekranı tamamen boyar (Black=Temizle, White=Doldur) */
void ssd1306_Fill(SSD1306_COLOR color) {
    memset(SSD1306_Buffer, (color == Black) ? 0x00 : 0xFF, sizeof(SSD1306_Buffer));
}

/* Buffer'daki görüntüyü I2C üzerinden gerçek ekrana gönderir */
void ssd1306_UpdateScreen(void) {
    for(uint8_t i = 0; i < 8; i++) {
        ssd1306_WriteCommand(0xB0 + i); // Sayfa seç
        ssd1306_WriteCommand(0x00);      // Sütun düşük bit
        ssd1306_WriteCommand(0x10);      // Sütun yüksek bit
        HAL_I2C_Master_Transmit(&hi2c1, SSD1306_I2C_ADDR, &SSD1306_Buffer[SSD1306_WIDTH * i], SSD1306_WIDTH, 100);
    }
}

/* Belirlenen noktaya pixel çizer */
void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color) {
    if(x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) return;
    if(color == White) SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |= (1 << (y % 8));
    else               SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
}

/* Yazı yazılacak koordinatı günceller */
void ssd1306_SetCursor(uint8_t x, uint8_t y) {
    CurrentX = x;
    CurrentY = y;
}

/* Ekrana metin yazar: Tek tek karakterleri DrawPixel ile çizer */
void ssd1306_WriteString(char* str, FontDef Font, SSD1306_COLOR color) {
    while(*str) {
        for(uint8_t i = 0; i < Font.FontHeight; i++) {
            uint16_t b = Font.data[(*str - 32) * Font.FontHeight + i];
            for(uint8_t j = 0; j < Font.FontWidth; j++) {
                if((b << j) & 0x8000) ssd1306_DrawPixel(CurrentX + j, CurrentY + i, color);
                else                  ssd1306_DrawPixel(CurrentX + j, CurrentY + i, (SSD1306_COLOR)!color);
            }
        }
        CurrentX += Font.FontWidth;
        str++;
    }
}
