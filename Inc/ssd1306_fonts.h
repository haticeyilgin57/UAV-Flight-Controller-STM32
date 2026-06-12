#ifndef __SSD1306_FONTS_H__
#define __SSD1306_FONTS_H__

#include <stdint.h>

/* Font yapısı: Harflerin boyunu ve verisini tutar */
typedef struct {
    const uint8_t FontWidth;    // Harf genişliği
    const uint8_t FontHeight;   // Harf yüksekliği
    const uint16_t *data;       // Harfin çizim (bit) tablosu
} FontDef;

/* main.c içinde kullandığın Font_7x10 burada dışarıya açılıyor */
extern FontDef Font_7x10;

#endif
