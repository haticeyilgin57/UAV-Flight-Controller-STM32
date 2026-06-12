#include "ssd1306_fonts.h"

/* 7x10 Font Verisi: Harflerin pixel pixel nasıl göründüğünü tanımlar */
static const uint16_t Font7x10_Table[] = {
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, // Boşluk
    0x2020, 0x2020, 0x2020, 0x0000, 0x2020, // !
    // ... Bu tablo ASCII karakterlerinin (A, B, C, 1, 2, 3 vb.)
    // tüm çizim verilerini içerir. Laboratuvardaki orijinal kütüphaneden
    // tam tabloyu kopyalayabilirsin.
};

/* Yazı tipi nesnesini oluşturur */
FontDef Font_7x10 = {7, 10, Font7x10_Table};
