#include <Adafruit_NeoPixel.h>

#define PIN 3
#define ROWS 6
#define COLS 16
#define NUM_LEDS (ROWS * COLS)

Adafruit_NeoPixel strip(NUM_LEDS, PIN, NEO_GRB + NEO_KHZ800);

int mapLED(int row, int col) {
  if (row % 2 == 0) {
    return row * COLS + col;
  } else {
    return row * COLS + (COLS - 1 - col);
  }
}

void setup() {
  strip.begin();
  strip.show();
}

void loop() {

  strip.clear();  // ✔ limpiar una sola vez antes de dibujar

  for (int r = 0; r < ROWS; r++) {

    if (r == 0) {
      for (int c = 0; c < COLS; c++) {
        strip.setPixelColor(mapLED(r, c), strip.Color(255, 255,0));
      }
    }

    if (r == 1) {
       strip.setPixelColor(mapLED(r, 1),  strip.Color(255, 255, 0));
       strip.setPixelColor(mapLED(r, 16), strip.Color(255, 255, 0));
    }

    else if (r == 2) {
      strip.setPixelColor(mapLED(r, 2),  strip.Color(255, 255, 0));
      strip.setPixelColor(mapLED(r, 15), strip.Color(255, 255, 0));
    }

    else if (r == 3) {
      strip.setPixelColor(mapLED(r, 3),  strip.Color(100, 255, 0));
      strip.setPixelColor(mapLED(r, 14), strip.Color(100, 255, 0));
    }

    else if (r == 4) {
      strip.setPixelColor(mapLED(r, 5),  strip.Color(100, 255, 0));
      strip.setPixelColor(mapLED(r, 12), strip.Color(100, 255, 0));
    }

    else if (r == 5) {
      for (int c = 7; c < 11; c++) {
        strip.setPixelColor(mapLED(r, c), strip.Color(100, 255, 0));
      }
    }
  }

  strip.show();   // ✔ ahora muestra toda la imagen completa
  delay(400);
}
