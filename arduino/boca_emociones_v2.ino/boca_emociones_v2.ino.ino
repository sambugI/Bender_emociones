#include <Adafruit_NeoPixel.h>

// ---------- Configuración de matriz ----------
#define PIN 2
#define ROWS 6
#define COLS 16
#define NUM_LEDS (ROWS * COLS)
#define BRILLO_INICIAL 60

Adafruit_NeoPixel strip(NUM_LEDS, PIN, NEO_GRB + NEO_KHZ800);

// Convierte coordenadas (fila, col) a índice físico en tira serpenteada
int mapLED(int row, int col) {
  if (row % 2 == 0) {
    return row * COLS + col;                  // fila par → izq→der
  } else {
    return row * COLS + (COLS - 1 - col);     // fila impar → der→izq
  }
}

// ============= ESTADOS Y ANIMACIONES =============
enum Mode { 
  MODE_OFF, 
  MODE_FELICIDAD, 
  MODE_TRISTEZA, 
  MODE_ENOJO, 
  MODE_SORPRESA,
  MODE_SI,
  MODE_NO
};

Mode modoActual = MODE_OFF;
Mode lastMode   = MODE_OFF;

// ENOJO
bool enojo_encendido = false;
unsigned long enojo_t0 = 0;
const unsigned long ENOJO_INTERVALO = 400;

// SORPRESA
unsigned long sorpresa_t0 = 0;
const unsigned long SORPRESA_INTERVALO = 100;
uint8_t sorpresa_frame = 0;

// ============= PROTOTIPOS =============
void leerComando();
void actualizarAnimacion();
void dibujarFelicidad();
void dibujarTristeza();
void animEnojoReset();
void animEnojoStep();
void animSorpresaReset();
void animSorpresaStep();
void clearAll();
void showAll();

// ============= SETUP =============
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(80);

  strip.begin();
  strip.setBrightness(BRILLO_INICIAL);
  clearAll(); showAll();

  Serial.println("Comandos: felicidad | tristeza | enojo | sorpresa | si | no | off");
}

// ============= LOOP =============
void loop() {
  leerComando();
  actualizarAnimacion();
}

// ============= Parser comandos =============
void leerComando() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim(); cmd.toLowerCase();

  if      (cmd == "felicidad") modoActual = MODE_FELICIDAD;
  else if (cmd == "tristeza")  modoActual = MODE_TRISTEZA;
  else if (cmd == "enojo")     modoActual = MODE_ENOJO;
  else if (cmd == "sorpresa")  modoActual = MODE_SORPRESA;
  else if (cmd == "si")       modoActual = MODE_SI;
  else if (cmd == "no")       modoActual = MODE_NO;
  else if (cmd == "off")       modoActual = MODE_OFF;
}

// ============= Motor animaciones =============
void actualizarAnimacion() {

  // Si el modo cambió
  if (modoActual != lastMode) {
    clearAll();
    showAll();

    switch (modoActual) {
      case MODE_FELICIDAD:
        dibujarFelicidad();
        showAll();
        break;

      case MODE_TRISTEZA:
        dibujarTristeza();
        showAll();
        break;

      case MODE_ENOJO:
        animEnojoReset();
        break;

      case MODE_SORPRESA:
        animSorpresaReset();
        break;
        
      case MODE_SI:
        dibujarSI();
        showAll();
        break;
      
      case MODE_NO:
        dibujarNO();
        showAll();
        break;
      
      case MODE_OFF:
        break;
    }

    lastMode = modoActual;
  }

  // Animaciones
  if (modoActual == MODE_ENOJO)    animEnojoStep();
  if (modoActual == MODE_SORPRESA) animSorpresaStep();
}

// =====================================================
//                 EMOCIÓN: FELICIDAD
// =====================================================
void dibujarFelicidad() {
  uint32_t amarillo = strip.Color(255, 200, 0);

  // Fila 0 completa
  for (int c = 0; c < COLS; c++)
    strip.setPixelColor(mapLED(0, c), amarillo);

  // Fila 1 → columnas 2 y 15
  strip.setPixelColor(mapLED(1, 0), amarillo);
  strip.setPixelColor(mapLED(1, 15), amarillo);

  // Fila 2 → 3,4 y 11,12
  strip.setPixelColor(mapLED(2, 1), amarillo);
  strip.setPixelColor(mapLED(2, 14), amarillo);

  // Fila 3 → 4 y 11
  strip.setPixelColor(mapLED(3, 2), amarillo);
  strip.setPixelColor(mapLED(3, 13), amarillo);

  // Fila 4 → 5 y 10
  strip.setPixelColor(mapLED(4, 4), amarillo);
  strip.setPixelColor(mapLED(4, 11), amarillo);

  // Fila 5 → sonrisa (de 6 a 9)
  for (int c = 6; c <= 9; c++)
    strip.setPixelColor(mapLED(5, c), amarillo);
}

// =====================================================
//                 EMOCIÓN: TRISTEZA
// =====================================================
void dibujarTristeza() {
  uint32_t azul = strip.Color(0, 0, 255);

  // Fila 5 → extremos
  strip.setPixelColor(mapLED(5, 0), azul);
  strip.setPixelColor(mapLED(5, 15), azul);

  // Fila 4 → 2 puntos
  strip.setPixelColor(mapLED(4, 0), azul);
  strip.setPixelColor(mapLED(4, 15), azul);

  // Fila 3 → 2 puntos
  strip.setPixelColor(mapLED(3, 0), azul);
  strip.setPixelColor(mapLED(3, 15), azul);

  // Fila 2 → 2 puntos
  strip.setPixelColor(mapLED(2, 1), azul);
  strip.setPixelColor(mapLED(2, 14), azul);
  // fila 1 → 2 puntos
  strip.setPixelColor(mapLED(1, 2), azul);
  strip.setPixelColor(mapLED(1, 13), azul);

  // Fila 1 → sonrisa invertida
  for (int c = 3; c <= 12; c++)
    strip.setPixelColor(mapLED(0, c), azul);
}

// =====================================================
//                  ENOJO (parpadeo)
// =====================================================
void animEnojoReset() {
  enojo_encendido = false;
  enojo_t0 = millis();
}

void animEnojoStep() {
  if (millis() - enojo_t0 < ENOJO_INTERVALO) return;
  enojo_t0 = millis();
  enojo_encendido = !enojo_encendido;

  if (!enojo_encendido) {
    clearAll(); 
    showAll();
    return;
  }

  uint32_t rojo = strip.Color(255, 0, 0);
  // Fila 6 completa
  for (int c = 2; c < 14; c++)
    strip.setPixelColor(mapLED(5, c), rojo);
  
  strip.setPixelColor(mapLED(4, 2), rojo);
  strip.setPixelColor(mapLED(4, 13), rojo);

  // Fila 4 → columnas 2 y 15
  strip.setPixelColor(mapLED(3, 3), rojo);
  strip.setPixelColor(mapLED(3, 12), rojo);

  // Fila 3 → 3,4 y 11,12
  strip.setPixelColor(mapLED(2, 4), rojo);
  strip.setPixelColor(mapLED(2, 11), rojo);
  
  // Fila 0 → sonrisa (de 6 a 9)
  for (int c = 6; c <= 9; c++)
    strip.setPixelColor(mapLED(1, c), rojo);
    
  showAll();
}

// =====================================================
//            SORPRESA (expansión radial)
// =====================================================
void animSorpresaReset() {
  sorpresa_frame = 0;
  sorpresa_t0 = millis();
  clearAll(); 
  showAll();
}

void animSorpresaStep() {
  if (millis() - sorpresa_t0 < SORPRESA_INTERVALO) return;
  sorpresa_t0 = millis();

  clearAll();
  uint32_t morado = strip.Color(30, 0, 250);

  switch(sorpresa_frame) {
    case 0:
      strip.setPixelColor(mapLED(2, 7), morado);
      strip.setPixelColor(mapLED(3, 7), morado);
      break;
    case 1:
      strip.setPixelColor(mapLED(2, 6), morado);
      strip.setPixelColor(mapLED(2, 8), morado);
      strip.setPixelColor(mapLED(3, 6), morado);
      strip.setPixelColor(mapLED(3, 8), morado);
      break;
    case 2:
      for (int r = 2; r <= 3; r++)
        for (int c = 6; c <= 8; c++)
          strip.setPixelColor(mapLED(r, c), morado);
      break;
    case 3:
      for (int r = 1; r <= 4; r++)
        strip.setPixelColor(mapLED(r, 5), morado),
        strip.setPixelColor(mapLED(r, 9), morado);
      break;
    case 4:
      for (int r = 1; r <= 4; r++)
        strip.setPixelColor(mapLED(r, 4), morado),
        strip.setPixelColor(mapLED(r, 10), morado);
      break;
    case 5:
      for (int r = 0; r <= 5; r++)
        strip.setPixelColor(mapLED(r, 3), morado),
        strip.setPixelColor(mapLED(r, 11), morado);
      break;
    case 6:
      for (int r = 0; r <= 5; r++)
        strip.setPixelColor(mapLED(r, 2), morado),
        strip.setPixelColor(mapLED(r, 12), morado);
      break;
  }

  showAll();
  sorpresa_frame = (sorpresa_frame + 1) % 7;
}

// Si
void dibujarSI() {
  uint32_t verde = strip.Color(0, 255, 0);

  // Fila 0 completa
  strip.setPixelColor(mapLED(0, 0), verde);
  strip.setPixelColor(mapLED(0, 15), verde);
  // Fila 1 → columnas 2 y 15
  strip.setPixelColor(mapLED(1, 0), verde);
  strip.setPixelColor(mapLED(1, 15), verde);

  // Fila 2 → 3,4 y 11,12
  strip.setPixelColor(mapLED(2, 1), verde);
  strip.setPixelColor(mapLED(2, 14), verde);

  // Fila 3 → 4 y 11
  strip.setPixelColor(mapLED(3, 2), verde);
  strip.setPixelColor(mapLED(3, 13), verde);

  // Fila 4 → 5 y 10
  strip.setPixelColor(mapLED(4, 4), verde);
  strip.setPixelColor(mapLED(4, 11), verde);

  // Fila 5 → sonrisa (de 6 a 9)
  for (int c = 6; c <= 9; c++)
    strip.setPixelColor(mapLED(5, c), verde);
}
// No
void dibujarNO() {
  uint32_t rojo = strip.Color(255, 0, 0);

  // Fila 0 completa
  strip.setPixelColor(mapLED(5, 0),rojo);
  strip.setPixelColor(mapLED(5, 15), rojo);
  // Fila 1 → columnas 2 y 15
  strip.setPixelColor(mapLED(4, 0), rojo);
  strip.setPixelColor(mapLED(4, 15), rojo);

  // Fila 2 → 3,4 y 11,12
  strip.setPixelColor(mapLED(3, 1), rojo);
  strip.setPixelColor(mapLED(3, 14), rojo);

  // Fila 3 → 4 y 11
  strip.setPixelColor(mapLED(2, 2), rojo);
  strip.setPixelColor(mapLED(2, 13), rojo);

  // Fila 4 → 5 y 10
  strip.setPixelColor(mapLED(1, 4), rojo);
  strip.setPixelColor(mapLED(1, 11), rojo);

  // Fila 5 → sonrisa (de 6 a 9)
  for (int c = 6; c <= 9; c++)
    strip.setPixelColor(mapLED(0, c), rojo);
}


// =====================================================
//          Utilidades
// =====================================================
void clearAll() {
  for (int i = 0; i < NUM_LEDS; i++)
    strip.setPixelColor(i, 0);
}

void showAll() {
  strip.show();
}
