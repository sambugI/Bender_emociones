#include <Adafruit_NeoPixel.h>
#include <Servo.h>

// ---------- Config ----------
#define PIN_OJO_IZQ 4
#define PIN_OJO_DER 6
#define NUM_LEDS 16
#define BRILLO_INICIAL 60
// ---------- Configuración de matriz ----------
#define PIN 2
#define ROWS 6
#define COLS 16  
#define NUM_LEDS_B (ROWS * COLS)
    


Servo orejaizq;
Servo orejader;
Servo cejaizq;
Servo cejader;

Adafruit_NeoPixel tiraIzq(NUM_LEDS, PIN_OJO_IZQ, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel tiraDer(NUM_LEDS, PIN_OJO_DER, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel* tiras[2] = { &tiraIzq, &tiraDer };
Adafruit_NeoPixel strip(NUM_LEDS_B, PIN, NEO_GRB + NEO_KHZ800); // BOCA

// Convierte coordenadas (fila, col) a índice físico en tira serpenteada
int mapLED(int row, int col) {
  if (row % 2 == 0) {
    return row * COLS + col;                  // fila par → izq→der
  } else {
    return row * COLS + (COLS - 1 - col);     // fila impar → der→izq
  }
}

// ---------- Estados ----------
enum Mode { MODE_OFF, MODE_FELICIDAD, MODE_TRISTEZA, MODE_ENOJO, MODE_SORPRESA, MODE_TRANQUI, MODE_SI, MODE_NO };
Mode modoActual = MODE_OFF;
Mode lastMode = MODE_OFF;   // para detectar cambio de modo
bool forceRedraw = false;   

// ---------- ENOJO: parpadeo ----------
bool enojo_encendido = false;
unsigned long enojo_t0 = 0;
const unsigned long ENOJO_INTERVALO = 400; // ms

// SORPRESA
unsigned long sorpresa_t0 = 0;
const unsigned long SORPRESA_INTERVALO = 100;
uint8_t sorpresa_frame = 0;

// ---------- FELICIDAD: orejas en antifase ----------
int  FELI_CENTER    = 90;   // centro neutro
int  FELI_AMPL      = 30;   // amplitud
int  feli_offset    = 0;
int  feli_dir       = +1;
int  FELI_STEP      = 2;
unsigned long FELI_INTERVAL = 20;
unsigned long feli_t0 = 0;


// ---------- Brillo ----------
int brillo = BRILLO_INICIAL;

// ---------- Prototipos ----------
void leerComando();
void actualizarAnimacion();
void dibujarModoUnaVez(uint32_t color);
void animEnojoReset();
void animEnojoStep();
void clearAll();
void showAll();
void setOrejas(int angIzq, int angDerRaw);
void animFelicidadReset();
void animFelicidadStep();
void animTRANQUIStep();
void animTRANQUIReset();
void animBreathStep();
void setBrightnessAll(int b);
void smoothMoveOrejas(int targetL, int targetR, int stepDelay);
uint32_t Wheel(byte WheelPos);

// ---------- ENOJO: marcando cejas (leds apagados) en los ojos
void setRedLowerThreeQuartersCustom() {
  uint32_t rojo = tiraIzq.Color(255, 0, 0);

  // Apagar ambos ojos primero
  for (int i = 0; i < NUM_LEDS; i++) {
    tiraIzq.setPixelColor(i, 0);
    tiraDer.setPixelColor(i, 0);
  }

  // Ejemplo: encender bloque central (índices 3 .. NUM_LEDS-5)
  for (int i = 3; i < NUM_LEDS - 5; i++) {
    // izquierda
    tiraDer.setPixelColor(i, rojo);
    int mirror = NUM_LEDS - 1 - i;
    tiraIzq.setPixelColor(mirror, rojo);
  }

  // Actualizar ambas tiras UNA vez
  tiraIzq.show();
  tiraDer.show();
}


// TRANQUI

unsigned long tranqui_t0 = 0;
const unsigned long TRANQUI_INTERVAL = 1000; // refrescar cada 1 segundo


// ======= TRISTEZA NO BLOQUEANTE =======
// Parpadeo suave azul sin usar delay(), permitiendo lectura serial continua.

unsigned long lastBlinkTime = 0;
int blinkStep = 0;
const int blinkInterval = 1000;  // ms entre parpadeos
bool tristezaInitialized = false;

void setBlueLowerSevenTenths_AdjustedLeftGap_NonBlocking() {
  static uint32_t azul = tiraIzq.Color(0, 179, 255);
  static uint32_t negro = 0;

  // Inicialización al entrar al modo tristeza
  if (!tristezaInitialized) {
    smoothMoveOrejas(0, 0, 8);

    // Encender franja azul inicial (mitad inferior)
    for (int i = 5; i < NUM_LEDS - 4; i++) {
      tiraIzq.setPixelColor(i, azul);
      tiraDer.setPixelColor(i, azul);
    }
    tiraIzq.show();
    tiraDer.show();

    lastBlinkTime = millis();
    blinkStep = 0;
    tristezaInitialized = true;
  }

  // Control de parpadeo sin bloquear
  if (millis() - lastBlinkTime < blinkInterval) return;
  lastBlinkTime = millis();

  switch (blinkStep) {
    case 0:
      tiraDer.setPixelColor(6, negro); tiraDer.show();
      break;
    case 1:
      tiraDer.setPixelColor(6, azul); tiraDer.show();
      break;
    case 2:
      tiraIzq.setPixelColor(10, negro); tiraIzq.show();
      break;
    case 3:
      tiraIzq.setPixelColor(10, azul); tiraIzq.show();
      break;
    case 4:
      tiraDer.setPixelColor(7, negro); tiraDer.show();
      break;
    case 5:
      tiraDer.setPixelColor(7, azul); tiraDer.show();
      break;
    case 6:
      tiraIzq.setPixelColor(9, negro); tiraIzq.show();
      break;
    case 7:
      tiraIzq.setPixelColor(9, azul); tiraIzq.show();
      break;
  }

  blinkStep = (blinkStep + 1) % 8;  // repetir ciclo
}




// ---------- Felicidad ----------
void setYellowUpperSevenTenths() {
  // Color amarillo cálido
  uint32_t amarillo = tiraIzq.Color(255, 180, 5);

  // Apagar todos los LEDs primero
  for (int i = 0; i < NUM_LEDS; i++) {
    tiraIzq.setPixelColor(i, 0);
    tiraDer.setPixelColor(i, 0);
  }

  // revisar si los leds que se prenden son físicamente los correctos
  for (int i = 0; i < 6; i++) {
    tiraIzq.setPixelColor(i, amarillo);
    tiraDer.setPixelColor(i, amarillo);
  }

  for (int j = 11; j < 16; j++) {
    tiraIzq.setPixelColor(j, amarillo);
    tiraDer.setPixelColor(j, amarillo);
  }

  // Actualizar ambas tiras solo una vez
  tiraIzq.show();
  tiraDer.show();
}


// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(80);

  orejaizq.attach(9);
  orejader.attach(10);
  cejaizq.attach(11);
  cejader.attach(12);

  tiraIzq.begin();
  tiraDer.begin();
  strip.begin();

  setBrightnessAll(brillo);

  // posición neutra
  setOrejas(90, 90);
  moverceja(90);
  clearAll();
  showAll();

  Serial.println("\nComandos: felicidad | tristeza | enojo | sorpresa | off | tranqui | si | no | brillo <0-255> | masbrillo | menosbrillo");
}



// ------------------- Loop -------------------
void loop() {
  leerComando();
  actualizarAnimacion();
}



// ------------------- Parser -------------------
void leerComando() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toLowerCase();

  if (cmd.length() == 0) return;

  Serial.print("Recibido: '");
  Serial.print(cmd);
  Serial.println("'");

  Mode previo = modoActual;

  if      (cmd == "felicidad")   modoActual = MODE_FELICIDAD;
  else if (cmd == "tristeza")    modoActual = MODE_TRISTEZA;
  else if (cmd == "enojo")       modoActual = MODE_ENOJO;
  else if (cmd == "sorpresa")    modoActual = MODE_SORPRESA;
  else if (cmd == "off" || cmd == "apagar") modoActual = MODE_OFF;
  else if (cmd == "tranqui")     modoActual = MODE_TRANQUI;
  else if (cmd == "si")          modoActual = MODE_SI;
  else if (cmd == "no")          modoActual = MODE_NO;

  else if (cmd.startsWith("brillo ")) {
      brillo = constrain(cmd.substring(7).toInt(), 0, 255);
      setBrightnessAll(brillo);
      Serial.print("Brillo seteado a ");
      Serial.println(brillo);
  }
  else if (cmd == "masbrillo") {
      brillo = constrain(brillo + 20, 0, 255);
      setBrightnessAll(brillo);
      Serial.print("Brillo aumentó a ");
      Serial.println(brillo);
  }
  else if (cmd == "menosbrillo") {
      brillo = constrain(brillo - 20, 0, 255);
      setBrightnessAll(brillo);
      Serial.print("Brillo bajó a ");
      Serial.println(brillo);
  }
  else {
      Serial.println("Comando no reconocido.");
  }

  forceRedraw = (modoActual != previo);
}



// ------------- Motor de animaciones -------------
void actualizarAnimacion() {

  if (modoActual != lastMode || forceRedraw) {

    forceRedraw = false;
    clearAll();
    showAll();

    switch (modoActual) {

      case MODE_FELICIDAD:
        moverceja(90);
        animFelicidadReset();
        setYellowUpperSevenTenths();
        smoothMoveOrejas(0, 0, 8);
        dibujarFelicidad();
        showAll();
        break;

      case MODE_TRISTEZA:
        moverceja(70);
        tristezaInitialized = false;
        dibujarTristeza();
        showAll();
        break;

      case MODE_ENOJO:
        moverceja(120);
        animEnojoReset();
        smoothMoveOrejas(90, 90, 6);
        showAll();
        break;

      case MODE_SORPRESA:
        moverceja(80);
        dibujarModoUnaVez(tiraIzq.Color(196, 0, 255)); // Morado
        dibujarSorpresa();
        smoothMoveOrejas(150, 150, 8);
        showAll();
        break;

      case MODE_OFF:
        moverceja(90);
        apagarOjos();
        smoothMoveOrejas(90, 90, 8);
        showAll();
        break;

      case MODE_TRANQUI:
        moverceja(90);
        smoothMoveOrejas(150, 150, 8);
        animTRANQUIReset();
        showAll();
        break;

      case MODE_SI:
        moverceja(90);
        setYesGreenEyes();
        smoothMoveOrejas(150, 150, 8);
        dibujarSI();
        showAll();
        break;

      case MODE_NO:
        moverceja(90);
        setNoRedEyes();
        smoothMoveOrejas(90, 90, 6);
        dibujarNO();
        showAll();
        break;
    }

    lastMode = modoActual;
  }


  // ------ Animaciones por modo ------
  if (modoActual == MODE_ENOJO)    animEnojoStep();
  if (modoActual == MODE_FELICIDAD)  animFelicidadStep();
  //if (modoActual == MODE_TRANQUI)    animTRANQUIStep();
  if (modoActual == MODE_TRISTEZA)   setBlueLowerSevenTenths_AdjustedLeftGap_NonBlocking();
}



// -------- Helper: apagar ojos ---------
void apagarOjos() {
  for (int i = 0; i < NUM_LEDS; i++) {
    tiraIzq.setPixelColor(i, 0);
    tiraDer.setPixelColor(i, 0);
  }
  showAll();
}


// --------- Ojos estáticos (una vez) ---------
void dibujarModoUnaVez(uint32_t color) {
  // Rellena las tiras completas con un solo comando (más eficiente)
  tiraIzq.fill(color, 0, NUM_LEDS);
  tiraDer.fill(color, 0, NUM_LEDS);

  // Actualiza ambas tiras
  showAll();
}

// --------- ENOJO: parpadeo  ---------
void animEnojoReset() {
  enojo_encendido = false;
  enojo_t0 = millis();
}

void animEnojoStep() {

  uint32_t ahora = millis();
  if (ahora - enojo_t0 < ENOJO_INTERVALO) return;
  enojo_t0 = ahora;

  // Alternar estado de parpadeo
  enojo_encendido = !enojo_encendido;

  // --- OJOS ---
  if (enojo_encendido) {
    setRedLowerThreeQuartersCustom();  
  } else {
    // Apagar solo ojos
    tiraIzq.fill(0, 0, NUM_LEDS);
    tiraDer.fill(0, 0, NUM_LEDS);
    tiraIzq.show();
    tiraDer.show();
  }

  // --- BOCA (siempre encendida) ---
  uint32_t rojo = Adafruit_NeoPixel::Color(255, 0, 0);

  // Fila 6 completa (col 2 a 13)
  for (int c = 2; c < 14; c++)
    strip.setPixelColor(mapLED(5, c), rojo);

  // Fila 5
  strip.setPixelColor(mapLED(4, 2), rojo);
  strip.setPixelColor(mapLED(4, 13), rojo);

  // Fila 4
  strip.setPixelColor(mapLED(3, 3), rojo);
  strip.setPixelColor(mapLED(3, 12), rojo);

  // Fila 3
  strip.setPixelColor(mapLED(2, 4), rojo);
  strip.setPixelColor(mapLED(2, 11), rojo);
  
  // Fila 1 → sonrisa (de 6 a 9)
  for (int c = 5; c <= 10; c++)
    strip.setPixelColor(mapLED(1, c), rojo);

  showAll();  // Actualiza la matriz y las tiras
}



// --------- FELICIDAD ----------
void animFelicidadReset() {
  feli_t0 = millis();
  feli_offset = 0;
  feli_dir = +1;
  setOrejas(FELI_CENTER, FELI_CENTER);
}

void animFelicidadStep() {

  uint32_t ahora = millis();
  if (ahora - feli_t0 < FELI_INTERVAL) return;
  feli_t0 = ahora;

  // Actualizar desplazamiento
  feli_offset += feli_dir * FELI_STEP;

  // Invertir sentido en límites
  if (feli_offset >= FELI_AMPL) {
    feli_offset = FELI_AMPL;
    feli_dir = -1;
  } 
  else if (feli_offset <= -FELI_AMPL) {
    feli_offset = -FELI_AMPL;
    feli_dir = +1;
  }

  // Ángulos finales
  int angIzq = constrain(FELI_CENTER + feli_offset, 0, 180);
  int angDer = constrain(FELI_CENTER - feli_offset, 0, 180);

  setOrejas(angIzq, angDer);
}


// ----------------------TRANQUI--------

void animTRANQUIReset() {

  // Color blanco suave
  uint32_t blanco = Adafruit_NeoPixel::Color(150, 150, 150);

  // --- OJOS: ambas tiras en blanco ---
  for (int i = 0; i < 17; i++) {
    tiraIzq.setPixelColor(i, blanco);
    tiraDer.setPixelColor(i, blanco);
  }
  // Actualiza ojos
  tiraIzq.show();
  tiraDer.show();

  // --- BOCA: usar 'strip' + mapLED(row,col)
  // Encender solo las dos filas centrales: row == 2 y row == 3 (0-based)
  for (int row = 0; row < ROWS; row++) {
    for (int col = 0; col < COLS; col++) {
      int idx = mapLED(row, col);
      if (row == 2 || row == 3) {
        strip.setPixelColor(idx, blanco);
      } else {
        strip.setPixelColor(idx, 0); // apagado
      }
    }
  }
  // Actualiza boca (matriz)
  strip.show();
}

//void animTRANQUIStep() {
//  animTRANQUIReset(); // Solo se ejecuta cada 1 segundo
//}




// ----------Si----------------
// ======= MODO SÍ =======
void setYesGreenEyes() {
  // Color verde brillante
  uint32_t verde = tiraIzq.Color(0, 255, 0);

  // Apagar ojos primero
  for (int i = 0; i < NUM_LEDS; i++) {
    tiraIzq.setPixelColor(i, 0);
    tiraDer.setPixelColor(i, 0);
  }

  // Encender todos los LEDs en verde
  for (int i = 0; i < NUM_LEDS; i++) {
    tiraIzq.setPixelColor(i, verde);
    tiraDer.setPixelColor(i, verde);
  }

  // Expresión física
  smoothMoveOrejas(60, 60, 6);  // orejas medianamente levantadas
  moverceja(45);                // cejas relajadas

  tiraIzq.show();
  tiraDer.show();
}

// ======= MODO NO =======
void setNoRedEyes() {
  // Color rojo intenso
  uint32_t rojo = tiraIzq.Color(255, 0, 0);

  // Apagar ojos primero
  for (int i = 0; i < NUM_LEDS; i++) {
    tiraIzq.setPixelColor(i, 0);
    tiraDer.setPixelColor(i, 0);
  }

  // Encender todos los LEDs en rojo
  for (int i = 0; i < NUM_LEDS; i++) {
    tiraIzq.setPixelColor(i, rojo);
    tiraDer.setPixelColor(i, rojo);
  }

  // Expresión física
  smoothMoveOrejas(120, 120, 6);  // orejas ligeramente hacia atrás
  moverceja(150);                 // cejas fruncidas

  tiraIzq.show();
  tiraDer.show();
}

// BOCA
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
  strip.setPixelColor(mapLED(4, 3), amarillo);
   strip.setPixelColor(mapLED(4, 12), amarillo);
  strip.setPixelColor(mapLED(4, 11), amarillo);

  // Fila 5 → sonrisa (de 6 a 9)
  for (int c = 5; c <= 10; c++)
    strip.setPixelColor(mapLED(5, c), amarillo);
}

// =====================================================
//                 EMOCIÓN: TRISTEZA
// =====================================================
void dibujarTristeza() {
  uint32_t azul = strip.Color(0, 0, 255);

  // Fila 5 → extremos
  //strip.setPixelColor(mapLED(5, 0), azul);
  //strip.setPixelColor(mapLED(5, 15), azul);

  // Fila 4 → 2 puntos
  strip.setPixelColor(mapLED(5, 1), azul);
  strip.setPixelColor(mapLED(5, 14), azul);

  // Fila 3 → 2 puntos
  strip.setPixelColor(mapLED(4, 1), azul);
  strip.setPixelColor(mapLED(4, 14), azul);

  // Fila 2 → 2 puntos
  strip.setPixelColor(mapLED(3, 2), azul);
  strip.setPixelColor(mapLED(3, 13), azul);
  // fila 1 → 2 puntos
  strip.setPixelColor(mapLED(2, 3), azul);
  strip.setPixelColor(mapLED(2, 12), azul);

  // Fila 1 → sonrisa invertida
  for (int c = 3; c <= 12; c++)
    strip.setPixelColor(mapLED(1, c), azul);
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
void dibujarSorpresa() {
  uint32_t morado = strip.Color(30, 0, 250);
  
  // Fila 0 (arriba)
  strip.setPixelColor(mapLED(0, 8), morado);
  strip.setPixelColor(mapLED(0, 9), morado);
  
  // Fila 1
  strip.setPixelColor(mapLED(1, 7), morado);
  strip.setPixelColor(mapLED(1, 10), morado);

  // Fila 2
  strip.setPixelColor(mapLED(2, 6), morado);
  strip.setPixelColor(mapLED(2, 11), morado);

  // Fila 3
  strip.setPixelColor(mapLED(3, 6), morado);
  strip.setPixelColor(mapLED(3, 11), morado);

  // Fila 4
  strip.setPixelColor(mapLED(4, 7), morado);
  strip.setPixelColor(mapLED(4, 10), morado);

  // Fila 5 (abajo)
  strip.setPixelColor(mapLED(5, 8), morado);
  strip.setPixelColor(mapLED(5, 9), morado);
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
  strip.setPixelColor(mapLED(4, 3), verde);
  strip.setPixelColor(mapLED(4, 4), verde);
  strip.setPixelColor(mapLED(4, 11), verde);
  strip.setPixelColor(mapLED(4, 12), verde);
  // Fila 5 → sonrisa (de 6 a 9)
  for (int c = 5; c <= 10; c++)
    strip.setPixelColor(mapLED(5, c), verde);
}
// No
void dibujarNO() {
  uint32_t rojo = strip.Color(255, 0, 0);

  // Fila 0 completa
  //strip.setPixelColor(mapLED(5, 0),rojo);
  //strip.setPixelColor(mapLED(5, 15), rojo);
  // Fila 1 → columnas 2 y 15
  strip.setPixelColor(mapLED(5, 0), rojo);
  strip.setPixelColor(mapLED(5, 15), rojo);

  // Fila 2 → 3,4 y 11,12
  strip.setPixelColor(mapLED(4, 1), rojo);
  strip.setPixelColor(mapLED(4, 14), rojo);

  // Fila 3 → 4 y 11
  strip.setPixelColor(mapLED(3, 2), rojo);
  strip.setPixelColor(mapLED(3, 13), rojo);

  // Fila 4 → 5 y 10
  strip.setPixelColor(mapLED(2, 4), rojo);
  strip.setPixelColor(mapLED(2, 11), rojo);

  // Fila 5 → sonrisa (de 6 a 9)
  for (int c = 6; c <= 9; c++)
    strip.setPixelColor(mapLED(1, c), rojo);
}


// --------- Utilidades ----------
void clearAll() {
  for (int i = 0; i < NUM_LEDS; i++) {
    tiraIzq.setPixelColor(i, 0);
    tiraDer.setPixelColor(i, 0);
  }
  for (int i = 0; i < NUM_LEDS_B; i++)
    strip.setPixelColor(i, 0);
}

void showAll() {
  tiraIzq.show();
  tiraDer.show();
  strip.show();
}

// Mueve cada oreja con ángulos independientes.
// ojo: el derecho se invierte como antes (180 - angRaw)
void setOrejas(int angIzq, int angDerRaw) {
  orejaizq.write(angIzq);
  orejader.write(180 - angDerRaw);
}

void moveroreja (int angulo) {
   setOrejas(angulo, angulo);
}

void moverceja (int angulo) {
   cejaizq.write(angulo);
   cejader.write(180 - angulo);
}

void setBrightnessAll(int b) {
  brillo = constrain(b, 0, 255);
  tiraIzq.setBrightness(brillo);
  tiraDer.setBrightness(brillo);
  strip.setBrightness(brillo);  // ← AGREGAR ESTA LÍNEA
  // showAll(); // normalmente lo llamamos desde el anim step
}

// Mueve orejas suavemente (no bloqueante simple usando delay interno pequeño).
// stepDelay en ms entre pasos.
void smoothMoveOrejas(int targetL, int targetR, int stepDelay) {
  int curL = orejaizq.read();
  int curRraw = 180 - orejader.read(); // recuperar raw
  curRraw = constrain(curRraw, 0, 180);

  int steps = max(abs(targetL - curL), abs(targetR - curRraw));
  if (steps == 0) return;

  int stepL = (targetL > curL) ? 1 : -1;
  int stepR = (targetR > curRraw) ? 1 : -1;

  // hacemos una pequeña interpolación pero con un for (esto bloquea un poco, si quieres no bloquearlo hacemos otra estrategia)
  for (int i = 0; i < steps; i++) {
    if (curL != targetL) curL += stepL;
    if (curRraw != targetR) curRraw += stepR;
    setOrejas(curL, curRraw);
    delay(stepDelay);
  }
  // asegurar posición final
  setOrejas(targetL, targetR);
}

// Helper para colores arcoiris
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return tiraIzq.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return tiraIzq.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return tiraIzq.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
