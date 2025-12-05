#include <Adafruit_NeoPixel.h>
#include <Servo.h>

// ---------- Config ----------
#define PIN_OJO_IZQ 4
#define PIN_OJO_DER 6
#define NUM_LEDS 16
#define BRILLO_INICIAL 80

Servo orejaizq;
Servo orejader;
Servo cejaizq;
Servo cejader;

Adafruit_NeoPixel tiraIzq(NUM_LEDS, PIN_OJO_IZQ, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel tiraDer(NUM_LEDS, PIN_OJO_DER, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel* tiras[2] = { &tiraIzq, &tiraDer };

// ---------- Estados ----------
enum Mode { MODE_OFF, MODE_FELICIDAD, MODE_TRISTEZA, MODE_ENOJO, MODE_SORPRESA, MODE_RAINBOW, MODE_SI, MODE_NO };
Mode modoActual = MODE_OFF;
Mode lastMode = MODE_OFF;   // para detectar cambio de modo
bool forceRedraw = false;   

// ---------- ENOJO: parpadeo ----------
bool enojo_encendido = false;
unsigned long enojo_t0 = 0;
const unsigned long ENOJO_INTERVALO = 400; // ms

// ---------- FELICIDAD: orejas en antifase ----------
int  FELI_CENTER    = 90;   // centro neutro
int  FELI_AMPL      = 35;   // amplitud
int  feli_offset    = 0;
int  feli_dir       = +1;
int  FELI_STEP      = 2;
unsigned long FELI_INTERVAL = 15;
unsigned long feli_t0 = 0;

// ---------- RAINBOW ----------
unsigned long rainbow_t0 = 0;
unsigned long RAINBOW_INTERVAL = 200; // ms
int rainbow_pos = 0;

// ---------- Tranqui ----------
unsigned long breath_t0 = 0;
unsigned long BREATH_INTERVAL = 100; // ms
float breath_phase = 0.0;
float BREATH_STEP = 0.1; // velocidad

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
void animRainbowStep();
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
    int mirror = NUM_LEDS - i;
    tiraIzq.setPixelColor(mirror, rojo);
  }

  // Actualizar ambas tiras UNA vez
  tiraIzq.show();
  tiraDer.show();
}





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

// Llama a esta función dentro de actualizarAnimacion():
//   if (modoActual == MODE_TRISTEZA) setBlueLowerSevenTenths_AdjustedLeftGap_NonBlocking();

// Cuando cambies de emoción, reinicia tristezaInitialized = false;







// ---------- Felicidad!! ----------
void setYellowUpperSevenTenths() {
  // Color amarillo cálido
  uint32_t amarillo = tiraIzq.Color(255, 180, 5);

  // Apagar todos los LEDs primero
  for (int i = 0; i < NUM_LEDS; i++) {
    tiraIzq.setPixelColor(i, 0);
    tiraDer.setPixelColor(i, 0);
  }

  // Encender del LED 1 al 7 (índices 0 a 6)
  for (int i = 0; i < 6; i++) {
    tiraIzq.setPixelColor(i, amarillo);
    tiraDer.setPixelColor(i, amarillo);
  }

  // Encender del LED 12 al 16 (índices 11 a 15)
  for (int j = 11; j < 16; j++) {
    tiraIzq.setPixelColor(j, amarillo);
    tiraDer.setPixelColor(j, amarillo);
  }

  // Actualizar ambas tiras solo una vez
  tiraIzq.show();
  tiraDer.show();
}


// ---------- Helper: apagar ojos ----------
void apagarOjos() {
  for (int i = 0; i < NUM_LEDS; i++) {
    tiraIzq.setPixelColor(i, 0);
    tiraDer.setPixelColor(i, 0);
  }
  showAll();
}



// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(80);
  orejaizq.attach(9); // Cambia si necesitas
  orejader.attach(10);
  cejaizq.attach(11);
  cejader.attach(12);
  tiraIzq.begin();
  tiraDer.begin();
  setBrightnessAll(brillo);
  // posición neutra
  setOrejas(90, 90);
  moverceja(90);
  clearAll(); showAll();

  Serial.println("\nComandos: felicidad | tristeza | enojo | sorpresa | off | rainbow | si | no | brillo <0-255> | masbrillo | menosbrillo");
}

// ---------------- Loop ----------------
void loop() {
  leerComando();
  actualizarAnimacion();
}

// ----------- Parser robusto -----------
void leerComando() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    if (cmd.length() == 0) return;
    cmd.trim();
    cmd.toLowerCase();

    Serial.print("Recibido: '"); Serial.print(cmd); Serial.println("'");
    Mode previo = modoActual;

    if (cmd == "felicidad") { modoActual = MODE_FELICIDAD; }
    else if (cmd == "tristeza")  { modoActual = MODE_TRISTEZA;  }
    else if (cmd == "enojo")     { modoActual = MODE_ENOJO;     }
    else if (cmd == "sorpresa")  { modoActual = MODE_SORPRESA;  }
    else if (cmd == "off" || cmd == "apagar") { modoActual = MODE_OFF; }
    else if (cmd == "rainbow") { modoActual = MODE_RAINBOW; }
    else if (cmd == "si") { modoActual = MODE_SI; }
    else if (cmd == "no") { modoActual = MODE_NO; }
    else if (cmd.startsWith("brillo ")) {
      int b = cmd.substring(7).toInt();
      b = constrain(b, 0, 255);
      brillo = b;
      setBrightnessAll(brillo);
      Serial.print("Brillo seteado a "); Serial.println(brillo);
    }
    else if (cmd == "masbrillo") {
      brillo = constrain(brillo + 20, 0, 255);
      setBrightnessAll(brillo);
      Serial.print("Brillo aumentó a "); Serial.println(brillo);
    }
    else if (cmd == "menosbrillo") {
      brillo = constrain(brillo - 20, 0, 255);
      setBrightnessAll(brillo);
      Serial.print("Brillo bajó a "); Serial.println(brillo);
    }
    else {
      Serial.println("Comando no reconocido. Usa:felicidad | tristeza | enojo | sorpresa | off | rainbow | si | no | brillo <0-255> | masbrillo | menosbrillo");
    }
    forceRedraw = (modoActual != previo);
  }
}

// --------- Motor de animaciones ---------
void actualizarAnimacion() {
  if (modoActual != lastMode || forceRedraw) {
    forceRedraw = false; 
    clearAll(); showAll();

    switch (modoActual) {
      case MODE_FELICIDAD:
        // enciende los 7/10 superiores en amarillo y mueve orejas/cejas
        moverceja(90); 
        animFelicidadReset();
        setYellowUpperSevenTenths();
        smoothMoveOrejas(0, 0, 8);   // orejas arriba               // cejas arriba
        showAll();
        break;


      case MODE_TRISTEZA:
        moverceja(70);
        tristezaInitialized = false; 
        showAll();
        break;


      case MODE_ENOJO:
        moverceja(120);
        animEnojoReset();
        smoothMoveOrejas(90, 90, 6); // orejas neutras
        showAll();
        break;

      case MODE_SORPRESA:
        moverceja(80);
        dibujarModoUnaVez(tiraIzq.Color(196, 0, 255)); // Morado
        //setOrejas(1, 1);
        smoothMoveOrejas(150, 150, 8);  // posición neutra
        showAll();
        break;

      case MODE_OFF:
        // apaga completamente ojos y deja orejas/cejas quietas
        moverceja(90);  
        apagarOjos();
        smoothMoveOrejas(90, 90, 8);  // posición neutra 
        showAll();            
        break;


      case MODE_RAINBOW:
        // arranca el rainbow
        moverceja(90);
        smoothMoveOrejas(150, 150, 8);  // posición neutra
        rainbow_pos = 0;
        rainbow_t0 = millis();
        animRainbowStep(); 
        showAll();            

        
        break;
      case MODE_SI:
        moverceja(90);
        setYesGreenEyes();
        smoothMoveOrejas(150, 150, 8);  // posición neutra
        showAll();
        break;
    
      case MODE_NO:
        moverceja(90);
        setNoRedEyes();
        smoothMoveOrejas(90, 90, 6);  // posición neutra
        showAll();
        break;


    }
    lastMode = modoActual;
  }

  // steps por modo
  if (modoActual == MODE_ENOJO) animEnojoStep();
  if (modoActual == MODE_FELICIDAD) animFelicidadStep();
  if (modoActual == MODE_RAINBOW) animRainbowStep();
  if (modoActual == MODE_TRISTEZA) setBlueLowerSevenTenths_AdjustedLeftGap_NonBlocking();
}

// --------- Emociones estáticas (una vez) ---------
void dibujarModoUnaVez(uint32_t color) {
  for (int i = 0; i < NUM_LEDS; i++) {
    tiraIzq.setPixelColor(i, color);
    tiraDer.setPixelColor(i, color);
  }
  showAll();
}

// --------- ENOJO: parpadeo  ---------
void animEnojoReset() {
  enojo_encendido = false;
  enojo_t0 = millis();
}

void animEnojoStep() {
  if (millis() - enojo_t0 < ENOJO_INTERVALO) return;
  enojo_t0 = millis();
  enojo_encendido = !enojo_encendido;

  if (enojo_encendido) {
    setRedLowerThreeQuartersCustom();  
  } else {
    clearAll();
  }
  showAll();
}

// --------- FELICIDAD ----------
void animFelicidadReset() {
  feli_t0 = millis();
  feli_offset = 0;
  feli_dir    = +1;
  setOrejas(FELI_CENTER, FELI_CENTER);
}

void animFelicidadStep() {
  if (millis() - feli_t0 < FELI_INTERVAL) return;
  feli_t0 = millis();

  feli_offset += feli_dir * FELI_STEP;

  if (feli_offset >=  FELI_AMPL) { feli_offset =  FELI_AMPL; feli_dir = -1; }
  if (feli_offset <= -FELI_AMPL) { feli_offset = -FELI_AMPL; feli_dir = +1; }

  int angIzq = FELI_CENTER + feli_offset;
  int angDer = FELI_CENTER - feli_offset;

  angIzq = constrain(angIzq, 0, 180);
  angDer = constrain(angDer, 0, 180);

  setOrejas(angIzq, angDer);
}

// --------- RAINBOW (no bloqueante) ----------
void animRainbowStep() {
  if (millis() - rainbow_t0 < RAINBOW_INTERVAL) return;
  rainbow_t0 = millis();
  for (int i = 0; i < NUM_LEDS; i++) {
    int idx = (i * 256 / NUM_LEDS + rainbow_pos) & 0xFF;
    uint32_t c = Wheel(idx);
    tiraIzq.setPixelColor(i, c);
    tiraDer.setPixelColor(i, c);
  }
  rainbow_pos = (rainbow_pos + 1) & 0xFF;
  showAll();
}

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




// --------- Utilidades ----------
void clearAll() {
  for (int i = 0; i < NUM_LEDS; i++) {
    tiraIzq.setPixelColor(i, 0);
    tiraDer.setPixelColor(i, 0);
  }
  showAll();
}

void showAll() {
  tiraIzq.show();
  tiraDer.show();
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
