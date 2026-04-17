/*
 * starcrwaler_mkr_traccion.ino
 * =====================
 * Firmware de prueba para el Arduino MKR WiFi 1010 - Modo 1 (Tracción diferencial).
 * Compatible con el script horu_control.py (sustituye a Simulink).
 *
 * Fixes respecto a control_motores_v2.ino:
 *   - Delay de 250 µs DESPUÉS de RR en cada ciclo (fix del fallo FL/0x141).
 *   - Mismo delay en pararTodo() para no saturar el bus CAN.
 *   - Reconexión WiFi no bloqueante.
 *   - Watchdog de seguridad: libera motores si no llegan paquetes en 500 ms.
 *   - Rate limiter software para aceleración/deceleración suave.
 *
 * Datagrama UDP esperado (18 bytes, 9 x int16 little-endian):
 *   [0] id               -> debe ser 1 (origen PC/script)
 *   [1] action_left_train
 *   [2] action_right_train
 *   [3..6] o0..o3        -> no usado en modo 1
 *   [7] mode             -> 1 = tracción
 *   [8] code_error
 *
 * Protocolo CAN motores RMD-X8 V4.2:
 *   Comando velocidad (0xA2): 8 bytes
 *   [0]=0xA2 [1-3]=0x00 [4-7]=velocidad_dps*100 en int32 little-endian
 *   Comando liberar  (0x80): 8 bytes todos 0x00 excepto [0]=0x80
 *
 * Hardware:
 *   - Arduino MKR WiFi 1010
 *   - Shield CAN (MCP2515) a 1 Mbps
 *   - Motores RMD-X8: FL=0x141, FR=0x142, RR=0x143, RL=0x144
 */

#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <CAN.h>

// ─── CONFIGURACIÓN RED ────────────────────────────────────────────────────────

const char SSID[] = "Horu";
const char PASS[] = "Horu27T-I";
const unsigned int PUERTO_LOCAL = 8885;

// ─── IDs MOTORES CAN ─────────────────────────────────────────────────────────

const int ID_FL = 0x141;   // Front Left
const int ID_FR = 0x142;   // Front Right
const int ID_RR = 0x143;   // Rear Right
const int ID_RL = 0x144;   // Rear Left

// ─── PARÁMETROS DE CONTROL ────────────────────────────────────────────────────

// Velocidad máxima permitida en dps (grados por segundo)
// El script ya aplica saturación en ±40 dps * 100 = ±4000 como int16
// Esta constante es una segunda capa de seguridad en el Arduino
const int16_t VEL_MAX_DPS = 40;

// Rate limiter: máxima variación de velocidad por ciclo (dps/ciclo)
// Con 50 paquetes/s del script, 8 dps/ciclo => rampa de ~250 ms de 0 a 40 dps
const float RATE_LIMIT = 8.0f;

// Watchdog: si no llega ningún paquete en este tiempo, parar motores
const unsigned long SAFETY_TIMEOUT_MS = 500;

// Intervalo mínimo entre envíos CAN al mismo motor (µs)
// A 1 Mbps, un frame de 8 bytes necesita ~130 µs. 250 µs da margen seguro.
const unsigned int CAN_INTER_FRAME_US = 250;

// ─── DATAGRAMA ────────────────────────────────────────────────────────────────

struct __attribute__((packed)) DatagramaPC {
  int16_t id;
  int16_t action_left_train;
  int16_t action_right_train;
  int16_t o0, o1, o2, o3;
  int16_t mode;
  int16_t code_error;
};

// ─── VARIABLES GLOBALES ───────────────────────────────────────────────────────

WiFiUDP udp;
DatagramaPC rxData;
int wifiStatus = WL_IDLE_STATUS;

unsigned long ultimoPaquete = 0;
bool motorParado = true;               // estado actual de los motores
bool avisoTimeoutMostrado = false;

// Velocidades actuales (con rate limiter aplicado)
float velIzqActual = 0.0f;
float velDerActual = 0.0f;

// ─── SETUP ────────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);  // espera max 3s al monitor serie

  Serial.println(F("\n╔══════════════════════════════════╗"));
  Serial.println(F("║  Horu MKR - Tracción diferencial  ║"));
  Serial.println(F("╚══════════════════════════════════╝\n"));

  // Iniciar bus CAN a 1 Mbps
  Serial.print(F("[CAN] Iniciando a 1 Mbps... "));
  if (!CAN.begin(1000E3)) {
    Serial.println(F("ERROR FATAL. Revisa el Shield CAN."));
    while (1);
  }
  Serial.println(F("OK"));

  // Liberar motores al arrancar (estado seguro)
  pararTodo();
  Serial.println(F("[CAN] Motores liberados (estado inicial seguro)."));

  // Conectar WiFi
  conectarWiFi();
  udp.begin(PUERTO_LOCAL);
  Serial.print(F("[UDP] Escuchando en puerto "));
  Serial.println(PUERTO_LOCAL);
  Serial.println(F("\nListo. Esperando paquetes del script Python...\n"));
}

// ─── LOOP PRINCIPAL ───────────────────────────────────────────────────────────

void loop() {
  // 1. Mantener conexión WiFi
  gestionarWiFi();

  // 2. Vaciar buffer UDP (nos quedamos con el paquete más reciente)
  bool hayDatosNuevos = leerUDP();

  // 3. Procesar datos recibidos
  if (hayDatosNuevos) {
    ultimoPaquete = millis();
    avisoTimeoutMostrado = false;
    motorParado = false;

    if (rxData.id == 1) {          // origen: PC / script Python
      procesarModo();
    }
    // Si id == 2 sería el ESP32 (no usado en este firmware de prueba)
  }

  // 4. Watchdog de seguridad
  if (millis() - ultimoPaquete > SAFETY_TIMEOUT_MS) {
    if (!motorParado) {
      Serial.println(F("\n[WATCHDOG] Timeout de conexión - parando motores."));
      pararTodo();
      velIzqActual = 0.0f;
      velDerActual = 0.0f;
      motorParado = true;
    }
    if (!avisoTimeoutMostrado) {
      Serial.println(F("[WATCHDOG] Esperando reconexión del script..."));
      avisoTimeoutMostrado = true;
    }
  }
}

// ─── LECTURA UDP ──────────────────────────────────────────────────────────────

bool leerUDP() {
  bool hayDatos = false;
  int tamano = udp.parsePacket();

  // Vaciamos el buffer completo; nos quedamos con el último paquete válido
  while (tamano > 0) {
    if (tamano >= (int)sizeof(DatagramaPC)) {
      udp.read((char*)&rxData, sizeof(DatagramaPC));
      hayDatos = true;
    } else {
      // Paquete demasiado pequeño: descartarlo
      udp.flush();
    }
    tamano = udp.parsePacket();
  }
  return hayDatos;
}

// ─── PROCESAMIENTO DE MODOS ───────────────────────────────────────────────────

void procesarModo() {
  switch (rxData.mode) {
    case 1:
      modoTraccion();
      break;

    default:
      // Modo no implementado en este firmware de prueba -> parar
      if (!motorParado) {
        pararTodo();
        motorParado = true;
      }
      break;
  }
}

// ─── MODO 1: TRACCIÓN DIFERENCIAL ────────────────────────────────────────────

void modoTraccion() {
  // Consignas recibidas del script (ya en dps escalados * 100 / GAIN)
  // El script envía: vel = joystick * GAIN * 100 (rango +-4000 aprox)
  // Convertimos a dps dividiendo por 100
  float velIzqConsigna = (float)rxData.action_left_train  / 100.0f;
  float velDerConsigna = (float)rxData.action_right_train / 100.0f;

  // Saturación de seguridad en Arduino (segunda capa)
  velIzqConsigna = saturar(velIzqConsigna, VEL_MAX_DPS);
  velDerConsigna = saturar(velDerConsigna, VEL_MAX_DPS);

  // Rate limiter: suaviza aceleración y deceleración
  velIzqActual = aplicarRateLimiter(velIzqActual, velIzqConsigna, RATE_LIMIT);
  velDerActual = aplicarRateLimiter(velDerActual, velDerConsigna, RATE_LIMIT);

  // Convertir a int16 para enviar por CAN (el motor espera dps * 100 como int32)
  int16_t cmdIzq = (int16_t)velIzqActual;
  int16_t cmdDer = (int16_t)velDerActual;

  // Invertir tren izquierdo (disposición mecánica de los motores)
  // Un valor positivo en FL/RL produce giro antihorario -> invertir para avanzar
  int16_t cmdIzqInv = cmdIzq * -1;

  // Debug (solo cuando hay movimiento real para no spammear)
  if (abs(cmdIzq) > 1 || abs(cmdDer) > 1) {
    Serial.print(F("[MODO1] IZQ="));
    Serial.print(cmdIzqInv);
    Serial.print(F(" dps  DER="));
    Serial.print(cmdDer);
    Serial.println(F(" dps"));
  }

  // Enviar a los 4 motores con delay entre tramas (FIX del fallo FL)
  enviarVelocidad(ID_FL, cmdIzqInv);  delayMicroseconds(CAN_INTER_FRAME_US);
  enviarVelocidad(ID_RL, cmdIzqInv);  delayMicroseconds(CAN_INTER_FRAME_US);
  enviarVelocidad(ID_FR, cmdDer);     delayMicroseconds(CAN_INTER_FRAME_US);
  enviarVelocidad(ID_RR, cmdDer);     delayMicroseconds(CAN_INTER_FRAME_US);  // <-- FIX: delay también aquí
}

// ─── ENVÍO CAN ────────────────────────────────────────────────────────────────

/*
 * Envía un comando de velocidad (0xA2) al motor indicado.
 * Incluye reintentos si el buffer TX está ocupado.
 * velocidad_dps: grados/segundo (se multiplica internamente por 100 para el protocolo)
 */
void enviarVelocidad(int id, int16_t velocidad_dps) {
  int32_t speedControl = (int32_t)velocidad_dps * 100;

  for (int intento = 0; intento < 5; intento++) {
    if (CAN.beginPacket(id)) {
      CAN.write(0xA2);
      CAN.write(0x00);
      CAN.write(0x00);
      CAN.write(0x00);
      CAN.write((uint8_t)( speedControl        & 0xFF));
      CAN.write((uint8_t)((speedControl >>  8) & 0xFF));
      CAN.write((uint8_t)((speedControl >> 16) & 0xFF));
      CAN.write((uint8_t)((speedControl >> 24) & 0xFF));
      CAN.endPacket();
      return;  // éxito
    }
    delayMicroseconds(100);  // buffer ocupado: esperar y reintentar
  }

  // Si llegamos aquí tras 5 intentos: problema físico o bus saturado
  Serial.print(F("[CAN] ERROR: no se pudo enviar a ID 0x"));
  Serial.println(id, HEX);
}

/*
 * Envía el comando de liberación (0x80) al motor indicado.
 * El motor queda en estado libre (sin par).
 */
void liberarMotor(int id) {
  for (int intento = 0; intento < 5; intento++) {
    if (CAN.beginPacket(id)) {
      CAN.write(0x80);
      CAN.write(0x00); CAN.write(0x00); CAN.write(0x00);
      CAN.write(0x00); CAN.write(0x00); CAN.write(0x00); CAN.write(0x00);
      CAN.endPacket();
      return;
    }
    delayMicroseconds(100);
  }
}

/*
 * Libera los 4 motores con delay entre tramas (mismo fix que modoTraccion).
 */
void pararTodo() {
  liberarMotor(ID_FL); delayMicroseconds(CAN_INTER_FRAME_US);
  liberarMotor(ID_RL); delayMicroseconds(CAN_INTER_FRAME_US);
  liberarMotor(ID_FR); delayMicroseconds(CAN_INTER_FRAME_US);
  liberarMotor(ID_RR); delayMicroseconds(CAN_INTER_FRAME_US);
}

// ─── UTILIDADES MATEMÁTICAS ───────────────────────────────────────────────────

float saturar(float valor, float maximo) {
  if (valor >  maximo) return  maximo;
  if (valor < -maximo) return -maximo;
  return valor;
}

/*
 * Limita la tasa de cambio de 'actual' hacia 'consigna'.
 * Evita aceleraciones bruscas sin necesidad de filtro adicional en el PC.
 */
float aplicarRateLimiter(float actual, float consigna, float maxDelta) {
  float delta = consigna - actual;
  if (delta >  maxDelta) return actual + maxDelta;
  if (delta < -maxDelta) return actual - maxDelta;
  return consigna;
}

// ─── GESTIÓN WIFI ─────────────────────────────────────────────────────────────

void conectarWiFi() {
  Serial.print(F("[WiFi] Conectando a '"));
  Serial.print(SSID);
  Serial.print(F("'..."));

  int intentos = 0;
  while (wifiStatus != WL_CONNECTED && intentos < 20) {
    wifiStatus = WiFi.begin(SSID, PASS);
    delay(500);
    Serial.print(F("."));
    intentos++;
  }

  if (wifiStatus == WL_CONNECTED) {
    Serial.println(F(" OK"));
    Serial.print(F("[WiFi] IP del Arduino: "));
    Serial.println(WiFi.localIP());
  } else {
    Serial.println(F(" FALLO - reintentando en loop"));
  }
}

/*
 * Reconexión WiFi no bloqueante: comprueba el estado en cada iteración
 * del loop y reconecta si se ha perdido la conexión, sin congelar el sistema.
 */
void gestionarWiFi() {
  static unsigned long ultimoCheckWiFi = 0;
  const unsigned long INTERVALO_CHECK_MS = 2000;

  if (millis() - ultimoCheckWiFi < INTERVALO_CHECK_MS) return;
  ultimoCheckWiFi = millis();

  wifiStatus = WiFi.status();
  if (wifiStatus != WL_CONNECTED) {
    Serial.println(F("[WiFi] Conexión perdida. Reconectando..."));
    pararTodo();   // seguridad: parar motores si se pierde el WiFi
    motorParado = true;
    WiFi.end();
    delay(100);
    wifiStatus = WiFi.begin(SSID, PASS);
  }
}