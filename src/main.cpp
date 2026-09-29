#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "pid.h"

Parametros P;

enum Estado : uint8_t { DETENIDO, AVANZANDO, GIRANDO };
Estado estado = DETENIDO;

// ============================================================================
// 2) ENCODERS - cuadratura completa, acceso directo a registros
// ============================================================================
// Con 4 encoders atendidos por interrupcion, leer el registro del puerto
// directamente (1 instruccion) en vez de digitalRead() (~50 instrucciones)
// importa de verdad para no perder pulsos.
volatile long cuentas[NUM_MOTORES] = { 0, 0, 0, 0 };
volatile uint8_t* regEncA[NUM_MOTORES];
volatile uint8_t* regEncB[NUM_MOTORES];
uint8_t mascaraEncA[NUM_MOTORES];
uint8_t mascaraEncB[NUM_MOTORES];
int8_t signoEnc[NUM_MOTORES];

static inline void flancoEncoder(uint8_t i) {
  bool a = (*regEncA[i] & mascaraEncA[i]) != 0;
  bool b = (*regEncB[i] & mascaraEncB[i]) != 0;
  cuentas[i] += (a != b) ? signoEnc[i] : -signoEnc[i];
}
void isrDI() { flancoEncoder(M_DI); }
void isrTI() { flancoEncoder(M_TI); }
void isrDD() { flancoEncoder(M_DD); }
void isrTD() { flancoEncoder(M_TD); }

// 'long' son 4 bytes en un chip de 8 bits: leerlo no es atomico.
// Se copian los 4 contadores con interrupciones apagadas (dura ~2us).
void leerCuentas(long out[NUM_MOTORES]) {
  noInterrupts();
  for (uint8_t i = 0; i < NUM_MOTORES; i++) out[i] = cuentas[i];
  interrupts();
}

void iniciarEncoders() {
  for (uint8_t i = 0; i < NUM_MOTORES; i++) {
    pinMode(PIN_ENC_A[i], INPUT_PULLUP);
    pinMode(PIN_ENC_B[i], INPUT_PULLUP);
    regEncA[i] = portInputRegister(digitalPinToPort(PIN_ENC_A[i]));
    regEncB[i] = portInputRegister(digitalPinToPort(PIN_ENC_B[i]));
    mascaraEncA[i] = digitalPinToBitMask(PIN_ENC_A[i]);
    mascaraEncB[i] = digitalPinToBitMask(PIN_ENC_B[i]);
    signoEnc[i] = ENCODER_INVERTIDO[i] ? -1 : 1;
  }
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A[M_DI]), isrDI, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A[M_TI]), isrTI, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A[M_DD]), isrDD, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A[M_TD]), isrTD, CHANGE);
}

// ============================================================================
// 3) MOTORES (L298N)
// ============================================================================
float pwmAplicado[NUM_MOTORES] = { 0, 0, 0, 0 };

void iniciarMotores() {
  for (uint8_t i = 0; i < NUM_MOTORES; i++) {
    pinMode(PIN_INA[i], OUTPUT);
    pinMode(PIN_INB[i], OUTPUT);
    pinMode(PIN_EN[i], OUTPUT);
  }
}

// u con signo: + = adelante, - = atras
void motorAplicar(uint8_t i, float u) {
  pwmAplicado[i] = u;
  if (MOTOR_INVERTIDO[i]) u = -u;
  int pwm = constrain((int)(fabs(u) + 0.5f), 0, (int)P.maxPwm);
  if (pwm == 0) {
    digitalWrite(PIN_INA[i], LOW);
    digitalWrite(PIN_INB[i], LOW);
  } else if (u > 0) {
    digitalWrite(PIN_INA[i], HIGH);
    digitalWrite(PIN_INB[i], LOW);
  } else {
    digitalWrite(PIN_INA[i], LOW);
    digitalWrite(PIN_INB[i], HIGH);
  }
  analogWrite(PIN_EN[i], pwm);
}

void frenarTodos() {
  for (uint8_t i = 0; i < NUM_MOTORES; i++) {
    pwmAplicado[i] = 0;
    digitalWrite(PIN_INA[i], HIGH);
    digitalWrite(PIN_INB[i], HIGH);
    analogWrite(PIN_EN[i], 255);   // freno activo
  }
}

// ============================================================================
// 4) MPU6050 - solo el eje Z del giroscopio (yaw). Nada de acelerometro:
//    la cascada nunca lo usa, asi que no se lee.
// ============================================================================
float yaw = 0;         // grados, continuo
float sesgoGz = 0;     // sesgo del giroscopio en cuentas crudas
bool imuOk = false;

void mpuEscribir(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_DIRECCION);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool mpuLeerGiroZ(int16_t &gz) {
  Wire.beginTransmission(MPU_DIRECCION);
  Wire.write(0x47);   // GYRO_ZOUT_H
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU_DIRECCION, (uint8_t)2) != 2) return false;
  uint8_t alto = Wire.read();
  uint8_t bajo = Wire.read();
  gz = (int16_t)((alto << 8) | bajo);
  return true;
}

bool mpuIniciar() {
  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(3000, true);   // si el ruido de motores traba el bus, no se cuelga
  mpuEscribir(0x6B, 0x01);   // PWR_MGMT_1: despertar
  delay(50);
  mpuEscribir(0x1A, 0x03);   // CONFIG: filtro pasa-bajas ~44Hz (quita vibracion de motores)
  mpuEscribir(0x1B, 0x08);   // GYRO_CONFIG: rango +-500 grados/s
  delay(50);
  int16_t prueba;
  return mpuLeerGiroZ(prueba);
}

// Robot QUIETO al llamar esto: promedia el giroscopio para hallar su sesgo.
void calibrarGiroscopio() {
  if (!imuOk) return;
  Serial.println(F("Calibrando giroscopio... no muevan el robot."));
  float suma = 0;
  int validas = 0;
  for (int k = 0; k < 300; k++) {
    int16_t gz;
    if (mpuLeerGiroZ(gz)) { suma += gz; validas++; }
    delay(3);
  }
  if (validas > 0) sesgoGz = suma / validas;
  yaw = 0;
  Serial.println(F("Giroscopio listo."));
}

void actualizarIMU(float dt) {
  if (!imuOk || dt > 0.1f) return;
  int16_t gz;
  if (!mpuLeerGiroZ(gz)) return;
  float velGrados = (gz - sesgoGz) / GIRO_LSB_POR_GPS * YAW_SIGNO;
  yaw += velGrados * dt;
}

// ============================================================================
// 5) LAZOS DE CONTROL - la cascada de 3 niveles
// ============================================================================
PID pidVel[NUM_MOTORES];
PID pidRumbo;
PID pidGiro;

float velMedida[NUM_MOTORES]   = { 0, 0, 0, 0 };   // mm/s
float velObjetivo[NUM_MOTORES] = { 0, 0, 0, 0 };   // mm/s
long cuentasPrevias[NUM_MOTORES] = { 0, 0, 0, 0 };

float yawObjetivo = 0;
float vBase = 0;
float distObjetivoMM = 0;
long cuentasInicio[NUM_MOTORES];

void aplicarParametros() {
  for (uint8_t i = 0; i < NUM_MOTORES; i++)
    pidVel[i].configurar(P.velKp, P.velKi, 0.0f, P.maxPwm, P.velIntMax);
  pidRumbo.configurar(P.rumboKp, P.rumboKi, P.rumboKd, P.rumboMax, P.rumboMax * 0.5f);
  pidGiro.configurar(P.giroKp, 0.0f, P.giroKd, P.giroMax, 0.0f);
}

void medirVelocidades(float dt) {
  long c[NUM_MOTORES];
  leerCuentas(c);
  for (uint8_t i = 0; i < NUM_MOTORES; i++) {
    float v = (c[i] - cuentasPrevias[i]) * MM_POR_TICK / dt;
    cuentasPrevias[i] = c[i];
    velMedida[i] += P.velFiltro * (v - velMedida[i]);
  }
}

// NIVEL 1: PI + feedforward por motor
void lazoVelocidad(float objIzq, float objDer, float dt) {
  velObjetivo[M_DI] = velObjetivo[M_TI] = objIzq;
  velObjetivo[M_DD] = velObjetivo[M_TD] = objDer;
  for (uint8_t i = 0; i < NUM_MOTORES; i++) {
    float obj = velObjetivo[i];
    float ff = 0;
    if (obj > 0.5f)       ff =   P.pwmArranque[i] + P.ffGanancia[i] * obj;
    else if (obj < -0.5f) ff = -(P.pwmArranque[i] - P.ffGanancia[i] * obj);
    float u = pidVel[i].calcular(obj, velMedida[i], dt, ff);
    motorAplicar(i, u);
  }
}

float recorridoMM() {
  long c[NUM_MOTORES];
  leerCuentas(c);
  float suma = 0;
  for (uint8_t i = 0; i < NUM_MOTORES; i++) suma += (c[i] - cuentasInicio[i]);
  return suma / NUM_MOTORES * MM_POR_TICK;
}

void terminarMovimiento() {
  frenarTodos();
  vBase = 0;
  estado = DETENIDO;
}

// NIVEL 3 + NIVEL 2a: avanzar recto una distancia
void pasoAvanzar(float dt) {
  float restante = distObjetivoMM - recorridoMM();
  if (fabs(restante) <= P.distTol || restante * distObjetivoMM < 0) {
    terminarMovimiento();
    return;
  }
  float vDeseada = constrain(P.distKp * fabs(restante), P.vMin, P.vMax);
  if (restante < 0) vDeseada = -vDeseada;
  if (fabs(vDeseada) > fabs(vBase)) {
    float paso = P.acel * dt;
    vBase += constrain(vDeseada - vBase, -paso, paso);
  } else {
    vBase = vDeseada;
  }
  float dv = pidRumbo.calcular(yawObjetivo, yaw, dt);
  lazoVelocidad(vBase - dv, vBase + dv, dt);
}

// NIVEL 2b: girar en su lugar (usa el giroscopio, no los encoders, porque
// las llantas derrapan al girar y los encoders mentirian sobre el angulo real)
void pasoGirar(float dt) {
  float error = yawObjetivo - yaw;
  if (fabs(error) <= P.giroTol) {
    terminarMovimiento();
    return;
  }
  float dv = pidGiro.calcular(yawObjetivo, yaw, dt);
  if (fabs(dv) < P.giroMin) dv = (error > 0) ? P.giroMin : -P.giroMin;
  lazoVelocidad(-dv, dv, dt);
}

// ============================================================================
// 6) COMANDOS MINIMOS POR SERIAL - solo para disparar una prueba
// ============================================================================
void iniciarAvance(float mm) {
  for (uint8_t i = 0; i < NUM_MOTORES; i++) pidVel[i].reiniciar();
  leerCuentas(cuentasInicio);
  distObjetivoMM = mm;
  vBase = 0;
  pidRumbo.reiniciar();
  yawObjetivo = yaw;   // ir derecho respecto al rumbo actual
  estado = AVANZANDO;
  Serial.print(F("Avanzando ")); Serial.print(mm); Serial.println(F(" mm"));
}

void iniciarGiro(float grados) {
  for (uint8_t i = 0; i < NUM_MOTORES; i++) pidVel[i].reiniciar();
  yawObjetivo = yaw + grados;
  pidGiro.reiniciar();
  estado = GIRANDO;
  Serial.print(F("Girando ")); Serial.print(grados); Serial.println(F(" grados"));
}

void detenerTodo() {
  frenarTodos();
  vBase = 0;
  estado = DETENIDO;
  Serial.println(F("Detenido."));
}

void procesarComando(char* linea) {
  char cmd = linea[0];
  float valor = atof(linea + 1);
  switch (cmd) {
    case 'f': iniciarAvance(valor); break;
    case 'g': iniciarGiro(valor); break;
    case 's': detenerTodo(); break;
    default:  Serial.println(F("Comandos: f<mm>  g<grados>  s"));
  }
}

void leerSerial() {
  static char linea[24];
  static uint8_t largo = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (largo > 0) { linea[largo] = '\0'; procesarComando(linea); largo = 0; }
    } else if (largo < sizeof(linea) - 1) {
      linea[largo++] = c;
    }
  }
}

// ============================================================================
// 7) DEBUG MINIMO - una linea de estado, nada de formato Teleplot
// ============================================================================
void imprimirDebug() {
  static uint32_t tUltimo = 0;
  if (millis() - tUltimo < 250) return;
  tUltimo = millis();
  const char* nombre = (estado == DETENIDO) ? "DETENIDO" : (estado == AVANZANDO) ? "AVANZANDO" : "GIRANDO";
  Serial.print(F("estado=")); Serial.print(nombre);
  Serial.print(F("  yaw="));  Serial.print(yaw, 1);
  Serial.print(F("  vDI="));  Serial.print(velMedida[M_DI], 0);
  Serial.print(F("  vTI="));  Serial.print(velMedida[M_TI], 0);
  Serial.print(F("  vDD="));  Serial.print(velMedida[M_DD], 0);
  Serial.print(F("  vTD="));  Serial.print(velMedida[M_TD], 0);
  Serial.println();
}

// ============================================================================
// SETUP y LOOP
// ============================================================================
uint32_t tControl = 0;
const uint32_t CONTROL_PERIODO_MS = 20;   // 50 Hz para toda la cascada

void setup() {
  Serial.begin(115200);
  iniciarMotores();
  frenarTodos();
  iniciarEncoders();
  aplicarParametros();

  imuOk = mpuIniciar();
  if (imuOk) {
    calibrarGiroscopio();
  } else {
    Serial.println(F("!! MPU6050 no responde: revisar SDA->20, SCL->21, VCC, GND."));
    Serial.println(F("   Avanzar en linea recta no va a corregir rumbo; girar no funcionara."));
  }

  Serial.println(F("Listo. Comandos: f<mm>  g<grados>  s"));
}

void loop() {
  uint32_t ahora = millis();
  if (ahora - tControl >= CONTROL_PERIODO_MS) {
    float dt = (ahora - tControl) / 1000.0f;
    tControl = ahora;

    actualizarIMU(dt);
    medirVelocidades(dt);

    switch (estado) {
      case DETENIDO:  frenarTodos();   break;
      case AVANZANDO: pasoAvanzar(dt); break;
      case GIRANDO:   pasoGirar(dt);   break;
    }
  }

  leerSerial();
  imprimirDebug();
}