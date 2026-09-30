#pragma once
#include <Arduino.h>

constexpr uint8_t NUM_MOTORES = 4; //En momento de compilacion
enum { M_DI = 0, M_TI = 1, M_DD = 2, M_TD = 3 }; //Crear constantes por motor

// Pines Motores -> DI, TI, DD, TD
const uint8_t PIN_INA[NUM_MOTORES] = { 22, 24, 26, 28 }; //IN1
const uint8_t PIN_INB[NUM_MOTORES] = { 23, 25, 27, 29 }; //IN2
const uint8_t PIN_EN[NUM_MOTORES]  = {  5,  6,  7,  8 }; //ENA

//Pines encoders con interrupcion -> DI, TI, DD, TD
const uint8_t PIN_ENC_A[NUM_MOTORES] = {  2,  3, 18, 19 }; //Cable Amarillo Señal A del Encoder
const uint8_t PIN_ENC_B[NUM_MOTORES] = { 30, 31, 32, 33 }; //Cable Negro Señal B del Encoder

//Para no recablear, cambiar valor aqui de signo si algo gira mal
const bool MOTOR_INVERTIDO[NUM_MOTORES]   = { false, false, true, true };
const bool ENCODER_INVERTIDO[NUM_MOTORES] = { false, false, true, true };

// Gira una rueda 10 vueltas completas a mano, cuenta los pulsos reales
// que reporta el encoder, y divide entre 10 para obtener este valor.
constexpr float PULSOS_POR_VUELTA = 1.0f; 

// Convierte pulso a mm reales que giraron
constexpr float MM_POR_TICK = 1.0f;

// MPU6050 - giroscopio
constexpr uint8_t MPU_DIRECCION = 0x68;
constexpr float GIRO_LSB_POR_GPS = 65.5f;   // sensibilidad para el rango +-500 grados/s
constexpr float YAW_SIGNO = 1.0f;           // cambiar a -1.0f si el yaw crece al reves

// Ajustar en vivo -> Ganancias PID
struct Parametros {
  // Velocidad de cada motor (PI + feedforward)
  float velKp = 1.5f; //Que tanto reacciona cada rueda frente al error
  float velKi = 0.0f; //Que tanto tiempo esta diespuesta a esperar si falla mucho
  float velIntMax = 100.0f; //Limite de frustracion p rueda
  float velFiltro = 0.3f;  //
  float pwmArranque[NUM_MOTORES] = { 40, 40, 40, 40 };          // Calibrar PWM minimo para que cada rueda empiece a girar
  float ffGanancia[NUM_MOTORES]  = { 1.0f, 1.0f, 1.0f, 1.0f };  // Calibrar qué tan rápido sube el PWM mientras pido velocidad

  // Mantener rumbo recto
  //Esto es por lado
  float rumboKp = 2.0f;
  float rumboKi = 0.0f;
  float rumboKd = 0.0f;
  float rumboMax = 60.0f;    // limite de la correccion diferencial entre lados

  // Girar donde este
  float giroKp = 2.0f;
  float giroKd = 0.0f;
  float giroMax = 150.0f;
  float giroMin = 40.0f;     // Vencer la friccion
  float giroTol = 2.0f;      // Grados para dar el giro terminado

  // Distancia a recorrer
  float distKp = 1.0f;
  float vMax = 200.0f;       // mm/s
  float vMin = 40.0f;
  float distTol = 5.0f;      // mm de tolerancia
  float acel = 400.0f;       // mm/s^2, rampa solo al acelerar

  float maxPwm = 200.0f;     // Limite de PWM real
};
