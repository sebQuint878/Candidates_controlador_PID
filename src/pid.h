#pragma once
#include <Arduino.h>

class PID {
  public:
    void configurar(float kp, float ki, float kd, float salidaMax, float intMax) {
      Kp = kp; Ki = ki; Kd = kd;
      this->salidaMax = salidaMax;
      this->intMax = intMax;
    }

    // feedforward, para sumar un valor fijo a la salida del PID.
    float calcular(float setpoint, float medido, float dt, float ff = 0.0f) {
      float error = setpoint - medido;

      integral += error * dt;
      integral = constrain(integral, -intMax, intMax);

      float derivada = (dt > 0.0f) ? (error - errorPrevio) / dt : 0.0f;
      errorPrevio = error;

      float salida = Kp * error + Ki * integral + Kd * derivada + ff;
      return constrain(salida, -salidaMax, salidaMax);
    }

    // Llamar SIEMPRE al iniciar un movimiento nuevo, o la integral y la
    // derivada arrastran datos del movimiento anterior.
    void reiniciar() {
      integral = 0;
      errorPrevio = 0;
    }

  private:
    float Kp = 0, Ki = 0, Kd = 0;
    float salidaMax = 255, intMax = 255;
    float integral = 0;
    float errorPrevio = 0;
};
