#pragma once
#include <Arduino.h>

class PID {
  public:
    void configurar(float kp, float ki, float kd, float salidaMax, float intMax) {
      Kp = kp; Ki = ki; Kd = kd;
      this->salidaMax = salidaMax;
      this->intMax = intMax;
    }

        // salida = ff + Kp*error + I + D
    float calcular(float setpoint, float medido, float dt, float ff = 0.0f) {
      if (dt <= 0.0f) return constrain(ff, -salidaMax, salidaMax);
      float error = setpoint - medido;

      float derivada = primera ? 0.0f : -(medido - medidoPrevio) / dt;
      medidoPrevio = medido;
      primera = false;

      float integralNueva = constrain(integral + Ki * error * dt, -intMax, intMax);
      float prueba = ff + Kp * error + integralNueva + Kd * derivada;
      bool saturaArriba = (prueba >  salidaMax) && (error > 0);
      bool saturaAbajo  = (prueba < -salidaMax) && (error < 0);
      if (!saturaArriba && !saturaAbajo) integral = integralNueva;

      float salida = ff + Kp * error + integral + Kd * derivada;
      return constrain(salida, -salidaMax, salidaMax);
    }

    // Llamar SIEMPRE al iniciar un movimiento nuevo
    void reiniciar() {
      integral = 0;
      primera = true;
    }

  private:
    float Kp = 0, Ki = 0, Kd = 0;
    float salidaMax = 255, intMax = 255;
    float integral = 0;      // ya multiplicada por Ki
    float medidoPrevio = 0;
    bool primera = true;
};