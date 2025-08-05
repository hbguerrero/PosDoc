#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

//MOtor1
#define ENC_A1 13
#define ENC_B1 14
#define PWM_PIN1 16
#define DIR11 17
#define DIR21 18

//Motor2
#define ENC_A2 34
#define ENC_B2 35
#define PWM_PIN2 23
#define DIR12 26
#define DIR22 27

// ⚙️ Parámetros físicos
const float PPR = 2370.6 * 4.0;   // pulsos por vuelta * cuadratura
const float Ts = 0.005;            // 5 ms
const float zonaMuerta = 5.0;     // voltios
const float omegaMax = 3.1;       // [rad/s]
const float voltajeMax = 12.0;    // [V]
const int pwmMax = 255;

// Controlador PI
float Kp = 16;
float Ki = 16;
float integral = 0;
float integral2 = 0;
float controlVolt1 = 0;
float controlVolt2 = 0;

// Variables de estado
volatile long encoderCount = 0;
float velocidadDeseada1 = 0.0;

volatile long encoderCount2 = 0;
float velocidadDeseada2 = 0.0;

unsigned long lastMillis = 0;

// 📉 Media móvil (N=5)
#define N 5
float omega1_hist[N] = {0};
float omega2_hist[N] = {0};
int idx_filtro = 0;

void IRAM_ATTR encoderISR_A1() {
  bool A = digitalRead(ENC_A1);
  bool B = digitalRead(ENC_B1);
  encoderCount += (A == B) ? 1 : -1;
}

void IRAM_ATTR encoderISR_B1() {
  bool A = digitalRead(ENC_A1);
  bool B = digitalRead(ENC_B1);
  encoderCount += (A != B) ? 1 : -1;
}

void IRAM_ATTR encoderISR_A2() {
  bool A = digitalRead(ENC_A2);
  bool B = digitalRead(ENC_B2);
  encoderCount2 += (A == B) ? 1 : -1;
}

void IRAM_ATTR encoderISR_B2() {
  bool A = digitalRead(ENC_A2);
  bool B = digitalRead(ENC_B2);
  encoderCount2 += (A != B) ? 1 : -1;
}

void setup() {
  Serial.begin(115200);

  pinMode(ENC_A1, INPUT_PULLUP);
  pinMode(ENC_B1, INPUT_PULLUP);
  pinMode(ENC_A2, INPUT_PULLUP);
  pinMode(ENC_B2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_A1), encoderISR_A1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B1), encoderISR_B1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A2), encoderISR_A2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B2), encoderISR_B2, CHANGE);

  pinMode(DIR11, OUTPUT);
  pinMode(DIR21, OUTPUT);
  digitalWrite(DIR11, HIGH);
  digitalWrite(DIR21, LOW);

  pinMode(DIR12, OUTPUT);
  pinMode(DIR22, OUTPUT);
  digitalWrite(DIR12, HIGH);
  digitalWrite(DIR22, LOW);

  ledcSetup(0, 5000, 8);
  ledcAttachPin(PWM_PIN1, 0);
  ledcSetup(1, 5000, 8);
  ledcAttachPin(PWM_PIN2, 1);

  if (!bno.begin()) {
    Serial.println("No BNO055 detected!");
    while (1);
  }
  bno.setExtCrystalUse(true);

  Serial.println("Listo. Ingresa la velocidad deseada en rad/s:");
}

void loop() {
  static long lastCount = 0;
  static long lastCount2 = 0;

  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    int commaIndex = input.indexOf(',');
    if (commaIndex != -1) {
      String vel1_str = input.substring(0, commaIndex);
      String vel2_str = input.substring(commaIndex + 1);
      float nuevaVel1 = vel1_str.toFloat();
      float nuevaVel2 = vel2_str.toFloat();
      if (nuevaVel1 >= 0 && nuevaVel1 <= 3.1) velocidadDeseada1 = nuevaVel1;
      if (nuevaVel2 >= 0 && nuevaVel2 <= 3.1) velocidadDeseada2 = nuevaVel2;
    }
  }

  if (millis() - lastMillis >= (Ts * 1000)) {
    long delta = encoderCount - lastCount;
    lastCount = encoderCount;

    long delta2 = encoderCount2 - lastCount2;
    lastCount2 = encoderCount2;

    float omega1 = (delta / PPR) * (2 * PI / Ts);
    float omega2 = (delta2 / PPR) * (2 * PI / Ts);

    // 🧮 Aplicar filtro de media móvil
    omega1_hist[idx_filtro] = omega1;
    omega2_hist[idx_filtro] = omega2;

    float omega1_sum = 0;
    float omega2_sum = 0;
    for (int i = 0; i < N; i++) {
      omega1_sum += omega1_hist[i];
      omega2_sum += omega2_hist[i];
    }

    float omega1_filt = omega1_sum / N;
    float omega2_filt = omega2_sum / N;

    idx_filtro = (idx_filtro + 1) % N;

    // ⚙️ Controlador PI con señal filtrada
    float error = velocidadDeseada1 - omega1_filt;
    integral += error * Ts;
    controlVolt1 = Kp * error + Ki * integral;

    float error2 = velocidadDeseada2 - omega2_filt;
    integral2 += error2 * Ts;
    controlVolt2 = Kp * error2 + Ki * integral2;

    // ✅ Compensar zona muerta
    if (controlVolt1 > 0) controlVolt1 += zonaMuerta;
    else if (controlVolt1 < 0) controlVolt1 -= zonaMuerta;
    controlVolt1 = constrain(controlVolt1, 0, voltajeMax);

    if (controlVolt2 > 0) controlVolt2 += zonaMuerta;
    else if (controlVolt2 < 0) controlVolt2 -= zonaMuerta;
    controlVolt2 = constrain(controlVolt2, 0, voltajeMax);

    // PWM
    int pwm1 = (int)((controlVolt1 / voltajeMax) * pwmMax);
    pwm1 = constrain(pwm1, 0, pwmMax);
    ledcWrite(0, pwm1);

    int pwm2 = (int)((controlVolt2 / voltajeMax) * pwmMax);
    pwm2 = constrain(pwm2, 0, pwmMax);
    ledcWrite(1, pwm2);

    // 🧭 IMU lectura
    sensors_event_t event;
    bno.getEvent(&event);
    int imu_heading = int(event.orientation.x);

    // 📊 Envío para ROS2 o Teleplot
    Serial.print(">omega1:"); Serial.println(omega1_filt, 3);
    Serial.print(">omega2:"); Serial.println(omega2_filt, 3);
    Serial.print(">imu:");    Serial.println(imu_heading);

    lastMillis = millis();
  }
}
