/*            Identifier v.1.0.1 
    v.1.0.1: Замена библиотеки DallasTemperature.h на microDS18B20.h
*/

#include "config.h"
#include "timer_blink.h"
#include <microDS18B20.h>

MicroDS18B20<TERM1_PIN> sensor1; // Режим без адресации(один датчик)
Timer echoDelay(1000);           // переиод вывод информации в uart 1000мс.

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(500);
  /* Initialization Input Pins */
  pinMode(CH1_PIN, INPUT);
  pinMode(CH2_PIN, INPUT);
  pinMode(CH3_PIN, INPUT);
  pinMode(BAT1_PIN, INPUT);
  pinMode(BAT2_PIN, INPUT);
  pinMode(CONNECT_PIN, INPUT);
  pinMode(TERM1_PIN, INPUT);
  pinMode(TERM2_PIN, INPUT);
  pinMode(BRIGHT_POT_PIN, INPUT);

  /* Initialization Output Pins */
  pinMode(REGIME_PIN, OUTPUT);
  pinMode(PWM_BRIGHT_DISPLAY_PIN, OUTPUT);
  pinMode(FAN1_PWM_PIN, OUTPUT);
  pinMode(FAN2_PIN, OUTPUT);
}

static bool manual_switch = false;

void loop() {
  sensor1.requestTemp(); // Запрос температуры
  
  if (echoDelay.ready()) {

    /* Print Regime Status */
    Serial.print("R: ");
    if (digitalRead(REGIME_PIN)) Serial.print("3");
    else Serial.print("4");
    Serial.print(", P: ");
    if (digitalRead(CONNECT_PIN)) Serial.println("OFF");
    else Serial.println("ON");

    /* Print Chanels Status */
    /* Chanel 1 */
    Serial.print("CH1: ");
    printChanelStatus(analogRead(CH1_PIN));
    Serial.print(", ");
    /* Chanel 2 */
    Serial.print("CH2: ");
    printChanelStatus(analogRead(CH2_PIN));
    Serial.print(", ");
    /* Chanel 3 */
    Serial.print("CH3: ");
    printChanelStatus(analogRead(CH3_PIN));
    Serial.print('\n');

    /* Print Battery Voltage */
    Serial.print("BAT1: ");
    Serial.print(voltageCalc5v(analogRead(BAT1_PIN)));
    Serial.print("v, ");
    Serial.print("BAT2: ");
    Serial.print(voltageCalc5v(analogRead(BAT2_PIN)));
    Serial.println("v");

    /* Print Temperature */
    Serial.print("Temp1: ");
        if (sensor1.readTemp()) { // Проверяем успешность чтения и выводим температуру
        Serial.print(sensor1.getTemp());
        Serial.println("C");
      } else Serial.println("Error");
    Serial.print('\n');
    Serial.println("-------------------------");
  }

  /* Parsing Serial */
  if (Serial.available() > 1) {
    char id = Serial.read();
    uint16_t value = Serial.parseInt();

    switch (id) {
      case 'r':
        if (value == 3) digitalWrite(REGIME_PIN, 1);
        if (value == 4) digitalWrite(REGIME_PIN, 0);
        break;

      case 'd':
        if (!manual_switch) {
          Serial.print("Light of screen = ");
          uint8_t constrVal = constrain(value, 0, 100);
          Serial.print(constrVal);
          Serial.println("%");
          analogWrite(PWM_BRIGHT_DISPLAY_PIN, map(constrVal, 0, 100, 0, 255));
        }
        break;

      case 's':
        Serial.print("lcd switch is ");
        if (value == 0) {
          manual_switch = false;
          Serial.println("OFF");
        }
        if (value == 1) {
          manual_switch = true;
          Serial.println("ON");
        }
        break;

        case 'f':
          Serial.print("Fan power = ");
          uint8_t constrValFan = constrain(value, 0, 100);
          Serial.print(constrValFan);
          Serial.println("%");
          analogWrite(FAN1_PWM_PIN, map(constrValFan, 0, 100, 0, 255));
          break;
    }
  }

  if (manual_switch) { // Управление яркостью с помощью потенциометра R10
    analogWrite(PWM_BRIGHT_DISPLAY_PIN, analogRead(BRIGHT_POT_PIN) / 4);
  }
}

void printChanelStatus(int adc) {
  if (adc < RANGE_COEFF) Serial.print("KZ");
  if (adc > NC_MIN && adc < NC_MAX) Serial.print("NC");
  if (adc > S1_MIN && adc < S1_MAX) Serial.print("S1");
  if (adc > S2_MIN && adc < S2_MAX) Serial.print("S2");
  if (adc > S3_MIN && adc < S3_MAX) Serial.print("S3");
  if (adc > B1_MIN && adc < B1_MAX) Serial.print("B1");
  if (adc > B2_MIN && adc < B2_MAX) Serial.print("B2");
  if (adc > B3_MIN && adc < B3_MAX) Serial.print("B3");
}

float voltageCalc5v(int adcValue) { // вычисление значения напряжения до 5в
  return  (float)adcValue * VA / 1024.0f;
}

float voltageCalc(int adcValue) { // вычисление значения напряжения с делителем напряжения(например для 12в)
  return voltageCalc5v(adcValue) * ((DIV_R1 + DIV_R2) / DIV_R2);
}
