/* Identifier v.1.0.0 */

#include "config.h"
#include "timer_blink.h"
#include <OneWire.h>
#include <DallasTemperature.h>

OneWire oneWire(TERM1_PIN);
DallasTemperature sensor1(&oneWire);
Timer tempDelay(800);   // Задержка для измерения температуры (750мс.)
Timer echoDelay(1000);  // переиод вывод информации в uart 1000мс.

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
  pinMode(FAN1_PIN, OUTPUT);
  pinMode(FAN2_PIN, OUTPUT);
  
  /* Initialization DS18S20_1 */
  sensor1.begin();
}

static bool manual_switch = false;

void loop() {
  
  if(echoDelay.ready()) {
    
    /* Print Regime Status */
    Serial.print("R: ");
    if(digitalRead(REGIME_PIN)) Serial.print("3");
    else Serial.print("4");
    Serial.print(", P: ");
    if(digitalRead(CONNECT_PIN)) Serial.println("OFF");
    else Serial.println("ON");
    
    /* Print Chanels Status */
    Serial.print("CH1: ");
    printChanelStatus(analogRead(CH1_PIN));
    Serial.print(", ");
  
    Serial.print("CH2: ");
    printChanelStatus(analogRead(CH2_PIN));
    Serial.print(", ");
  
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
    sensor1.requestTemperatures(); // Send the command to get temperatures
    Serial.print("Temp1: ");
    if(tempDelay.ready()) {
      float tempC = sensor1.getTempCByIndex(0);
      Serial.print(tempC);
      Serial.println("C");
    } else {
      Serial.print('\n');
    }
    Serial.println("-------------------------");
 }

  if(Serial.available() > 1) {
    char id = Serial.read();
    uint16_t value = Serial.parseInt();

    switch(id) {
      case 'r':
        if(value == 3) digitalWrite(REGIME_PIN, 1);
        if(value == 4) digitalWrite(REGIME_PIN, 0);
        break;
        
      case 'l':
        if(!manual_switch) {
          Serial.print("Light of screen = ");
          Serial.print(value);
          Serial.println("%");
          analogWrite(PWM_BRIGHT_DISPLAY_PIN, map(constrain(value, 0, 100), 0, 100, 0, 255));
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
    }
  }
  
  if(manual_switch) {
    analogWrite(PWM_BRIGHT_DISPLAY_PIN, analogRead(BRIGHT_POT_PIN) / 4);
  }
}

void printChanelStatus(int adc) {
  if(adc < RANGE_COEFF) Serial.print("KZ");
  if(adc > NC_MIN && adc < NC_MAX) Serial.print("NC");
  if(adc > S1_MIN && adc < S1_MAX) Serial.print("S1");
  if(adc > S2_MIN && adc < S2_MAX) Serial.print("S2");
  if(adc > S3_MIN && adc < S3_MAX) Serial.print("S3");
  if(adc > B1_MIN && adc < B1_MAX) Serial.print("B1");
  if(adc > B2_MIN && adc < B2_MAX) Serial.print("B2");
  if(adc > B3_MIN && adc < B3_MAX) Serial.print("B3");
}

float voltageCalc5v(int adcValue) { // вычисление значения напряжения до 5в
  return  (float)adcValue * VA / 1024.0f;
}

float voltageCalc(int adcValue) { // вычисление значения напряжения с делителем напряжения(например для 12в)
  return voltageCalc5v(adcValue) * ((DIV_R1 + DIV_R2) / DIV_R2);
}
