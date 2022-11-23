#ifndef _CONFIG_H_
#define _CONFIG_H_

#define CH1_PIN   A3
#define CH2_PIN   A4
#define CH3_PIN   A5
#define BAT1_PIN  A1
#define BAT2_PIN  A2
#define TERM1_PIN 3
#define TERM2_PIN 4
#define FAN1_PIN  5
#define FAN2_PIN  6

#define REGIME_PIN  7
#define CONNECT_PIN 2

#define BRIGHT_POT_PIN A0
#define PWM_BRIGHT_DISPLAY_PIN 9

#define NC_VALUE 1023
#define S1_VALUE 340
#define S2_VALUE 630
#define S3_VALUE 750
#define B1_VALUE 810
#define B2_VALUE 840
#define B3_VALUE 878
#define RANGE_COEFF 10
#define NC_MIN NC_VALUE - RANGE_COEFF
#define NC_MAX NC_VALUE + RANGE_COEFF
#define S1_MIN S1_VALUE - RANGE_COEFF
#define S1_MAX S1_VALUE + RANGE_COEFF
#define S2_MIN S2_VALUE - RANGE_COEFF
#define S2_MAX S2_VALUE + RANGE_COEFF
#define S3_MIN S3_VALUE - RANGE_COEFF
#define S3_MAX S3_VALUE + RANGE_COEFF
#define B1_MIN B1_VALUE - RANGE_COEFF
#define B1_MAX B1_VALUE + RANGE_COEFF
#define B2_MIN B2_VALUE - RANGE_COEFF
#define B2_MAX B2_VALUE + RANGE_COEFF
#define B3_MIN B3_VALUE - RANGE_COEFF
#define B3_MAX B3_VALUE + RANGE_COEFF

// GND -- [ R2 ] -- A0 -- [ R1 ] -- VIN
#define VA 5.0        // Напряжение на пине 5V (зависит от стабилизатора)
#define DIV_R1 10000  // Точное значение 10 кОм резистора
#define DIV_R2 4700   // Точное значение 4.7 кОм резистора

#endif /* _CONFIG_H_ */
