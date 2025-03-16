/* This file is used for mapping the GPIO to the software constants based on the board being built. */

#pragma once

#define __XRP_PIN_UNDEF 127

// Ultra Sonic max pulse
#define ULTRASONIC_MAX_PULSE_WIDTH 23200

// Servo pulses
#define XRP_SERVO_MIN_PULSE_US 500
#define XRP_SERVO_MAX_PULSE_US 2500

#ifdef PICO_RP2350

// Non-Beta board
#define PIN_LAYOUT_IDENT "Non-Beta"
#define __XRP_NONBETA

#else
// Beta
#define PIN_LAYOUT_IDENT "Beta"

/****************************************************************************************/
/******** Keep below until can upgrade platformio to sparkfun_xrp_controller_beta *******/
/****************************************************************************************/
#define MOTOR_L_IN_1 (6u)
#define MOTOR_L_IN_2 (7u)
#define MOTOR_R_IN_1 (14u)
#define MOTOR_R_IN_2 (15u)
#define MOTOR_3_IN_1 (2u)
#define MOTOR_3_IN_2 (3u)
#define MOTOR_4_IN_1 (10u)
#define MOTOR_4_IN_2 (11u)
#define MOTOR_L_ENCODER_A (4u)
#define MOTOR_L_ENCODER_B (5u)
#define MOTOR_R_ENCODER_A (12u)
#define MOTOR_R_ENCODER_B (13u)
#define MOTOR_3_ENCODER_A (0u)
#define MOTOR_3_ENCODER_B (1u)
#define MOTOR_4_ENCODER_A (8u)
#define MOTOR_4_ENCODER_B (9u)
#define SERVO_1 (16u)
#define SERVO_2 (17u)
#define I2C_SDA_1 (18u)
#define I2C_SCL_1 (19u)
#define DISTANCE_TRIGGER (20u)
#define DISTANCE_ECHO (21u)
#define LINE_L (26u)
#define LINE_R (27u)
#define BOARD_VIN_MEASURE (28u)
#define BOARD_USER_BUTTON (22u)
#define BOARD_LED (PIN_LED)
/*******************************************************************************/
/*******************************************************************************/

#define SERVO_3    __XRP_PIN_UNDEF
#define SERVO_4    __XRP_PIN_UNDEF

#endif