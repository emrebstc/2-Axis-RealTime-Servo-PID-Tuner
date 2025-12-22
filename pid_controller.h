#ifndef __PID_CONTROLLER_H
#define __PID_CONTROLLER_H

#include "main.h"

//  Kontrol Veri Yapısı
typedef struct {
    float setpoint_x;       // Derece cinsinden
    float setpoint_y;
    uint16_t adc_x_value;
    uint16_t adc_y_value;
    uint16_t current_pwm_x;
    uint16_t current_pwm_y;
} ControlData_t;

// PID Kontrolcü Yapısı
typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float previous_error;
    float dt;
    float out_min;
    float out_max;
    float filtered_d;      // Her Eksenin Kendi D filtresi
} PID_Controller_t;

// Global Degiskenler
extern ControlData_t g_control_data;
extern PID_Controller_t pid_x;
extern PID_Controller_t pid_y;
extern uint16_t g_adc_raw_buffer[2];
#define ADC_BUF_SIZE 2

// Fonksiyon Prototipleri
float Calculate_PID(PID_Controller_t *pid, float current_value, float setpoint);
float ADC_To_Degree(uint16_t adc);

#endif
