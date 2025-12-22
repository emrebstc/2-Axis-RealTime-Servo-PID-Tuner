#include "pid_controller.h"

// ADC den Dereceye mapping
#define ADC_MIN_MAP   930.0f
#define ADC_MAX_MAP   3490.0f

#define DEG_MIN   0.0f
#define DEG_MAX   180.0f

// Servo PWM limitleri
#define PWM_MIN     400.0f
#define PWM_MAX     2400.0f
#define PWM_CENTER  1400.0f

float filtered_degree = 0.0f;
const float ALPHA = 0.01f; // Filtre katsayısı

// Yonu duzeltilmis adc den dereceye cevirme
float ADC_To_Degree(uint16_t adc)
{
    if (adc < ADC_MIN_MAP) adc = ADC_MIN_MAP;
    if (adc > ADC_MAX_MAP) adc = ADC_MAX_MAP;

    // Önce normal (0-180) haritalamayı hesaplama
    float calculated_deg =
        (adc - ADC_MIN_MAP) * (DEG_MAX - DEG_MIN)
        / (ADC_MAX_MAP - ADC_MIN_MAP);

    // Sonucu ters çevirme (180 - calculated_deg)
    // ADC_MIN > 0 yerine 180 e gidecek
    // ADC_MAX > 180 yerine 0 a gidecek
    return DEG_MAX - calculated_deg;
}

// Control data
ControlData_t g_control_data =
{
    .setpoint_x = 90.0f, //Baslangic degerleri
    .setpoint_y = 90.0f, //Baslangic degerleri
};

// Pİd katsayilari X
PID_Controller_t pid_x = {
    .Kp = 0.6f, .Ki = 0.1f, .Kd = 0.005f, .dt = 0.005f,
    .out_min = -1500.0f, .out_max = 1500.0f,
    .integral = 0.0f, .previous_error = 0.0f
};

// Pİd katsayilari Y
PID_Controller_t pid_y = {
    .Kp = 0.5f, .Ki = 0.1f, .Kd = 0.005f, .dt = 0.005f,
    .out_min = -1500.0f, .out_max = 1500.0f,
    .integral = 0.0f, .previous_error = 0.0f
};

// PID hesaplama
float Calculate_PID(PID_Controller_t *pid, float current_deg, float target_deg)
{
    float error = target_deg - current_deg;
    float deadband = 0.5f;

    // Statik offseti disarida hesaplama
    float target_pwm_base = ((target_deg / 180.0f) * 2000.0f) + 400.0f;

    // Ölü bölge kontrolu
    if (error < deadband && error > -deadband)
    {
        pid->integral *= 0.9f;
        pid->previous_error = error;
        return target_pwm_base;
    }

    // İntegral yanasma bölgesinde calisip calismamasini kontrol etme yoksa hata birikiyor
    if (error < 3.0f && error > -3.0f) {
        pid->integral += error * pid->dt;
    } else {
        pid->integral = 0;
    }

    // Turev
    float delta_error = error - pid->previous_error;
    float derivative = delta_error / pid->dt;

    // Struct icindeki filtered_d kullanma
    pid->filtered_d = (0.2f * derivative) + (0.8f * pid->filtered_d);

    pid->previous_error = error;

    // cikis fonksiyonu
    float output = (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * pid->filtered_d);

    // Düzeltmeyi sinirlama (+-300 pwm arasi)
    if (output > 300.0f)  output = 300.0f;
    if (output < -300.0f) output = -300.0f;

    return output + target_pwm_base;
}
