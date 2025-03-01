#include <cmath>
#include <unistd.h>

#include "config.h"
#include "motor_control.h"
#include "pigpiod_if2.h"

void motor_driver_init(int pi) {
    gpio_write(pi, IN1, 0);
    gpio_write(pi, IN2, 0);
    set_PWM_frequency(pi, PWM, PWM_FREQ);
    set_PWM_range(pi, PWM, 100);
    set_PWM_dutycycle(pi, PWM, 0);

    set_PWM_frequency(pi, IN1, PWM_FREQ);
    set_PWM_range(pi, IN1, 100);
    set_PWM_dutycycle(pi, IN1, 0);

    set_PWM_frequency(pi, IN2, PWM_FREQ);
    set_PWM_range(pi, IN2, 100);
    set_PWM_dutycycle(pi, IN2, 0);
}
void motor_control_update(bool update_motor) {
    if (!update_motor) {
        // モーター制御をリセット
        motor_value = 0;
        pwm_duty = 0;
        set_PWM_dutycycle(pi, PWM, pwm_duty);
        gpio_write(pi, IN1, 0);
        gpio_write(pi, IN2, 0);
        gpio_write(pi, LED_G, 0);
        gpio_write(pi, LED_R, 0);
        motor_direction = 0;
        return;
    }

    // モーター制御の更新処理
    motor_value = 0;

    // ステートフィードバックゲインを用いて制御入力を計算
    for (int i = 0; i < 4; i++) {
        motor_value += Gain[i] * x_data[i][0];
    }

    // オフセットを考慮
    if (motor_value > 0) {
        motor_value += motor_offset;
    } else {
        motor_value -= motor_offset;
    }

    if (motor_value > MAX_VOLTAGE) {
        motor_value = MAX_VOLTAGE;
    } else if (motor_value < -MAX_VOLTAGE) {
        motor_value = -MAX_VOLTAGE;
    }

    // PWMデューティサイクルの計算
    pwm_duty = static_cast<int>(motor_value * 100.0f / BATTERY_VOLTAGE);

    // モーターの駆動方向に応じてGPIOピンを制御
    if (pwm_duty >= 0) {
        // 順転
        set_PWM_dutycycle(pi, PWM, 100);

        set_PWM_dutycycle(pi, IN1, pwm_duty);
        set_PWM_dutycycle(pi, IN2, 0);

        gpio_write(pi, LED_G, 1);
        motor_direction = 1;
    } else {
        // PWMデューティサイクルの絶対値を計算
        pwm_duty = -pwm_duty;
        // 逆転
        set_PWM_dutycycle(pi, PWM, 100);

        set_PWM_dutycycle(pi, IN1, 0);
        set_PWM_dutycycle(pi, IN2, pwm_duty);

        gpio_write(pi, LED_R, 1);
        motor_direction = 2;
    }
}
