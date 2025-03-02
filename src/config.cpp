#include "config.h"

// グローバル変数の定義
int pi;
int bus_acc;
int bus_gyr;
int sample_num = 100;
float meas_interval = 10000.0f; // us micro seconds
float theta_mean;
float theta_variance;
float theta_dot_mean;
float theta_dot_variance;
float y[4][1];

const double PI = 3.14159265358979323846; // 円周率
const double rad2deg = 180.0 / PI;        // ラジアンを度に変換する定数
const double deg2rad = PI / 180.0;
// フォルダ名は最後に"/"を付けること
const std::string BASE_DIR = "/home/ubuntu/gcl-pendulum";
const std::string LOG_DATA_DIR = BASE_DIR + "/data/";
const std::string PARAM_DATA_DIR = BASE_DIR + "/param/gain.json";

int encoder_update_rate = 25; // usec
int encoder_resolution = 100;
int encoder_value = 0;
int table[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};
float pre_theta2 = 0.0f;

float theta_update_freq = 400.0f; // Hz
float theta_update_interval = 1.0f / theta_update_freq;
int th1_dura =
    1000000 * 1.0f / theta_update_freq; // 2500usec for theta_update_freq=400
float theta_data_predict[2][1] = {{0}, {0}};
float theta_data[2][1] = {{0}, {0}};
float P_theta_predict[2][2] = {{1, 0}, {0, 0}};
float P_theta[2][2] = {{0}};
float A_theta[2][2] = {{1, -theta_update_interval}, {0, 1}};
float B_theta[2][1] = {{theta_update_interval}, {0}};
float C_theta[1][2] = {{1, 0}};

float x_data_predict[4][1] = {{0}, {0}, {0}, {0}};
float x_data[4][1] = {{0}, {0}, {0}, {0}};
float P_x_predict[4][4] = {{0}};
float P_x[4][4] = {{0}};

// matrix A_x
float A_x[4][4] = {
    {1.002095034906568, 0.01000698521167948, 0, 4.373991285837984e-05},
    {0.4188267072641069, 1.002095034906568, 0, 0.008664755775936872},
    {-0.001109702169674044, -3.716839929508615e-06, 1, 0.0097068833062686},
    {-0.2198289309671251, -0.001109702169674044, 0, 0.9419534619478218}};

// matrix B_x
float B_x[4][1] = {{-0.0002812494396757963},
                   {-0.05571473621358585},
                   {0.001884752403108296},
                   {0.3732416284219275}};

// matrix C_x
float C_x[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};

// matrix Gain
float Gain[4];

// measurement noise matrix
float measure_variance_mat[4][4] = {{0}};
float voltage_error = 0.01f; // volt
float voltage_variance = voltage_error * voltage_error;

float feedback_rate = 0.01f; // sec
int feedback_dura = 10;      // msec
float motor_value = 0.0f;
int pwm_duty = 0;
int motor_direction = 1;
float motor_offset = 0.17f; // volt
// float motor_offset = 0.00f;   // volt
float MAX_VOLTAGE = 3.3f;     // モーターに入力する最大電圧 [V]
float BATTERY_VOLTAGE = 5.0f; // バッテリー電圧

// encoder
int enc_syn = 1;
int update_theta_syn_flag = 1;
int code;

// imu
float xAccl = 0.0f, yAccl = 0.0f, zAccl = 0.0f;
float xGyro = 0.0f, yGyro = 0.0f, zGyro = 0.0f;
