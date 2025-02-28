# -----------------
# you need to install libraries below
# pip install pandas
# pip install numpy
# pip install matplotlib
# -----------------



import os
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# CSVファイルのパス
csv_file = "../../data/log_2025-02-28_23-47-56_Gain_94.1_15.2_0.8_1.2_MaxV3.3.csv"

# データを読み込む
df = pd.read_csv(csv_file)

# rad -> deg 変換（角度は deg, 角速度は deg/s に変換）
df["theta_p"] = np.rad2deg(df["theta_p"])
df["theta_p_dot"] = np.rad2deg(df["theta_p_dot"])
df["theta_w"] = np.rad2deg(df["theta_w"])
df["theta_w_dot"] = np.rad2deg(df["theta_w_dot"])
df["theta_p_kf"] = np.rad2deg(df["theta_p_kf"])
df["theta_p_dot_kf"] = np.rad2deg(df["theta_p_dot_kf"])
df["theta_w_kf"] = np.rad2deg(df["theta_w_kf"])
df["theta_w_dot_kf"] = np.rad2deg(df["theta_w_dot_kf"])

# 出力ディレクトリを決定
output_dir = f"figure/{os.path.splitext(os.path.basename(csv_file))[0]}"
os.makedirs(output_dir, exist_ok=True)

# elapsed_timeを横軸に、それぞれの値をプロット
figures = []

def plot_and_save(x, y, y_kf, ylabel, filename):
    plt.figure()
    plt.plot(df[x].to_numpy(), df[y].to_numpy(), label=y, color='b')
    plt.plot(df[x].to_numpy(), df[y_kf].to_numpy(), label=y_kf, color='r', linestyle='dashed')
    plt.xlabel(x)
    plt.ylabel(ylabel)
    plt.legend()
    plt.grid()
    output_path = os.path.join(output_dir, filename)
    plt.savefig(output_path)
    figures.append(output_path)
    plt.close()

# 各プロット
plot_and_save("elapsed_time", "theta_p", "theta_p_kf", "Attitude angle [deg]", "theta_p_vs_kf.pdf")
plot_and_save("elapsed_time", "theta_p_dot", "theta_p_dot_kf", "Attitude angular vel. [deg/s]", "theta_p_dot_vs_kf.pdf")
plot_and_save("elapsed_time", "theta_w", "theta_w_kf", "Wheel angle [deg]", "theta_w_vs_kf.pdf")
plot_and_save("elapsed_time", "theta_w_dot", "theta_w_dot_kf", "Wheel angular vel. [deg/s]", "theta_w_dot_vs_kf.pdf")

# log_motor_valueの時間履歴をプロット
plt.figure()
plt.plot(df["elapsed_time"], df["log_motor_value"], label="log_motor_value", color='g')
plt.xlabel("elapsed_time")
plt.ylabel("Control input [V]")
plt.legend()
plt.grid()
motor_plot_path = os.path.join(output_dir, "log_motor_value.pdf")
plt.savefig(motor_plot_path)
figures.append(motor_plot_path)
plt.close()

print("保存されたPDFファイル:")
for f in figures:
    print(f)
