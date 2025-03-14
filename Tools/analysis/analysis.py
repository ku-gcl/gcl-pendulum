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
# csv_file = "../../data/log_2025-03-02_14-27-27_Gain_31.0_4.9_0.1_0.4_MaxV3.3.csv"
csv_file = "../../data/log_2025-03-02_15-35-25_Gain_31.0_4.9_0.1_0.4_MaxV3.3.csv"

# データを読み込む。最初の5行はゲイン情報などなので、スキップ
df = pd.read_csv(csv_file, skiprows=5)

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
    plt.xlabel("Time [s]")
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
plt.plot(df["elapsed_time"].to_numpy(), df["log_motor_value"].to_numpy(), label="log_motor_value", color='g')
plt.xlabel("Time [s]")
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
