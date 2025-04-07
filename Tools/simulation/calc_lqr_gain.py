from InvertedPendulum import InvertedPendulum
import numpy as np

# LQR のパラメータ
# x=[振子角度, 振子角速度, 車輪角度, 車輪角速度] の順番

# Q = np.diag([10, 10, 1, 1])
# R = 100.0

Q = np.diag([10, 10, 1, 1])
R = 1.0

pend = InvertedPendulum()
pend.calc_discrete_system()

# LQR 計算
Pc, Lc, Gc = pend.lqr(Q, R)
Pd, Ld, Gd = pend.dlqr(Q, R)

# 安定化システムの固有値計算 (continuous system)
A_BK = pend.A + np.dot(pend.B, Gc)
eigen_value, eigen_vector = pend.eig(A_BK)
eig_c = [[ev.real, ev.imag] for ev in eigen_value]    # 複素数を実数・虚数に分解

# 安定化システムの固有値計算 (discrete system)
A_BK = pend.Ad + np.dot(pend.Bd, Gd)
eigen_value, eigen_vector = pend.eig(A_BK)
eig_d = [[ev.real, ev.imag] for ev in eigen_value]    # 複素数を実数・虚数に分解

print("**********************************")
print("*        Calculation Result      *")
print("**********************************")
print("LQR Weight")
print("Q = " + ", ".join(map(str, np.diag(Q))))
print("R = " + "{}".format(R))
print("")
print("----------------------------------")
print("Gain of the continuous system")
print(Gc)
print("")
print("----------------------------------")
print("Eigenvalue of the stable system (continuous system)")
print(", ".join(map(str, eig_c)))
print("")
print("----------------------------------")

# JSON データを作成
gain_data = {
    "type": "LQR",
    "Q": np.diag(Q).tolist(),  # 対角成分をリスト化
    "R": R,
    "Eigenvalue_continuous": eig_c,
    "Eigenvalue_discrete": eig_d,
    "Gain_continuous": Gc.flatten().tolist(),  # 1D 配列に変換してリスト化
    "Gain_discrete": Gd.flatten().tolist(),       # used for C++ code
}

# JSON ファイルに保存
file_path = "../../param/gain.json"
pend.output_gain(file_path, gain_data)
