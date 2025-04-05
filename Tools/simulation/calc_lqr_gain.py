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
P, L, G = pend.lqr(Q, R)

# 安定化システムの固有値計算
A_BK = pend.Ad + np.dot(pend.Bd, G)
eigen_value, eigen_vector = pend.eig(A_BK)

# 複素数を実数・虚数に分解
eigen_value_serializable = [[ev.real, ev.imag] for ev in eigen_value]

print("**********************************")
print("*        Calculation Result      *")
print("**********************************")
print("LQR Weight")
print("Q = " + ", ".join(map(str, np.diag(Q))))
print("R = " + "{}".format(R))
print("")
print("----------------------------------")
print("Gain")
print(G)
print("")
print("----------------------------------")
print("Eigenvalue of the stable system")
print(", ".join(map(str, eigen_value)))
print("")
print("----------------------------------")

# JSON データを作成
gain_data = {
    "type": "LQR",
    "Q": np.diag(Q).tolist(),  # 対角成分をリスト化
    "R": R,
    "Eigenvalue": eigen_value_serializable,  # 複素数を実数・虚数のリストに変換
    "Gain": G.flatten().tolist()  # 1D 配列に変換してリスト化
}

# JSON ファイルに保存
file_path = "../../param/gain.json"
pend.output_gain(file_path, gain_data)
