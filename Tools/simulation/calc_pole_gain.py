from InvertedPendulum import InvertedPendulum
import numpy as np

# 連続系の極を入力（負の値）
pole_c = np.array([-0.5427245011638713,
                -8.748851049822434,
                -5.553343483855886+0.6289469190780128j,
                -5.553343483855886-0.6289469190780128j])

pend = InvertedPendulum()
pend.calc_discrete_system()

# gain and eigen value of the continuous system
Gc = pend.continuous_acker(pole_c)
eig_c = [[ev.real, ev.imag] for ev in pole_c]    # 複素数を実数・虚数に分解

# gain and eigen value of the discrete system
pole_d = pend.c2d_poles(pole_c)
Gd = pend.discrete_acker(pole_d)
eig_d = [[ev.real, ev.imag] for ev in pole_d]

print("**********************************")
print("*        Calculation Result      *")
print("**********************************")
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
    "type": "Pole",
    "Eigenvalue_continuous": eig_c,
    "Eigenvalue_discrete": eig_d,
    "Gain_continuous": Gc.flatten().tolist(),  # 1D 配列に変換してリスト化, (not used for C++ code)
    "Gain_discrete": Gd.flatten().tolist(),       # used for C++ code
}

# JSON ファイルに保存
file_path = "../../param/gain.json"
pend.output_gain(file_path, gain_data)
