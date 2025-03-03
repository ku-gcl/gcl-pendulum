from InvertedPendulum import InvertedPendulum
import numpy as np

# 極を入力
pole = np.array([0.9945874558754403, 
                 0.916229398609705, 
                 0.9459616842284228+0.005949675320093892j,
                 0.9459616842284228-0.005949675320093892j])

pend = InvertedPendulum()
pend.calc_discrete_system()

G = pend.acker(pole)
eigen_value = pole

# 複素数を実数・虚数に分解
eigen_value_serializable = [[ev.real, ev.imag] for ev in eigen_value]

print("**********************************")
print("*        Calculation Result      *")
print("**********************************")
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
    "type": "Pole",
    "Eigenvalue": eigen_value_serializable,  # 複素数を実数・虚数のリストに変換
    "Gain": G.flatten().tolist()  # 1D 配列に変換してリスト化
}

# JSON ファイルに保存
file_path = "../../param/gain.json"
pend.output_gain(file_path, gain_data)
