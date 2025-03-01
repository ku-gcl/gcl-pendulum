from InvertedPendulum import InvertedPendulum
import numpy as np

# pip install control
# python3 calculate_gain.py


Q = np.array([[10, 0, 0, 0], 
              [0, 10, 0 ,0], 
              [0, 0, 1, 0], 
              [0, 0, 0, 1]])
R = 100.0

pend = InvertedPendulum()
pend.calc_discrete_system()

P, L, G = pend.lqr(Q, R)

pole = L
F = pend.acker(L)

print(G)
print("--------")
print(F)

# TODO: gain.txtの出力
# 出力するのはQd, RdではなくQ, Rでよいのか？
# type=lqr: Q, R, G
# type=pole: pole, G
