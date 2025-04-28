


######
# cd ~/gcl-pendulum/Tools/simulation/test
# python3 test_c2d_d2c_poles.py
######


import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


from InvertedPendulum import InvertedPendulum
import numpy as np

discrete_poles = np.array([0.9945874558754403, 
                 0.916229398609705, 
                 0.9459616842284228+0.005949675320093892j,
                 0.9459616842284228-0.005949675320093892j])

pend = InvertedPendulum()
pend.calc_discrete_system()



print(pend.A)
print(pend.B)


# mb = 0.395
# mw = 0.085
mw = 0.026
mb = 0.208
Jw = 1.0192e-05
Jb = 0.0018407151666666667
Jm = 2.8125e-07
rw = 0.028
n = 64.8
l = 0.0941

d1 = (mw + mb) * rw*rw + Jw + Jm * n * n
d2 = (mw + mb) * rw*rw + Jw + mb * l * rw + Jm * n
d3 = d2
d4 = (mw + mb) * rw*rw + 2 * mb * l * rw + mb*l*l + Jw + Jb + Jm
g = 9.81
c = 7.56E-3
Kw = 1.80E-3
Rm = 2.4
a = n * Kw / Rm

print("Test Parameters")
print("d1 = " + str(d1))
print("d2 = " + str(d2))
print("d3 = " + str(d3))
print("d4 = " + str(d4))

delta = d2 * d3 - d1 * d4
a21 = -d1 * mb * g * l / delta
a24 = -d3 * c / delta
a41 = d2 * mb * g * l / delta
a44 = d4 * c / delta

A = np.array([[0, 1, 0, 0],
              [a21, 0, 0, a24],
             [0, 0, 0, 1],
             [a41, 0, 0, a44]])

b2 = d3*a /delta
b4 = -d4*a/delta
B = np.array([[0], [b2], [0], [b4]])

print("--------------")
print(A)
print("--------------")
print(B)