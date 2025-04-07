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


continuous_poles = pend.d2c_poles(discrete_poles)
discrete_poles_2 = pend.c2d_poles(continuous_poles)


print("discrete_poles")
print(discrete_poles)
print("")
print("continuous_poles")
print(continuous_poles)
print("")
print("discrete_poles2")
print(discrete_poles_2)




