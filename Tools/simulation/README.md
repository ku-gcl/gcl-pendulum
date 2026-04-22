## Install

```bash
cd ~/gcl-pendulum/Tools
pip install -r requirements.txt
```

## Execution

```bash
cd ~/gcl-pendulum/Tools/simulation

# lqr
python3 calc_lqr_gain.py

# pole placement
python3 calc_pole_gain.py
```

## Expected Output (gcl-pendulum ver1)

正しく動作する gcl-pendulum ver1 の場合の `calc_lqr_gain.py` の出力例です。

<details>
<summary>calc_lqr_gain.py の出力</summary>

```
python3 calc_lqr_gain.py
Unable to init server: Could not connect: Connection refused
Unable to init server: Could not connect: Connection refused

(calc_lqr_gain.py:3029): Gdk-CRITICAL **: 15:41:00.558: gdk_cursor_new_for_display: assertion 'GDK_IS_DISPLAY (display)' failed
******************************
*        Parameters          *
******************************
Wheel Parameters
m_wheel = 0.026 (kg)
r_wheel = 0.028 (m)
I_wheel = 1.0192e-05 (kg.m^2)

------------------------------
Pendulum Parameters
m_pendulum = 0.20800000000000002 (kg)
The length between the center of gravity and the axis r_pendulum (m)
r_pendulum = 0.09407227087423742 (m)
I_pendulum = 0.0018407151666666667 (kg.m^2)

------------------------------
Rotator Parameters
m_rotator = 0.01 (kg)
r_rotator = 0.0075 (m)
Im = 2.8125e-07 (kg.m^2)

------------------------------
******************************
*        Ac, Bc, Cc          *
******************************
System Matrix
A = 
[[  0.           1.           0.           0.        ]
 [ 41.95235235   0.           0.           0.89200718]
 [  0.           0.           0.           1.        ]
 [-22.63064197   0.           0.          -5.97959415]]
B = 
[[ 0.        ]
 [-5.7356429 ]
 [ 0.        ]
 [38.44903644]]
C = 
[[1 0 0 0]
 [0 1 0 0]
 [0 0 1 0]
 [0 0 0 1]]
******************************
*    Pole of the system      *
******************************
Eigenvalue of A = 
[ 0.          6.34941089 -7.45737714 -4.8716279 ]
******************************
*        Ad, Bd, Cd, Ts          *
******************************
System Matrix
Ad = 
[[ 1.00209503e+00  1.00069852e-02  0.00000000e+00  4.37399129e-05]
 [ 4.18826707e-01  1.00209503e+00  0.00000000e+00  8.66475578e-03]
 [-1.10970217e-03 -3.71683993e-06  1.00000000e+00  9.70688331e-03]
 [-2.19828931e-01 -1.10970217e-03  0.00000000e+00  9.41953462e-01]]
Bd = 
[[-2.81249440e-04]
 [-5.57147362e-02]
 [ 1.88475240e-03]
 [ 3.73241628e-01]]
Cd = 
[[1 0 0 0]
 [0 1 0 0]
 [0 0 1 0]
 [0 0 0 1]]
Ts = 
0.01
**********************************
*        Calculation Result      *
**********************************
LQR Weight
Q = 10, 10, 1, 1
R = 1.0

----------------------------------
Gain of the continuous system
[[116.11973638  18.86085646   1.           1.5214221 ]]

----------------------------------
Eigenvalue of the stable system (continuous system)
[-43.162824680944816, 0.0], [-0.9885023979020285, 0.0], [-5.755095323268113, 1.2814379849016346], [-5.755095323268113, -1.2814379849016346]

----------------------------------
```

</details>

## 修正履歴

### `InvertedPendulum.calc_continuous_system` の係数修正

最新の git 版 (HEAD) のロジックでは実機で倒立できなかったため、以前のバージョン (gcl-pendulum ver1) と等価なロジックに戻しつつ、変数名は最新版のスタイル (`d1〜d4`, `a11, a12, a41, a44`, `b2, b4`) に揃えました。

<details>
<summary>最新 git 版との差分</summary>

2×2 のマス行列を反転して状態方程式の係数を導出する箇所で、最新 git 版には以下の不整合がありました。

**1. マス行列中間量 `d2, d3, d4` への `Im` 項追加 (不要)**

最新版では `d2 (= d3)` に `+ Im * gear_ratio`、`d4` に `+ Im` を追加していました。もとの導出では車輪側の成分 `d1` のみに `gear_ratio² * Im` が入るのが正しいので、これらの追加項を削除しました。

| 項目 | 最新 git 版 | 修正後 (ver1 と等価) |
|---|---|---|
| `d1` (車輪側) | `m_whole·r_wheel² + I_wheel + gear_ratio²·Im` | 同左 |
| `d2 = d3` (結合項) | `... + I_wheel + Im·gear_ratio` | `... + I_wheel` |
| `d4` (振子側) | `... + I_pendulum + I_wheel + Im` | `... + I_pendulum + I_wheel` |

**2. `b4` の符号反転**

マス行列の逆行列を 2×2 の公式 `M⁻¹ = (1/det)·[[d, -b],[-c, a]]` で展開した場合、入力ベクトル成分の符号のうち `b4` が反転する必要があります。

| 項目 | 最新 git 版 | 修正後 (ver1 と等価) |
|---|---|---|
| `b2` | `+ d3·gear·kt/Rm / det` | 同左 |
| `b4` | `+ d4·gear·kt/Rm / det` | `- d4·gear·kt/Rm / det` |

`a11, a12, a41, a44` は `d3 = d2` のもとで最新版と修正後で同じ式になるため、実質的な差分は上記 2 点のみです。

</details>
