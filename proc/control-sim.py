import numpy as np
import control
Mmotor = 0.186
Mfly = 0.0357
rdisk = 0.0617318752905765
La = 0.10
Ifly = Mfly * rdisk ** 2 / 2
Ifly_ctr = Ifly + Mfly * La ** 2
Itot = (Ifly_ctr + Mmotor * La) * 2 # I _think_ it's x2 because in the inertial frame the Earth is the other part of that arm
g = 9.8
b = 0.01
A = np.array([[ 0, 1, 0, 0 ], [ 0, 0, 1, 0 ], [ 0, Itot * g, -b, 0 ], [ 0, 0, -Itot / Ifly, 0 ]])[:3, :3]
B = np.array([ 0, 0, 1, 0 ]).reshape((-1, 1))[:3]
Q = np.diag([ 10, 1, 0.1, 10 ])[:3, :3]
R = np.array([ 100 ])
K = control.lqr(A, B, Q, R)[0]
print(K)

# soln: K = matrix([[3.16227766, 4.9763414 , 3.16061237]]): this is a controller that applies this in units of N-m torque, which I just need to translate to angular acceleration through the flywheel inertia I think
# array([145.27566309, 682.80899015, 787.61792219]) in units of angular acceleration of the flywheel ((rad/s)/s) at 32Hz ticks (this seems very aggressive)
# array([ 41.68598654, 195.92797422, 226.00227323]) units of timing (ms) from the ESC assuming proportionality, with 400ms -> 1394rad/s
# array([10, 48, 56], dtype=uint8) rads expressed in 4096-units, these represented in 256-units, rounded
# final calc: (np.array([0.31622777, 1.48629963, 1.71444173]) / Ifly / 32 * 400 / 1394 / 1024 * 256).astype(np.uint8)