import control
import scipy.integrate
import numpy as np
import matplotlib.pyplot as plt
import math
from sortedcontainers import SortedDict

plt.ion()
plt.close('all')
plt.show()

def run():
	Mmotor = 0.080
	Mfly = 0.0357
	rdisk = 0.0617318752905765
	La = 0.10
	Ifly = 4.85E-5 # Mfly * rdisk ** 2 / 2
	Ifly_ctr = Ifly + Mfly * La ** 2
	Itot = (Ifly_ctr + Mmotor * La) * 2 # I _think_ it's x2 because in the inertial frame the Earth is the other part of that arm
	g = 9.8
	b = 0.01

	A = np.array([[ 0, 1, 0, 0 ], [ 0, 0, 1, 0 ], [ 0, Itot * g, -b, 0 ], [ 0, 0, -Itot / Ifly, 0 ]])
	B = np.array([ 0, 0, 1, 0 ]).reshape((-1, 1))
	Q = np.diag([ 0.5, 10, 1 ])
	R = np.array([ 100 ])
	K = control.lqr(A[:3,:3], B[:3], Q, R)[0]
	K = np.array(list(np.array(K)[0]) + [0]).reshape((1,-1)) # pad by 1
	print(K)
	T0 = 0.6
	VDD = 11.8
	KV = 1000
	Vmax = VDD * KV / 60 * math.pi * 2

	T = np.arange(0, 20, 0.1)
	N = np.random.normal(scale=0.1, size=T.shape)
	SS = control.StateSpace(A - np.dot(B, K), np.array([ 0, 0, 1, 0 ]).reshape((-1, 1)), np.eye(A.shape[0]), np.zeros((A.shape[1], 1)))
	Tout, _, xout = control.forced_response(SS, T, N, X0=(0, 0.1, 1.0, 0))
	
	# D = 0.4
	# buf = SortedDict([])
	# def f(t, y):
	# 	while(len(buf) > 1 and buf.peekitem(1)[0] < t - D):
	# 		buf.popitem(0)
	# 	buf[t] = y
	# 	print(t, len(buf))
	# 	u = np.random.normal(scale=0.4)
	# 	u_ = 0
	# 	if t > D:
	# 		bufT = buf.keys()
	# 		y_ = np.array(buf.peekitem(1)[:-1]) # delay by D, hope the linear interp isn't too bad
	# 		u_ = np.dot(K, y_.reshape((-1,1)))
	# 		u -= u_
			
	# 	return list((np.dot(A, y[:4].reshape((-1,1))) + np.dot(B, u)).flatten()) + [u_]
		
	# sol = scipy.integrate.solve_ivp(f, (0, 5), (0, 0.1, 0.1, 0, 0), rtol=2E-3) # , t_eval=T)
	# Tout, xout = (sol.t, sol.y)
	
	plt.figure()
	[plt.plot(Tout, x) for x in xout[:3]]
	plt.xlabel('Time (s)')
	plt.ylabel('Error (rad-s) / Angle (rad) / Speed (rad/s)')
	plt.title('Simulation with Gaussian torque noise (σ = 0.1) with LQR controller\n with Q = diag(0.01, 10, 1), R = 100, X0 = (0, 0.1, 0.1)')
	plt.plot(Tout[1:], (xout[4,1:] - xout[4,:-1]) / (Tout[1:] - Tout[:-1]), 'tab:red')
	plt.plot(Tout, -(1 - abs(xout[3]) / Vmax) * T0, 'tab:purple')
	plt.plot(Tout, (1 - abs(xout[3]) / Vmax) * T0, 'tab:purple')
	plt.legend(['Integral error (rad-s)', 'Arm angle (rad)', 'Arm speed (rad/s)', 'Control torque (N-m)', 'Maximum feasible torque (N-m)'])

	plt.figure()
	plt.plot(Tout, xout[3], 'tab:blue')
	input()

run()