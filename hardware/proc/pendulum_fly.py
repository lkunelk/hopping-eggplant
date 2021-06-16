from scipy.integrate import solve_ivp
import numpy as np
import math
import matplotlib.pyplot as plt
import sys
import pdb

plt.ion()
plt.show()
plt.close('all')

def run():
	Las = np.linspace(0.08, 0.16, 8) # m
	Iflys = np.linspace(0.0008, 0.002, 8) # kg-m
	# Las = np.array([0.1])
	# Iflys = np.array([0.00152])
	Mfly = 0.05 # kg
	Mmotor = 0.070
	nturns = [21.5] # [10.5, 13.5, 17.5, 21.5, 25.5]
	eta = 0.48
	T0s = np.array([0.272]) # np.array([0.344, 0.326, 0.297, 0.272, 0.246]) # N-m # np.array([110/(4620*7.4/60*2*math.pi/2)*2*eta]) # 
	Vmaxs = np.array([222]) * math.pi * 2 # np.array([444, 351.5, 267.63, 222, 187.47]) * math.pi * 2 # rad/s
	tfs = np.empty((2, T0s.shape[0], Las.shape[0], Iflys.shape[0]))
	# tfs[:] = np.nan
	tfs[:] = 0
	
	zerocross = lambda _, y: y[0] - math.pi / 2
	zerocross.terminal = True
	
	for i, La in enumerate(Las):
		for j, Ifly in enumerate(Iflys):
			for k, (Vmax, T0) in enumerate(zip(Vmaxs, T0s)):
				Ifly_ctr = Ifly + Mfly * La ** 2
				Itot = (Ifly_ctr + Mmotor * La) * 2 # I _think_ it's x2 because in the inertial frame the Earth is the other part of that arm
				Tg0 = (Mfly + Mmotor) * 9.8 * La
				def f(_, y):
					Tm = (1 - (y[1] + y[2]) / Vmax) * T0
					return [y[1], (Tm - Tg0 * math.cos(y[0])) / Itot, Tm / Ifly]
				sol = solve_ivp(f, [0, 5], [0, 0, 0], events=zerocross)
				# pdb.set_trace()
				if len(sol.t_events[0]) > 0:
					tfs[:,k,i,j] = [sol.t_events[0][0], (sol.y[1] + sol.y[2])[-1]] # sol.t_events[0][0]
	
	for tf_ in tfs:
		M = np.max(tf_)
		for nturn, tf in zip(nturns, tf_):
			plt.figure()
			plt.imshow(tf, vmax=M, extent=(Iflys[0], Iflys[-1], Las[-1], Las[0]), aspect='auto')
			plt.colorbar()
			plt.xlabel('Flywheel inertia (kg-m)')
			plt.ylabel('Length of arm (m)')
			plt.title('Zero-crossing time for %.1fT Quicrun motor \nand %dg flywheel' % (nturn, Mfly * 1E3))
			plt.show()
			plt.savefig('%.1f.png' % nturn)
	
	pdb.set_trace()
run()