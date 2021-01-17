import sys
import csv
import datetime
import time
import matplotlib.pyplot as plt
import pdb
import numpy as np

plt.ion()
plt.show()
def run():
	with open(sys.argv[1], 'r') as f, open(sys.argv[1] + '.csv', 'r') as g, open(sys.argv[1] + '.volts.csv', 'w') as h:
		D = [(datetime.datetime.strptime(m[0], '%H:%M:%S.%f'), m[1]) for l in f for m in [l.split(' -> ')] if len(m) == 2]
		D = np.array([((d[0] - D[0][0]).total_seconds() * 29.97, int(d[1].strip())) for d in D])
		D_idxs = np.argsort(D[:,0])
		D_sorted = D[D_idxs,:]
		E = np.array([[float(r_) for r_ in r] for r in csv.reader(g)])
		
		DE = (np.interp(E[:,0], D_sorted[:,0], D_sorted[:,1]), E[:,1])
		
		filt = np.hanning(13)
		DE_smooth = np.array([np.convolve(filt, DE[0], mode='valid'), np.convolve(filt, DE[1], mode='valid')]) / np.sum(filt)
		DE_idxs = np.argsort(DE_smooth[0,:])
		DE_smooth_sorted = DE_smooth[:,DE_idxs]
		Esub = np.arange(np.amin(DE_smooth[0]) // 8 * 8, np.amax(DE_smooth[0]), 8)
		Dsub = np.abs((np.interp(Esub, DE_smooth_sorted[0,:], DE_smooth_sorted[1,:]) * (1 << 10))).astype(np.uint16)
		h.write('\n'.join(['%d,%d' % r for r in zip(Esub, Dsub)]))
		# pdb.set_trace()
		# input()

run()