import os
import re
import sys
import numpy as np
import scipy.signal
from pydub import AudioSegment
import matplotlib.pyplot as plt

plt.ion()
plt.show()

def run():
	S = {}
	
	s = AudioSegment.from_file(sys.argv[1])
	dty = { 1: np.int8, 2: np.int16, 4: np.int32 }[s.sample_width]
	S = np.frombuffer(s.raw_data, dtype=dty).astype(np.float32)
	F = s.frame_rate
	STRIDE = 0.2
	Fmaxs = []
	ts = np.arange(5.5, len(s) / 1000 - STRIDE, STRIDE)
	for r in ts:
		seg = (int((r - STRIDE) * F), int((r + STRIDE) * F))
		Fs, P = scipy.signal.welch(S[seg[0]:seg[1]], fs=F, nperseg=(seg[1] - seg[0]) // 2)
		plt.clf()
		plt.loglog(Fs, P)
		plt.ylim((1E-7, 1E4))
		plt.title('%.1f' % r)
		input()
		nmax = np.where(Fs > 200)[0][0]
		nmin = np.where(Fs > 30)[0][0]
		Fmaxs.append(Fs[np.argmax(P[:nmax])])
	
	plt.plot(ts, Fmaxs)
	input()

if __name__ == '__main__':
	run()