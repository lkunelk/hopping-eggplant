import sys
import numpy as np
import cv2
import math
import pdb

NUM_AVG_FRAMES = 10

if __name__ == '__main__':
	def run():
		V = cv2.VideoCapture(sys.argv[1])
		
		ref = np.uint8([[[126, 209, 112]]])
		ref_h = cv2.cvtColor(ref, cv2.COLOR_BGR2HSV)[0,0,0]
		frame = 0
		yx = np.mgrid[0:V.get(cv2.CAP_PROP_FRAME_HEIGHT),0:V.get(cv2.CAP_PROP_FRAME_WIDTH)]
		yx = yx.reshape((2, int(yx.size / 2)))
		y, x = yx
		ctr = (101, 134)
		while True:
			frame += 1
			valid, im = V.read()
			if not valid:
				break
			# if frame % 20 != 0:
			# 	continue
			
			hsvi = cv2.GaussianBlur(cv2.cvtColor(im, cv2.COLOR_BGR2HSV), (5, 5), 0)
			# pdb.set_trace()
			# hsvf = hsvf, (21,21), 0)
			hsvf = (((np.cos((hsvi[:,:,0].astype(np.float64) - ref_h) / 180.0 * 2 * math.pi) + 1) / 2.0) ** 16 * hsvi[:,:,1]) ** 4
			hsvf_center = np.sum(hsvf.flatten() * (x, y), axis=1) / np.sum(hsvf)
			hsvf_rng = [np.amin(hsvf), np.amax(hsvf)]
			# A = np.vstack((x.flatten(), np.ones(y.size))).T * hsvf.reshape((hsvf.size, 1))
			# print(np.linalg.lstsq(A, y)[0])
			# print(hsvf_center)
			print('%d,%.4f' % (frame, math.atan2(hsvf_center[1] - ctr[1], hsvf_center[0] - ctr[0])))
			I = (((hsvf - hsvf_rng[0]) / (hsvf_rng[1] - hsvf_rng[0])) * 255).astype(np.uint8)
			cv2.circle(I, tuple(hsvf_center.astype(np.uint8)), 5, 255, -1)
			cv2.circle(I, ctr, 3, 255, -1)
			cv2.imshow('1', I)
			cv2.waitKey(20)
	run()