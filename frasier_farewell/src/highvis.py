import cv2
import numpy as np
import glob

yellows = [[ 32, 192, 192], [ 32, 128, 128], [ 32, 160, 224], [ 32, 160, 192]]
yellows = [np.array(i) for i in yellows]

def training_images():
	most_common_colors = []
	for i in glob.glob('/home/naoki-joanne/test_photos/highvis/*.png'):
		img = cv2.imread(i)
		denoised = cv2.blur(img,(5,5))
		denoised = cv2.cvtColor(denoised, cv2.COLOR_BGR2HLS)
		hist = cv2.calcHist([denoised], range(3), None, [8, 8, 8], [0, 255, 0, 255, 0, 255])
		max_ind = np.unravel_index(np.argmax(hist, axis=None), hist.shape)
		most_common_color = np.array([ind * 32 for ind in max_ind])
		print most_common_color

def extract_yellow(img):
	denoised = cv2.blur(img,(10,10))
	denoised = cv2.cvtColor(denoised, cv2.COLOR_BGR2HLS)
	max_mask_sum, max_mask = 0, None
	for yellow in yellows:
		mask = cv2.inRange(denoised, yellow*6/10, yellow*14/10)
		if np.sum(mask) >= max_mask_sum:
			max_mask_sum = np.sum(mask)
			max_mask2 = max_mask if max_mask is not None else mask
			max_mask = mask

	mask = max_mask2 + max_mask
	output = cv2.bitwise_and(img, img, mask=mask)

	return output

if __name__ == '__main__':
	cap = cv2.VideoCapture(1)
	while True:
		ret, frame = cap.read()
		dst = extract_yellow(frame)
		dst = np.hstack((frame, dst))
		cv2.imshow('', dst)
		cv2.waitKey(1)