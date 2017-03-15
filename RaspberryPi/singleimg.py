import cv2

junk_frames = 30
camera = cv2.VideoCapture(0)

def get_image():
    retval, img = camera.read()
    return img

for i in xrange(junk_frames):
    temp = get_image()

capture = get_image()

cv2.imshow("img",capture)
cv2.waitKey()


del (camera)