import picamera as PiCamera
import time

camera = PiCamera()

camera.start_preview()
time.sleep(5)
camera.stop_preview()
