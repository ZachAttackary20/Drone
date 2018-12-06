import time
from picamera import PiCamera

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 30
camera.iso = 400
camera.shutter_speed = 3000
ext = 'h264'
fileName = './captures/BallThrow' + time.strftime('%d-%m-%Y_%H-%M-%S') + "." + ext
camera.start_recording(fileName, ext)
camera.wait_recording(10)
camera.stop_recording()
camera.close()
