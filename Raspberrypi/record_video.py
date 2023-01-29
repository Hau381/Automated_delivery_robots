import picamera

camera = picamera.PiCamera()
camera.resolution = (1280,854 )
camera.start_recording('my_video.h264')
camera.wait_recording(500)
camera.stop_recording()