from bluedot import BlueDot
from signal import pause
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD) 
GPIO.setwarnings(False)
GPIO.setup(7, GPIO.OUT)
GPIO.setup(11, GPIO.OUT)
GPIO.setup(13, GPIO.OUT)
GPIO.setup(15, GPIO.OUT)

def dpad(pos):
    #print(pos)
    if pos.top:
        print("Forward")
    elif pos.bottom:
        print("Reverse")

    elif pos.left:
        print("Left")
        
    elif pos.right:
        print("Right")
        
    elif pos.middle:
        print("Code Killed")

bd = BlueDot()
bd.when_pressed = dpad
#bd.when_moved = dpad

   