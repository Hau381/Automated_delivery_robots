from bluedot import BlueDot
from bluedot import BlueDotPosition
import time
bd1 = BlueDotPosition(0,0,0,0)
bd = BlueDot()
bd.wait_for_press()
print ("You pressed the blue dot !")
while True :
    a=bd1.__str__()
    print(a)
    bd.wait_for_press()
