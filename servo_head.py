import RPi.GPIO as GPIO
import time


# engrenage: cote servo: Zroueservo = 40, et cote axe tete robot: Zrouerobot = 20 --> rapport de transmission = k = 40/20 = 2
# --> la relation de "angle robot" en fonction de "angle servo" est : "angle robot" = "angle servo"*2

# pour ce servomoteur, la periode est egale a 20ms
# le th est egual a 2,5 ms  pour angle servo = - 135 cad angle robot = -270 --> duty cycle = 2,5/20
# le th est egual a 1,5 ms  pour angle servo = 0     cad angle robot = 0 --> duty cycle = 1,5/20
# le th est egual a 0,5 ms  pour angle servo = + 135 cad angle robot = 270 --> duty cycle = 0,5/20

# --> la relation du duty cycle en fonction de l'"angle robot" (appele "angle" dans le programme") est:
# dc = (((angle/270)+1,5)/20)*100 =((angle/270)+1,5)*5



def turn_head(servo,angle):
    if angle <= 190.0 and angle > -190.0: # test pour savoir si "angle" est compris entre - 190 (exclus) et 190 (inclus) lancer le servo avec le duty cycle correspondant a la valeur de "angle"
        servo.ChangeDutyCycle((((-float(angle)/270)*1.0+1.5)/20)*100)# cad l'"angle robot" que celui-ci fait 