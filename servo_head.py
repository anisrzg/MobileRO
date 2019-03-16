import RPi.GPIO as GPIO
import time


# engrenage: cote servo: Zroueservo = 40, et cote axe tete robot: Zrouerobot = 20 --> rapport de transmission = k = 2
# --> la relation de "angle robot" en fonction de "angle servo" est : angle servo*2

# pour ce servomoteur, la periode est egale a 20ms
# le th est egual a 2.5ms  pour angle servo = -135 cad angle robot = -270 --> duty cycle = 12.5
# le th est egual a 1.5ms  pour angle servo = 0     cad angle robot = 0 --> duty cycle = 7.5
# le th est egual a 0.5ms  pour angle servo = 135 cad angle robot = 270 --> duty cycle = 2.5

# --> la relation du duty cycle en fonction de l'"angle robot" (appele "angle" dans le programme") est:
# [(-angle robot/270 +1.5)/20]*100 = [(-angle robot / 270) +1.5]*5



def turn_head(servo, angle):
    if -190 < angle and angle <= 190: # test pour savoir si "angle" est compris entre - 190 (exclus) et 190 (inclus)
        servo.ChangeDutyCycle(((-float(angle)*1.0/270.0)+1.5)*5.0) # lancer le servo avec le duty cycle correspondant a la valeur de "angle"
                                                                 # cad l'"angle robot" que celui-ci fait
                                                                 # 1.0 correspond au coeff. de correction calculer par la calibration du servo_head
