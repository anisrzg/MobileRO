import RPi.GPIO as GPIO
import time


# engrenage: cote servo: Zroueservo = 40, et cote axe tete robot: Zrouerobot = 20 --> rapport de transmission = k = 2
# --> la relation de "angle robot" en fonction de "angle servo" est : angle servo*2

# pour ce servomoteur, la periode est egale a 20ms
# le th est egual a 0.5ms  pour angle servo = -135 cad angle robot = -270 --> duty cycle = 0.5/20
# le th est egual a 1.5ms  pour angle servo = 0     cad angle robot = 0 --> duty cycle = 1.5/20
# le th est egual a 2.5ms  pour angle servo = 135 cad angle robot = 270 --> duty cycle = 2.5/20

# --> la relation du duty cycle en fonction de l'"angle robot" (appele "angle" dans le programme") est:
# [(angle robot/135 +1.5)/20]*50



def turn_head(servo_head, head_angle):
    if -190 < head_angle <= 190: # test pour savoir si "angle" est compris entre - 190 (exclus) et 190 (inclus)
        dc=(((head_angle/135)+1.5)/20)*50
        servo.ChangeDutyCycle(dc) # lancer le servo avec le duty cycle correspondant a la valeur de "angle"
                           # cad l'"angle robot" que celui-ci fait
