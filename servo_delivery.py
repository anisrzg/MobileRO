import RPi.GPIO as GPIO
import time


# pour ce servomoteur, la periode est egale a 20ms
# le th est egual a 2ms  pour angle du servo = 0     -->   duty cycle = 5
# le th est egual a 1ms  pour angle du servo = - 120 -->   duty cycle = 10


def delivery(servo):

    servo.ChangeDutyCycle(10) # lancer le servo avec le duty cycle max = 1/20 (5%) correspondant a l'angle=-120 pour ouverture trappe
    time.sleep(7) # laisser la trappe ouverte pendant 7s
    servo.ChangeDutyCycle(5) # lancer le servo avec le duty cycle min = 2/20 (10%) correspondant a angle = 0 pour fermeture trappe
    time.sleep(2) # attente de 2s