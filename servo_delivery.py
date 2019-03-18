import RPi.GPIO as GPIO
import time


# pour ce servomoteur, la periode est egale a 20 ms
# le th est egual a 2 ms  pour angle du servo = 0     -->   duty cycle = 0.05
# le th est egual a 1 ms  pour angle du servo = - 120 -->   duty cycle = 0.1



def delivery(servo):

    servo.ChangeDutyCycle(10)# lancer le servo avec le duty cycle max = 0.1 ms correspondant a l'angle= 0 pour ouverture trappe
    time.sleep(7) # laisser la trappe ouverte pendant 7s
    servo.ChangeDutyCycle(5)# lancer le servo avec le duty cycle min = 0.05 ms correspondant a angle = 0 pour fermeture trappe
    time.sleep(2) # attente de 2s
    
# lancer le servo avec le duty cycle correspondant a la valeur de "angle"