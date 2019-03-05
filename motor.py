__author__ = 'Sylvain'

import time
from threading import Thread
from math import*
import RPi.GPIO as GPIO




class Motor(Thread):

    PWM_FREQUENCY = 100  # in Hz
    PWM_PERIOD = 1.0/PWM_FREQUENCY # in s


    def __init__(self, my_motor_ID, my_pin_clockwise, my_pin_counterclockwise):
        Thread.__init__(self)
        self.motor_ID = my_motor_ID
        self.pin_clockwise = my_pin_clockwise
        self.pin_counterclockwise = my_pin_counterclockwise
        self.direction = 0
        self.velocity = 0
        self.debug = 0
        self.exit = False


    def set_motor_velocity(self, my_velocity):
        if my_velocity >= -100 and my_velocity <=100:
            self.velocity = my_velocity
            self.set_motor_direction()


    def set_motor_direction(self):
        if self.velocity < 0:
            self.direction = -1
        else:
            if self.velocity > 0:
                self.direction = 1
            else:
                self.direction = 0


# Ecriture sur la meme ligne de 3 parametres lies au moteur
    def state_print(self):
        print(self.motor_ID, self.direction, self.velocity)


    def run(self):
        """Code a executer pendant l'execution du thread."""


# Definition des 2 lignes du GPIO CW et CCW en E_S
        .........................
        .........................


        while self.exit != True:



# Calcul des durees du niveau haut et du niveau bas du signal PWM
# en fonction de la vitesse choisie et de la periode

            high_state_time = .........................
            low_state_time = .........................



# le but est de faire tourner le moteur dans un sens
# la patte CW = 0 et la patte CCW realise le PWM
# debut et fin sont des variables dans lesquelles sont stockees
# les instants de debut et de fin du niveau haut du signal PWM

            if self.direction == 1:
                GPIO.output(self.pin_clockwise, GPIO.LOW)
                debut = time.time()
                .........................
                .........................
                .........................
                .........................
                .........................



# le but est de faire tourner le moteur dans l'autre sens
# la patte CW ......................... et la patte CCW .........................
# debut et fin sont des variables dans lesquelles sont stockees
# les instants de debut et de fin du niveau haut du signal PWM

            if self.direction == -1:
                .........................
                .........................
                .........................
                .........................
                .........................
                .........................
                .........................



# le but est de faire arreter le moteur
# les variables debut et fin sont mises a 0

            if self.direction == 0:
                .........................
                .........................
                .........................
                .........................
                .........................



# appel a la fonction self.state_print (pour pouvoir ecrire sur la meme ligne les 3 parametres lies au moteur).........................
# et ecriture sur  2 lignes differentes des valeurs des variables high_state_time et low_state_time
# et ecriture sur de la duree s'ecoulant entre les instants debut et fin

            if self.debug == 1:
                .........................
                .........................
                .........................
                .........................



