import RPi.GPIO as GPIO
import time



def obstacle_detected(TRIG,ECHO,seuil):
    .................. # creation de l impulsion du signal Trigger au NH (pour que le capteur lance sa salve US)
    .................. # pendant 15us
    .................. # le signal trigger repasse au NB

    .................. # initialisation de la variable "pulse_start"
    .................. # initialisation de la variable "pulse_end"


    .................. # attendre que le signal ECHO passe au NH
        .................. # "pulse_start" prend la valeur de l'instant ou le signal ECHO passe au NH
    .................. # attendre que le signal ECHO passe au NB
        .................. # "pulse_end" prend la valeur de l'instant ou le signal ECHO repasse au NB
    pulse_duration = .................. # calcul de la duree de l'impulsion du signal ECHO


    distance = .................. # calcul de la distance "en cm" entre l'obstacle et le capteur
    .................. # arrondir la distance a 2 decimales
    .................. # affichage de la distance en cm


    .................. # si la valeur de la distance est <= a la valeur du seuil (initialise a 25 cm dans le main)
        .................. # affichage du message stop
        .................. # et retour au "Main" avec la valeur de retour "1" (car obstacle detecte)
   .................. # sinon
        .................. # retour au "Main" avec la valeur de retour "0" (car obstacle non detecte ou trop loin)

