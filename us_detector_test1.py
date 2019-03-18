import RPi.GPIO as GPIO
import time


def obstacle_detected(TRIG,ECHO,seuil):
    GPIO.output (TRIG, GPIO.HIGH) # creation de l impulsion du signal Trigger au NH (pour que le capteur lance sa salve US)
    time.sleep(15.0/1000000.0) # pendant 15us
    GPIO.output(TRIG, GPIO.LOW) # le signal trigger repasse au NB

    global obstacle = 0
    pulse_start = 0 # initialisation de la variable "pulse_start"
    pulse_end = 0 # initialisation de la variable "pulse_end"

    while GPIO.input(ECHO) == 1:  # attendre que le signal ECHO passe au NH  
        pulse_start = time.time() # "pulse_start" prend la valeur de l'instant ou le signal ECHO passe au NH
        while  GPIO.input(ECHO) == 1: # attendre que le signal ECHO passe au NB
            
    pulse_end = time.time() # "pulse_end" prend la valeur de l'instant ou le signal ECHO repasse au NB
    pulse_duration = pulse_end - pulse_start  # calcul de la duree de l'impulsion du signal ECHO


    distance = (pulse_duration * 34000)/2  # calcul de la distance "en cm" entre l'obstacle et le capteur
    distance = round(distance, 2) # arrondir la distance a 2 decimales
    print ("la distance est de", distance, "centimetre") # affichage de la distance en cm


    if distance <= seuil: # si la valeur de la distance est <= a la valeur du seuil (initialise a 25 cm dans le main)
        print ("STOP")# affichage du message stop
        obstacle = 1
        return obstacle  # et retour au "Main" avec la valeur de retour "1" (car obstacle detecte)
    else : # sinon
        obstacle = 0 # retour au "Main" avec la valeur de retour "0" (car obstacle non detecte ou trop loin)
        return obstacle
    
