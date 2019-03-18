import RPi.GPIO as GPIO
import time



def obstacle_detected(TRIG,ECHO,seuil):
    GPIO.output(TRIG,GPIO.HIGH) # creation de l impulsion du signal Trigger au NH (pour que le capteur lance sa salve US)
    time.sleep(0.000015) # pendant 15us
    GPIO.output(TRIG,GPIO.LOW) # le signal trigger repasse au NB

    pulse_start = 0 # initialisation de la variable "pulse_start"
    pulse_end = 0 # initialisation de la variable "pulse_end"


    while GPIO.input(ECHO)==0 :# attendre que le signal ECHO passe au NH
        pulse_start = time.time() # "pulse_start" prend la valeur de l'instant ou le signal ECHO passe au NH
    while GPIO.input(ECHO)==1 :# attendre que le signal ECHO passe au NB
        pulse_end = time.time() # "pulse_end" prend la valeur de l'instant ou le signal ECHO repasse au NB
    pulse_duration = pulse_end - pulse_start  # calcul de la duree de l'impulsion du signal ECHO


    distance = 17000 * pulse_duration   #calcul de la distance "en cm" entre l'obstacle et le capteur
    distance=round(distance,2) # arrondir la distance a 2 decimales
    print("Distance:",distance,"cm") # affichage de la distance en cm


    if distance <= seuil:  # si la valeur de la distance est <= a la valeur du seuil (initialise a 25 cm dans le main)
        print("STOP") # affichage du message stop
        return(1) # et retour au "Main" avec la valeur de retour "1" (car obstacle detecte)
    else: # sinon
        return(0) # retour au "Main" avec la valeur de retour "0" (car obstacle non detecte ou trop loin)
