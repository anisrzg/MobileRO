import RPi.GPIO as GPIO


def line_detected(line_finder):

# Return HIGH when black line is detected, and LOW when white area is detected
    etat = GPIO.input(line_finder) # la valeur du sortie du capteur teste est stockee dans la variable "etat"
    return (etat)  # retour au "Main" avec la variable "etat"

