

import serial
import time

def rfid(ser):
    GPIO.SETUP(15, GPIO.IN)
        string = GPIO.input(15)# dans la variable "string" est stockee la chaine des 14 caracteres lus par liaison serie
        if(string == 0): # si "string" est vide
            print('no tag') # affichage de "no tag"
        else # sinon
            string = string[1:12] # dans la variable "string" ne garder que les 12 caracteres utiles (sans start et stop)
            print 'Carte scanne = ', string # affichage de la valeur de "string"
