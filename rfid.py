

import serial
import time

def rfid(ser):
    string = ser.read(14) # dans la variable "string" est stockee la chaine des 14 caracteres lus par liaison serie
    if len(string)==0: # si "string" est vide
        print ("no tag") # affichage de "no tag"
    else: # sinon
        string = string[1:13]# dans la variable "string" ne garder que les 12 caracteres utiles (sans start et stop)
        print("string"), string # affichage de la valeur de "string"
    return string # retour au "Main" avec la variable "string"
