import os
import RPi.GPIO as GPIO

from motor import*
from line_finder import*
from rfid import*
from servo_delivery import*
from servo_head import*
from us_detector import*
from angular_sensor import*
import time


GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)



####################################################################################################################

# calibration du servomoteur de tete (l'appel a cette fonction est a faire la 1ere fois avant de lancer le mode auto)


def calibration_servo_head():
    # setup servo head
    GPIO.setup(18,GPIO.OUT)
    servo_head = GPIO.PWM(18,50)
    servo_head.start(7.5)
    calib_angle_90 = 90
    calib_angle_m90 = -90
    calib_angle_120 = 120
    calib_angle_m120 = -120
    calib_angle_180 = 180
    calib_angle_m180 = -180
    input=raw_input()
    print("calibration ANGLE = 90")
    print("click on + or - to set an accurate angle then press any other key")
    turn_head(servo_head,calib_angle_90)
    input=raw_input()
    while input == "+" or input=="-":
        if input=="+":
            calib_angle_90 = calib_angle_90 + 1
        else:
            calib_angle_90 = calib_angle_90 - 1
        turn_head(servo_head,calib_angle_90)
        input=raw_input()

    print("calibration ANGLE = -90")
    print("click on + or - to set an accurate angle then press any other key")
    turn_head(servo_head,calib_angle_m90)
    input=raw_input()
    while input == "+" or input=="-":
        if input=="+":
            calib_angle_m90 = calib_angle_m90 + 1
        else:
            calib_angle_m90 = calib_angle_m90 - 1
        turn_head(servo_head,calib_angle_m90)
        input=raw_input()

    print("calibration ANGLE = 120")
    print("click on + or - to set an accurate angle then press any other key")
    turn_head(servo_head,calib_angle_120)
    input=raw_input()
    while input == "+" or input=="-":
        if input=="+":
            calib_angle_120 = calib_angle_120 + 1
        else:
            calib_angle_120 = calib_angle_120 - 1
        turn_head(servo_head,calib_angle_120)
        input=raw_input()

    print("calibration ANGLE = -120")
    print("click on + or - to set an accurate angle then press any other key")
    turn_head(servo_head,calib_angle_m120)
    input=raw_input()
    while input == "+" or input=="-":
        if input=="+":
            calib_angle_m120 = calib_angle_m120 + 1
        else:
            calib_angle_m120 = calib_angle_m120 - 1
        turn_head(servo_head,calib_angle_m120)
        input=raw_input()

    print("calibration ANGLE = 180")
    print("click on + or - to set an accurate angle then press any other key")
    turn_head(servo_head,calib_angle_180)
    input=raw_input()
    while input == "+" or input=="-":
        if input=="+":
            calib_angle_180 = calib_angle_180 + 1
        else:
            calib_angle_180 = calib_angle_180 - 1
        turn_head(servo_head,calib_angle_180)
        input=raw_input()

    print("calibration ANGLE = -180")
    print("click on + or - to set an accurate angle then press any other key")
    turn_head(servo_head,calib_angle_m180)
    input=raw_input()
    while input == "+" or input=="-":
        if input=="+":
            calib_angle_m180 = calib_angle_m180 + 1
        else:
            calib_angle_m180 = calib_angle_m180 - 1
        turn_head(servo_head,calib_angle_m180)
        input=raw_input()

    corr90=float(calib_angle_90)/90.0
    corrm90=float(calib_angle_m90)/-90.0
    corr120=float(calib_angle_120)/120.0
    corrm120=float(calib_angle_m120)/-120.0
    corr180=float(calib_angle_180)/180.0
    corrm180=float(calib_angle_m180)/-180.0
    print("90 : ",corr90)
    print("-90 : ",corrm90)
    print("120 : ",corr120)
    print("-120 : ",corrm120)
    print("180 : ",corr180)
    print("-180 : ",corrm180)
    print("moy=",(corr90+corrm90+corr120+corrm120+corr180+corrm180)/6)



####################################################################################################################

# calibration du magnetometre (l'appel a cette fonction est a faire la 1ere fois avant de lancer le mode auto)


def calibration_mag():
    patate = Angular()
    listX=[]
    listY=[]
    raw_input("start : press ENTER")
    rotate(7)
    start_calib=time.time()
    while time.time()-start_calib < 240:
        mag=patate.get_raw_vector()
        print(mag)
        listX.append(mag[0])
        listY.append(mag[1])
        time.sleep(0.1)
    stop_motion()
    maxX=-10000
    minX=10000
    for i in range(len(listX)):
        if listX[i]<minX:
            minX=listX[i]
        if listX[i]>maxX:
            maxX=listX[i]
    maxY=-10000
    minY=10000
    for i in range(len(listY)):
        if listY[i]<minY:
            minY=listY[i]
        if listY[i]>maxY:
            maxY=listY[i]

    print("min X = ",minX)
    print("max X = ",maxX)
    print("min Y = ",minY)
    print("max Y = ",maxY)
    print(" ")

    offsetX=float(maxX+minX)/2.0
    offsetY=float(maxY+minY)/2.0

    print("offset X",offsetX)
    print("offset Y = ",offsetY)

    ampX = maxX-minX
    ampY = maxY-minY

    coeff_corr_Y = float(ampX)/float(ampY)
    print("coeff correction amp Y = ",coeff_corr_Y)
    action = raw_input("save ? y/n\n")
    if action=="y" or action=="Y":
        file = open("config.conf","w")
        file.write(str(offsetX)+"\n")
        file.write(str(offsetY)+"\n")
        file.write(str(coeff_corr_Y)+"\n")
        file.close()
        print("save DONE\n")

    file = open("config.conf","r")
    for line in file.readlines():
        print(line)
    file.close()
    print("end of calibration")



  ####################################################################################################################

# Determination du signe d'un nombre


def signe(number):
     return (float(number)/float(abs(number))) # retour avec la valeur +1 ou -1 si le nombre reel "number" est pos ou neg



####################################################################################################################

#########################       fonction correspondant au mode automatique        ###################################


def start_mode_auto(choix):



####################################################################################################################

# Initialisation des differents capteurs et actionneurs


# setup RFID
    ser = serial.Serial('/dev/ttyS0', 9600, timeout=0.1) # definition de la ligne serie
    time.sleep(1)
    print ("RFID setup ---> READY") # ecriture pour montrer que le RFID est READY
    time.sleep(0.2) # petite attente de 0,2s


# setup detecteur de ligne
    line_finder_right = 8
    line_finder_left = 7
    GPIO.setup(line_finder_right, GPIO.IN)# configuration en E/S du GPIO correspondant au line_finder_right
    GPIO.setup(line_finder_left, GPIO.IN) # configuration en E/S du GPIO correspondant au line_finder_left
    print ("line_finder_right ---> READY") # ecriture pour montrer que le 1er line finder  est READY
    time.sleep(0.2) # petite attente de 0,2s
    print ("line_finder_left ---> READY") # ecriture pour montrer que le 2eme line finder est READY
    time.sleep(0.2)# petite attente de 0,2s


# setup servo head
    GPIO.setup(18, GPIO.OUT) # configuration en E/S du GPIO 18 correspondant au servomoteur de tete
    servo_head = GPIO.PWM(18, 50) # configuration du PWM sur le GPIO 18 avec une periode de 20ms
    servo_head.start(7.5) # initialisation du servo sur l'angle 0 (position milieu sachant que angle de -135 a +135)
    print ("servo head setup ---> READY") # ecriture pour montrer que le servo head est READY
    time.sleep(0.2) # petite attente de 0,2s


# setup servo delivery
    GPIO.setup(25, GPIO.OUT) # configuration en E/S du GPIO 25 correspondant au servomoteur de livraison de materiel
    servo_delivery = GPIO.PWM(25, 50) # configuration du PWM sur le GPIO 25 avec une periode de 20ms
    servo_delivery.start(5) # initialisation du servomo sur l'angle 0 (position trappe fermee sachant que angle de -120 a 0)
    print ("servo delivery ---> READY") # ecriture pour montrer que le servo delivery est READY
    time.sleep(0.2) # petite attente de 0,2s


# setup us_detector
    TRIG = 16
    ECHO = 12
    GPIO.setup(TRIG, GPIO.OUT) # configuration en E/S du GPIO correspondant au signal TRIG
    GPIO.setup(ECHO, GPIO.IN) # configuration en E/S du GPIO correspondant au signal ECHO
    GPIO.output(TRIG, GPIO.LOW) # initialisation du signal TRIG au niveau bas
    print ("us_detector ---> READY") # ecriture pour montrer que le us_detector est READY
    time.sleep(0.2) # petite attente de 0,2s


# setup angular sensor (magnetometre)
    refresh= 1
    rotation_speed = 7
    patate = Angular() # "patate" est un objet de la classe "Angular"
    print("angle origine = ",patate.get_origin_angle()) #appel a "get_origin_angle" puis affichage de l angle d'origine
    start_clock_angular = time.time() # la variable "start_clock_angular" = l'heure ou l'angle d'origine a ete mesure
    max_error=5
    print("angular_detector ---> READY") # ecriture pour montrer que l'angular detector est READY
    time.sleep(0.2) # petite attente de 0,2s



####################################################################################################################

# definition et initialisation des parameters


    vitesse = 45 # la vitesse du robot est fixee par defaut a 45% de sa vitesse maximale
    head_angle= 0
    motor_angle = 0

    alarm = 0 # initialisation variable "alarm" (alarm peut = 0, 1 ou 2 en fonction de la perte de la ligne noire)
# alarm = 0 si pas de perte de ligne noire
# alarm = 1 quand on s'apercoit la 1ere fois qu'il y a la perte de ligne
# alarm = 2 si il y a toujours la perte malgre la tentative de retrouver la ligne

    seuil = 25 # le seuil de detection d obstacle est fixe a 25 cm

    a=0

# chacune des 8 variables "lock" est associee a un tag
# quand une des variables "lock" = 1, on ne peut plus prendre en compte le tag associe deja detecte --> verrouillage
    lock1=0 # initialisation des 8 variables "lock"
    lock2=0
    lock3=0
    lock4=0
    lock5=0
    lock6=0
    lock7=0
    lock8=0

    rfid1="020034E92DF2" # valeurs des 12 caracteres utiles de chacun des codes des 8 tags rencontres sur le parcours
    rfid2="020034EA06DA"
    rfid3="0400A85CD424"
    rfid4="0400A85CF202"
    rfid5="03002557A7D6"
    rfid6="020036D944A9"
    rfid7="0300C8D6E9F4"
    rfid8="0300C8D69F82"
    list_rfid=[rfid1,rfid2,rfid3,rfid4,rfid5,rfid6,rfid7,rfid8] # codes des 8 tags mis dans la liste "list_rfid"

    print ("tag rfid setup ---> READY") # ecriture pour montrer que l'initialisation des parametres est READY



####################################################################################################################

# Attente pour le lancement du programme en mode auto


    raw_input("start : press ENTER") # affichage du message "start : press ENTER"
                                     # afin d'attendre qu'une touche soit tapee (ENTER par ex) pour continuer



####################################################################################################################


    while lock1==0 and lock8==0:


####################################################################################################################

# Test de la presence d'obstacle


        if obstacle_detected(TRIG, ECHO, seuil) == 1 : # si obstacle detecte ( en etant alle a la fonction "obstacle_detected(TRIG,ECHO,seuil)" )
            stop_motion() # arret
            time.sleep(5) # pendant 5s
            alarm = 0 # la variable "alarm" reste a 0



####################################################################################################################

# Suivi de la ligne noire

        else:
            if (line_detected(line_finder_right)==0 and line_detected(line_finder_left)==0):
                if alarm == 0 :
                    debut_alarme = time.time()
                    alarm = 1

                if (time.time()-debut_alarme)<1:
                    motor_angle = head_angle+90
                else:
                    if(time.time()-debut_alarme)<3:
                        motor_angle = head_angle-90
                    else:
                        alarm = 2
            if (line_detected(line_finder_right)==0 and line_detected(line_finder_left)==1):
                alarm = 0
                motor_angle = head_angle+10
            if (line_detected(line_finder_right)==1 and line_detected(line_finder_left)==0):
                alarm = 0
                motor_angle = head_angle-10
            if (line_detected(line_finder_right)==1 and line_detected(line_finder_left)==1):
                alarm = 0
                motor_angle = head_angle

            if alarm!=2:
                if alarm == 1:
                   vitesse = 15
                else:
                   vitesse = 45
                linear_motion("1", vitesse, motor_angle)
            else:
                print("lost")
                stop_motion()
                time.sleep(1)




####################################################################################################################

# Test de la presence de tags


            tag = rfid(ser) # la variable "tag" prend la valeur du tag detecte (en etant alle a la fonction "rfid")
            if len(tag)!=0: # si la chaine de caracteres n'est pas vide
                stop_motion() # arret du robot
                while tag not in list_rfid:
                    time.sleep(0.1) # attente 0,1s
                    tag=rfid(ser) #


# "head_angle" est toujours reference par rapport a l'axe du robot avec la roue "1" comme roue de proue

# le rfid1 permet de faire tourner la tete avec l'angle "head_angle" (verticale montante)
# et de faire avancer le robot suivant sa roue de proue "1" a la "vitesse 45" et avec l'angle "head_angle" pendant 1s
# puis arret du robot car il est revenu a l atelier de depart

                if tag==rfid1: # si la variable "tag" a pris la valeur du tag "rfid1"
                    if lock1==0: # si il n'y a pas de verrouillage (grace a la variable "lock1")
                        lock1=1 # realisation du verrouillage pour ne plus prendre en compte le tag deja detecte
                        alarm=0 # la variable "alarm" reste a 0
                        stop_motion() # arret du robot
                        head_angle = 0 # la variable "head_angle" prend la valeur ......
                        turn_head(servo_head,head_angle) # saut a fonction turn_head(servo_head,head_angle) pour tourner tete
                        time.sleep(2) # attente de 2s
                        linear_motion("1", vitesse, head_angle) # redemarrage avec la fonction "linear_motion"
                        time.sleep(1) # pendant 1s
                        stop_motion() # arret du robot


# le rfid2 permet de faire tourner la tete avec l'angle "head_angle"
# (horizontale vers la droite si la variable de selection de l'atelier "choix" = 1)
# (droite oblique vers la gauche du triangle equilateral si la variable de selection de l'atelier "choix" non = 1)
# et de faire avancer le robot suivant sa roue de proue "1" a la "vitesse 45" et avec l'angle "head_angle" pendant 1s

                if tag==rfid2:
                    if lock2==0:
                        lock2=1
                        alarm=0
                        stop_motion()
                        if choix==1:
                            head_angle = -90
                        else:
                            head_angle = 120
                        turn_head(servo_head,head_angle)
                        time.sleep(2)
                        linear_motion("1", vitesse, head_angle)
                        time.sleep(1)


# le rfid3 permet d'arreter le robot, puis de livrer le materiel ( appel a la fonction "delivery(servo_delivery)" )
# puis de faire avancer le robot suivant sa roue de proue "1" a la "vitesse 45" et avec l'angle "head_angle" precedent
# pendant 1s

                if tag==rfid3:
                    if lock3==0:
                        lock3=1
                        alarm=0
                        stop_motion()
                        time.sleep(2)
                        delivery(servo_delivery)
                        linear_motion("1", vitesse, head_angle)
                        time.sleep(1)


# le rfid4 permet de faire tourner la tete avec l'angle "head_angle" (verticale descendante)
# et de faire avancer le robot suivant sa roue de proue "1" a la "vitesse 45" et avec l'angle "head_angle" pendant 1s

                if tag==rfid4:
                    if lock4==0:
                        lock4=1
                        alarm=0
                        stop_motion()
                        head_angle = 180
                        turn_head(servo_head,head_angle)
                        time.sleep(2)
                        linear_motion("1", vitesse, head_angle)
                        time.sleep(1)


# le rfid5 permet de faire tourner la tete avec l'angle "head_angle" (horizontale vers la gauche)
# et de faire avancer le robot suivant sa roue de proue "1" a la "vitesse 45" et avec l'angle "head_angle" pendant 1s

                if tag==rfid5:
                    if lock5==0:
                        lock5=1
                        alarm=0
                        stop_motion()
                        head_angle = 90
                        turn_head(servo_head,head_angle)
                        time.sleep(2)
                        linear_motion("1", vitesse, head_angle)
                        time.sleep(1)


# le rfid6 permet d'arreter le robot, puis de livrer le materiel ( appel a la fonction "delivery(servo_delivery)" )
# puis de faire avancer le robot suivant sa roue de proue "1" a la "vitesse 45" et avec l'angle "head_angle" precedent
# pendant 1s

                if tag==rfid6:
                    if lock6==0:
                        lock6=1
                        alarm=0
                        stop_motion()
                        time.sleep(2)
                        delivery(servo_delivery)
                        linear_motion("1", vitesse, head_angle)
                        time.sleep(1)


# rfid7 permet de faire tourner la tete avec angle "head_angle" (droite oblique vers la droite du triangle equilateral)
# et de faire avancer le robot suivant sa roue de proue "1" a la "vitesse 45" et avec l'angle "head_angle" pendant 1s

                if tag==rfid7:
                    if lock7==0:
                        lock7=1
                        alarm=0
                        stop_motion()
                        head_angle = -120
                        turn_head(servo_head,head_angle)
                        time.sleep(2)
                        linear_motion("1", vitesse, head_angle)
                        time.sleep(1)


# le rfid8 permet de faire tourner la tete avec l'angle "head_angle" (verticale montante)
# et de faire avancer le robot suivant sa roue de proue "1" a la "vitesse 45" et avec l'angle "head_angle" pendant 1s
# puis arret du robot car il est revenu a l atelier de depart

                if tag==rfid8:
                    if lock8==0:
                        lock8=1
                        alarm=0
                        stop_motion()
                        head_angle = 0
                        turn_head(servo_head,head_angle)
                        time.sleep(2)
                        linear_motion("1", vitesse, head_angle)
                        time.sleep(1)


####################################################################################################################

# Test de la position du robot grace au magnetometre et reajustement de la position du robot


            if time.time() - start_clock_angular > refresh :
                stop_motion()
                print("test")
                #time.sleep(1)
                error = patate.get_error()
                if abs(error) >= max_error:
                    rotate(int(signe(error)*rotation_speed))
                    #rotate(6)
                    while abs(error) >= max_error :
                        #print(error)
                        time.sleep(0.5)
                        error = patate.get_error()
                    stop_motion()
                    #time.sleep(1)
                start_clock_angular = time.time()



#####################################################################################################################

# Definition des fonctions correspondant aux differents mouvements realisables par le robot


def rotate(velocity):
        motor_1.set_motor_velocity(-velocity)
        motor_2.set_motor_velocity(-velocity)
        motor_3.set_motor_velocity(-velocity)


def go_straight(motor_ID, velocity):
    # le moteur dont l'ID est donnee en argument sera le moteur de proue du mouvement

        if motor_ID == motor_1.motor_ID:
            motor_1.set_motor_velocity(0)
            motor_2.set_motor_velocity(velocity)
            motor_3.set_motor_velocity(-velocity)

        if motor_ID == motor_2.motor_ID:
            motor_1.set_motor_velocity(-velocity)
            motor_2.set_motor_velocity(0)
            motor_3.set_motor_velocity(velocity)

        if motor_ID == motor_3.motor_ID:
            motor_1.set_motor_velocity(velocity)
            motor_2.set_motor_velocity(-velocity)
            motor_3.set_motor_velocity(0)



def linear_motion(motor_ID,velocity,angle):
    # le moteur dont l'ID est donnee en argument sera le moteur de proue du mouvement

        if motor_ID == motor_1.motor_ID:
            motor_1.set_motor_velocity(-velocity*sin(radians(angle)))
            motor_2.set_motor_velocity(velocity*sin(radians(angle+60)))
            motor_3.set_motor_velocity(-velocity*sin(radians(60-angle)))

        if motor_ID == motor_2.motor_ID:
            motor_1.set_motor_velocity(-velocity*sin(radians(60-angle)))
            motor_2.set_motor_velocity(-velocity*sin(radians(angle)))
            motor_3.set_motor_velocity(velocity*sin(radians(angle+60)))

        if motor_ID == motor_3.motor_ID:
            motor_1.set_motor_velocity(velocity*sin(radians(angle+60)))
            motor_2.set_motor_velocity(-velocity*sin(radians(60-angle)))
            motor_3.set_motor_velocity(-velocity*sin(radians(angle)))



def stop_motion():
        motor_1.set_motor_velocity(0)
        motor_2.set_motor_velocity(0)
        motor_3.set_motor_velocity(0)



####################################################################################################################


# Creation des 3 threads en donnant l'ID du moteur les numeros des pattes CW et CCW
motor_1 = Motor("1",13,17)
motor_2 = Motor("2",21,22)
motor_3 = Motor("3",23,24)



# Lancement des 3 threads : la fonction self.start() fait appel a la fonction self.run()
motor_1.start()
motor_2.start()
motor_3.start()



######################################################################################################################

# saisie des commandes clavier


try:

    action = "" # initialisation de la variable "action" (chaine de caracteres vide)

    while action != "exit" : # tant que la commande clavier est differente de "exit"
        action = raw_input() # la chaine de caracteres tapee au clavier est stockee dans variable "action"


# Les commandes clavier suivantes correspondent a des tests de debbuggage
# pendant que le robot fonctionne on verifie certains parametres associes a chaque moteur

        if action == "on1":
            motor_1.debug=1
        if action == "off1":
            motor_1.debug=0
        if action == "on2":
            motor_2.debug=1
        if action == "off2":
            motor_2.debug=0
        if action == "on3":
            motor_3.debug=1
        if action == "off3":
            motor_3.debug=0



# Les commandes clavier suivantes correspondent aux differents mouvements realisables par le robot
# lors de la phase de tests


# test de la fonction "stop_motion"
        if action == "s":
            stop_motion()


# test des differentes fonctions "go_straight"
        if action == "go1":
            vitesse = int(input())
            go_straight("1",vitesse)
        if action == "go2":
            vitesse = int(input())
            go_straight("2",vitesse)
        if action == "go3":
            vitesse = int(input())
            go_straight("3",vitesse)



# test des differentes fonctions "linear_motion"
        if action == "l1":
            vitesse = int(input())
            angle = int(input())
            linear_motion("1", vitesse, angle)
        if action == "l2":
            vitesse = int(input())
            angle = int(input())
            linear_motion("2", vitesse, angle)
        if action == "l3":
            vitesse = int(input())
            angle = int(input())
            linear_motion("3", vitesse, angle)


# test de la fonction "rotate"
        if action == "rot":
            vitesse = int(input())
            rotate(vitesse)



# Les commandes clavier suivantes correspondent au mode automatique de fonctionnement reel


# lancement du mode automatique de fonctionnement reel
# saisie du numero de l'atelier ou le materiel doit etre livre (1 ou 2) et mise dans la variable "choix"
# appel a la fonction ""start_mode_auto" pour lancer le mode automatique
        if action == "auto":
            choix = int(input())
            start_mode_auto(choix)


# lancement de la calibration du magnetometre (a faire la 1ere fois avant de lancer le mode auto)
        if action=="calib_mag":
            calibration_mag()


# lancement de la calibration du servomoteur de tete (a faire la 1ere fois avant de lancer le mode auto)
        if action =="calib_servo":
            calibration_servo_head()




finally:
    motor_1.exit = True
    motor_2.exit = True
    motor_3.exit = True


    motor_1.join()
    motor_2.join()
    motor_3.join()

#GPIO.cleanup()
