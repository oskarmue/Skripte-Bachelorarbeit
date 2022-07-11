#!/usr/bin/env python3

#scp pi@10.42.0.2:/home/pi/catkin_ws/src/orientation/src/send_move_lidar.py /home/oskar/Desktop/send_move_lidar.py
#scp /home/oskar/Desktop/scripte_bachelor/Code/planer_option_eins.py pi@10.42.0.3:/home/pi/catkin_ws/src/orientation/src/

from std_msgs.msg import Float64MultiArray
import time
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from collections import deque
import serial
import time
'''
Aufgaben die der Subscriber übernimmt

bei jedem neuen Punktepaar wird zu beginn:
ein winkel berechnet, dieser wird dann angesteuert

1. Position, Orientierung empfangen
2. checken ob die Zielposition erreich wurde
3. wenn ja
    3.1 neue Zielposition einstellen
    3.3 dann neuen winkel zur Zielposition berechnen 
    3.4 an regeler schicken
4. wenn nein
    4.1 gefahrenen Winkel berechnen
    4.2 neuen zu fahrenden Winkel mit dem PI Controller berechnen
    4.3 an regler schicken
'''

punkt_nr = 1
Trajektorie = [[50, -30], [0, 0], [70,70], [140,0], [210,0], [210,70], [280,70], [360,-10], [280,-90], [140,-90], [120,-70], 
[140,-50], [160,-70], [140,-90], [90,-90], [90,-110], [70,-110], [70,-90], [50,-90], [50, -110], [30, -110], 
[30, -90], [0, -90], [0, -77.5], [75, -57.5], [-30, -25], [0, -12.5], [0,0]]#das hier kann auch aus einer txt file oder so gelesen werden

#Trajektorie = [[0,0], [-100, 0]]
#Trajektorie = [[0,0], [0, 100]]
'''
punkt_nr = 1
Trajektorie = [[50, -30], [0, 0], [70,70], [140,0], [210,0], [210,70], [280,70], [360,-10], [280,-90], [140,-90], [120,-70], 
[140,-50], [160,-70], [140,-90], [90,-90], [90,-110], [70,-110], [70,-90], [50,-90], [50, -110], [30, -110], 
[30, -90], [0, -90], [0, -77.5], [75, -57.5], [-30, -25], [0, -12.5], [0,0]]#das hier kann auch aus einer txt file oder so gelesen werden
'''
for i in range(1, len(Trajektorie)):
    Trajektorie[i][0] = Trajektorie[i][0] * 2
    Trajektorie[i][1] = Trajektorie[i][1] * 2
for i in range(1, len(Trajektorie)):
    Startpunkt = Trajektorie[i-1]
    Zielpunkt = Trajektorie[i]

    if Startpunkt[1] == Zielpunkt[1]:#f(x) = m * x + b
        steigung = 0 #wird nicht verwendet
    elif Startpunkt[0] == Zielpunkt[0]:
        steigung = 0 #unendlich - aber wird nicht verwendet, deswegen egal
    else:
        gerade = np.polyfit([Startpunkt[0], Zielpunkt[0]], [Startpunkt[1], Zielpunkt[1]], 1)
        steigung = gerade

    Trajektorie[i].append(steigung)
#Trajektorie = [[100,0], [0,0], [100,0], [0,0]] #das hier kann auch aus einer txt file oder so gelesen werden
#Trajektorie = [[0,0], [-50,0], [-100,-100]]
Zielposition = Trajektorie[punkt_nr]
Position = Trajektorie[0]#[50,-30]

#punkt_nr = 1
#Trajektorie = [[0,0], [50, 50], [100, 0]]
'''
for i in range(1, len(Trajektorie)):
    Startpunkt = Trajektorie[i-1]
    Zielpunkt = Trajektorie[i]

    if Startpunkt[1] == Zielpunkt[1]:#f(x) = m * x + b
        steigung = 0 #wird nicht verwendet
    elif Startpunkt[0] == Zielpunkt[0]:
        steigung = 0 #unendlich - aber wird nicht verwendet, deswegen egal
    else:
        gerade = np.polyfit([Startpunkt[0], Zielpunkt[0]], [Startpunkt[1], Zielpunkt[1]], 1)
        steigung = gerade

    Trajektorie[i].append(steigung)
'''
#Trajektorie = [[0,0], [100,0], [0,0], [100,0], [0,0]] #das hier kann auch aus einer txt file oder so gelesen werden
#Trajektorie = [[0,0], [-50,0], [-100,-100]]
#Zielposition = Trajektorie[punkt_nr]
Position = [0,0]

Position_alt = Position
Orientierung = 0
Istbahn = []
Schritte = 0
vergangene_pos = deque(maxlen = 25)

Zielwinkel = 0
neuer_punkt = True

pub = rospy.Publisher('real_und_gewuenscht', Float64MultiArray, queue_size = 0)
pub_bahn = rospy.Publisher('bahn', Float64MultiArray, queue_size = 1)

werte_für_regler = Float64MultiArray() #[r_real, r_gewuenscht, o_real, o_gewünscht, punkt_nr]
bahn = Float64MultiArray()

Zeit_1 = 0
Zeit_2 = 0

offset_x = 0
offset_y = 0

index = 0
'''
ser = serial.Serial(
        '/dev/ttyS0', 
        baudrate = 115200,
        parity = serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize =serial.EIGHTBITS,
        timeout = 1
        )

ser.close()
ser.open()
'''

def calc_winkel(p1, p2):
    if p1[0] == p2[0]:
        if p2[1] > p1[1]:
            return 0
        else:
            return 180

    elif p1[1] == p2[1]:
        if p2[0] > p1[0]:
            return 90
        else:
            return -90
    else:
        if p2[0] > p1[0]:#p2 liegt rechts von p1
            if p2[1] > p1[1]:#p2 liegt über p1
                return np.arctan((p2[0] - p1[0])/(p2[1] - p1[1])) * (180/np.pi)
            else:
                return 90 + np.arctan(abs(p2[1] - p1[1])/(p2[0] - p1[0]))* (180/np.pi)
        else:#p2 liegt links von p1
            if p2[1] > p1[1]:#p2 liegt über p1
                return -np.arctan(abs(p2[0] - p1[0])/(p2[1] - p1[1]))* (180/np.pi)
            else:
                return -90 + np.arctan(abs(p2[1] - p1[1])/(p2[0] - p1[0]))* (180/np.pi)

def calc_delta(winkel, distanz):
    if winkel > 0:
        if winkel >= 90:
            winkel = winkel - 90
            winkel = winkel *(np.pi/180)
            x = np.cos(winkel) * distanz
            y = -np.sin(winkel) * distanz 
        else:
            winkel = winkel *(np.pi/180)
            x = np.sin(winkel) * distanz
            y = np.cos(winkel) * distanz        
    else:
        if winkel <= -90:
            winkel = abs(winkel) - 90
            winkel = winkel *(np.pi/180)
            x = -np.cos(winkel) * distanz
            y = -np.sin(winkel) * distanz 
        else:
            winkel = winkel * (np.pi/180)
            x = np.sin(winkel) * distanz
            y = np.cos(winkel) * distanz
    return x, y

def calc_richtung(Position, Zielpunkt):
    if Zielpunkt[1] == Position[1]:
        if Zielpunkt[0] > Position[0]:
            richtung = 0
        else:
            richtung = 180

    if Zielpunkt[0] == Position[0]:
        if Zielpunkt[1] > Position[1]:
            richtung = 90
        else:
            richtung = 270
    
    elif Zielpunkt[1] > Position[1]:#der Zeilpunkt liegt auf der y achse vor dem roboter
        if Position[0] > Zielpunkt[0]:#der punkt liegt links --> negativer winkel
            richtung = 90 + np.arctan(abs(Zielpunkt[0]-Position[0])/abs(Zielpunkt[1]-Position[1]))*(180/np.pi)
            #print('hallo')
        else:#der punkt liegt rechts --> positiver winkel
            richtung = np.arctan((Zielpunkt[1]-Position[1])/(Zielpunkt[0]-Position[0]))*(180/np.pi)

    else:#der Zeilpunkt liegt auf der y achse hinter dem roboter
        if Position[0] > Zielpunkt[0]:#der punkt liegt links --> negativer winkel
            richtung = 180 + (np.arctan(abs(Zielpunkt[1] - Position[1])/abs(Zielpunkt[0] - Position[0])))*(180/np.pi)
        else:#der punkt liegt rechts --> positiver winkel
            richtung = 360 - np.arctan(abs(Zielpunkt[1] - Position[1])/(Zielpunkt[0] - Position[0]))*(180/np.pi)

    return richtung

def calc_distanz(Position, Zielpunkt):

    return (abs(Position[0] - Zielpunkt[0])**2 + abs(Position[1] - Zielpunkt[1])**2)**(1/2)

def position_berichtigen(Bahn, index):
    position = [0, 0, 0, 0, 0]#x_real, y_real, x, y, orientierung, Zielpunkt
    orientierungen = []
    Bahn_berichtigt = []
    for i in range(index, len(Bahn)-4, 4):
        Bahn_berichtigt.append(position)
        p1 = [Bahn[i], Bahn[i+1]]
        p2 = [Bahn[i+4], Bahn[i+5]]
        orientierung = Bahn[i+2]
        orientierungen.append(orientierung)
        distanz = calc_distanz(p1, p2)
        winkel = calc_winkel(p1, p2)
        absoluter_winkel = winkel + orientierung
        d_x, d_y = calc_delta(absoluter_winkel, distanz)
        position = [position[0] + d_x, position[1] + d_y, Bahn[i], Bahn[i+1], orientierung, Bahn[i+3]]

    index = len(Bahn) - 8
    return Bahn_berichtigt, orientierungen

def move_befehl(befehl):
    global ser
    array = bytes(f'<{befehl[0]},{befehl[1]},{befehl[2]},{befehl[3]}>\n', 'utf-8')
    ser.write(array)

def callback(data):
    global punkt_nr
    global Trajektorie
    global Zielposition
    global Istbahn
    global Orientierung
    global Position
    global Schritte
    global vergangene_pos
    global Zeit_1
    global Zeit_2
    global offset_x
    global offset_y
    global index

    '''
    Aufgaben die der Subscriber übernimmt

    1. Position, Orientierung empfangen
    2. checken ob die Zielposition erreich wurde
    3. wenn ja
        3.1 neue Zielposition einstellen
        3.3 dann neuen winkel zur Zielposition berechnen 
        3.4 neue werte an regler schicken
    4. wenn nein
        4.1 gefahrenen Winkel berechnen
        4.2 neuen zu fahrenden Winkel berechnen
        4.3 werte an regler schicken
    '''

    #1. Position, Orientierung empfangen
    Position = [offset_x + Trajektorie[0][0] + data.pose.position.x, offset_y + Trajektorie[0][1] + data.pose.position.y] #nochmal neu 
    vergangene_pos.append(Position)
    Orientierung = data.pose.orientation.z

    #rospy.loginfo(f'Position: {Position}, Orientierung: {Orientierung}')
    Istbahn.append(Position[0]) 
    Istbahn.append(Position[1])
    Istbahn.append(Orientierung)
    Istbahn.append(float(punkt_nr))

    #2. checken ob die Zielposition erreich wurde
    distanz = (abs(Position[0] - Zielposition[0])**2 + abs(Position[1] - Zielposition[1])**2)**(1/2)

    if distanz <= 1:
        bahn.data = Istbahn
        pub_bahn.publish(bahn)

    #3. Zielposition erreicht
    if distanz <= 0.2:#mm
        punkt_nr += 1
        if punkt_nr >= len(Trajektorie):#angekommen --> stoppen   

            #3.1.1 Werte an Regler schicken
            werte_für_regler.data = [0,0,0,0,-1]      
            pub.publish(werte_für_regler)
            rospy.loginfo(f'angekommen')
            bahn.data = Istbahn
            pub_bahn.publish(bahn)
            rospy.signal_shutdown('we are done here')

        else:#angekommen aber mehr Punkte vorhanden --> nächster Punkt

            #3.2.1 neue Zielposition einstellen
            Zielposition = Trajektorie[punkt_nr]

            #3.2.2 dann neuen winkel zur Zielposition berechnen 
            r_gewollt = calc_richtung(Position, Zielposition)  

            r_real = calc_richtung(vergangene_pos[0], vergangene_pos[-1])
            #
            vergangene_pos = deque(maxlen = 25)
            vergangene_pos.append(Position)

            #3.2.4 Werte an Regler schicken
            werte_für_regler.data = [r_real, r_gewollt, Orientierung, 0, punkt_nr]
            pub.publish(werte_für_regler)
            bahn.data = Istbahn
            pub_bahn.publish(bahn)
            rospy.loginfo(f'es wird ein neuer Punkt angefahren, gewünschter Winkel: {r_gewollt}')
            rospy.loginfo(f'{Schritte}')
            #Schritte = 0
            Zeit_1 = time.time()
            dt = Zeit_1 - Zeit_2
            Zeit_2 = Zeit_1
            #rospy.loginfo(f'{dt}')
            #if Schritte % 25 < 5:
                #Schritte += 5
            #Schritte = 0
            #####
            neue_bahn, orientierungen = position_berichtigen(Istbahn, index)
            #offset_x = Position[0] - neue_bahn[-1][0] - Trajektorie[0][0]
            #offset_y = Position[1] - neue_bahn[-1][1] - Trajektorie[0][1]
            rospy.loginfo(f'das ist der offset:  {offset_x}, {offset_y}')
            #####
    #4. Zielposition nicht erreicht
    #elif (len(vergangene_pos) >= 10) and (Schritte % 25 == 0): #60
    elif (len(vergangene_pos) >= 10) and (Schritte % 20 == 0):
        #4.1 gefahrenen Winkel berechnen
        r_real = calc_richtung(vergangene_pos[0], vergangene_pos[-1])

        #Punkt auf der Geraden finden
        entfernungswert = 5#10
        Zielpunkt = Trajektorie[punkt_nr]
        Startpunkt = Trajektorie[punkt_nr-1]

        if distanz <= 10:
            r_gewollt = calc_richtung(Position, Zielpunkt)
        else:
            if Startpunkt[0] == Zielpunkt[0]:#f(x) = m * x + b
                x = Startpunkt[0]
                if Zielpunkt[1] > Startpunkt[1]:      
                    y = Position[1] + (abs(entfernungswert**2 - (Zielpunkt[0] - Position[0])**2))**(1/2) #derzeitige y koordinate
                else:
                    y = Position[1] - (abs(entfernungswert**2 - (Zielpunkt[0] - Position[0])**2))**(1/2) #derzeitige y koordinate

            elif Startpunkt[1] == Zielpunkt[1]:
                y = Startpunkt[1]
                if Zielpunkt[0] > Startpunkt[0]:
                    x = Position[0] + (abs(entfernungswert**2 - (Zielpunkt[1] - Position[1])**2))**(1/2) #derzeitige y koordinate
                else:
                    x = Position[0] - (abs(entfernungswert**2 - (Zielpunkt[1] - Position[1])**2))**(1/2) #derzeitige y koordinate
            else:
                gerade = Trajektorie[punkt_nr][2]

                #Schnittpunkt und Abstand berechnen
                normale = -1/gerade[0]
                d = Position[1] - normale * Position[0]
                x_s = (d - gerade[1])/(gerade[0] - normale)
                y_s = gerade[0] * x_s + gerade[1]
                abstand = calc_distanz(Position, [x_s, y_s]) 

                #Hypothenuse berechnen
                k = (abs(entfernungswert**2 - abstand**2))**(1/2)

                #Winkel aus Steigung der Zielgeraden berechnen
                beta = abs(np.arctan(Trajektorie[punkt_nr][2][0]))

                #neuen Punkt berechnen
                if Zielpunkt[1] > Startpunkt[1]:
                    if Zielpunkt[0] > Startpunkt[0]:
                        x = x_s + np.cos(beta) * k
                        y = y_s + np.sin(beta) * k
                    else:
                        x = x_s - np.cos(beta) * k
                        y = y_s + np.sin(beta) * k
                else: 
                    if Zielpunkt[0] > Startpunkt[0]:
                        x = x_s + np.sin(beta) * k
                        y = y_s - np.cos(beta) * k
                    else:
                        x = x_s - np.sin(beta) * k
                        y = y_s - np.cos(beta) * k

            #print('Zielpunkt: ', x, y)
            r_gewollt = calc_richtung(Position, [x, y])          

            #r_gewollt = calc_richtung(Position, Zielposition)

        #4.2 Werte an Regler schicken
        werte_für_regler.data = [r_real, r_gewollt, Orientierung, 0, punkt_nr]
        Zeit_1 = time.time()
        dt = Zeit_1 - Zeit_2
        Zeit_2 = Zeit_1
        #rospy.loginfo(f'{dt}')
        if dt < 0.09:
            time.sleep(0.1)
            rospy.loginfo(f'wurde hoch gesetzt {dt + 0.1}')

        pub.publish(werte_für_regler)
        rospy.loginfo(f'distanz: {round(distanz, 2)}, real/gewünscht: {round(r_real, 2)}/{round(r_gewollt, 2)}')
    
        
    Schritte += 1

def move():
    rospy.init_node('controle', anonymous = True)
    rospy.Subscriber('pose', PoseStamped, callback)
    rospy.spin()#lets run node continously

if __name__ == "__main__":	
    try:
        move()
    except rospy.ROSInterruptException:
        pass
