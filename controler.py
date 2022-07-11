#!/usr/bin/env python3

#scp pi@10.42.0.2:/home/pi/catkin_ws/src/orientation/src/send_move_lidar.py /home/oskar/Desktop/send_move_lidar.py
#scp /home/oskar/Desktop/scripte_bachelor/Code/controler.py pi@10.42.0.3:/home/pi/catkin_ws/src/orientation/src/

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
import time
import rospy
import numpy as np
import serial

'''
1. winkel, orientierung & gewollter winkeler7orientierung empfangen und punkte_nr
2. wenn sich punkte nummer zur letzten unterscheidet, dann iterme auf null setzen
3. in regler reingeben
4. neuen move befehl senden
'''
'''
50, -30
0,0
70,70
140,0
210,0
210,70
280,70
360,-10
280,-90
140,-90
120,-70
140,-50
160,-70
140,-90
90,-90
90,-110
70,-110
70,-90
50,-90
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

Schritte = 0
punkt_nr = 0

Trajektorie = [[0, 0],[70,70], [140,0]]#, [210,0], [210,70]] #das hier kann auch aus einer txt file oder so gelesen werden
#Trajektorie = [[100,0], [0,0], [0,100], [0,0]] #das hier kann auch aus einer txt file oder so gelesen werden
#Trajektorie = [[-50,0], [100,100]]
Zielposition = Trajektorie[punkt_nr]
Position = [50,-30]


#Trajektorie = [[70,70], [140,0]]#, [210,0], [210,70]] #das hier kann auch aus einer txt file oder so gelesen werden
#Trajektorie = [[100,0], [0,0], [0,100], [0,0]] #das hier kann auch aus einer txt file oder so gelesen werden
#Trajektorie = [[-100, 0], [100, 0], [0, 100], [100,100]]
#Zielposition = Trajektorie[punkt_nr]
#Position = [0,0]

#diese sind am Anfang null, das ist der I teil der Regler
o_integral_fehler = 0
r_integral_fehler = 0

o_real_alt = 0
r_real_alt = 0

d_alt = 0

time_alt = 0.25
richtung_alt = 0

#zu dem ersten Punkt auch schon eine Fallunterscheidung
modus = 0 #wenn die richtung nahe der 0 ist damit es keine verwirrung zwischen 0/360° gibt
angekommen = False
pub = rospy.Publisher('rot_befehl', Float64MultiArray, queue_size = 1)#queue_size: wenn die nachrichten schnelle reinkommen als sie verarbeotet werden,
rot_befehl = Float64MultiArray()
beginn = True
Zeit_1 = time.time()
Zeit_2 = time.time()
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
        else:#der punkt liegt rechts --> positiver winkel
            richtung = np.arctan((Zielpunkt[1]-Position[1])/(Zielpunkt[0]-Position[0]))*(180/np.pi)

    else:#der Zeilpunkt liegt auf der y achse hinter dem roboter
        if Position[0] > Zielpunkt[0]:#der punkt liegt links --> negativer winkel
            richtung = 180 + (np.arctan(abs(Zielpunkt[1] - Position[1])/abs(Zielpunkt[0] - Position[0])))*(180/np.pi)
        else:#der punkt liegt rechts --> positiver winkel
            richtung = 360 - np.arctan(abs(Zielpunkt[1] - Position[1])/(Zielpunkt[0] - Position[0]))*(180/np.pi)

    return richtung

def pid(sp,pv,pv_last,ierr,dt, KP, KI, KD, regeln = True):
    if regeln:
        #sp = gewünschter wert
        #pv_aktuelle_wert
        #pv_last letzter wert

        # Parameters in terms of PID coefficients
        KP = KP#Kc
        KI = KI#00001#Kc/tauI
        KD = KD#.002#0.001#Kc*tauD
        # ubias for controller (initial heater)
        op0 = 0
        # upper and lower bounds on heater level
        # calculate the error
        error = sp-pv
        # calculate the integral error
        ierr = ierr + KI * error * dt
        # calculate the measurement derivative
        dpv = (pv - pv_last) / dt
        # calculate the PID output
        P = KP * error
        I = ierr
        D = -KD * dpv
        op = op0 + P + I + D

        # implement anti-reset windup

        # return the controller output and PID terms
    else: #fals man den regler zum vergleich mal ausschalten möchte
        op,I = 0,0

    return [op, I]

def regler_Übersetzer(einstellungswinkel, fahrwinkel, regleroutput):

    a_1 = einstellungswinkel - 0.25 * 180
    a_2 = einstellungswinkel + 0.25 * 180
    v_1 = -np.sin(a_1 * np.pi/180)
    v_2 = -np.sin(a_2 * np.pi/180)
    v = max(abs(v_1), abs(v_2))

    b_1 = fahrwinkel - 0.25 * 180
    b_2 = fahrwinkel + 0.25 * 180
    w_1 = -np.sin(b_1 * np.pi/180)
    w_2 = -np.sin(b_2 * np.pi/180)
    w = max(abs(w_1), abs(w_2))

    if regleroutput == 0:
        regleroutput = 0.0001 
    x = (regleroutput/(regleroutput + w)) * ((regleroutput + v)/regleroutput)
    #print(output * x = verhältnismäßiger aoutput beim fahrwinkel)
    return x

def move_befehl(befehl):
    global ser
    array = bytes(f'<{befehl[0]},{befehl[1]},{befehl[2]},{befehl[3]}>\n', 'utf-8')
    ser.write(array)

def callback(data):
    global Zeit_1
    global Zeit_2
    global Schritte
    global punkt_nr
    global o_integral_fehler
    global r_integral_fehler
    global o_real_alt
    global r_real_alt
    global time_alt
    global d_alt
    global modus
    global beginn 
    global angekommen
    global richtung_alt
    Zeit_1 = time.time()

    #1. winkel, orientierung & gewollter winkeler7orientierung empfangen und punkte_nr
    r_real = data.data[0]
    r_desired = data.data[1]
    o_real = data.data[2]
    o_desired = data.data[3]

    #3.regeln und move befehle senden
    if data.data[4] == -1:#dann wurde der letzte Punkt erreicht
        angekommen = True

    time_neu = time.time()
    dt = time_neu - time_alt
    time_alt = time_neu
    if beginn:
        beginn = False
        dt = 0.25
 
    d_o, o_integral_fehler = pid(o_desired, o_real, o_real_alt, o_integral_fehler, dt, -50,-2, -6.5, regeln = True)#-50,-2, -6.5#-80,-5, -5#sp,pv,pv_last,ierr,dt, KP, KI, KD, regeln = True
    d_o = regler_Übersetzer(0, r_real, d_o) * d_o
    o_real_alt = o_real
    o_neu = d_o
    rospy.loginfo(f'{dt}')

    d_alt = o_neu

    if o_neu < 0:
        if o_neu > -0.06:
            if o_neu > -0.03:
                o_neu = 0
            else:
                o_neu = -0.06
    else:
        if o_neu < 0.06:
            if o_neu < 0.03:
                o_neu = 0
            else:
                o_neu = 0.06

    if not angekommen:

        rospy.loginfo(f'neuer move befehl: {[round(r_desired, 4), round(o_neu, 4), 1, 0]}, dt: {dt}')
        richtung_alt = r_desired
        move_befehl([round(r_desired, 4), round(o_neu, 4), 1, 0])
        rot_befehl.data = [r_desired, o_neu]
        pub.publish(rot_befehl)
    else:
        move_befehl([0,0,0,0])
        rospy.loginfo('angekommen')
        rospy.signal_shutdown('we are done here')

    Zeit_2 = time.time()

    
def move():
    rospy.init_node('controle', anonymous = True)


    #hier einen anfangsbefehl, damit das system ins "rollen" kommt und die Maussensoren senden
    r_gewollt = calc_richtung(Position, Trajektorie[punkt_nr])
    rospy.loginfo(f'Startwinkel: {r_gewollt}')
    move_befehl([r_gewollt, 0, 1, 0]) 
    rospy.loginfo(f'{[r_gewollt, 0, 1, 0]}')
    time.sleep(1)

    rospy.Subscriber('real_und_gewuenscht', Float64MultiArray, callback)
    rospy.spin()#lets run node continously

if __name__ == "__main__":	
    try:
        move()
    except rospy.ROSInterruptException:
        pass
