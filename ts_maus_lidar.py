#!/usr/bin/env python3

#scp /home/oskar/Desktop/scripte_bachelor/Code/ts_maus_lidar.py pi@10.42.0.3:/home/pi/catkin_ws/src/orientation/src/

'''
nachrichten nicht synchronisieren, sondern
1. listener: lidar --> updatet nur den winkel
2. listener: maus --> (höhere frequenz) --> triggert berechnung
'''
'''
hier wird noch der Kalman Filter mit reingesetzt

'''
from collections import namedtuple
import message_filters
from std_msgs.msg import Float64
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import numpy as np
import random

import rospy

pub = rospy.Publisher('pose', PoseStamped, queue_size = 10)#queue_size: wenn die nachrichten schnelle reinkommen als sie verarbeotet werden,
my_pose_for_publishinp = PoseStamped()
pub_2 = rospy.Publisher('varianz_kalman', Float64, queue_size = 10)#queue_size: wenn die nachrichten schnelle reinkommen als sie verarbeotet werden,

winkel = 0
maus_1 = [0,0]
maus_2 = [0,0]

x = 0
y = 0 
x_alt = 0
y_alt = 0
pos = [0, 0]
#################################Kalman Initlisierungsstep
#Kalman Filter Stuff
gaussian = namedtuple('Gaussian', ['mean', 'var'])
gaussian.__repr__ = lambda s: f'Gaussian: mean={s.mean}, Varianz={s[1]}'
richtungs_befehl = 0
rot_befehl = 0
belief = gaussian(0,0)
winkel_lidar_1 = 0
winkel_lidar_2 = 0

winkel_lidar_1_alt = 0

abstand_1 = 0
abstand_2 = 0

abstand_anfang_1 = 0
abstand_anfang_2 = 0
##########################################################

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

def calc_distanz(p1, p2):

    return ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)**(1/2)

def calc_orientierung(Position, Zielpunkt):
    entf = 146
    Zielpunkt[1] = Zielpunkt[1] + entf
    if Zielpunkt[0] == Position[0]:
            richtung = 0

    elif Zielpunkt[1] > Position[1]:#der Zeilpunkt liegt auf der y achse vor dem roboter
        if Position[0] > Zielpunkt[0]:#der punkt liegt links --> negativer winkel
            richtung = -np.arctan(abs(Zielpunkt[0]-Position[0])/abs(Zielpunkt[1]-Position[1]))*(180/np.pi)
        else:#der punkt liegt rechts --> positiver winkel
            richtung = 90 - np.arctan((Zielpunkt[1]-Position[1])/(Zielpunkt[0]-Position[0]))*(180/np.pi)

    return richtung

def interpolieren(G1, G2, r):

    return G1[1] + (G2[1] - G1[1])/(G2[0] - G1[1]) * (r - G1[0])

def Einfluss(regleroutput, richtungs_befehl):
    v_1 = -np.sin(richtungs_befehl * np.pi/180 - 0.25 * np.pi) + regleroutput/100
    v_2 = -np.sin(richtungs_befehl * np.pi/180 + 0.25 * np.pi) + regleroutput/100
    v_3 = -np.sin(richtungs_befehl * np.pi/180 - 0.25 * np.pi) - regleroutput/100
    v_4 = -np.sin(richtungs_befehl * np.pi/180 + 0.25 * np.pi) - regleroutput/100
    
    maximal = np.max([abs(v_1), abs(v_2), abs(v_3), abs(v_4)])

    e = regleroutput/(100 * maximal) #das müsste jetzt eig noch "übersetzt" werden

    #rospy.loginfo(f'das ist der berechnete, unübersetzte Einfluss: {e}')

    return e #0.2

def adding_gaussians(g1, g2):
    g12 = gaussian(g1.mean + g2.mean, g1.var + g2.var)
    return g12

def prediction(belief, richtungs_befehl, rot_befehl):

    mean_0, var_0 = 0.000320940920442, 0.00119798106678
    mean_45, var_45 = 0.000257312417382,  0.001116923798578
    mean_90, var_90 = -0.000247504668,  0.000903403575585
    mean_135, var_135 = -0.000472933355764, 0.000998138753948
    mean_180, var_180 = -0.000410267624573, 0.001005022715405
    mean_225, var_225 = 0.000389967626126,  0.00096406615878
    mean_270, var_270 = -0.000204892887457, 0.001092452385994
    mean_315, var_315 = 0.000455636422407,  0.000910054553732

    if richtungs_befehl < 45:
        m = interpolieren([0, mean_0], [45, mean_45], richtungs_befehl) 
        v = interpolieren([0, var_0], [45, var_45], richtungs_befehl)
    elif richtungs_befehl < 90:
        m = interpolieren([45, mean_45], [90, mean_90], richtungs_befehl) 
        v = interpolieren([45, var_45], [90, var_90], richtungs_befehl)
    elif richtungs_befehl < 135:
        m = interpolieren([90, mean_90], [135, mean_135], richtungs_befehl) 
        v = interpolieren([90, var_90], [135, var_135], richtungs_befehl)
    elif richtungs_befehl < 180:
        m = interpolieren([135, mean_135], [180, mean_180], richtungs_befehl) 
        v = interpolieren([135, var_135], [180, var_180], richtungs_befehl)
    elif richtungs_befehl < 225:
        m = interpolieren([180, mean_180], [225, mean_225], richtungs_befehl) 
        v = interpolieren([180, var_180], [225, var_225], richtungs_befehl)
    elif richtungs_befehl < 270:
        m = interpolieren([225, mean_270], [270, mean_270], richtungs_befehl) 
        v = interpolieren([225, var_270], [270, var_270], richtungs_befehl)
    elif richtungs_befehl < 315:
        m = interpolieren([270, mean_270], [315, mean_315], richtungs_befehl) 
        v = interpolieren([270, var_270], [315, var_315], richtungs_befehl)
    else:
        m = interpolieren([315, mean_315], [360, mean_0], richtungs_befehl) #bei 270 ist gewollt null
        v = interpolieren([315, mean_315], [360, var_0], richtungs_befehl)
    
    process_noise_g = gaussian(m, v)
    v_max = 4.6 #mm/s
    r_rod_max = v_max/160
    Einfluss_ =  Einfluss(rot_befehl, richtungs_befehl)
    if rot_befehl > 0:
        winkelveränderung = Einfluss_ * r_rod_max 
    elif rot_befehl < 0:
        winkelveränderung = Einfluss_ * r_rod_max * -1
    else:
        winkelveränderung = 0

    controle_noise = gaussian(belief.mean + winkelveränderung, 0.005)  

    prediction = adding_gaussians(belief, process_noise_g)
    prediction = adding_gaussians(prediction, controle_noise)   
    #prediction = adding_gaussians(belief, process_noise_g)   
  
    return prediction

def multiply_gaussians(g1, g2):
    mean = (g1.var * g2.mean + g2.var * g1.mean)/(g1.var + g2.var)
    variance = (g1.var * g2.var) / (g1.var + g2.var)
    return gaussian(mean, variance)

def update(prior, likelihood):#hier wird die prediction mit den messungen verrechnet
    posterior = multiply_gaussians(likelihood, prior)
    return posterior

class sensor:
    def __init__(self, varianz):
        self.v = varianz

    def set_variance(self, abstand, abstand_anfang):
        if abstand <= 0.4:
            if (abstand_anfang > 0.4) and (abstand_anfang < 0.8):
                self.v = 0.0005859
            else:
                self.v = 0.0016
        if abstand <= 0.8:
            if (abstand_anfang > 0.4) and (abstand_anfang < 0.8):
                self.v = 0.00333
            else:
                self.v = 0.0057
        else:
            if (abstand_anfang > 0.4) and (abstand_anfang < 0.8):
                self.v = 0.006
            else:
                self.v = 0.0104

    def measure(self, Messwert):
        return gaussian(Messwert, self.v)

mäuse = sensor(1)
lidar_1 = sensor(1)
lidar_2 = sensor(1)

def callback_kallman(data):
    global richtungs_befehl
    global rot_befehl#

    richtungs_befehl = data.data[0]
    rot_befehl = data.data[1]

def callback_winkel(data):
    global maus_1
    global maus_2
    global winkel_lidar_1
    global winkel_lidar_2
    global abstand_1
    global abstand_2
    global winkel
    global belief

    winkel_lidar_1 = data.data[0]
    winkel_lidar_2 = data.data[1]

    abstand_1 = data.data[2]
    abstand_2 = data.data[3]

    abstand_anfang_1 = data.data[4]
    abstand_anfang_2 = data.data[5]

    #############################################hier findet die Kalman Action Statt
    #das findet immer mit neuer lidar messung statt
    
    belief = prediction(belief, richtungs_befehl, rot_befehl)
    
    lidar_1.set_variance(abstand_1, abstand_anfang_1)
    lidar_2.set_variance(abstand_2, abstand_anfang_2)

    Messung_1 = lidar_1.measure(winkel_lidar_1* 180/np.pi)
    Messung_2 = lidar_2.measure(winkel_lidar_2* 180/np.pi)

    #belief = update(Messung_1, Messung_2)
    #winkel = belief.mean
    
    belief = update(Messung_1, belief)
    belief = update(Messung_2, belief)

    winkel = belief.mean
    pub_2.publish(belief.var)
    #winkel_varianz = belief.var

    #winkel_lidar_1_alt = winkel_lidar_1
    
    ############################################

def callback_maus_2(data):
    global maus_1
    global maus_2
    global winkel 
    maus_2 = [-data.point.x, -data.point.y]

def calc_winkel_maus(p1, p2):
    '''
    dabei gehe ich davon aus, dass sich der Roboter nicht zu weit von der verlangten Orientierung entfernt
    abstand beide maussensoren: 146mm
    '''
    #print(p1, p2)

    entf = 146
    p2[1] = p2[1] + entf

    '''
    if p2[0] > p1[0]:
        delta_x = abs(p2[0] - p1[0])
        return np.arcsin(delta_x/entf) * 180/np.pi

    elif p2[0] > p1[0]:
        delta_x = abs(p2[0] - p1[0])
        return -np.arcsin(delta_x/entf) * 180/np.pi

    else:
        return 0
    '''

def callback(data):
    global maus_1
    global maus_2
    global winkel
    global x
    global y
    global x_alt
    global y_alt
    global pos

    '''
    x = data.point.x 
    y = data.point.y 
    d_x = x - x_alt
    d_y = y - y_alt

    winkel_b = calc_winkel([0,0], [d_x, d_y])
    abstand_b = calc_distanz([0,0], [d_x, d_y])
    absoluter_winkel = winkel + winkel_b
    d_x_neu, d_y_neu = calc_delta(absoluter_winkel, abstand_b)
    my_pose_for_publishinp.pose.position.x += d_x_neu
    my_pose_for_publishinp.pose.position.y += d_y_neu
    '''
    if winkel != my_pose_for_publishinp.pose.orientation.z:
        rospy.loginfo(f'{winkel}')#, Maus: {round(calc_orientierung(maus_1, maus_2), 4)}')

    my_pose_for_publishinp.header.stamp = data.header.stamp
    my_pose_for_publishinp.pose.position.x = data.point.x
    my_pose_for_publishinp.pose.position.y = data.point.y
    my_pose_for_publishinp.pose.position.z = 0
    my_pose_for_publishinp.pose.orientation.x = 0
    my_pose_for_publishinp.pose.orientation.y = 0
    my_pose_for_publishinp.pose.orientation.z = winkel
    my_pose_for_publishinp.pose.orientation.w = 0

    #x_alt = d_x
    #y_alt = d_y
    pub.publish(my_pose_for_publishinp)
    #rospy.loginfo(f'x: {round(my_pose_for_publishinp.pose.position.x, 2)}, y: {round(my_pose_for_publishinp.pose.position.y, 2)}, orientierung: {round(my_pose_for_publishinp.pose.orientation.z, 4)}')#, orientation_maus: {round(calc_orientierung(maus_1, maus_2), 4)}')
    #rospy.loginfo(f'LiDar: {round(my_pose_for_publishinp.pose.orientation.z, 4)}')
#angle = message_filters.Subscriber('winkel_eine_wand', Float64)#das muss noch in eine sinnvolle stamped message geändert werden
rospy.init_node('sync_and_controle')

rospy.Subscriber('winkel_ein_wand', Float64MultiArray, callback_winkel)#das muss noch in eine sinnvolle stamped message geändert werden
rospy.Subscriber('m_1', PointStamped, callback)
rospy.Subscriber('m_2', PointStamped, callback_maus_2)
rospy.Subscriber('rot_befehl', Float64MultiArray, callback_kallman)

rospy.spin()

