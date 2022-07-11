#!/usr/bin/env python3

#scp /home/oskar/Desktop/scripte_bachelor/Code/winkel_publishen.py pi@10.42.0.3:/home/pi/catkin_ws/src/orientation/src/


import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray
import numpy as np
import time
from collections import deque

'''
Laserscan Message
header: 
  seq: 536
  stamp: 
    secs: 1638868956
    nsecs: 819774900
  frame_id: "base_scan"
angle_min: -3.1415927410125732
angle_max: 3.1415927410125732
angle_increment: 0.006919807754456997
time_increment: 0.0001111109959310852
scan_time: 0.10011100769042969
range_min: 0.10000000149011612
range_max: 16.0
ranges [] (len = 909) #weiß nicht ob sich die länge des arrays verändert, vlt nach dem kallibrieren
intensities [] (len = 909)
'''

pub = rospy.Publisher('winkel_ein_wand', Float64MultiArray, queue_size = 1)#queue_size: wenn die nachrichten schnelle reinkommen als sie verarbeotet werden,
#könne diese in der queue gespeichert werden, denke aber, dass das die zu einem delay führen kann, deshalb kein großer wert, ein ständiger Datenfluss sollte eh
#vorhanden sein, sonst gibt es ganz andere Probleme, alte daten helfen auch nicht sonderlich beim steuern, da sich das System woanders befindet

geraden_folge = deque(maxlen = 4) #eigentlich reicht auch maxlen = 2
null_posi_1 = 0
null_posi_2 = 0
winkel_zu_eins_beginn = 0
winkel_zu_zwei_beginn = 0

Anfangs_winkel_1 = []
Anfangs_winkel_2 = []

Anfangs_Abstand_1 = []
Anfangs_Abstand_2 = []

null_winkel_1 = 0
null_winkel_2 = 0

null_abstand_1 = 0
null_abstand_2 = 0

abstand_1 = 0
abstand_2 = 0

abstand_seite_1 = 0
abstand_seite_2 = 0

geraden_alt = []

my_array_for_publishing = Float64MultiArray()

def pol2cart(rho, phi):
	#polar koordinaten in karthesische um berechnen
	x = rho * np.cos(phi)
	y = rho * np.sin(phi)
	return x, y

def dist(x3, y3, gerade): 
	#den kürzstens abstand zwischen einem Punkt und einer Geraden berechnen
	#x3,y3 der Punkt
	#gerade = np.polyfit([x1[0], x2[0]], [x1[1], x2[1]], 1), da das für eine Kombination immer das gleiche ist, 
	#macht es keinen Sinn, dass für jeden neuen Punkt neu u berechnen
	normale = -1/gerade[0]
	d = y3 - normale * x3
	x_s = (d - gerade[1])/(gerade[0] - normale)
	y_s = gerade[0] * x_s + gerade[1]
	dist = np.sqrt(((x3-x_s)**2)+((y3-y_s)**2))
	return dist

def find_angles_to_walls_cart_90(LaserScan, schwelle):
	global geraden_alt
	#1. Die Polarkoordinaten (aus dem laserscan) in karthesische umrechnen
	scan = LaserScan.ranges
	angle_increment = LaserScan.angle_increment
	angles = []
	counter = 0
	for i in range(len(scan)):
		angles.append(angle_increment * counter)
		counter += 1


	scan_cart = []
	for i in range(len(scan)):
		if scan[i] >= 3:
			x, y = 0, 0
		else:
			x, y = pol2cart(scan[i], angles[i])
		scan_cart.append([x, y])

	#2. In den koordinaten nach potentiellen Ebenen suchen, die zur orientierung aufgestellt worden sind
	geraden = []
	i = 0
	while True:
		p_1 = scan_cart[i]
		p_2 = scan_cart[i + 6]
		p_3 = scan_cart[i + 12]
		if (p_1[0] != 0) and (p_2[1] != 0):
			gerade = np.polyfit([p_1[0], p_2[0], p_3[0]], [p_1[1], p_2[1], p_3[1]], 1)
			s = i + 1
			counter = 0
			consecutive_zeros = 0
			while True and (s < len(scan_cart)):
				if (scan_cart[s][0] != 0) and (scan_cart[s][1] != 0):
					consecutive_zeros = 0
					a = dist(scan_cart[s][0], scan_cart[s][1], gerade)
					
					if a <= 0.1: #mit diesem Wert eventuell noch weiter rumprobieren
						counter += 1
						
					else:
						break
					if consecutive_zeros >= 2:
						break

				else:
					consecutive_zeros += 1
				s += 1

				if s >= len(scan_cart)-5:
					break
								
			if counter >= schwelle:
				#print(counter)
				geraden.append([gerade[0], gerade[1], i, counter])
				i = s #das machen und dann den Zeitunterschied betrachten  0.0411992073059082 s vs 0.005056619644165039 s
								
		i += 20
		if i >= len(scan_cart)-20:
			break
	for i in range(len(geraden)):
		ende = geraden[i][2] + geraden[i][3] 
		anfang = geraden[i][2]
		m = geraden[i][0]
		b = geraden[i][1]
		gerade = [m,b]
		scan_cart_part = scan_cart[:ende]

		cons_zero = 0
		index = anfang
		counter = 0
		while True:
			punkt = scan_cart[index]
			abstand = dist(punkt[0], punkt[1], gerade)
			#print(abstand, index)
			
			counter += 1

			if punkt[0] == 0:
				cons_zero += 1
			if cons_zero > 2:
				break
			if abstand >= 0.1 and punkt[0] != 0:
				break

			anfang = index
			index -= 1

		geraden[i][2] = anfang
			
		relevante_punkte = scan_cart[anfang:ende]
		relevante_punkte_ohne_null = []
		for s in range(len(relevante_punkte)):
			if relevante_punkte[s][0] != 0:
				relevante_punkte_ohne_null.append(relevante_punkte[s])

		relevante_punkte_ohne_null = np.array(relevante_punkte_ohne_null)
		x_s = relevante_punkte_ohne_null[:,0]
		y_s = relevante_punkte_ohne_null[:,1]

		gerade = np.polyfit(x_s, y_s, 1)

		geraden[i][0] = gerade[0]
		geraden[i][1] = gerade[1]
		geraden[i].append(anfang) #index 4

	#3. die Stellung der Ebenen zueinander und den Schnittpunkt berechnen
	#ist eigentlich unwichtig, aber ganz interessant zu sehen, ob es wirklich 90° sind, sollte das zu weit off sein, muss der
	#Aufbau nochmal verbessert werden
	'''
	for i in range(len(geraden)):
		m, b = geraden[i][0], geraden[i][1] 
		for s in range(i + 1, len(geraden)):
			n, d = geraden[s][0], geraden[s][1]

			if m * n != -1:
				schnittwinkel = np.arctan((m - n)/(1 + m * n))
				#print(f'winkel zwischen {i + 1} und {s + 1}: {round(schnittwinkel, 4)}rad {round(schnittwinkel * (180/np.pi), 4)}°')
			else:
				schnittwinkel = np.pi/2
				#print(f'winkel zwischen {i + 1} und {s + 1}: {round(schnittwinkel, 4)}rad {90}°')
	'''
	#4. Die Winkel von Sensor zu Bezugsebenen berechnen, dieser wird dann für die Orientierung verwendet
	#die nullachse des Sensors ist bei dem kabel, dieses liegt in dieser anschauung auf der positioven x achse
	for i in range(len(geraden)):
		m = geraden[i][0]
		b = geraden[i][1]

		if m > 0:
			if b > 0:
				winkel = np.pi/2 +  np.arctan(m)
			else:
				winkel = np.pi * 2 - np.arctan(m)
		else:
			if b > 0:
				winkel = np.pi/2 - np.arctan(abs(m))
			else:
				winkel = np.pi * 3/2 - np.arctan(abs(m))

		geraden[i][2] = winkel
		#print('das ist der Winkel: ',winkel, 'm: ', m, 'b: ', b)

	#5. Abstand zu den Geraden berechnen berechen, um damit die Position des Sensors relativ zu den Bezugsebenen zu bestimmen
	for i in range(len(geraden)):
		m, b = geraden[i][0], geraden[i][1]

		m_2 = -1/m
		b_2 = 0 #da wir den kürzesten weg zwischen gerade und ursprung haben wollen

		Schnittpunkt_x = -(b/(m + 1/m))
		Schnittpunkt_y = b/(m**2 + 1)		

		abstand_Schnittpunkt = np.sqrt(Schnittpunkt_x**2 + Schnittpunkt_y**2)
		geraden[i][3] = abstand_Schnittpunkt
		abstand_seite = np.sqrt((Schnittpunkt_x - scan_cart[geraden[i][4]][0])**2 + (Schnittpunkt_y- scan_cart[geraden[i][4]][1])**2)
		geraden[i][4] = abstand_seite


	if len(geraden) > 2:
		geraden = geraden_alt
		#dann wird halt einmal der alte wert veröffentlicht

	geraden_alt = geraden

	return geraden #[[[(erste gerade) m, b, winkel, abstand], [(zweite gerade) m, b, winkel, abstand]]

def callback(data):
	global geraden_folge
	global my_array_for_publishing
	global null_posi_1
	global null_posi_2
	global winkel_zu_eins_beginn
	global winkel_zu_zwei_beginn

	global Anfangs_winkel_1
	global Anfangs_winkel_2

	global Anfangs_Abstand_1
	global Anfangs_Abstand_2

	global null_winkel_1
	global null_winkel_2

	global abstand_1
	global abstand_2

	global null_abstand_1
	global null_abstand_2

	global abstand_seite_1
	global abstand_seite_2

	#neue Werte berechnen
	geraden_folge.append(find_angles_to_walls_cart_90(data, 50))

	#Zuordnen welche gerade zu welcher Geradengruppe gehört
	if len(geraden_folge) == 1:
		#Festlegen welche Gerade eins und welche zwei ist
		geraden_folge[0][0].append(1)
		geraden_folge[0][1].append(2)

	else:
		#Die zuordnung welche der neuen Geraden Gerade eins ist, passiert über die Steigung, die die näher an der alten dran ist, ist die eins
		m_1_alt = geraden_folge[-2][0][0]

		m_1_neu = geraden_folge[-1][0][0]
		m_2_neu = geraden_folge[-1][1][0]

		if abs(m_1_alt - m_1_neu) < abs(m_1_alt - m_2_neu):
			geraden_folge[-1][0].append(geraden_folge[-2][0][-1])
			geraden_folge[-1][1].append(geraden_folge[-2][1][-1])
		else:
			geraden_folge[-1][1].append(geraden_folge[-2][0][-1])
			geraden_folge[-1][0].append(geraden_folge[-2][1][-1])

		#neue Koordinaten und Winkel berechnen
		alte_posi = np.array(geraden_folge[-2])
		index_alt_eins = np.where(alte_posi[:, -1] == 1)[0][0]
		index_alt_zwei = np.where(alte_posi[:, -1] == 2)[0][0]

		neue_posi = np.array(geraden_folge[-1])
		index_neu_eins = np.where(neue_posi[:, -1] == 1)[0][0]
		index_neu_zwei = np.where(neue_posi[:, -1] == 2)[0][0]

		if len(Anfangs_winkel_1) <= 10:
			Anfangs_winkel_1.append(neue_posi[index_neu_eins][2])
			Anfangs_winkel_2.append(neue_posi[index_neu_zwei][2])
			Anfangs_Abstand_1.append(neue_posi[index_neu_eins][3])
			Anfangs_Abstand_2.append(neue_posi[index_neu_zwei][3])

			if len(Anfangs_winkel_1) == 10:
				null_winkel_1 = np.mean(Anfangs_winkel_1)
				null_winkel_2 = np.mean(Anfangs_winkel_2)

				null_abstand_1 = np.mean(Anfangs_Abstand_1)
				null_abstand_2 = np.mean(Anfangs_Abstand_2)

			rospy.loginfo(f'es werden genaue Anfangswerte gemessen')

		else:
			winkel_1 = neue_posi[index_neu_eins][2] - null_winkel_1
			winkel_2 = neue_posi[index_neu_zwei][2] - null_winkel_2

			abstand_1 = (neue_posi[index_neu_eins][3] - null_abstand_1) * 10**3
			abstand_2 = (neue_posi[index_neu_zwei][3] - null_abstand_2) * 10**3

			abstand_seite_1 = neue_posi[index_neu_eins][4]
			abstand_seite_2 = neue_posi[index_neu_zwei][4]

			#Die neuen daten publishen (topic =  /winkel_publisher)
			my_array_for_publishing.data = [winkel_1, winkel_2, abstand_1, abstand_2, abstand_seite_1, abstand_seite_2]
			pub.publish(my_array_for_publishing)

			#rospy.loginfo(f'Erhaltenen Daten wurden verarbeitet: {len(geraden_folge[-1])} Bezugsebenen wurden gefunden')

def scan_verarbeiten():
	rospy.init_node("winkel_publisher", anonymous = True)
	rospy.Subscriber('scan', LaserScan, callback)

	#dann wird callback ausgeführt

	rospy.spin()#lets run node continously

if __name__ == "__main__":	
	try:
		scan_verarbeiten()
	except rospy.ROSInterruptException:
		pass
