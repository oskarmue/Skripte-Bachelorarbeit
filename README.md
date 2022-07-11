# Skripte-Bachelorarbeit

Sensoren_launch.launch:

Hier werden bis auf controler.py alle nötigen Notes gestartet, die zum Abfahren der Trajektorie nötig sind.

Bestromung.py

Bevor die Sensoren gestartet werden, werden die Motoren bestromt um einen sicheren festen Stand zu gewährleisten. 
Beim ersten Bestromen der Motoren können Ruckler auftreten, diese würde bereits Messungen der Maussensoren führen,
womit die Startposition nicht mehr bei 0/0 wäre. Das ist ein weiterer Grund, weshalb zu Beginn des Fahrstarts die Bestromung
stattfindet.

winkel_publishen.py

In dieser node werden die Winkel zu den in dem Laserscan lokalisierten aufgestellten Wänden berechnet
und diesen fortlaufend zugeordnet.

lm1.py & lm2.py

Dies sind die Nodes zum auslesen und veröffentlichen der gemachten Maussensoremessungen

ts_maus_lidar.py

In dieser Node werden die neusten Messungen des Lidar Sensors und die neusten Messungen der Mäuse 
in einer Message kombiniert und dann veröffentlicht.
Weiterhin finden in dieser Node der Filterschritt der Daten Statt.

planer_option_eins.py

Hier wird die gemessene Pose des mobilen Robotersystem entgegengenommen und es werden
anschließen neue gewünschte Fahrtwerte berechnet und veröffentlicht.

controller.py

In dieser Node befindet sich der PID Regler für die Regelung der Orientierung.
Nach empfang der Ist-Pose und der gewünschten Pose (veröffentlicht durch planer_option_eins.py)
werden hier neue Fahrtwerte berechnet und dieser über die Serielle Verbindung zwischen Raspberry Pi und Arduiono
weitergegeben.
Außerdem wird der neue "Bewegungsbefehl" veröffentlicht und an ts_maus_lidar.py gesendet um diesen in dem Kalman Filter zu verwenden.
Controler.py gibt den ersten Bewegungsbefehl an die Motoren. Startet man diese Node in der Launch file, so fährt das System 
mit starten der Launch FIle los.

Stop_move.py

Sollte controller ein Error haben, werden die Motoren nicht automatisch gestoppt. Stop_move.py
sendet eine einmalige Message and die Motoren um diese zu stoppen.

