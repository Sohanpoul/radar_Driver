#!/usr/bin/env python3

import numpy as np
from sklearn.neighbors import KNeighborsClassifier
from sklearn.neighbors import NearestCentroid
from sklearn.svm import SVC
from sklearn.metrics import f1_score
from sklearn.model_selection import train_test_split
from sklearn import metrics
from sklearn.pipeline import Pipeline
import os
import math
import rospy
from std_msgs.msg import Bool
from radar_driver.msg import radar
from timeit import default_timer as timer
from nav_msgs.msg import Odometry

global dec 
global dec_pub
global speed 
speed = 0

def callback(data):
    global dec
    global dec_pub
    target_id = data.target_id
    speed_dat = data.speed_dat
    distance_x = data.distance_x
    distance_y = data.distance_y
    try:
        scenario = np.array([arrange_data(speed_dat,distance_x, distance_y)])
        decision = bool(False)
        print(scenario[0][0][0],scenario[0][0][1],abs(scenario[0][0][2]),scenario[0][1][0],scenario[0][1][1],abs(scenario[0][1][2]))
        if abs(scenario[0][1][2]) == 0.0 or abs(scenario[0][0][2]) == 0.0:
            decision = bool(False)
                
        else:
            print("check")
            
            decision = bool(dec.predict([[scenario[0][0][0],scenario[0][0][1],abs(scenario[0][0][2]),scenario[0][1][0],scenario[0][1][1],abs(scenario[0][1][2])]]))
            print("check2")
        dec_pub.publish(decision)
    except:
        #print("error")
        decision = bool(True)
        pass

    #scenario = np.array([scenario])
        

def gps_callback(msg):
    speed = msg.speed
        

def arrange_data(speed_dat,distance_x, distance_y):
    global speed
    d = sorted(list(zip(distance_x, distance_y, speed_dat)), key = lambda x: x[2], reverse = True)
    
    return d
    
class decesion_loop:
    def __init__(self):
        self.start = 0
        self.end = 0
        self.decision = True
        self.data = np.loadtxt('data.txt')
        self.labels = np.loadtxt('labels.txt')
        self.train()
        

    def train(self):
        
        score = 0
        train_X, val_X, train_y, val_y = train_test_split(self.data, self.labels, random_state=1)
        self.clf = Pipeline([('clf', KNeighborsClassifier()), ])
        self.clf.fit(train_X, train_y)
        predicted = self.clf.predict(val_X)
        k = metrics.confusion_matrix(val_y, predicted)
        

    def predict(self, scenario):
        self.start = timer()
        decision = self.clf.predict(scenario)
        self.end = timer()
        return decision

    def log_info(self):
        print('Time to make a decision: ' + str(round((self.end - self.start)*1000, 2)) + 'ms')


dec = decesion_loop()

def main():
   
    global dec_pub
    
    rospy.init_node('radar_decision_making', anonymous=True)
    
    rospy.Subscriber('/radar_track',radar,callback)
    #rospy.Subscriber('/gps_data/speed',Odometry,gps_callback)
    dec_pub = rospy.Publisher('/decision_making', Bool, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    rospy.spin()

if __name__ == '__main__':
    main()












#test_secnario = np.array([[2.992846824926438531e+01, 4.254972507212197108e+00, -1.110000000000006537e+01, 1.580217938527905019e+01, -4.21645313819245171e-01, -1.97999992370605469e+01]])

