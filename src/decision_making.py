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


global dec 
global dec_pub


def callback(data):
    global dec
    global dec_pub
    target_id = data.target_id
    speed_dat = data.speed_dat
    distance_x = data.distance_x
    distance_y = data.distance_y
    scenario = np.array([arrange_data(speed_dat,distance_x, distance_y)])    
    #scenario = np.array([scenario])
    decision = bool(dec.predict(scenario))
    dec_pub.publish(decision)

    

def arrange_data(speed_dat,distance_x, distance_y):
    d = sorted(list(zip(distance_x, distance_y, speed_dat)), key = lambda x: x[0], reverse = True)
    
    return [d[1][0], d[1][1], d[1][2], d[0][0], d[0][1], d[0][2]] 

    
class decesion_loop:
    def __init__(self):
        self.start = 0
        self.end = 0
        self.decision = True
        self.data = np.loadtxt('data.txt')
        self.labels = np.loadtxt('labels.txt')
        self.train()
        

    def train(self):
        for i in range(500):
            score = 0
            train_X, val_X, train_y, val_y = train_test_split(self.data, self.labels, random_state=i)
            self.clf = Pipeline([('clf', KNeighborsClassifier()), ])
            self.clf.fit(train_X, train_y)
            predicted = self.clf.predict(val_X)
            k = metrics.confusion_matrix(val_y, predicted)
            # for k in range(0, len(val_y) - 1):
            #     print(str(val_X[k]) + '     ' + str(val_y[k]) + '   :   ' + str(predicted[k]))
            fscore = float(f1_score(val_y, predicted, average='weighted')) #, zero_division=1))
            score += fscore
        print('accuracy = ' + str(score/500))

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
    dec_pub = rospy.Publisher('/decision_making', Bool, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    rospy.spin()

if __name__ == '__main__':
    main()












#test_secnario = np.array([[2.992846824926438531e+01, 4.254972507212197108e+00, -1.110000000000006537e+01, 1.580217938527905019e+01, -4.21645313819245171e-01, -1.97999992370605469e+01]])

