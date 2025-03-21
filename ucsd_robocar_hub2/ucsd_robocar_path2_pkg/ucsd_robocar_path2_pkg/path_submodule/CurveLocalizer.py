from re import S
from tracemalloc import start
import numpy as np
import os,pickle, math
from scipy.ndimage import convolve1d
from scipy import optimize

class CurveLocalizer:
    def __init__(self, mapDir, sampleDist, history_size, startIdx, mapWindow, initTime):
        self.map = mapDir
        
        self.sampling_dist = sampleDist
        self.Xprev = np.zeros((3,1))
        self.prevTime = initTime
        self.currIdx = startIdx

        self.history_size = history_size
        self.x = [0.0]
        self.y = [0.0]

        self.measurement_hist = np.zeros(history_size)
        self.mapWindow = mapWindow

    def addMeasurement(self, curv):
        self.measurement_hist = np.delete(self.measurement_hist,0)
        self.measurement_hist = np.append(self.measurement_hist,curv)

    def predict(self, vx, steeringangle, dt, X_prev):
        l = 0.33
        lr = 0.1
        vx = 0.5
        steeringangle = 0
        beta = np.arctan2(lr*(steeringangle),l)
        xdot = vx*np.cos(beta)
        ydot = vx*np.sin(beta)
        thetadot = vx*np.tan(steeringangle)*np.cos(beta)/l

        X = X_prev + dt*np.array([[xdot], [ydot], [thetadot]])

        travelDist = np.linalg.norm(X[:2]-X_prev[:2])

        deltaIdx = int(travelDist/self.sampling_dist)

        self.Xprev = np.copy(X)

        return deltaIdx

    def convolve(self, map_array, window, start, win_size):
        mLen = len(window)
        submap = map_array[int(start - 0.5*win_size):int(start + 0.5*win_size)+1]
        min_sum = 10000
        min_index = 0
        for i in range(len(submap)-len(window)+1):
            curr_sum = np.sum(abs(submap[i:i+len(window)]-window))
            if curr_sum<min_sum:
                min_sum = curr_sum
                min_index = int(start - 0.5*win_size)+i+mLen-1

        return min_index

    # Functions to fit circle to set of points
    def calcRadius(self, xc, yc):
        return np.sqrt((self.x-xc)**2 + (self.y-yc)**2)

    def f_2(self, center):
        Ri = self.calcRadius(*center)
        return Ri-Ri.mean()

    def fitcircle(self, x, y):
        self.x = x
        self.y = y
        x_m = np.mean(x)
        y_m = np.mean(y)

        center_est = x_m, y_m
        center, ier = optimize.leastsq(self.f_2, center_est)

        if math.isinf(np.sum(center)) or np.isnan(np.sum(center)):
            curvature = 0
        else:
            curvature = np.mean(1/self.calcRadius(*center))

        print('center: ', center, 'curvature: ', curvature)

        return center, curvature

    def computePosition(self, vx, steeringangle, currTime, lidarAvailable):
        dt = currTime - self.prevTime
        deltaIdx = self.predict(vx, steeringangle, dt, self.Xprev)
        nextIdx = self.currIdx + deltaIdx

        if lidarAvailable:
            nextIdx = self.convolve(self.map, self.measurement_hist, nextIdx, self.mapWindow)

        nextIdx = nextIdx%len(self.map)
        self.prevTime = currTime

        return nextIdx
