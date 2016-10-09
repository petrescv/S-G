import serial
import time
import numpy as np
import string
from lib_robotis import *


ser = serial.Serial('COM4', 9600)

dyn = USB2Dynamixel_Device('COM3')
#find_servos(dyn)

s0 = Robotis_Servo(dyn,8)#base
s2 = Robotis_Servo(dyn,2)#shoulder,positive up
s3 = Robotis_Servo(dyn,3)#wrist
s7 = Robotis_Servo(dyn,7)#gripper
s5 = Robotis_Servo(dyn,5)#elbow
s6 = Robotis_Servo(dyn,6)#shoulder,negative up


RADIANS = 0
DEGREES = 1

l1 = 17.5 #Length of link 1
l2 = 19.5 #length of link 2

def invKin(x, y, angleMode=DEGREES):
    """Returns the angles of the first two links
    in the robotic arm as a list.
    returns -> (th1, th2)
    input:
    x - The x coordinate of the effector
    y - The y coordinate of the effector
    angleMode - tells the function to give the angle in
                degrees/radians. Default is degrees
    output:
    th1 - angle of the first link w.r.t ground
    th2 - angle of the second link w.r.t the first"""

    #stuff for calculating th2
    r_2 = x**2 + y**2
    l_sq = l1**2 + l2**2
    term2 = (r_2 - l_sq)/(2*l1*l2)
    term1 = ((1 - term2**2)**0.5)*-1
    #calculate th2
    th2 = math.atan2(term1, term2)
    #optional line. Comment this one out if you 
    #notice any problems
    th2 = -1*th2

    #Stuff for calculating th2
    k1 = l1 + l2*math.cos(th2)
    k2 = l2*math.sin(th2)
    r  = (k1**2 + k2**2)**0.5
    gamma = math.atan2(k2,k1)
    #calculate th1
    th1 = math.atan2(y,x) - gamma

    if(angleMode == RADIANS):
        return th1, th2
    else:
        return math.degrees(th1), math.degrees(th2)


print np.absolute(invKin(36,-5))

def goTo(arrayXY):
    theta1 = np.absolute(arrayXY[0])
    theta2 = np.absolute(arrayXY[1])
    
    s3.move_angle(math.radians(np.absolute(theta1-theta2)+12))
    time.sleep(0.3)
    s5.move_angle(math.radians(theta2),blocking = False)
    time.sleep(0.3)
    s2.move_angle(math.radians(theta1),blocking = False)
    s6.move_angle(math.radians(-theta1),blocking = True)
    
def goToRest():
    theta1 = 95
    theta2 = 0
    s7.move_angle(math.radians(90),blocking = False)
    s5.move_angle(math.radians(theta2),blocking = False)
    time.sleep(0.5)
    s2.move_angle(math.radians(theta1),blocking = False)
    s6.move_angle(math.radians(-theta1),blocking = True)
    time.sleep(0.5)
    s3.move_angle(math.radians(0))
    
def convertToDistance(val):
    if (val <= 175) and (val >= 94):
        return (9015*val**(-1.119) -0.0006*val**2 + 0.1969*val - 8.2976) 
    if val > 175:
        return (9015*val**(-1.119) - 0.0097*val + 7.9069)
    else:
        return 0

def getDistance():
    
    a = np.array(())
    
    all=string.maketrans('','')
    nodigs=all.translate(all, string.digits)
    
    start = time.time()
    
    while (time.time()-start)<=0.1:
        reading = ser.readline()
        reading = reading.translate(all, nodigs)
        if reading != '':
            reading = int(reading)
            a = np.hstack((a,reading))
        
    a = np.sort(a)
    x = a[int(len(a)/4):3*int(len(a)/4)]
    
    return convertToDistance(np.average(x))

def fastScan():
    
    startAngle = -45.
    endAngle = 45.
    s0.move_angle(math.radians(startAngle),blocking = False)
    print "Starting FAST scan..."
    angleRange = np.array(())
    angle = startAngle
    
    while angle < endAngle:
        s0.move_angle(math.radians(angle),blocking = False)
        angle = angle + 6*0.29
        print "Angle is:",angle
        print "Distance is:",getDistance()
        if getDistance() != 0:
            angleRange = np.hstack((angleRange,angle))    
        time.sleep(0.11)
    print angleRange
    print "FAST scan finished"
    return angleRange
    
def slowScan():
    
    slowAngleRange = fastScan()
    startAngle = slowAngleRange[-1]
    endAngle = slowAngleRange[0]
    s0.move_angle(math.radians(startAngle),blocking = False)
    
    angleRange = np.array(())
    distanceRange = np.array(())
    angle = startAngle
    print "Starting SLOW scan..."
    print "Starting angle is ",startAngle
    print "End angle is ",endAngle
    while angle > endAngle:
        s0.move_angle(math.radians(angle),blocking = False)
        angle = angle - 0.29
        print "Angle is:",angle
        print "Distance is:",getDistance()
        if getDistance() != 0:
            angleRange = np.hstack((angleRange,angle))
            distanceRange = np.hstack((distanceRange,getDistance()))  
        time.sleep(0.11)
    print angleRange
    print distanceRange
    print "SLOW scan finished"
    return angleRange, distanceRange
       
def getDepth():
    trueY = distanceRange
    trueY = trueY[int(len(trueY)/2):len(trueY)]
    return np.average(trueY) + 2.7

def getBaseRotation():
    return (angleRange[0] + angleRange[-1])/2.

def goToTrash():
    theta1 = 60
    theta2 = 45
    s0.move_angle(math.radians(-90),blocking = False)
    time.sleep(0.5)
    s7.move_angle(math.radians(45),blocking = False)
    s5.move_angle(math.radians(theta2),blocking = False)
    time.sleep(0.5)
    s2.move_angle(math.radians(theta1),blocking = False)
    s6.move_angle(math.radians(-theta1),blocking = True)
    s3.move_angle(math.radians(np.absolute(theta1-theta2)))
    time.sleep(0.5)
    s7.move_angle(math.radians(45),blocking = False)


goToRest()

angleRange, distanceRange = slowScan()

print getDepth()
print getBaseRotation()

s0.move_angle(math.radians(getBaseRotation()),blocking = False)

if getDepth() <= 40.:
    goTo(invKin(getDepth()-15,0))
if getDepth() >= 40. and getDepth() < 45.:
    goTo(invKin(getDepth()-16,-2))
else:
    goTo(invKin(getDepth()-15,-5))

s7.move_angle(math.radians(45),blocking = False)

time.sleep(1)

goToTrash()

goToRest()
