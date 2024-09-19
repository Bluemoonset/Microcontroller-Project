import time
import csv
from adafruit_servokit import ServoKit
import board
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
import math

i2c = board.I2C()
pca = PCA9685(i2c)
pca.frequency = 50

#1 ft
armLength = 1
#arm distance from ground (ft) (0, 0.4]
armHeight = 0.3

kit = ServoKit(channels=16)

#Writes an updated servo position to file.
def writePosition(servoNum, angle):
    copyDict = {}
    with open('servoPosition.csv', mode='r') as file:   
        reader = csv.reader(file)
        for line in reader:
            copyDict.update({str(line[0]):line[1]})
        #print(copyDict)
        with open('servoPosition.csv', 'w') as csv_file:  
            writer = csv.writer(csv_file)
            copyDict.update({str(servoNum):str(angle)})
            for key, value in copyDict.items():
                writer.writerow([key, value]) 
                     
#Gets the current position of a servo.
def getPosition(servoNum):
    copyDict = {}
    with open("servoPosition.csv", 'r') as file:
        reader = csv.reader(file)
        for line in reader:
            copyDict.update({str(line[0]):line[1]})
        return copyDict[str(servoNum)]

#Moves a servo to an angle.
def move(servo, num, ang):
    initialAngle = float(getPosition(num))
    angleDiff = (abs(ang - initialAngle))
    #Does not move if there is less than 1 degree of change.
    if (angleDiff < 1):
        return
    #How many steps for each movement
    numStep = round(1.2 * angleDiff)
    increment = (ang - initialAngle)/(numStep)
    currentAngle = initialAngle
    for x in range(0, numStep):
        currentAngle += increment
        if (currentAngle < 0):
            currentAngle = 1
        elif (currentAngle > 180):
            currentAngle = 180
        #servo[int(num)].angle = round(currentAngle,1)
        servo.angle = round(currentAngle,1)
        writePosition(int(num), str(round(currentAngle,1)))
        time.sleep(abs(increment/60))
        
#TODO, returns the arm to a known default position.
def returnDefault(servo):
    pass
        
#Where ABC are wanted angles, and abc are known sides
#Calculates angles needed for an input distance.
def processor(a,b,c):
    #calculate the angles for a triangle at the position, but above ground level.
    angles0 = lawCosine(a,b,c)
    #calculate the angles for a triangle at the position, updated to be closer to the ground.
    angles = RadToDeg(groundAdjust(angles0, c))
    #angles0 = RadToDeg(angles0)
    #print('orig deg', angleAdjust(angles0[0], angles0[1], angles0[2],c))
    
    return angleAdjust(angles[0], angles[1], angles[2],c)

#Law of cosine to find all the angles of a triangle given all of its sides.
def lawCosine(a, b, c):
    A = math.acos((b**2 + c**2 - a**2)/(2*b*c)) # + 16.7
    B = math.acos((a**2 + c**2 - b**2)/(2*a*c))
    C = math.acos((a**2 + b**2 - c**2)/(2*a*b))
    return([A,B,C])

#Adjusts theoretical angles for the servos, which are not positioned ideally.
#Values here are all from manual testing, and WILL change based on PWM.
def angleAdjust(A,B,C,c):
    B = (-B / 2) + 103
    C = (-C / 2) + 129
    return [A,B,C]


#adjust angles of triangle to bring arm cloesr to the ground
""" def groundAdjust(angles):
    #imagine arm as triangle. 
    #original vertical distance of triangle
    oVertical = math.sin(angles[1])
    #new vertical distance of triangle
    nVertical = oVertical + 0.3
    #The length of the temporary side created.
    tempSide = math.sin(angles[2] / 2) * 1
    #apply law of cosine to calculate new angle for servo2.
    angles[1] = math.acos((1 + (tempSide ** 2) - (nVertical**2)) / (2*1*tempSide))
    #apply law of sines to find new angle for servo4
    angles[2] = math.asin(math.sin(angles[1])*tempSide/nVertical)
    return angles """
    
#Adjust the angles of a triangle to bring the arm closer to the ground
def groundAdjust(angles, distance):
    #The length of the temporary side created.
    #theta = math.atan((0.3) / distance)
    #phi = angles[1] + theta
    #angles[1] = phi
    #angles[2] = math.pi - (2 * theta)
    #tempDist = distance / math.acos(theta)
    #angles[2] = math.asin(tempDist * math.sin(phi))
    #tempDist = 0.3/math.sin(theta)
    #tempDist = distance / math.cos(theta)
    #angles[2] = math.asin(tempDist * math.sin(phi))
    
    #Creating another triangle below current triangle the input distance and set armHeight. 
    theta = math.atan(armHeight/distance)
    #Gets the distance between the new end of the arm and servo2
    tempDist = armHeight/math.sin(theta)
    #create the new triangle with law of cosine after accounting for the extra vertical distance.
    angles2 = lawCosine(armLength, armLength,tempDist)
    #The true angle. There is overlap, but it shouldn't matter for this purpose.
    overlap = angles[1] + angles2[1] - theta
    #angles2[1] = angles[1] + angles2[1] - overlap
    
    #Adjust the first angle; Law cosine will only create triangles where "tempDist" is parallel to the ground.
    #Therefore the first angle must be "lowered."
    #IRL this should be added together, but the servo is positioned so that
    #the higher the number the servo receives, the lower arm position.
    #Since the servos scale inversly with angle, we have to inverse the previous statement.
    #Therefore instead of the higher the number the lower the arm, its the lower the number the lower the arm.
    angles2[1] = -theta + angles[1]
    return (angles2)

def RadToDeg(angles):
    angles[0] = angles[0] * 180 / math.pi
    angles[1] = angles[1] * 180 / math.pi
    angles[2] = angles[2] * 180 / math.pi
    return angles

def DegToRad(angles):
    angles[0] = angles[0] * math.pi / 180
    angles[1] = angles[1] * math.pi / 180
    angles[2] = angles[2] * math.pi / 180
    return angles

#Given angle for servo0 and distance from the base, move the end of the arm near the ground there.
def moveGround(servo, ang0, distance):
    #angle[0] = do not use (angle between arm end and servo2), angle[1] = angle of servo2, angle[2] = angle of servo4
    #Calculate the angles needed
    angles = processor(float(armLength), float(armLength), float(distance))
    print("Practical After: ", angles)
    
    #Move servos.
    #move(servo, 0, ang0)
    move(servo0,0, ang0)
    time.sleep(0.5)
    servo2Pos = float(getPosition(2))
    #if servo2 arm go up, do first
    if (angles[1] - servo2Pos < 0):
        #move(servo, 2, angles[1])
        move(servo2,2, angles[1])
        time.sleep(0.5)
        #move(servo, 4, angles[2])
        move(servo4,4, angles[2])
    else:
        #move(servo, 4, angles[2])
        move(servo4,4, angles[2])
        time.sleep(0.5)
        #move(servo, 2, angles[1])
        move(servo2,2, angles[1])
       
#Setting default conditions

""" kit.servo[0].set_pulse_width_range(150,4000)
kit.servo[2].set_pulse_width_range(150,4000)
#kit.servo[4].set_pulse_width_range(500,2650)
kit.servo[4].set_pulse_width_range(150,4000)

kit.servo[0].angle = 60
time.sleep(1)
kit.servo[2].angle = 60
time.sleep(1)
kit.servo[4].angle = 170 """

servo0 = servo.Servo(pca.channels[0], min_pulse=150, max_pulse=4000)
servo2 = servo.Servo(pca.channels[2], min_pulse=150, max_pulse=4000)
servo4 = servo.Servo(pca.channels[4], min_pulse=150, max_pulse=4000)

servo0.angle = 60
time.sleep(.5)
servo2.angle =60
time.sleep(.5)
servo4.angle = 60
time.sleep(.5)

writePosition('0', '60')
writePosition('2', '60')
writePosition('4', '60')

quit = False
print("Enter 'q' to quit")

#For manual movement 
while True:
    while True:
        try:
            angle = input("Please select angle to face [5, 120]: ")
            if (angle == 'q'):
                quit = True
                break
            angle = float(angle)
            break
        except ValueError:
            print('Input is not a number, try again.')
    if (quit):
        pca.deinit()
        break
    while True:
        try:
            distance = input("Please select distance (in feet) (.3, 2): ")
            if (distance == 'q'):
                quit = True
                break
            distance = float(distance)
            break
        except ValueError:
            print('Input is not a number, try again.')
    #move(kit.servo, servo, angle)
    if (quit):
        pca.deinit()
        break
    moveGround(kit.servo, angle, distance)
    #returnDefault(kit.servo)
    time.sleep(.5)
