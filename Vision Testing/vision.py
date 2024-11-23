import cv2
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import math
df = pd.read_csv('edge_points.csv')  

known_shape = np.array(df[['x', 'y']].values, dtype=np.int32)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # Adjust width as needed
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480) # Adjust height as needed

kp = 0.15
ki = 0.001
kd = -1
 
#random comment

totalErrorM1 = 0
totalErrorM2 = 0
totalErrorM3 = 0

motor1 = 0
motor2 = 0
motor3 = 0
            
lastM1Dist = 0
lastM2Dist = 0
lastM3Dist = 0

totalErrorX = 0
totalErrorY = 0

lastX = 0
lastY = 0

motor1 = 0.5
motor2 = 0
motor3 = 0
from smbus2 import SMBus

addr = 0x8
bus = SMBus(1)

number = 1

def send_signed_int(value):
    if not (-32768 <= value <= 32767):
        return 0
    
    highByte = (value >> 8) & 0xFF
    lowByte = value & 0xFF
    
    bus.write_i2c_block_data(addr, highByte, [lowByte])

def send_motor_data(M1_Byte, M2_Byte, M3_Byte):
    highByte = int(M1_Byte) & 0xFF
    mid_byte = int(M2_Byte) & 0xFF
    lowByte = int(M3_Byte) & 0xFF
    
    bus.write_i2c_block_data(addr, highByte, [mid_byte, lowByte])
    
    
def distance(coords0, coords1):
    return math.sqrt((coords0[0] - coords1[0])**2 + (coords0[1] - coords1[1])**2)


def getPerp(linePoint1, linePoint2):
    x1,y1 = linePoint1
    x2,y2 = linePoint2
    x3,y3 = linePoint2
    
    if(x2 == x1):
        return (0,0)
    
    originalSlope = (y2-y1)/(x2-x1)
    perp = -1/originalSlope
    
    b = y3 - perp*x3
    
    return(perp, b)

def pointToLineDist(point, slope, intercept,motorType):
    x1,y1 = point
    A = slope
    B = -1
    C = intercept
    
    yline = slope*x1 + intercept
    diff = y1 - yline
    
    numerator = abs(A*x1 + B*y1 + C)
    denom = math.sqrt((A)**2 + (B)**2)

    if(denom == 0):
        return 0
    if(motorType == "m1"):
        if(y1 > yline):
            return -numerator/denom
        else:
            return numerator/denom   
    if(diff > 0):
        return numerator/denom
    else:
        return -numerator/denom


def getMotorDist(linePoint1, linePoint2, point,motorType):
    slope, intercept = getPerp(linePoint1,linePoint2)
    return pointToLineDist(point, slope, intercept,motorType)
    
'''
def distFromLine(linePoint1, linePoint2, point):
    
    x1,y1 = linePoint1
    x2,y2 = linePoint2
    x3,y3 = point
    
    
    numerator = ((y2-y1)*x3 - (x2-x1) *y3 + (x2*y1) - (y2*x1))
    denom = math.sqrt((y2-y1)**2 + (x2-x1)**2)
    
    
    cross = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1)
    
    input(cross)
    
    if(denom == 0):
        return 0
    if(cross > 0):
        return numerator/denom
    else:
        return -numerator/denom
'''
   
    

def findClosest(m1Loc, m2Loc, m3Loc, ballLoc):
    closestName = "M3"
    closestDist = distance(ballLoc, m3Loc)
    
    if(distance(ballLoc, m2Loc) < closestDist):
        closestDist = distance(ballLoc, m2Loc)
        closestName = "M2"
    
    
    if(distance(ballLoc, m1Loc) < closestDist):
        closestDist = distance(ballLoc, m2Loc)
        closestName = "M1"
    
    return closestName
    
def getSign(num):
    if num < 0:
        return (0,0,255)
    return (0,255,0)

def drawInterceptLine(frame,m1Loc,midX,midY):
    m, i = getPerp(m1Loc,(int(midX),int(midY)))
    x = midX
    y = midY
    
    
    deltaX = 50
    
    x1 = int(x - deltaX)
    y1 = int(y - m *deltaX)
    
    x2 = int(x + deltaX)
    y2 = int(y + m*deltaX)
    
    cv2.line(frame,(x1,y1),(x2,y2),(0,255,0),2)
    return frame

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    
    # Convert the frame from BGR to HSV color space to easily identify a colour
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 

    
    plat_color_lower = np.array([0, 0, 105]) # [lower Hue, lower Saturation, lower Value]
    plat_color_upper = np.array([180, 50, 200]) # [upper Hue, upper Saturation, upper Value]
    
    hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Threshold the HSV image to get the colors defined above
    # Pixels in the range are set to white (255) and those that aren't are set to black (0), creating a binary mask 
    mask = cv2.inRange(hsvFrame, plat_color_lower, plat_color_upper)
    cv2.imshow("white mask", mask)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
   
    if contours:
        # Determines the larget contour size using the cv.contour Area function
        largest_contour = max(contours, key=cv2.contourArea)

        # Draw the smoothed contour
       # cv2.drawContours(frame, [smooth_contour], -1, (0, 255, 0), thickness=2)
        cv2.drawContours(frame, [largest_contour], -1, (214, 112, 218), thickness=2)

        #cv2.drawContours(frame, largest_contour, -1, (255, 0, 0), 2)
        ((midX, midY), radius) = cv2.minEnclosingCircle(largest_contour)
        cv2.circle(frame, (int(midX), int(midY)), 2, (0, 0, 255), -1)  # (image to draw dot on, x,y pixel coordinates, radius in pixels, RGB values in this case red, -1 indicates to fill the circle)
        
        approx = cv2.approxPolyDP(largest_contour, epsilon=5, closed = True)
        
        m1Loc = (int(midX), int(midY) + 150)
        m2Loc = (int(midX) + 100, int(midY) - 75)
        m3Loc = (int(midX) - 100, int(midY) - 75)

        
       
        # *3 Define the range of yellow color in HSV [Hue, Saturation, Value]
        # SET THESE VALUES VIA THE METHOD EXPLAINED IN THE TUTORIAL
        ball_color_lower = np.array([10, 150, 100]) # [lower Hue, lower Saturation, lower Value]
        ball_color_upper = np.array([25, 255, 255]) # [upper Hue, upper Saturation, upper Value]


        # Threshold the HSV image to get the colors defined above
        # Pixels in the range are set to white (255) and those that aren't are set to black (0), creating a binary mask 
        mask = cv2.inRange(gray, ball_color_lower, ball_color_upper)

        #cv2.imshow("orange mask", mask)
        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Edge detection
        edges = cv2.Canny(blurred, threshold1=50, threshold2=150, apertureSize=3)

        # Find contours in the edge-detected image
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Threshold the HSV image to get the colors defined above
        # Pixels in the range are set to white (255) and those that aren't are set to black (0), creating a binary mask 
        mask = cv2.inRange(hsvFrame, ball_color_lower, ball_color_upper)
        cv2.imshow("orange mask", mask)


        # Find contours in the mask
        # RETR_TREE retrieves all hierarchical contours and organizes them
        # CHAIN_APPROX_SIMPLE compresses horizontal, vertical, and diagonal segments, leaving only their end points
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        
        
        
   # cv2.line(frame,m1Loc,(int(midX),int(midY)),(255,0,0),2)
  #  cv2.line(frame,m2Loc,(int(midX),int(midY)),(255,0,0),2)
 #   cv2.line(frame,m3Loc,(int(midX),int(midY)),(255,0,0),2)

        # Find the index of the largest contour/
        if contours:
            # Determines the larget contour size using the cv.contour Area function
            largest_contour = max(contours, key=cv2.contourArea)
            # Computes the minimum enclosing circle aroudn the largest contour
            ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)
            # * 4 Only consider large enough objects. If it only detects a small portion of your ball, you can test higher radius values to capture more of the ball
            if radius > 10:
                # Draw a yellow circle around the ball
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                # Draw a red dot in the center of the ball
            cv2.circle(frame, (int(x), int(y)), 2, (0, 255, 255), -1)  # (image to draw dot on, x,y pixel coordinates, radius in pixels, RGB values in this case red, -1 indicates to fill the circle)
            # Display the position of the ball
            
            errorX = int(x)-int(midX)
            errorY = int(y)-int(midY)
            totalErrorX += errorX
            totalErrorY += errorY
            dx = x - lastX
            dy = y - lastY
            lastX = x 
            lastY = y 
            
            
            m1Dist = getMotorDist(m1Loc,(midX,midY),(int(x),int(y)),"m1")
            m2Dist = getMotorDist(m2Loc,(midX,midY),(int(x),int(y)),"m2")
            m3Dist = getMotorDist(m3Loc,(midX,midY),(int(x),int(y)),"m3")
            drawInterceptLine(frame,m3Loc,midX,midY)
            
            xVector = -(errorX)
            yVector = -(errorY)
            
            cv2.arrowedLine(frame, (int(x), int(y)), (int(x + xVector), int(y + yVector)), (255, 0, 0), thickness=2, tipLength=0.3)
            cv2.arrowedLine(frame, (int(x), int(y)), (int(x + dx), int(y + dy)), (0, 25, 0), thickness=2, tipLength=0.3)

            totalErrorM1 += m1Dist
            totalErrorM2 += m2Dist
            totalErrorM3 += m3Dist 
            
            
            
            motor1 = -(kp*m1Dist + ki*totalErrorM1 + kd*(lastM1Dist-m1Dist))
            motor2 = -(kp*m2Dist + ki*totalErrorM2 + kd*(lastM2Dist-m2Dist))
            motor3 = -(kp*m3Dist + ki*totalErrorM3 + kd*(lastM3Dist-m3Dist))
            
            lastM1Dist = m1Dist
            lastM2Dist = m2Dist
            lastM3Dist = m3Dist
            
       #     closest = (findClosest(m1Loc,m2Loc, m3Loc, (int(x),int(y))))
            
    '''        
            if "M1" == closest:
                motor1 = -yVector
                motor2 = -xVector - motor1
                motor3 = xVector - motor1
            else:
                if("M3" == closest):
                    motor3 = xVector
                    motor1 = -yVector - motor3 
                    motor2 = yVector - motor3

                else:
                    motor2 = -xVector
                    motor1 = -yVector - motor2
                    motor3 = yVector - motor2
 '''                   
        
        
    cv2.putText(frame,"M1"+str(int(motor1)), m1Loc, cv2.FONT_HERSHEY_SIMPLEX, 1, getSign(motor1), 2, cv2.LINE_AA)
    cv2.putText(frame,"M2"+str(int(motor2)), m2Loc, cv2.FONT_HERSHEY_SIMPLEX, 1, getSign(motor2), 2, cv2.LINE_AA)
    cv2.putText(frame,"M3"+str(int(motor3)), m3Loc, cv2.FONT_HERSHEY_SIMPLEX, 1, getSign(motor3), 2, cv2.LINE_AA)

    
    send_motor_data(motor1,motor2,motor3)
    


    # Display the frame with overlaid contours

    cv2.imshow("Contours Overlay with Matches", frame)

        # Exit on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
