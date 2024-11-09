import cv2
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

df = pd.read_csv('edge_points.csv')  

known_shape = np.array(df[['x', 'y']].values, dtype=np.int32)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # Adjust width as needed
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480) # Adjust height as needed

kpx = 1
kix = 0
kdx = 0

kpy = 1
kiy = 0
kdy = 0

totalErrorX = 0
totalErrorY = 0

lastX = 0
lastY = 0


while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert the frame from BGR to HSV color space to easily identify a colour
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 

    # Apply Gaussian blur
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Edge detection
    edges = cv2.Canny(blurred, threshold1=50, threshold2=150, apertureSize=3)

    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
   
    if contours:
        # Determines the larget contour size using the cv.contour Area function
        largest_contour = max(contours, key=cv2.contourArea)
        epsilon = 0.01 * cv2.arcLength(largest_contour, True)
        smooth_contour = cv2.approxPolyDP(largest_contour, epsilon, True)

        # Draw the smoothed contour
       # cv2.drawContours(frame, [smooth_contour], -1, (0, 255, 0), thickness=2)
        cv2.drawContours(frame, [largest_contour], -1, (0, 0, 255), thickness=2)

        #cv2.drawContours(frame, largest_contour, -1, (255, 0, 0), 2)
        ((midX, midY), radius) = cv2.minEnclosingCircle(largest_contour)
        cv2.circle(frame, (int(midX), int(midY)), 2, (0, 0, 255), -1)  # (image to draw dot on, x,y pixel coordinates, radius in pixels, RGB values in this case red, -1 indicates to fill the circle)

   
    # *3 Define the range of yellow color in HSV [Hue, Saturation, Value]
    # SET THESE VALUES VIA THE METHOD EXPLAINED IN THE TUTORIAL
    ball_color_lower = np.array([10, 100, 100]) # [lower Hue, lower Saturation, lower Value]
    ball_color_upper = np.array([50, 255, 255]) # [upper Hue, upper Saturation, upper Value]


    # Threshold the HSV image to get the colors defined above
    # Pixels in the range are set to white (255) and those that aren't are set to black (0), creating a binary mask 
    mask = cv2.inRange(gray, ball_color_lower, ball_color_upper)

    # Apply Gaussian blur
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Edge detection
    edges = cv2.Canny(blurred, threshold1=50, threshold2=150, apertureSize=3)

    # Find contours in the edge-detected image
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Threshold the HSV image to get the colors defined above
    # Pixels in the range are set to white (255) and those that aren't are set to black (0), creating a binary mask 
    mask = cv2.inRange(gray, ball_color_lower, ball_color_upper)
    

    # Find contours in the mask
    # RETR_TREE retrieves all hierarchical contours and organizes them
    # CHAIN_APPROX_SIMPLE compresses horizontal, vertical, and diagonal segments, leaving only their end points
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Find the index of the largest contour
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
            
            xVector = -(kpx*errorX + kix*totalErrorX + kdx*dx)
            yVector = -(kpy*errorY + kiy*totalErrorY + kdy*dy)
            
            print(f"({xVector}, {yVector})")
            cv2.arrowedLine(frame, (int(x), int(y)), (int(x + xVector), int(y + yVector)), (255, 0, 0), thickness=2, tipLength=0.3)
            cv2.arrowedLine(frame, (int(x), int(y)), (int(x + dx), int(y + dy)), (0, 25, 0), thickness=2, tipLength=0.3)



    # Display the frame with overlaid contours

    cv2.imshow("Contours Overlay with Matches", frame)

    # Exit on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

