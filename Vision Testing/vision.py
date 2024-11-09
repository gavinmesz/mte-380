import cv2
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

df = pd.read_csv('edge_points.csv')  # Make sure to replace with your file path

known_shape = np.array(df[['x', 'y']].values, dtype=np.int32)

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert to grayscale for easier processing
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Edge detection
    edges = cv2.Canny(blurred, threshold1=50, threshold2=150, apertureSize=3)

    # Find contours in the edge-detected image
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Iterate over the contours to draw them on the frame
    for contour in contours:
        # Draw all contours in blue
        cv2.drawContours(frame, [contour], -1, (255, 0, 0), 2)

        # Approximate the contour to a polygon
        approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)

        # Match the shape with the known shape
        match = cv2.matchShapes(known_shape, approx, cv2.CONTOURS_MATCH_I1, 0.0)

        # If the match score is low enough, consider it a match
        if match < 0.1:  # Adjust this threshold as needed
            # Draw matched contours in green
            cv2.drawContours(frame, [approx], -1, (0, 255, 0), 2)

    # Display the frame with overlaid contours
    cv2.imshow("Contours Overlay with Matches", frame)

    # Exit on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

