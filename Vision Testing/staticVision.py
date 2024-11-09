import cv2
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Load the input image and the known shape image (template)
input_image = cv2.imread('IMG_1003.jpg')  # Replace with your input image path
height, width = input_image.shape[:2]
new_width = int(width * 0.15)  # Resize to 50% of the original width
new_height = int(height * 0.15)  # Resize to 50% of the original height
input_image = cv2.resize(input_image, (new_width, new_height))

df = pd.read_csv('edge_points.csv')  # Make sure to replace with your file path
known_shape_contour  = np.array(df[['x', 'y']].values, dtype=np.int32)

known_shape_center = np.mean(known_shape_contour, axis=0)

# Recenter the known shape points to make the center (0, 0)
recentered_known_shape_points = known_shape_contour - known_shape_center

# Plot the recentered shape points (in red)



# Convert both images to grayscale
# Convert the input image to grayscale
gray_input = cv2.cvtColor(input_image, cv2.COLOR_BGR2GRAY)

blurred_gray_input = cv2.GaussianBlur(gray_input, (7, 7), 0)  # (5, 5) is the kernel size


# Apply Canny edge detection
edges_input = cv2.Canny(blurred_gray_input, threshold1=10, threshold2=150)

# Apply closing operation to close any gaps in the edges
kernel = np.ones((7, 7), np.uint8)  # Adjust the kernel size for the closing operation
closed_edges_input = cv2.morphologyEx(edges_input, cv2.MORPH_CLOSE, kernel)

# Find contours in the input image
contours_input, _ = cv2.findContours(closed_edges_input, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

# Function to match contours from the input image to the known shape
def match_contours(contours_input, known_shape_contour):
    matched_contours = []
    for contour_input in contours_input:
        # Compare the contour with the known shape contour using cv2.matchShapes()
        similarity = cv2.matchShapes(contour_input, known_shape_contour, cv2.CONTOURS_MATCH_I2, 0.0)
        
        # If the similarity is below a threshold, consider it a match (adjust as needed)
        if similarity < 0.1:  # You can adjust this threshold
            matched_contours.append(contour_input)

    return matched_contours

# Match the contours in the resized input image to the known shape
matched_contours = match_contours(contours_input, known_shape_contour)

# Draw the contours from the input image in blue
cv2.drawContours(input_image, contours_input, -1, (255, 0, 0), 2)

# Draw the matched contours in green
cv2.drawContours(input_image, matched_contours, -1, (0, 255, 0), 2)

# Overlay the known shape in red at the centroid of the matched contours
for contour in matched_contours:
    # Calculate the moments of the contour to find the centroid
    M = cv2.moments(contour)
    if M["m00"] != 0:  # Check to avoid division by zero
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        # Translate the recentered known shape points to the centroid position
        translated_shape = recentered_known_shape_points + np.array([cX, cY])

        print(translated_shape)
        input("hm") 
        
        plt.plot(translated_shape[:, 0], translated_shape[:, 1], 'ro-', label="Recentered Shape")

        # Add labels and title
        plt.title('Original vs. Recentered Known Shape Points')
        plt.xlabel('X')
        plt.ylabel('Y')

        # Show the legend
        plt.show()

        # Check if translated_shape has valid points
        if translated_shape.shape[0] > 0:  # Ensure there are points to draw
            # Draw the translated known shape in red
            cv2.drawContours(input_image, [translated_shape.reshape((-1, 1, 2))], -1, (0, 0, 255), 2)
        else:
            print("Warning: Translated shape has no points to draw.")



# Wait until a key is pressed
cv2.waitKey(0)

# Destroy all OpenCV windows
cv2.destroyAllWindows()
