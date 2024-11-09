import ezdxf
import numpy as np
import csv
import pandas as pd
from scipy.spatial import ConvexHull

# Open a CSV file to write the coordinates
with open('coordinates.csv', mode='w', newline='') as csvfile:
    fieldnames = ['x', 'y']
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

    # Write the header to the CSV
    writer.writeheader()

    # Load the DXF file
    doc = ezdxf.readfile("ME380_001_Platform.dxf")
    msp = doc.modelspace()

    # Extract and process entities
    # Extract and process entities
    for entity in msp:
        if entity.dxftype() == "LINE":
            start, end = entity.dxf.start, entity.dxf.end
            # Write coordinates of the line's start and end points
            writer.writerow({'x': start.x, 'y': start.y})
            writer.writerow({'x': end.x, 'y': end.y})
        
        elif entity.dxftype() == "CIRCLE":
            center = entity.dxf.center
            radius = entity.dxf.radius
            # Write the center of the circle (or you can sample points)
            writer.writerow({'x': center.x, 'y': center.y})

        elif entity.dxftype() == "ARC":
            # Arc data
            center = entity.dxf.center
            radius = entity.dxf.radius
            start_angle = np.radians(entity.dxf.start_angle)  # Convert to radians
            end_angle = np.radians(entity.dxf.end_angle)      # Convert to radians

            # Handle counterclockwise arcs by adjusting the end angle if it's smaller than the start angle
            if end_angle < start_angle:
                end_angle += 2 * np.pi  # Wrap around for counterclockwise arcs

            # Generate points along the arc
            angles = np.linspace(start_angle, end_angle, num=100)  # Adjust num for smoothness
            for angle in angles:
                x = center.x + radius * np.cos(angle)
                y = center.y + radius * np.sin(angle)
                writer.writerow({'x': x, 'y': y})

        elif entity.dxftype() == "SPLINE":
            # Spline data (handle both control and fit points)
            spline_points = entity.control_points  # Access control points

            # Write each control point to the CSV
            for point in spline_points:
                writer.writerow({'x': point[0], 'y': point[1]})

print("Coordinates have been written to 'coordinates.csv'.")

# Load the CSV file into a DataFrame
df = pd.read_csv('coordinates.csv')

# Assuming the points are in columns named 'x' and 'y'
points = df[['x', 'y']].values

# Compute the convex hull
hull = ConvexHull(points)

# Get the indices of the points that are on the hull
edge_points_indices = hull.vertices

# Filter the original DataFrame to only include edge points
edge_points = df.iloc[edge_points_indices]

# Save the edge points to a new CSV file
edge_points.to_csv('edge_points.csv', index=False)

print("Filtered edge points saved to edge_points.csv")