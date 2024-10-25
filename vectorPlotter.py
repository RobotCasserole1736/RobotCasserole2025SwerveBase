
import math
import matplotlib.pyplot as plt
import numpy as np
from navigation.repulsorFieldPlanner import RepulsorFieldPlanner
from navigation.navConstants import GOAL_PICKUP, GOAL_SPEAKER
from wpimath.geometry import Translation2d
import matplotlib.pyplot as plt
import numpy as np
from navigation.navConstants import *

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import cm
from scipy.interpolate import griddata
from scipy.ndimage import gaussian_filter
from scipy.signal import argrelextrema

from utils.constants import FIELD_X_M, FIELD_Y_M

class VectorPlotter:
    def __init__(self, width_meters, height_meters, fig_size=(10, 8), dpi=100):
        """
        Initializes the VectorPlotter.

        Parameters:
        - width_meters (float): Width of the space in meters.
        - height_meters (float): Height of the space in meters.
        - fig_size (tuple): Size of the figure in inches (width, height).
        - dpi (int): Dots per inch for the figure.
        """
        print("Initializing VectorPlotter...")
        self.width_meters = width_meters
        self.height_meters = height_meters
        self.fig_size = fig_size
        self.dpi = dpi
        self.vectors = []  # List to store vectors as tuples
        print(f"Defined space: {self.width_meters}m (width) x {self.height_meters}m (height)")

        # Initialize the plot
        print("Setting up the plot...")
        self.fig, self.ax = plt.subplots(figsize=self.fig_size, dpi=self.dpi)
        self.ax.set_xlim(0, self.width_meters)
        self.ax.set_ylim(0, self.height_meters)
        self.ax.set_aspect('equal')  # Ensure equal scaling for x and y
        self.ax.set_xlabel('X (meters)')
        self.ax.set_ylabel('Y (meters)')
        self.ax.set_title('Vector Plotter with Gradient Lines and Local Minima')

        # Optional: Add grid
        self.ax.grid(True, which='both', linestyle='--', linewidth=0.5)
        print("Plot setup complete.\n")

    def add_vector(self, anchor_x, anchor_y, vector_x, vector_y):
        """
        Adds a vector to the plot.

        Parameters:
        - anchor_x (float): X-coordinate of the anchor point in meters.
        - anchor_y (float): Y-coordinate of the anchor point in meters.
        - vector_x (float): X-component of the vector in arbitrary units.
        - vector_y (float): Y-component of the vector in arbitrary units.
        """
        print(f"Adding vector: Anchor=({anchor_x}, {anchor_y}), Vector=({vector_x}, {vector_y})")
        self.vectors.append((anchor_x, anchor_y, vector_x, vector_y))

    def plot(self):
        """
        Plots all added vectors with color-coding based on magnitude,
        gradient lines, and marks local minima.
        """
        print("\nStarting plot process...")
        if not self.vectors:
            print("No vectors to plot. Exiting plot process.")
            return

        print(f"Number of vectors to plot: {len(self.vectors)}")

        # Extract vector components
        print("Extracting vector components...")
        vectors_np = np.array(self.vectors)
        anchor_x = vectors_np[:, 0].astype(float)
        anchor_y = vectors_np[:, 1].astype(float)
        vector_x = vectors_np[:, 2].astype(float)
        vector_y = vectors_np[:, 3].astype(float)
        print("Extraction complete.")

        # Calculate magnitudes
        print("Calculating vector magnitudes...")
        magnitudes = np.linalg.norm(vectors_np[:, 2:4], axis=1)
        max_magnitude = np.max(magnitudes)
        min_magnitude = np.min(magnitudes)
        print(f"Maximum magnitude: {max_magnitude}")
        print(f"Minimum magnitude: {min_magnitude}")

        if max_magnitude == 0:
            print("All vectors have zero magnitude. Exiting plot process.")
            return

        # Normalize magnitudes for color mapping
        print("Normalizing magnitudes for color mapping...")
        norm = plt.Normalize(vmin=min_magnitude, vmax=max_magnitude) 
        cmap = cm.get_cmap('jet')  # Blue -> Green -> Red
        colors = cmap(norm(magnitudes))
        print("Color normalization complete.")

        # Plot each vector with color based on magnitude
        print("Plotting vectors...")
        for i in range(len(self.vectors)):
            mag = math.sqrt(vector_x[i]**2 + vector_y[i]**2)
            self.ax.arrow(anchor_x[i], anchor_y[i],
                          vector_x[i]/mag*0.2, vector_y[i]/mag*0.2,
                          head_width=0.06, head_length=0.1,
                          fc=colors[i], ec=colors[i],
                          length_includes_head=True)
            if (i + 1) % 10 == 0 or (i + 1) == len(self.vectors):
                print(f"Plotted {i + 1}/{len(self.vectors)} vectors.")
        print("All vectors plotted.")

        # Create a grid for gradient lines (streamlines)
        print("Creating grid for gradient lines...")
        grid_size = 20  # Adjust for resolution
        grid_x, grid_y = np.mgrid[0:self.width_meters:complex(grid_size),
                                  0:self.height_meters:complex(grid_size)]
        print(f"Grid created with size {grid_size}x{grid_size}.")


        # Interpolate vector components onto the grid
        print("Interpolating vector components onto the grid...")
        grid_u = griddata((anchor_x, anchor_y), vector_x, (grid_x, grid_y), method='cubic', fill_value=0)
        grid_v = griddata((anchor_x, anchor_y), vector_y, (grid_x, grid_y), method='cubic', fill_value=0)
        print("Interpolation complete.")


        # Detect local minima
        print("Detecting local minima in vector magnitudes...")
        # Smooth magnitudes for better minima detection
        magnitudes_smooth = gaussian_filter(magnitudes, sigma=1)

        # Find indices of local minima
        local_min_indices = argrelextrema(magnitudes_smooth, np.less)[0]
        print(f"Found {len(local_min_indices)} potential local minima.")

        # Define a threshold to avoid marking trivial minima
        threshold = min_magnitude + (max_magnitude - min_magnitude) * 0.1  # 10% above min
        print(f"Applying threshold: {threshold} to filter minima.")

        # Filter minima based on threshold
        filtered_min_indices = [i for i in local_min_indices if magnitudes[i] <= threshold]
        print(f"{len(filtered_min_indices)} local minima passed the threshold.")

        # Plot local minima
        if filtered_min_indices:
            print("Marking local minima on the plot...")
            minima_x = anchor_x[filtered_min_indices]
            minima_y = anchor_y[filtered_min_indices]
            self.ax.scatter(minima_x, minima_y, color='magenta', marker='X',
                            s=10, label='Local Minima')
            for x, y in zip(minima_x, minima_y):
                self.ax.annotate('', (x, y), textcoords="offset points",
                                  ha='center', color='magenta')
            print("Local minima marked.")
        else:
            print("No significant local minima found.")

        print("Finalizing and displaying the plot...")
        plt.show()
        print("Plot displayed successfully.\n")

    def clear_vectors(self):
        """
        Clears all vectors from the plot.
        """
        print("Clearing all vectors and resetting the plot...")
        self.vectors = []
        self.ax.cla()
        self.ax.set_xlim(0, self.width_meters)
        self.ax.set_ylim(0, self.height_meters)
        self.ax.set_aspect('equal')
        self.ax.set_xlabel('X (meters)')
        self.ax.set_ylabel('Y (meters)')
        self.ax.set_title('Vector Plotter with Gradient Lines and Local Minima')
        self.ax.grid(True, which='both', linestyle='--', linewidth=0.5)
        print("Plot reset complete.\n")

# Example Usage
if __name__ == "__main__":
    print("=== Vector Plotter Program Started ===\n")
    
    # Define the space dimensions in meters
    width = FIELD_X_M  # meters
    height = FIELD_Y_M  # meters

    # Create a VectorPlotter instance
    print("\nCreating VectorPlotter instance...")
    plotter = VectorPlotter(width_meters=width, height_meters=height,
                            fig_size=(12, 9), dpi=100)
    print("VectorPlotter instance created.\n")

    # Add vectors: add_vector(anchor_x, anchor_y, vector_x, vector_y)
    print("Adding vectors...")
    rpp = RepulsorFieldPlanner()
    rpp.setGoal(GOAL_PICKUP)

    XCount = 0.1
    YCount = 0.1
    while YCount < 8.1:
        while XCount < 16.4:
            force = rpp._getForceAtTrans(Translation2d(XCount, YCount))
            plotter.add_vector(XCount, YCount, force.x, force.y)
            XCount += .2
        YCount += .2
        XCount = 0.1

    print("All vectors added.\n")

    # Plot the vectors along with gradient lines and local minima
    print("Initiating plotting of vectors...")
    plotter.plot()
    print("=== Vector Plotter Program Finished ===")



