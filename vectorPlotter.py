import tkinter as tk
from tkinter import Canvas
import matplotlib.pyplot as plt
import numpy as np
from navigation.repulsorFieldPlanner import RepulsorFieldPlanner
from wpimath.geometry import Pose2d, Translation2d, Rotation2d

class VectorPlotter:
    def __init__(self, master):
        self.master = master
        self.master.title("Vector Field Plotter")
        
        # Create a canvas to draw on
        self.canvas = Canvas(master, width=16.54 * 90, height=8.21 * 90, bg='white')
        self.canvas.pack()

        # Store vectors
        self.vectors = []

    def plot_vector(self, x, y, dx, dy):
        # Store the vector as (x, y, dx, dy)
        self.vectors.append((x, y, dx, dy))
        self.draw_vector(x, y, dx, dy)

    def draw_vector(self, x, y, dx, dy):
        # Draw the vector on the canvas
        self.canvas.create_line(x, y, x + dx, y + dy, arrow=tk.LAST, fill='blue', width=.01)

# Example usage:
if __name__ == "__main__":
    root = tk.Tk()
    vector_plotter = VectorPlotter(root)
    
    rpp = RepulsorFieldPlanner()
    goal = Pose2d(Translation2d(2,2), Rotation2d())
    rpp.setGoal(goal)
    vector_plotter.plot_vector(goal.translation().X() * 90, goal.translation().Y()* 90, 0.2, 0.2)

    XCount = 0
    YCount = 0
    while YCount < 8.21:
        while XCount < 16.54:
            vector_plotter.plot_vector(XCount * 90, YCount * 90, rpp.getForceAtTrans(Translation2d(XCount, YCount)).x / 100, rpp.getForceAtTrans(Translation2d(XCount, YCount)).y / 100)
            XCount += .1
        YCount += .1
        XCount = .1

    # Show the matplotlib plot when the GUI is closed
    root.protocol("WM_DELETE_WINDOW", lambda: root.destroy())
    root.mainloop()