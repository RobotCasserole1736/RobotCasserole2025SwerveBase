import tkinter as tk
from typing import Tuple
from navigation.forceGenerators import Lane, LinearForceGenerator, PointObstacle
from navigation.navForce import Force
from utils.constants import *
from navigation.repulsorFieldPlanner import RepulsorFieldPlanner
from navigation.navConstants import *
from wpimath.geometry import Translation2d

class ScaledCanvas:
    def __init__(self, root: tk.Tk, width_m: float, height_m: float):
        self.canvas = tk.Canvas(root, bg="white", width=1400, height=800)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.vizMargin_m = 1.0
        self.width_m = width_m + 2.0 * self.vizMargin_m
        self.height_m = height_m + 2.0 * self.vizMargin_m
        self.scale = 1.0
        self.offset_x = 0
        self.offset_y = 0
        self.objects = []

        # Recalculate scale on resize
        self.canvas.bind("<Configure>", self._on_resize)

    def _on_resize(self, event):
        # Calculate scale to maintain aspect ratio
        scale_width = event.width / self.width_m
        scale_height = event.height / self.height_m
        self.scale = min(scale_width, scale_height)

        # Calculate offsets to center the drawing
        self.offset_x = (event.width - self.scale * self.width_m) / 2
        self.offset_y = (event.height - self.scale * self.height_m) / 2

        self.canvas.delete("all")
        for obj in self.objects:
            obj()

    def add_rectangle(self, x_m: float, y_m: float, width_m: float, height_m: float, color: str = "black"):
        def draw():
            x1, y1 = self._to_pixels(x_m, y_m)
            x2, y2 = self._to_pixels(x_m + width_m, y_m + height_m)
            self.canvas.create_rectangle(x1, y1, x2, y2, outline=color, width=2)

        self.objects.append(draw)
        draw()

    def add_arrow(self, start_m: Tuple[float, float], end_m: Tuple[float, float], color: str = "red"):
        def draw():
            x1, y1 = self._to_pixels(*start_m)
            x2, y2 = self._to_pixels(*end_m)
            self.canvas.create_line(x1, y1, x2, y2, arrow=tk.LAST, fill=color, width=2)

        self.objects.append(draw)
        draw()

    def add_line(self, start_m: Tuple[float, float], end_m: Tuple[float, float], color: str = "red"):
        def draw():
            x1, y1 = self._to_pixels(*start_m)
            x2, y2 = self._to_pixels(*end_m)
            self.canvas.create_line(x1, y1, x2, y2, fill=color, width=3)

        self.objects.append(draw)
        draw()

    def add_circle(self, location_m: Tuple[float, float], radius_m:float,  color: str = "red"):
        def draw():
            x1, y1 = self._to_pixels(location_m[0]-radius_m, location_m[1]-radius_m)
            x2, y2 = self._to_pixels(location_m[0]+radius_m, location_m[1]+radius_m)
            self.canvas.create_oval(x1,y1,x2,y2,outline=color, width=3)

        self.objects.append(draw)
        draw()

    def _to_pixels(self, x_m: float, y_m: float) -> Tuple[int, int]:
        """
        Apply margin offset, rotate to wpi visiual conventions
        """
        return (
            int( (x_m + self.vizMargin_m) * self.scale + self.offset_x),
            int( (self.height_m - y_m - self.vizMargin_m) * self.scale + self.offset_y),
        )

# Create the main Tkinter application
def main():
    arrowSpacing_m = 0.15

    root = tk.Tk()
    root.title("Potential Fields")
    width_m, height_m = FIELD_X_M, FIELD_Y_M # Visualization dimensions in meters
    canvas = ScaledCanvas(root, width_m, height_m)

    # Set up the actual repulsor field planner
    rpf = RepulsorFieldPlanner()
    rpf.setGoal(GOAL_SPEAKER)

    # Field boundary
    canvas.add_rectangle(0.0, 0.0, FIELD_X_M, FIELD_Y_M, color="black")

    # Sample Points
    num_x_samples = int(FIELD_X_M / arrowSpacing_m + 1)
    num_y_samples = int(FIELD_Y_M / arrowSpacing_m + 1)
    forces: dict[tuple[int, int], Force]= {}


    # Add some arrows
    for x_idx in range(num_x_samples):
        for y_idx in range(num_y_samples):
            x_pos = x_idx * arrowSpacing_m
            y_pos = y_idx * arrowSpacing_m
            force = rpf._getForceAtTrans(Translation2d(x_pos, y_pos))

            forces[(x_idx,y_idx)] = force

            start = (x_pos, y_pos)
            end = (start[0] + force.unitX() * arrowSpacing_m * 0.75, start[1] + force.unitY() * arrowSpacing_m * 0.75)
            canvas.add_arrow(start, end, color="green")

    # Plot Obstacles
    for obs in rpf.fixedObstacles:
        if(isinstance(obs, PointObstacle)):
            canvas.add_circle((obs.location.X(), obs.location.Y()), obs.radius, color="grey")
        elif(isinstance(obs,LinearForceGenerator)):
            if(isinstance(obs, Lane)):
                linecolor="yellow"
            else:
                linecolor = "grey"
            canvas.add_line((obs.start.X(), obs.start.Y()), (obs.end.X(), obs.end.Y()), color=linecolor)

    # Exhaustively detect local minima
    DELTA_M = arrowSpacing_m/2.0
    for x_idx in range(num_x_samples):
        for y_idx in range(num_y_samples):

            if(x_idx == 0 or y_idx == 0 or x_idx == num_x_samples-1 or y_idx == num_y_samples-1):
                # boundary
                continue
            
            x_pos = x_idx * arrowSpacing_m
            y_pos = y_idx * arrowSpacing_m

            # Look for minima in X
            forcePrev = rpf._getForceAtTrans(Translation2d(x_pos - DELTA_M, y_pos))
            forceNext = rpf._getForceAtTrans(Translation2d(x_pos + DELTA_M, y_pos))
            xHasMinima = (forcePrev.x > 0 and forceNext.x < 0)

            # Look for minima in Y
            forcePrev = rpf._getForceAtTrans(Translation2d(x_pos, y_pos - DELTA_M))
            forceNext = rpf._getForceAtTrans(Translation2d(x_pos, y_pos + DELTA_M))
            yHasMinima = (forcePrev.y > 0 and forceNext.y < 0)

            if(yHasMinima and xHasMinima):
                canvas.add_circle((x_pos, y_pos), arrowSpacing_m/8.0)

    root.mainloop()

if __name__ == "__main__":
    main()
