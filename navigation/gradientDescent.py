import numpy as np
import matplotlib.pyplot as plt
import math
import random
from wpimath.geometry import Pose2d, Rotation2d

FIELD_X_M = 16.54
FIELD_Y_M = 8.21

class CostMap:
    def __init__(self, grid_size_m:float, end_pose: Pose2d, base_grid:np.ndarray|None=None):
        self.grid_size_m = grid_size_m
        self.end_pose = end_pose
        self.grid_shape = (round(FIELD_Y_M/grid_size_m), round(FIELD_X_M/grid_size_m))
        self.end_point = round(end_pose.Y()/grid_size_m), round(end_pose.X()/grid_size_m)
        if(base_grid is None):
            self.base_grid = self._create_base_cost_map()
        else:
            self.base_grid = base_grid

    def _create_base_cost_map(self) -> np.ndarray:
        # Defines the base cost map
        y_end, x_end = self.end_point
        y, x = np.indices(self.grid_shape)
        grid = np.sqrt((x - x_end) ** 2 + (y - y_end) ** 2)

        # Apply cost of hitting walls
        self._block_walls(grid)

        return grid

    def _block_walls(self, grid: np.ndarray, wall_height: float = 100):
        # Make sure hitting walls is high cost
        grid[0, :] = wall_height       # Top row
        grid[-1, :] = wall_height      # Bottom row
        grid[:, 0] = wall_height       # Left column
        grid[:, -1] = wall_height      # Right column

    def get_copy(self) -> 'CostMap':
        # Returns a new CostMap instance with a copy of the base grid
        base_grid_copy = np.copy(self.base_grid)
        return CostMap(self.grid_size_m, self.end_pose, base_grid_copy)

    def add_cosine_squared_bump(self, peak_center: Pose2d, peak_height: float, peak_radius_m: float):
        y_peak = peak_center.Y()/self.grid_size_m
        x_peak = peak_center.X()/self.grid_size_m
        peak_radius = peak_radius_m/cost_map_copy.grid_size_m

        y_min = max(0,round(y_peak - peak_radius))
        x_min = max(0,round(x_peak - peak_radius))
        y_max = min(round(y_peak + peak_radius),self.grid_shape[0])
        x_max = min(round(x_peak + peak_radius),self.grid_shape[1])


        for y in range(y_min, y_max, 1):
            for x in range(x_min, x_max, 1):
                # Calculate distance from peak center (circular radius)
                distance = np.sqrt((y - y_peak) ** 2 + (x - x_peak) ** 2)
                within_radius = distance < peak_radius
                if within_radius:
                    bump = peak_height * math.cos(math.pi * distance / (2 * peak_radius)) ** 2
                    self.base_grid[y, x] += bump

    def calculate_path(self, start_pose: Pose2d, step_size_m: float = 0.25) -> np.ndarray:
        # Calculate a gradient descent path using the base grid
        start = (round(start_pose.Y()/self.grid_size_m), round(start_pose.X()/self.grid_size_m))
        step_size = step_size_m/self.grid_size_m
        grid_path = self._gradient_descent_on_function(start, step_size)
        grid_path *= self.grid_size_m
        return grid_path

    def _gradient_descent_on_function(self, start: tuple[int, int], step_size: float) -> np.ndarray:

        proximity_threshold = step_size * 1.25

        y, x = start
        y_end, x_end = self.end_point
        
        # Calculate max iterations based on the distance we're traversing
        straightline_x = x_end - x
        straightline_y = y_end - y
        max_iter = int(math.sqrt(straightline_x ** 2 + straightline_y ** 2) / step_size * 20.0)

        path = []
        
        for step in range(max_iter):
            if (x < 0 or x >= self.grid_shape[1] or y < 0 or y >= self.grid_shape[0]):
                raise ValueError("Starting point or end point is out of bounds.")
            
            # Calculate interpolated gradients
            grad_x, grad_y = self._interpolate_gradient(x, y)
            
            grad_mag = np.sqrt(grad_x ** 2 + grad_y ** 2)
            
            delta_x = -1.0 * step_size * grad_x / grad_mag
            delta_y = -1.0 * step_size * grad_y / grad_mag

            # Update positions based on gradient direction
            new_x = x + delta_x
            new_y = y + delta_y
            
            new_x = max(0, min(new_x, self.grid_shape[1] - 1))
            new_y = max(0, min(new_y, self.grid_shape[0] - 1))
            
            new_point = np.array([new_x, new_y])
            distance_to_target = np.sqrt((new_x - x_end) ** 2 + (new_y - y_end) ** 2)

            # Print debug information
            print(f"Step {step}: Distance to target = {distance_to_target:.2f}, Delta = ({delta_x:.2f},{delta_y:.2f})")
            
            if distance_to_target >= proximity_threshold:
                # Too far away, we'll keep going
                path.append(new_point)
                x, y = new_x, new_y
                
            else:
                # Close enough to the endpoint. Append it and get out.
                path.append(np.array([x_end, y_end]))
                break
        
        # If max iterations reached, return path without the final point
        if step == max_iter - 1:
            print("Max iterations reached. Stopping.")

        # Simplify the path
        #return np.array(path)
        return self._resample_path(path, 2.5)

    def _interpolate_gradient(self, x: float, y: float) -> tuple[float, float]:
        # Get the integer parts and the fractional parts
        grid = self.base_grid
        x0, y0 = int(np.floor(x)), int(np.floor(y))
        x1, y1 = min(x0 + 1, grid.shape[1] - 1), min(y0 + 1, grid.shape[0] - 1)
        
        dx = x - x0
        dy = y - y0
        
        # Interpolate the gradients at the four corners
        grad_x00 = grid[y0, x1] - grid[y0, x0]  # Gradient in x-direction at (x0, y0) to (x1, y0)
        grad_x10 = grid[y1, x1] - grid[y1, x0]  # Gradient in x-direction at (x0, y1) to (x1, y1)
        
        grad_y00 = grid[y1, x0] - grid[y0, x0]  # Gradient in y-direction at (x0, y0) to (x0, y1)
        grad_y01 = grid[y1, x1] - grid[y0, x1]  # Gradient in y-direction at (x1, y0) to (x1, y1)
        
        # Interpolate in the x-direction for grad_x
        grad_x0 = self._linear_interpolate(grad_x00, grad_x10, dy)
        
        # Interpolate in the y-direction for grad_y
        grad_y0 = self._linear_interpolate(grad_y00, grad_y01, dx)
        
        # Interpolated gradients at (x, y)
        grad_x = grad_x0
        grad_y = grad_y0
        
        return grad_x, grad_y

    @staticmethod
    def _linear_interpolate(value1: float, value2: float, ratio: float) -> float:
        return value1 * (1 - ratio) + value2 * ratio

    @staticmethod
    def _resample_path(path: list[np.ndarray], min_dist: float) -> np.ndarray:
        smoothed_path = []
        num_points = len(path)

        # First point should always be in the path
        smoothed_path.append(path[0])
        x_last, y_last = path[0]

        # Only place points that are properly spaced out
        for i in range(num_points):
            x_cur, y_cur = path[i]
            delta_x = x_cur - x_last
            delta_y = y_cur - y_last
            dist = math.sqrt(delta_x ** 2 + delta_y ** 2)
            if dist > min_dist:
                smoothed_path.append(path[i])
                x_last, y_last = x_cur, y_cur
        
        # Last point should always be in the path
        smoothed_path.append(path[-1])

        return np.array(smoothed_path)

def plot_results(values: np.ndarray, path: np.ndarray, grid_shape: tuple[int, int], grid_size_m: float):
    global ax1, cid
    grid = np.reshape(values, grid_shape)
    
    ax1.clear()

    ax1.imshow(grid, cmap='viridis', origin='lower')
    ax1.plot(path[:, 0]/grid_size_m, path[:, 1]/grid_size_m, 'r-', marker='o', markersize=5, label='Path')
    ax1.set_title('Gradient Descent Path')
    ax1.legend()
    ax1.set_xlabel('X (grid)')
    ax1.set_ylabel('Y (grid)')

    plt.draw()

def onclick(event):
    global cost_map, cost_map_copy, peak_height, start, path
    
    if event.button == 1:  # Left click
        # Create cosine-squared bump
        obstructionPose = Pose2d(event.xdata*cost_map_copy.grid_size_m, event.ydata*cost_map_copy.grid_size_m, Rotation2d.fromDegrees(0.0))
        cost_map_copy.add_cosine_squared_bump(obstructionPose, 200.0, 1.0)
        
    elif event.button == 3: # Right Click
        # Reset and find a new path
        cost_map_copy = cost_map.get_copy()

    # Calc and Plot the path
    path = cost_map_copy.calculate_path(start)
    plot_results(cost_map_copy.base_grid, path, cost_map_copy.grid_shape, cost_map_copy.grid_size_m)

def main():
    global cost_map, cost_map_copy, start, path, ax1, cid

    end_point = Pose2d.fromFeet(40,20,Rotation2d.fromDegrees(0.0))
    start = Pose2d.fromFeet(2,5,Rotation2d.fromDegrees(0.0))
        
    cost_map = CostMap(0.3, end_point)
    cost_map_copy = cost_map.get_copy()

    # Set up interactive plot
    fig, ax1 = plt.subplots()
    plt.subplots_adjust(bottom=0.2)
    cid = fig.canvas.mpl_connect('button_press_event', onclick)

    # Plot initial result
    path = cost_map_copy.calculate_path(start)
    plot_results(cost_map_copy.base_grid, path, cost_map_copy.grid_shape, cost_map.grid_size_m)
    
    plt.show()

if __name__ == '__main__':
    main()
