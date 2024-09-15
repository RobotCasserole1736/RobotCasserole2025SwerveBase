import numpy as np
import matplotlib.pyplot as plt
import math
import random

class CostMap:
    def __init__(self, grid_shape: tuple[int, int], end_point: tuple[int, int]):
        self.grid_shape = grid_shape
        self.end_point = end_point
        self.base_grid = self.create_base_cost_map()

    def create_base_cost_map(self) -> np.ndarray:
        # Defines the base cost map
        y_end, x_end = self.end_point
        y, x = np.indices(self.grid_shape)
        grid = np.sqrt((x - x_end) ** 2 + (y - y_end) ** 2)

        # Apply cost of hitting walls
        self.block_walls(grid)

        return grid

    def block_walls(self, grid: np.ndarray, wall_height: float = 100):
        # Make sure hitting walls is high cost
        grid[0, :] = wall_height       # Top row
        grid[-1, :] = wall_height      # Bottom row
        grid[:, 0] = wall_height       # Left column
        grid[:, -1] = wall_height      # Right column

    def get_copy(self) -> 'CostMap':
        # Returns a new CostMap instance with a copy of the base grid
        new_cost_map = CostMap(self.grid_shape, self.end_point)
        new_cost_map.base_grid = np.copy(self.base_grid)
        return new_cost_map

    def add_cosine_squared_bump(self, peak_center: tuple[int, int], peak_height: float, peak_radius: float):
        y_peak, x_peak = peak_center  

        y_min = round(y_peak - peak_radius)
        x_min = round(x_peak - peak_radius)
        y_max = round(y_peak + peak_radius)
        x_max = round(x_peak + peak_radius)

        for y in range(y_min, y_max, 1):
            for x in range(x_min, x_max, 1):
                # Calculate distance from peak center (circular radius)
                distance = np.sqrt((y - y_peak) ** 2 + (x - x_peak) ** 2)
                within_radius = distance < peak_radius
                if within_radius:
                    bump = peak_height * math.cos(math.pi * distance / (2 * peak_radius)) ** 2
                    self.base_grid[y, x] += bump

    def calculate_path(self, start: tuple[int, int], step_size: float = 0.75) -> np.ndarray:
        # Calculate a gradient descent path using the base grid
        return self.gradient_descent_on_function(start, step_size)

    def gradient_descent_on_function(self, start: tuple[int, int], step_size: float = 0.75) -> np.ndarray:
        grid = self.base_grid

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
            grad_x, grad_y = self.interpolate_gradient(x, y)
            
            grad_mag = np.sqrt(grad_x ** 2 + grad_y ** 2)

            # Add a small factor of randomness to the next step choice to help avoid cases of getting stuck in cycles.
            random_perturbation = 0.1
            
            delta_x = -1.0 * step_size * grad_x / grad_mag + random.uniform(step_size * -random_perturbation, step_size * random_perturbation)
            delta_y = -1.0 * step_size * grad_y / grad_mag + random.uniform(step_size * -random_perturbation, step_size * random_perturbation)

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
        return self.resample_path(path, 3.0)

    def interpolate_gradient(self, x: float, y: float) -> tuple[float, float]:
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
        grad_x0 = self.linear_interpolate(grad_x00, grad_x10, dy)
        
        # Interpolate in the y-direction for grad_y
        grad_y0 = self.linear_interpolate(grad_y00, grad_y01, dx)
        
        # Interpolated gradients at (x, y)
        grad_x = grad_x0
        grad_y = grad_y0
        
        return grad_x, grad_y

    @staticmethod
    def linear_interpolate(value1: float, value2: float, ratio: float) -> float:
        return value1 * (1 - ratio) + value2 * ratio

    @staticmethod
    def resample_path(path: list[np.ndarray], min_dist: float) -> np.ndarray:
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

def plot_results(values: np.ndarray, path: np.ndarray, grid_shape: tuple[int, int]):
    global ax1, cid
    grid = np.reshape(values, grid_shape)
    
    ax1.clear()

    ax1.imshow(grid.T, cmap='viridis', origin='lower', extent=[0, 54, 0, 27])
    ax1.plot(path[:, 1], path[:, 0], 'r-', marker='o', markersize=5, label='Path')
    ax1.set_title('Gradient Descent Path')
    ax1.legend()
    ax1.set_xlabel('X (feet)')
    ax1.set_ylabel('Y (feet)')

    plt.draw()

def onclick(event):
    global cost_map, cost_map_copy, peak_radius, peak_height, start, path
    
    # Create cosine-squared bump
    cost_map.add_cosine_squared_bump((round(event.xdata), round(event.ydata)), peak_height, peak_radius)
    
    # Reset and find a new path
    cost_map_copy = cost_map.get_copy()
    path = cost_map_copy.calculate_path(start)
    
    # Plot the path
    plot_results(cost_map_copy.base_grid, path, cost_map.grid_shape)

def main():
    global cost_map, cost_map_copy, peak_radius, peak_height, start, path, ax1, cid

    grid_shape = (54, 27)
    end_point = (40, 20)
    start = (5, 2)
    
    peak_radius = 3.0
    peak_height = 30.0
    
    cost_map = CostMap(grid_shape, end_point)
    cost_map_copy = cost_map.get_copy()

    # Set up interactive plot
    fig, ax1 = plt.subplots()
    plt.subplots_adjust(bottom=0.2)
    cid = fig.canvas.mpl_connect('button_press_event', onclick)

    # Plot initial result
    path = cost_map_copy.calculate_path(start)
    plot_results(cost_map.base_grid, path, cost_map.grid_shape)
    
    plt.show()

if __name__ == '__main__':
    main()
