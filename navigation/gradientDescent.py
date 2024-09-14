import numpy as np
import matplotlib.pyplot as plt
import math

def create_base_cost_map(grid_shape, end_point):
    #defines the base cost map.

    # Points further from the goal are costly
    # This should, by default, "Funnel" all paths toward the goal
    x_end, y_end = end_point
    y, x = np.indices(grid_shape)
    grid = np.sqrt((x - x_end)**2 + (y - y_end)**2) 

    # Apply cost of hitting walls
    block_walls(grid)

    return grid

def block_walls(grid, wall_height=100):
    # Make sure hitting walls is high cost
    grid[0, :] = wall_height       # Top row
    grid[-1, :] = wall_height      # Bottom row
    grid[:, 0] = wall_height       # Left column
    grid[:, -1] = wall_height      # Right column

def add_cosine_squared_bump(grid, peak_center, peak_height, peak_radius):
    x_peak, y_peak = peak_center
    y, x = np.indices(grid.shape)
    
    # Calculate distance from peak center (circular radius)
    distance = np.sqrt((x - x_peak)**2 + (y - y_peak)**2)
    within_radius = distance < peak_radius
    
    # Apply the cosine squared bump function within the circular region
    bump = np.where(
        within_radius, 
        peak_height * np.maximum(np.cos(np.pi * distance / (2 * peak_radius))**2, 0), 
        0
    )
    
    # Add bump to the grid
    grid += bump
    return grid

def add_hard_stop_circle(grid, peak_center, peak_radius):
    x_peak, y_peak = peak_center
    y, x = np.indices(grid.shape)
    
    # Calculate distance from peak center (circular radius)
    distance = np.sqrt((x - x_peak)**2 + (y - y_peak)**2)
    within_radius = distance < peak_radius
    
    # Apply the cosine squared bump function within the circular region
    bump = np.where(
        within_radius, 
        100.0,
        0
    )
    
    # Add bump to the grid
    grid += bump
    return grid


def linear_interpolate(value1, value2, ratio):
    return value1 * (1 - ratio) + value2 * ratio

def interpolate_gradient(grid, x, y):
    # Get the integer parts and the fractional parts
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
    grad_x0 = linear_interpolate(grad_x00, grad_x10, dy)
    
    # Interpolate in the y-direction for grad_y
    grad_y0 = linear_interpolate(grad_y00, grad_y01, dx)
    
    # Interpolated gradients at (x, y)
    grad_x = grad_x0
    grad_y = grad_y0
    
    return grad_x, grad_y

## TODO - this does not work yet.
def detect_backtracking(path, step_size, num_points=8):
    if len(path) < num_points:
        return False
    
    dist_range_thresh = step_size * 1.5

    most_recent_points = np.array(path[-num_points:])
    print("=============")
    
    # Compute pairwise distances between the most recent points
    distances = []
    for i in range(num_points):
        for j in range(i + 1, num_points):
            delta = np.linalg.norm(most_recent_points[i] - most_recent_points[j])
            distances.append(delta)
    
    # Calculate the maximum distance between any two points
    maxDist = np.max(distances)
    print(f"MaxDistances={maxDist}")

    # Check if the points are not spaced out enough
    if maxDist < dist_range_thresh:
        return True
    return False

def gradient_descent_on_function(values, start, end, grid_shape, step_size=0.75):
    grid = np.reshape(values, grid_shape)

    proximity_threshold = step_size * 1.25

    x, y = start
    x_end, y_end = end
    
    # Calcualte a max iterations based on the distance we're traversing
    straightline_x = x_end - x
    straightline_y = y_end - y
    max_iter = int( math.sqrt(straightline_x**2 + straightline_y**2) / step_size * 10.0)


    path = []
    
    for step in range(max_iter):
        if (x < 0 or x >= grid_shape[1] or y < 0 or y >= grid_shape[0]):
            raise ValueError("Starting point or end point is out of bounds.")
        
        # Calculate interpolated gradients
        grad_x, grad_y = interpolate_gradient(grid, x, y)
        
        grad_mag = np.sqrt(grad_x**2 + grad_y**2)
        
        delta_x = -1.0 * step_size * grad_x / grad_mag
        delta_y = -1.0 * step_size * grad_y / grad_mag

        # Update positions based on gradient direction
        new_x = x + delta_x
        new_y = y + delta_y
        
        new_x = max(0, min(new_x, grid_shape[1]-1))
        new_y = max(0, min(new_y, grid_shape[0]-1))
        
        new_point = np.array([new_x, new_y])
        distance_to_target = np.sqrt((new_x - x_end)**2 + (new_y - y_end)**2)

        # Print debug information
        print(f"Step {step}: Distance to target = {distance_to_target:.2f}, Delta = ({delta_x:.2f},{delta_y:.2f})")
        
        if distance_to_target >= proximity_threshold:
            # Too far away, we'll keep going
            path.append(new_point)
        
            # Check for backtracking (four points within 2 step sizes)
            # if detect_backtracking(path, step_size):
            #     print(f"Backtracking detected at step {step}. Stopping.")
            #     return np.array(path[:-1])  # Return path without final backtracking point
                
            # Move to the new point
            x, y = new_x, new_y
            
        else:
            # Close enough to the endpoint. append it and get out.
            path.append(np.array([x_end, y_end]))
            break
    

    # If max iterations reached, return path without the final point
    if(step == max_iter-1):
        print("Max iterations reached. Stopping.")

    # Simplyify the path
    return resample_path(path, 3.0)

def resample_path(path, min_dist):

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
        dist = math.sqrt(delta_x**2 + delta_y**2)
        if(dist > min_dist):
            smoothed_path.append(path[i])
            x_last, y_last = x_cur, y_cur
    
    #Last point should always be in the path
    smoothed_path.append(path[-1])

    return np.array(smoothed_path)

def plot_results(values, path, grid_shape):
    global ax1, ax2, cid
    grid = np.reshape(values, grid_shape)
    
    ax1.clear()
    ax2.clear()

    ax1.imshow(grid, cmap='viridis', origin='lower', extent=[0, 54, 0, 27])
    ax1.set_title('Function Values')
    ax1.set_xlabel('X (feet)')
    ax1.set_ylabel('Y (feet)')
    
    ax2.imshow(grid, cmap='viridis', origin='lower', extent=[0, 54, 0, 27])
    ax2.plot(path[:, 0], path[:, 1], 'r-', marker='o', markersize=5, label='Path')
    ax2.set_title('Gradient Descent Path')
    ax2.legend()
    ax2.set_xlabel('X (feet)')
    ax2.set_ylabel('Y (feet)')

    plt.draw()

def onclick(event):
    global values, peak_radius, peak_height, end_point, start, path, grid_shape
    
    # Ignore clicks outside the axes
    if event.inaxes not in [ax1, ax2]:
        return
    
    x_click, y_click = event.xdata, event.ydata
    
    if event.button == 1:  # Left click
        print(f"Adding peak at ({x_click:.2f}, {y_click:.2f})")
        # Add cosine squared bump at clicked point
        values = add_cosine_squared_bump(values, (int(x_click), int(y_click)), peak_height, peak_radius)
        #values = add_hard_stop_circle(values, (int(x_click), int(y_click)), peak_radius)
        
        # Recompute the gradient descent path after adding the peak
        path = gradient_descent_on_function(values, start, end_point, grid_shape)
        plot_results(values, path, grid_shape)
        
    elif event.button == 3:  # Right click
        print("Resetting to initial funnel profile")
        # Reset to initial funnel profile
        values = create_base_cost_map(grid_shape, end_point)
        
        # Recompute the gradient descent path with the reset profile
        path = gradient_descent_on_function(values, start, end_point, grid_shape)
        plot_results(values, path, grid_shape)

# Global variables to track the state
peak_radius = 4
peak_height = 50
start = (10, 5)  # Starting point
end_point = (40, 20)  # End point
grid_shape = (27, 54)  # Y-axis is 27 units tall, X-axis is 54 units wide

# Create initial funnel
values = create_base_cost_map(grid_shape, end_point)

# Compute initial path
path = gradient_descent_on_function(values, start, end_point, grid_shape)

# Set up the plot
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))
plot_results(values, path, grid_shape)

# Connect the click event
cid = fig.canvas.mpl_connect('button_press_event', onclick)

plt.show()
