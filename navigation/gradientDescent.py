import numpy as np
import matplotlib.pyplot as plt

def create_funnel(grid_shape, end_point):
    x_end, y_end = end_point
    y, x = np.indices(grid_shape)
    funnel = np.sqrt((x - x_end)**2 + (y - y_end)**2)
    return funnel

def add_cosine_bump(grid, peak_center, peak_height, peak_radius):
    x_peak, y_peak = peak_center
    y, x = np.indices(grid.shape)
    
    # Define the square region
    within_radius = (np.abs(x - x_peak) <= peak_radius) & (np.abs(y - y_peak) <= peak_radius)
    
    # Calculate distance from peak center only within the square
    distance = np.sqrt((x - x_peak)**2 + (y - y_peak)**2)
    
    # Apply the cosine bump function only within the square region
    bump = np.where(within_radius, peak_height * np.maximum(np.cos(np.pi * distance / peak_radius), 0), 0)
    
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

def gradient_descent_on_function(values, start, end, grid_shape, step_size=1.0, proximity_threshold=1.5, max_iter=10000):
    grid = np.reshape(values, grid_shape)

    x, y = start
    x_end, y_end = end
    
    path = []
    last_point = np.array([x, y])
    
    for step in range(max_iter):
        if (x < 0 or x >= grid_shape[1] or y < 0 or y >= grid_shape[0]):
            raise ValueError("Starting point or end point is out of bounds.")
        
        # Calculate interpolated gradients
        grad_x, grad_y = interpolate_gradient(grid, x, y)
        
        grad_mag = np.sqrt(grad_x**2 + grad_y**2)
        
        # Update positions based on gradient direction
        new_x = x - step_size * grad_x / grad_mag
        new_y = y - step_size * grad_y / grad_mag
        
        new_x = max(0, min(new_x, grid_shape[1]-1))
        new_y = max(0, min(new_y, grid_shape[0]-1))
        
        new_point = np.array([new_x, new_y])
        height_delta = grid[int(new_y), int(new_x)] - grid[int(y), int(x)]
        distance_to_target = np.sqrt((new_x - x_end)**2 + (new_y - y_end)**2)

        # Print debug information
        print(f"Step {step}: Distance to target = {distance_to_target:.2f}, Height delta = {height_delta:.2f}")
        
        # Append points to path if distance is more than proximity threshold
        if np.linalg.norm(new_point - last_point) >= proximity_threshold:
            path.append(new_point)
            last_point = new_point
        
        # Check if close enough to the endpoint
        if distance_to_target < proximity_threshold:
            path.append(np.array([x_end, y_end]))
            break
        
        # Move to the new point
        x, y = new_x, new_y
    
    # Append the endpoint if it wasn't added in the loop
    if len(path) == 0 or np.linalg.norm(path[-1] - np.array([x_end, y_end])) >= proximity_threshold:
        path.append(np.array([x_end, y_end]))
    
    return np.array(path), grid

def plot_results(values, path, grid_shape):
    grid = np.reshape(values, grid_shape)
    
    plt.figure(figsize=(12, 6))
    
    plt.subplot(1, 2, 1)
    plt.imshow(grid, cmap='viridis', origin='lower', extent=[0, 54, 0, 27])
    plt.colorbar(label='Function value')
    plt.title('Function Values')
    
    plt.subplot(1, 2, 2)
    plt.imshow(grid, cmap='viridis', origin='lower', extent=[0, 54, 0, 27])
    plt.colorbar(label='Function value')
    plt.plot(path[:, 0], path[:, 1], 'r-', marker='o', markersize=5, label='Path')
    plt.title('Gradient Descent Path')
    plt.legend()
    
    plt.show()

# Example usage
grid_shape = (27, 54)  # Y-axis is 27 units tall, X-axis is 54 units wide
end_point = (40, 20)  # This is in (x, y) format
values = create_funnel(grid_shape, end_point)

# Add a cosine-shaped bump within a square region
peak_center = (15, 15)  # This is in (x, y) format
peak_height =10  # Peak height 10 times its original
peak_radius = 15
values = add_cosine_bump(values, peak_center, peak_height, peak_radius)

peak_center = (27,5)
values = add_cosine_bump(values, peak_center, peak_height, peak_radius)


# Define start point
start = (1, 1)  # This is in (x, y) format

# Perform gradient descent with the proximity threshold
path, grid = gradient_descent_on_function(values, start, end_point, grid_shape, step_size=0.5, proximity_threshold=1.5)
plot_results(values, path, grid_shape)
