from cscore import CameraServer, VideoMode
from typing import TYPE_CHECKING
import numpy as np
from navigation.navConstants import FIELD_X_M, FIELD_Y_M
from wpimath.geometry import Translation2d

if TYPE_CHECKING:
    from navigation.repulsorFieldPlanner import RepulsorFieldPlanner 


_M_PER_GRID = 1
_PX_PER_GRID = 20
GRID_SIZE_X = int(FIELD_X_M / _M_PER_GRID)
GRID_SIZE_Y = int(FIELD_Y_M / _M_PER_GRID)

class CostMapTelemetry:
    def __init__(self, name):
        self.imgWidth = GRID_SIZE_X*_PX_PER_GRID
        self.imgHeight = GRID_SIZE_Y*_PX_PER_GRID
        self.outputStream = CameraServer.putVideo(f"CostMap_{name}", self.imgWidth, self.imgHeight)
        self.outputStream.setPixelFormat(VideoMode.PixelFormat.kBGR)
        self.outImgMat =  np.zeros((self.imgHeight, self.imgWidth, 3), dtype=np.uint8)

    @staticmethod
    def viridis_color(value: float, maxVal:float = 100) -> tuple[int, int, int]:
        """
        Convert a value in the range 0-100 to an RGB color on the Viridis colormap.
        """
        # Ensure the value is within the range [0, 50]
        value = max(0, min(maxVal, value))
        
        # Viridis colormap data: [r, g, b] values scaled to [0, 1] (excerpt of 6 points)
        # Full colormap is typically more detailed, here is a scaled-down approximation
        viridis_data = [
            [0.267004, 0.004874, 0.329415],
            [0.229739, 0.322361, 0.545706],
            [0.127568, 0.566949, 0.550556],
            [0.369214, 0.788888, 0.382914],
            [0.678489, 0.863742, 0.189503],
            [0.993248, 0.906157, 0.143936]
        ]
        
        # Normalize value to select within the viridis data
        # Since we have 6 points, we divide by 20 to get an index range [0-5]
        num_colors = len(viridis_data) - 1
        index = value * num_colors / maxVal
        
        # Find the indices for interpolation
        lower_index = int(np.floor(index))
        upper_index = min(lower_index + 1, num_colors)
        
        # Interpolation factor
        factor = index - lower_index
        
        # Linearly interpolate between the two colors
        color = [
            viridis_data[lower_index][i] * (1 - factor) + viridis_data[upper_index][i] * factor
            for i in range(3)
        ]
        
        # Convert color to 8-bit RGB values
        r, g, b = [int(c * 255) for c in color]
        
        return (b, g, r)  # Return as (Blue, Green, Red) since BGR is needed


    def update(self, rpp:'RepulsorFieldPlanner'):
        return
        # This is way too slow need to get good
        if(rpp.goal is not None):
            for x_grid in range(GRID_SIZE_X):
                for y_grid in range(GRID_SIZE_Y):

                    x_center = (x_grid+0.5) * _M_PER_GRID
                    y_center = (y_grid+0.5) * _M_PER_GRID

                    val = rpp.getForceAtTrans(Translation2d(x_center, y_center)).mag()

                    # Correct for graphics conventions
                    x = x_grid
                    y = GRID_SIZE_Y - 1 - y_grid

                    x0 = x*_PX_PER_GRID
                    x1 = (x+1)*_PX_PER_GRID
                    y0 = y*_PX_PER_GRID
                    y1 = (y+1)*_PX_PER_GRID
                    self.outImgMat[y0:y1, x0:x1] = self.viridis_color(val)
                
        self.outputStream.putFrame(self.outImgMat)
