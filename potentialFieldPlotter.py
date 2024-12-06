import tkinter as tk
from typing import List, Tuple

class ScaledCanvas:
    def __init__(self, root: tk.Tk, width_m: float, height_m: float):
        self.canvas = tk.Canvas(root, bg="white")
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.width_m = width_m
        self.height_m = height_m
        self.scale = 1
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

    def _to_pixels(self, x_m: float, y_m: float) -> Tuple[int, int]:
        return (
            int(x_m * self.scale + self.offset_x),
            int(y_m * self.scale + self.offset_y),
        )

# Create the main Tkinter application
def main():
    root = tk.Tk()
    root.title("Scaled Canvas Example with Fixed Aspect Ratio")
    width_m, height_m = 10.0, 5.0  # Visualization dimensions in meters
    canvas = ScaledCanvas(root, width_m, height_m)

    # Add a rectangle
    canvas.add_rectangle(1.0, 1.0, 8.0, 3.0, color="blue")

    # Add some arrows
    arrows = [
        ((1.0, 1.0), (2.0, 2.0)),
        ((3.0, 1.0), (4.0, 2.5)),
        ((8.0, 1.0), (7.0, 3.0)),
        ((5.0, 4.0), (5.0, 2.5))
    ]
    for start, end in arrows:
        canvas.add_arrow(start, end, color="green")

    root.mainloop()

if __name__ == "__main__":
    main()
