use kiss3d::window::Window;
use na::{Point2, Point3};

/// Minimal 2D plotting utility using KISS3D
/// Displays in a EKG style with
pub struct Plot {
    last: usize,
    data: Vec<f32>,
}

impl Plot {
    /// Create a new plotting object with `size` slots for data points
    pub fn new(size: usize) -> Plot {
        Plot {
            last: 0,
            data: (0..size).map(|_| 0.0).collect::<Vec<_>>(),
        }
    }

    /// add a point to the cyclic buffer
    pub fn add_point(&mut self, y: f32) {
        self.data[self.last] = y;
        self.last = (self.last + 1) % self.data.len();
    }

    /// draw the plot to the window. (x,y) is the starting corner, dx is the horizontal distance
    /// between points, dy is the scaling factor of the data values to screen values.
    pub fn draw(&self, w: &mut Window, color: &Point3<f32>, x: f32, y: f32, dx: f32, dy: f32) {
        for i in 0..(self.data.len() - 1) {
            let x0 = x + dx * i as f32;
            let x1 = x0 + dx;
            let y0 = y + dy * self.data[i];
            let y1 = y + dy * self.data[i + 1];
            w.draw_planar_line(&Point2::new(x0, y0), &Point2::new(x1, y1), color);
        }
    }
}
