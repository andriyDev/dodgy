use glam::Vec2;

pub fn determinant(a: Vec2, b: Vec2) -> f32 {
  a.x * b.y - a.y * b.x
}
