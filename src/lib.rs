mod linear_programming;

use glam::Vec2;
use linear_programming::Line;

pub struct Agent {
  pub position: Vec2,
  pub velocity: Vec2,
  pub preferred_velocity: Vec2,

  pub radius: f32,
  pub max_velocity: f32,
}

impl Agent {
  pub fn compute_avoiding_velocity(
    &self,
    neighbours: &[&Agent],
    time_horizon: f32,
    time_step: f32,
  ) -> Vec2 {
    todo!()
  }
}
