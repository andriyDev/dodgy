mod linear_programming;

use glam::Vec2;
use linear_programming::{solve_linear_program, Line};

// A single agent in the simulation.
pub struct Agent {
  // The position of the agent.
  pub position: Vec2,
  // The current velocity of the agent.
  pub velocity: Vec2,

  // The radius of the agent. Agents will use this to avoid bumping into each other.
  pub radius: f32,
  // The maximum velocity the agent is allowed to move at.
  pub max_velocity: f32,
}

impl Agent {
  // Computes a velocity based off the agent's preferred velocity (usually the
  // direction to its current goal/waypoint). This new velocity is intended to
  // avoid running into the agent's `neighbours`. This is not always possible,
  // but agents will attempt to resolve any collisions in a reasonable fashion.
  // The `time_horizon` determines how long in the future should collisions be
  // considered between agents. The `time_step` helps determine the velocity in
  // cases of existing collisions.
  pub fn compute_avoiding_velocity(
    &self,
    neighbours: &[&Agent],
    preferred_velocity: Vec2,
    time_horizon: f32,
    time_step: f32,
  ) -> Vec2 {
    let lines = neighbours
      .iter()
      .map(|neighbour| {
        self.get_line_for_neighbour(neighbour, time_horizon, time_step)
      })
      .collect::<Vec<Line>>();

    solve_linear_program(&lines, preferred_velocity, self.max_velocity)
  }

  // Creates a line to describe the half-plane of valid velocities that should
  // not collide with `neighbour`.
  fn get_line_for_neighbour(
    &self,
    neighbour: &Agent,
    time_horizon: f32,
    time_step: f32,
  ) -> Line {
    todo!()
  }
}
