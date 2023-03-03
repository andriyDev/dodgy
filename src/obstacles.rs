use glam::Vec2;

use crate::Agent;

// A single obstacle in the simulation.
pub enum Obstacle {
  // A closed obstacle. The obstacle is closed in that the last vertex will have
  // an edge connecting it to the first vertex. The edges cannot cross, and the
  // interior of the obstacle is to the "left" of the edges. In other words,
  // obstacles with vertices going counter-clockwise will prevent objects from
  // getting into the loop, and obstacles with vertices going clockwise will
  // prevent objects from leaving the loop.
  Closed { vertices: Vec<Vec2> },
  // An open obstacle. The vertices are assumed to be a part of some closed
  // obstacle, so the left of the edge is solid, and the right is clear.
  Open { vertices: Vec<Vec2> },
}

// Computes the lines describing the half-planes of valid velocities for `agent`
// induced by `obstacle`. `time_horizon` determines how much time in the future
// should collisions be considered for this obstacle.
pub fn get_lines_for_agent_and_obstacle(
  agent: &Agent,
  obstacle: &Obstacle,
  time_horizon: f32,
) -> Vec<crate::Line> {
  todo!()
}
