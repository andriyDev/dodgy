// The contents of this file were primarily ported from Agent.cc from RVO2 with
// significant alterations. As per the Apache-2.0 license, the original
// copyright notice has been included, excluding those notices that do not
// pertain to the derivate work:
//
// Agent.cc
// RVO2 Library
//
// SPDX-FileCopyrightText: 2008 University of North Carolina at Chapel Hill
//
// The authors may be contacted via:
//
// Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
// Dept. of Computer Science
// 201 S. Columbia St.
// Frederick P. Brooks, Jr. Computer Science Bldg.
// Chapel Hill, N.C. 27599-3175
// United States of America
//
// <https://gamma.cs.unc.edu/RVO2/>

mod common;
mod linear_programming;
mod obstacles;
mod simulator;
mod visibility_set;

use common::*;
use glam::Vec2;
use linear_programming::{solve_linear_program, Line};
use obstacles::get_lines_for_agent_to_obstacle;

pub use obstacles::Obstacle;

pub use simulator::{AgentParameters, Simulator};
pub use visibility_set::VisibilitySet;

// A single agent in the simulation.
pub struct Agent {
  // The position of the agent.
  pub position: Vec2,
  // The current velocity of the agent.
  pub velocity: Vec2,

  // The radius of the agent. Agents will use this to avoid bumping into each
  // other.
  pub radius: f32,
  // The maximum velocity the agent is allowed to move at.
  pub max_velocity: f32,

  // The amount of responsibility an agent has to avoid other agents. The
  // amount of avoidance between two agents is then dependent on the ratio of
  // the responsibility between the agents. Note this does not affect
  // avoidance of obstacles.
  pub avoidance_responsibility: f32,
}

impl Agent {
  // Computes a velocity based off the agent's preferred velocity (usually the
  // direction to its current goal/waypoint). This new velocity is intended to
  // avoid running into the agent's `neighbours`. This is not always possible,
  // but agents will attempt to resolve any collisions in a reasonable fashion.
  // The `time_horizon` determines how long in the future should collisions be
  // considered between agents. The `obstacle_time_horizon` determines how long
  // in the future should collisions be considered for obstacles. The
  // `time_step` helps determine the velocity in cases of existing collisions,
  // and must be positive.
  pub fn compute_avoiding_velocity(
    &self,
    neighbours: &[&Agent],
    obstacles: &[&Obstacle],
    preferred_velocity: Vec2,
    time_horizon: f32,
    obstacle_time_horizon: f32,
    time_step: f32,
  ) -> Vec2 {
    assert!(time_step > 0.0, "time_step must be positive, was {}", time_step);

    let lines = obstacles
      .iter()
      .flat_map(|o| {
        get_lines_for_agent_to_obstacle(self, o, obstacle_time_horizon)
      })
      .chain(neighbours.iter().map(|neighbour| {
        self.get_line_for_neighbour(neighbour, time_horizon, time_step)
      }))
      .collect::<Vec<Line>>();

    // Since each neighbour generates one line, the number of obstacle lines is
    // just the other lines.
    let obstacle_line_count = lines.len() - neighbours.len();

    solve_linear_program(
      &lines,
      obstacle_line_count,
      self.max_velocity,
      preferred_velocity,
    )
  }

  // Creates a line to describe the half-plane of valid velocities that should
  // not collide with `neighbour`.
  fn get_line_for_neighbour(
    &self,
    neighbour: &Agent,
    time_horizon: f32,
    time_step: f32,
  ) -> Line {
    // There are two parts to the velocity obstacle induced by `neighbour`.
    // 1) The cut-off circle. This is where the agent collides with `neighbour`
    // after some time (either `time_horizon` or `time_step`).
    // 2) The cut-off shadow. Any velocity that is just scaled up from a
    // velocity in the cut-off circle will also hit `neighbour`.
    //
    // If the relative position and velocity is used, the cut-off for the shadow
    // will be directed toward the origin.

    let relative_neighbour_position = neighbour.position - self.position;
    let relative_agent_velocity = self.velocity - neighbour.velocity;

    let distance_squared = relative_neighbour_position.length_squared();

    let sum_radius = self.radius + neighbour.radius;
    let sum_radius_squared = sum_radius * sum_radius;

    let vo_normal;
    let relative_velocity_projected_to_vo;
    let inside_vo;

    // Find out if the agent is inside the cut-off circle. Note: since both the
    // distance to the cut-off circle and the radius of the cut-off circle is
    // scaled by `time_horizon` (or `time_step` depending on the situation),
    // factoring out those terms and cancelling yields this simpler expression.
    if distance_squared > sum_radius_squared {
      // No collision, so either project on to the cut-off circle, or the
      // cut-off shadow.
      //
      // The edges of the cut-off shadow lies along the tangents of the circle
      // that intersects the origin (since the tangents are the lines that just
      // graze the cut-off circle and so these line divide the "shadowed"
      // velocities from the "unshadowed" velocities).
      //
      // Since the shadows are caused by the tangent lines, velocities should be
      // projected to the cut-off circle when they are on one-side of the
      // tangent points, and should be projected to the shadow when on the
      // other-side of the tangent points.

      let cutoff_circle_center = relative_neighbour_position / time_horizon;
      let cutoff_circle_center_to_relative_velocity =
        relative_agent_velocity - cutoff_circle_center;
      let cutoff_circle_center_to_relative_velocity_length_squared =
        cutoff_circle_center_to_relative_velocity.length_squared();

      let dot = cutoff_circle_center_to_relative_velocity
        .dot(relative_neighbour_position);

      // TODO: Figure out why this works. Something to do with circle tangents,
      // right triangles with those tangents, and the angle between
      // `cutoff_circle_center_to_relative_velocity` and
      // `relative_neighbour_position`.
      if dot < 0.0
        && dot * dot
          > sum_radius_squared
            * cutoff_circle_center_to_relative_velocity_length_squared
      {
        // The relative velocity has not gone past the cut-off circle tangent
        // points yet, so project onto the cut-off circle.

        let cutoff_circle_radius = sum_radius / time_horizon;

        vo_normal =
          cutoff_circle_center_to_relative_velocity.normalize_or_zero();
        relative_velocity_projected_to_vo =
          vo_normal * cutoff_circle_radius + cutoff_circle_center;
        inside_vo = cutoff_circle_center_to_relative_velocity_length_squared
          < cutoff_circle_radius * cutoff_circle_radius;
      } else {
        // The relative velocity is past the cut-off circle tangent points, so
        // project onto the shadow.

        let tangent_triangle_leg =
          (distance_squared - sum_radius_squared).sqrt();

        // Consider the right-triangle describing the tangent point (one side
        // has length `sum_radius`, hypotenuse has side length
        // `cutoff_circle_center_to_relative_velocity_length_squared`). The last
        // side will have length `tangent_triangle_leg`. A similar triangle can
        // then be created using the same triangle leg lengths, but oriented
        // such that the hypotenuse is in the direction of the tangent and
        // composed of directions `relative_position` and the perpendicular of
        // `relative_position`.

        // Determine whether the relative velocity is nearer the left or right
        // side of the shadow.
        let tangent_side = determinant(
          relative_neighbour_position,
          cutoff_circle_center_to_relative_velocity,
        )
        .signum();

        // Compute the shadow direction using the tangent triangle legs, and
        // make sure to use the correct orientation of that direction (the
        // correct side of the line is invalid).
        let shadow_direction =
          relative_neighbour_position * tangent_triangle_leg * tangent_side
            + relative_neighbour_position.perp() * sum_radius;

        // Renormalize the shadow direction.
        let shadow_direction = shadow_direction / distance_squared;

        vo_normal = shadow_direction.perp();
        // Project onto the shadow.
        relative_velocity_projected_to_vo =
          relative_agent_velocity.project_onto_normalized(shadow_direction);
        // The velocity is inside the VO if it is to the left of the left
        // shadow, or the right of the right shadow.
        inside_vo = determinant(relative_agent_velocity, shadow_direction)
          * tangent_side
          >= 0.0;
      }
    } else {
      // Collision. Project on cut-off circle at time `time_step`.

      // Find the velocity such that after `time_step` the agent would be at the
      // neighbours position.
      let cutoff_circle_center = relative_neighbour_position / time_step;
      let cutoff_circle_radius = sum_radius / time_step;

      // The direction of the velocity from `cutoff_circle_center` is therefore
      // the normal to the velocity obstacle.
      vo_normal =
        (relative_agent_velocity - cutoff_circle_center).normalize_or_zero();
      // Get the point on the cut-off circle in that direction (which is the
      // agent's velocity projected to the circle).
      relative_velocity_projected_to_vo =
        vo_normal * cutoff_circle_radius + cutoff_circle_center;
      inside_vo = true;
    }

    // As in the paper, `u` is the vector from the relative velocity to the
    // nearest point outside the velocity obstacle.
    let u = relative_velocity_projected_to_vo - relative_agent_velocity;

    let responsibility = if inside_vo {
      self.avoidance_responsibility
        / (self.avoidance_responsibility + neighbour.avoidance_responsibility)
    } else {
      1.0
    };

    Line {
      point: self.velocity + u * responsibility,
      direction: -vo_normal.perp(),
    }
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  mod get_line_for_neighbour_tests {
    use glam::Vec2;

    use super::{Agent, Line};

    macro_rules! assert_line_eq {
      ($a: expr, $b: expr) => {{
        let a = $a;
        let b = $b;

        assert!(
          a.point.distance_squared(b.point) < 1e-5,
          "\n  left: {:?}\n right: {:?}",
          a,
          b
        );
        assert!(
          a.direction.distance_squared(b.direction) < 1e-5,
          "\n  left: {:?}\n right: {:?}",
          a,
          b
        );
      }};
    }

    #[test]
    fn velocity_projects_on_cutoff_circle() {
      let position = Vec2::new(1.0, 2.0);
      let radius = 2.0;

      let agent = Agent {
        position: Vec2::ZERO,
        velocity: Vec2::ZERO,
        radius: radius - 1.0,
        avoidance_responsibility: 1.0,
        max_velocity: 0.0,
      };

      let neighbour = Agent {
        position: position,
        velocity: Vec2::ZERO,
        radius: 1.0,
        avoidance_responsibility: 1.0,
        max_velocity: 0.0,
      };

      let actual_line = agent.get_line_for_neighbour(
        &neighbour, /* time_horizon= */ 1.0, /* time_step= */ 1.0,
      );
      // The agent's velocity projects directly onto the cut-off circle.
      assert_line_eq!(
        actual_line,
        Line {
          point: position.normalize() * (position.length() - radius),
          direction: position.perp().normalize(),
        }
      );
    }

    #[test]
    fn velocity_projects_to_shadow() {
      let mut agent = Agent {
        position: Vec2::ZERO,
        velocity: Vec2::new(-1.0, 3.0),
        radius: 1.0,
        max_velocity: 0.0,
        avoidance_responsibility: 1.0,
      };

      let neighbour = Agent {
        position: Vec2::new(2.0, 2.0),
        velocity: Vec2::ZERO,
        radius: 1.0,
        max_velocity: 0.0,
        avoidance_responsibility: 1.0,
      };

      let inside_shadow_line = agent.get_line_for_neighbour(
        &neighbour, /* time_horizon= */ 1.0, /* time_step= */ 1.0,
      );
      assert_line_eq!(
        inside_shadow_line,
        Line { point: Vec2::new(0.0, 3.0), direction: Vec2::new(0.0, 1.0) }
      );

      agent.velocity = Vec2::new(10.0, -1.0);

      let outside_shadow_line = agent.get_line_for_neighbour(
        &neighbour, /* time_horizon= */ 1.0, /* time_step= */ 1.0,
      );
      assert_line_eq!(
        outside_shadow_line,
        Line { point: Vec2::new(10.0, -0.5), direction: Vec2::new(-1.0, 0.0) }
      );
    }

    #[test]
    fn collision_uses_time_step() {
      let agent = Agent {
        position: Vec2::ZERO,
        velocity: Vec2::new(0.0, 0.0),
        radius: 2.0,
        max_velocity: 0.0,
        avoidance_responsibility: 1.0,
      };

      let neighbour = Agent {
        position: Vec2::new(2.0, 2.0),
        velocity: Vec2::ZERO,
        radius: 2.0,
        max_velocity: 0.0,
        avoidance_responsibility: 1.0,
      };

      let collision_line = agent.get_line_for_neighbour(
        &neighbour, /* time_horizon= */ 1.0, /* time_step= */ 0.5,
      );
      assert_line_eq!(
        collision_line,
        Line {
          point: (Vec2::ONE.normalize() * -8.0 + Vec2::new(4.0, 4.0)) * 0.5,
          direction: Vec2::new(-1.0, 1.0).normalize(),
        }
      );
    }

    #[test]
    fn no_collision_uses_time_horizon() {
      let agent = Agent {
        position: Vec2::ZERO,
        velocity: Vec2::new(0.0, 0.0),
        radius: 1.0,
        max_velocity: 0.0,
        avoidance_responsibility: 1.0,
      };

      let neighbour = Agent {
        position: Vec2::new(2.0, 2.0),
        velocity: Vec2::ZERO,
        radius: 1.0,
        max_velocity: 0.0,
        avoidance_responsibility: 1.0,
      };

      let collision_line = agent.get_line_for_neighbour(
        &neighbour, /* time_horizon= */ 2.0, /* time_step= */ 0.5,
      );
      assert_line_eq!(
        collision_line,
        Line {
          point: -Vec2::ONE.normalize() + Vec2::new(1.0, 1.0),
          direction: Vec2::new(-1.0, 1.0).normalize(),
        }
      );
    }

    #[test]
    fn uses_avoidance_responsibility() {
      let agent = Agent {
        position: Vec2::ZERO,
        velocity: Vec2::new(1.5, 0.0),
        radius: 1.0,
        avoidance_responsibility: 1.0,
        max_velocity: 0.0,
      };

      let neighbour = Agent {
        position: Vec2::new(4.0, 0.0),
        velocity: Vec2::ZERO,
        radius: 1.0,
        avoidance_responsibility: 3.0,
        max_velocity: 0.0,
      };

      let actual_line = agent.get_line_for_neighbour(
        &neighbour, /* time_horizon= */ 2.0, /* time_step= */ 0.5,
      );
      assert_line_eq!(
        actual_line,
        Line { point: Vec2::new(1.375, 0.0), direction: Vec2::new(0.0, 1.0) }
      );
    }

    #[test]
    fn uses_avoidance_responsibility_only_when_inside_vo() {
      let agent = Agent {
        position: Vec2::ZERO,
        velocity: Vec2::new(0.5, 0.0),
        radius: 1.0,
        avoidance_responsibility: 1.0,
        max_velocity: 0.0,
      };

      let neighbour = Agent {
        position: Vec2::new(4.0, 0.0),
        velocity: Vec2::ZERO,
        radius: 1.0,
        avoidance_responsibility: 3.0,
        max_velocity: 0.0,
      };

      let actual_line = agent.get_line_for_neighbour(
        &neighbour, /* time_horizon= */ 2.0, /* time_step= */ 0.5,
      );
      assert_line_eq!(
        actual_line,
        Line { point: Vec2::new(1.0, 0.0), direction: Vec2::new(0.0, 1.0) }
      );
    }
  }
}
