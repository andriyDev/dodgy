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

    solve_linear_program(
      &lines,
      /*rigid_constraint_count=*/ 0,
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
    // 2) The cut-off "legs". This is effectively the velocity obstacle's
    // shadow. Any velocity that is just scaled up from a velocity in the
    // cut-off circle will also hit `neighbour`.
    //
    // If the relative position and velocity is used, the cut-off for the legs
    // will be directed toward the origin.

    let relative_neighbour_position = neighbour.position - self.position;
    let relative_agent_velocity = self.velocity - neighbour.velocity;

    let distance_squared = relative_neighbour_position.length_squared();

    let sum_radius = self.radius + neighbour.radius;
    let sum_radius_squared = sum_radius * sum_radius;

    let vo_normal;
    let relative_velocity_projected_to_vo;

    // Find out if the agent is inside the cut-off circle. Note: since both the
    // distance to the cut-off circle and the radius of the cut-off circle is
    // scaled by `time_horizon` (or `time_step` depending on the situation),
    // factoring out those terms and cancelling yields this simpler expression.
    if distance_squared > sum_radius_squared {
      // No collision, so either project on to the cut-off circle, or the
      // cut-off legs.
      //
      // As mentioned earlier, the legs act as the shadow of the
      // cut-off circle. The cut-off legs lie along the tangents of the circle
      // that intersects the origin (since the tangents are the lines that just
      // graze the cut-off circle and so these line divide the "shadowed"
      // velocities from the "unshadowed" velocities).
      //
      // Since the shadows are caused by the tangent lines, velocities should be
      // projected to the cut-off circle when they are on one-side of the
      // tangent points, and should be projected to the legs when on the
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
      } else {
        // The relative velocity is past the cut-off circle tangent points, so
        // project onto the legs.

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
        // leg.
        let tangent_side = determinant(
          relative_neighbour_position,
          cutoff_circle_center_to_relative_velocity,
        )
        .signum();

        // Compute the leg direction using the tangent triangle legs, and make
        // sure to use the correct orientation of that direction (the correct
        // side of the line is invalid).
        let leg_direction =
          relative_neighbour_position * tangent_triangle_leg * tangent_side
            + relative_neighbour_position.perp() * sum_radius;

        // Renormalize the leg direction.
        let leg_direction = leg_direction / distance_squared;

        vo_normal = leg_direction.perp();
        // Project onto the leg.
        relative_velocity_projected_to_vo =
          relative_agent_velocity.project_onto_normalized(leg_direction);
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
    }

    // As in the paper, `u` is the vector from the relative velocity to the
    // nearest point outside the velocity obstacle.
    let u = relative_velocity_projected_to_vo - relative_agent_velocity;

    Line { point: self.velocity + u * 0.5, direction: -vo_normal.perp() }
  }
}

fn determinant(a: Vec2, b: Vec2) -> f32 {
  a.x * b.y - a.y * b.x
}

#[cfg(test)]
mod tests {
  use super::*;

  const EPSILON: f32 = 0.00001;

  mod get_line_for_neighbour_tests {
    use glam::Vec2;

    use super::{Agent, Line, EPSILON};

    fn line_test_impl(
      position_relative: Vec2,
      velocity_relative: Vec2,
      sum_radius: f32,
      time_horizon: f32,
      time_step: f32,
      expected_line: Line,
    ) {
      let agent = Agent {
        position: Vec2::ZERO,
        velocity: velocity_relative,
        radius: sum_radius - 1.0,
        max_velocity: 0.0,
      };

      let neighbour = Agent {
        position: position_relative,
        velocity: Vec2::ZERO,
        radius: 1.0,
        max_velocity: 0.0,
      };

      let actual_line =
        agent.get_line_for_neighbour(&neighbour, time_horizon, time_step);
      let actual_direction_length = actual_line.direction.length();
      assert!(
        (actual_direction_length - 1.0).abs() < EPSILON,
        "\nLine: {:?}\nDirection Length: {}",
        actual_line,
        actual_direction_length
      );
      assert!(
        (actual_line.direction.dot(expected_line.direction) - 1.0).abs()
          < EPSILON,
        "\nActual line: {:?}\nExpected line: {:?}",
        actual_line,
        expected_line
      );

      let distance_between_lines = (actual_line.point - expected_line.point)
        .dot(expected_line.direction.perp())
        .abs();
      assert!(
        distance_between_lines < EPSILON,
        "\nActual line: {:?}\nExpected line: {:?}\nDistance between lines: {}",
        actual_line,
        expected_line,
        distance_between_lines
      );
    }

    #[test]
    fn velocity_projects_on_cutoff_circle() {
      let position = Vec2::new(1.0, 2.0);
      let radius = 2.0;

      // The agent's velocity projects directly onto the cut-off circle, then
      // the agent takes 50% of the responsibility of avoidance.
      line_test_impl(
        /*position=*/ position,
        /*velocity=*/ Vec2::new(0.0, 0.0),
        /*sum_radius=*/ radius,
        /*time_horizon=*/ 1.0,
        /*time_step=*/ 1.0,
        /*expected_line=*/
        Line {
          point: position.normalize() * (position.length() - radius) * 0.5,
          direction: position.perp().normalize(),
        },
      );
    }

    #[test]
    fn velocity_projects_to_legs() {
      line_test_impl(
        Vec2::new(2.0, 2.0),
        Vec2::new(-1.0, 3.0),
        2.0,
        1.0,
        1.0,
        Line { point: Vec2::new(-0.5, 0.0), direction: Vec2::new(0.0, 1.0) },
      );

      line_test_impl(
        Vec2::new(2.0, 2.0),
        Vec2::new(10.0, -1.0),
        2.0,
        1.0,
        1.0,
        Line { point: Vec2::new(0.0, -0.5), direction: Vec2::new(-1.0, 0.0) },
      );
    }

    #[test]
    fn collision_uses_time_step() {
      line_test_impl(
        Vec2::new(2.0, 2.0),
        Vec2::new(0.0, 0.0),
        4.0,
        1.0,
        0.5,
        Line {
          point: (Vec2::ONE.normalize() * -8.0 + Vec2::new(4.0, 4.0)) * 0.5,
          direction: Vec2::new(-1.0, 1.0).normalize(),
        },
      );
    }

    #[test]
    fn no_collision_uses_time_horizon() {
      line_test_impl(
        Vec2::new(2.0, 2.0),
        Vec2::new(0.0, 0.0),
        2.0,
        2.0,
        0.5,
        Line {
          point: (-Vec2::ONE.normalize() + Vec2::new(1.0, 1.0)) * 0.5,
          direction: Vec2::new(-1.0, 1.0).normalize(),
        },
      );
    }
  }
}
