use glam::Vec3;

// A half-space to act as a constraint on the linear program. This is
// represented as a point and a normal, where the valid half-space resides in
// the direction of the normal.
#[derive(Clone, Debug)]
pub struct Plane {
  pub point: Vec3,
  // Must always have length = 1
  pub normal: Vec3,
}

impl Plane {
  fn signed_distance_to_plane(&self, point: Vec3) -> f32 {
    (point - self.point).dot(self.normal)
  }
}

#[derive(Clone, Debug)]
struct Line {
  point: Vec3,
  // Must always have length = 1
  direction: Vec3,
}

const RVO_EPSILON: f32 = 0.00001;

// The definition of the optimal value ignoring all constraints.
enum OptimalValue {
  // The best value of the linear program should be the one nearest to this
  // point (that satisfies the constraints).
  Point(Vec3),
  // The best value of the linear program should be the one furthest in this
  // direction (that satisfies the constraints). This must be a unit vector.
  Direction(Vec3),
}

// Solves the linear program restricted to `line`, and within the sphere defined
// by `radius`. In addition, all `constraints` are used to further restrict the
// resulting value. The best value is defined by `optimal_value`.
fn solve_linear_program_along_line(
  line: &Line,
  radius: f32,
  constraints: &[Plane],
  optimal_value: &OptimalValue,
) -> Result<Vec3, ()> {
  // Find the intersecting "times" of the line between `line` and the sphere
  // with `radius`. This is fairly straightforward by using the equation of a
  // ray and a sphere and solving. Note that `line.direction` is a unit
  // vector. The following is the result of expanding out the quadratic
  // equation.

  let line_dot_product = line.point.dot(line.direction);
  let discriminant = line_dot_product * line_dot_product + radius * radius
    - line.point.dot(line.point);

  if discriminant < 0.0 {
    // `line` does not intersect the sphere with `radius`, so the linear program
    // is infeasible.
    return Err(());
  }

  let discriminant = discriminant.sqrt();
  // The right time is the furthest distance in `line.direction` still in the
  // sphere, and the left time is the furthest distance in the opposite
  // direction.
  let mut t_left = -line_dot_product - discriminant;
  let mut t_right = -line_dot_product + discriminant;

  for constraint in constraints {
    // Solve for the time of intersect between `line` and `constraint`. This can
    // be done by solving for when the line and the plane intersects (using some
    // linear algebra).

    let direction_dot = line.direction.dot(constraint.normal);
    let numerator = constraint.signed_distance_to_plane(line.point);

    if direction_dot * direction_dot <= RVO_EPSILON {
      // `line` and `constraint` are nearly parallel.

      if numerator < -RVO_EPSILON {
        // `line` is parallel to and on the invalid side of `constraint`, so all
        // `line` values are invalid, and so the result is infeasible.
        return Err(());
      }

      // `line` is parallel to `constraint` but is on the valid side of
      // `constraint`, so all `line` values remain valid. `constraint` can be
      // ignored.
      continue;
    }

    // The time of intersection for `line` between `line` and `constraint`.
    let t = -numerator / direction_dot;

    // Cut the remaining values along `line` based on how the half-space of
    // `constraint` is oriented.
    if direction_dot >= 0.0 {
      t_left = t_left.max(t);
    } else {
      t_right = t_right.min(t);
    }

    if t_left > t_right {
      // Since t_left > t_right, all values have been invalidated by performing
      // the last cut, so the problem is infeasible.
      return Err(());
    }
  }

  let t = match optimal_value {
    &OptimalValue::Direction(direction) => {
      // If the optimal value is determined by a direction, just pick the most
      // extreme value in that direction. This will always either be t_right or
      // t_left.
      if direction.dot(line.direction) > 0.0 {
        t_right
      } else {
        t_left
      }
    }
    &OptimalValue::Point(point) => {
      // If the optimal value is determined by a point, project that point onto
      // the line segment [t_left, t_right].

      // Project to the line unconstrained.
      let t = line.direction.dot(point - line.point);

      // Clamp that point to the correct range.
      t.clamp(t_left, t_right)
    }
  };

  // Compute the actual optimal value using the time along `line`.
  Ok(line.point + t * line.direction)
}

#[cfg(test)]
mod tests {
  use super::*;

  macro_rules! assert_vec3_near {
    ($a: expr, $b: expr) => {{
      let a = $a;
      let b = $b;

      assert!(
        (a - b).length_squared() < super::RVO_EPSILON,
        "\n  left: {}\n right: {}",
        a,
        b
      );
    }};
  }

  mod solve_linear_program_along_line_tests {
    use glam::Vec3;

    use crate::linear_programming::Plane;

    use super::{solve_linear_program_along_line, Line, OptimalValue};

    #[test]
    fn projects_optimal_point_with_no_constraints() {
      // Compute what the circle height should be at the 0.5 mark.
      let circle_height_at_half = (1.0f32 - 0.5 * 0.5).sqrt();

      let valid_line = Line {
        direction: Vec3::new(0.0, 1.0, 0.0),
        point: Vec3::new(0.5, 0.0, 0.0),
      };

      assert_eq!(
        solve_linear_program_along_line(
          &valid_line,
          1.0,
          Default::default(),
          &OptimalValue::Point(Vec3::new(5.0, 0.25, 0.0)),
        ),
        Ok(Vec3::new(0.5, 0.25, 0.0))
      );

      assert_eq!(
        solve_linear_program_along_line(
          &valid_line,
          1.0,
          Default::default(),
          &OptimalValue::Point(Vec3::new(5.0, 2.0, 0.0)),
        ),
        Ok(Vec3::new(0.5, circle_height_at_half, 0.0))
      );

      assert_eq!(
        solve_linear_program_along_line(
          &valid_line,
          1.0,
          Default::default(),
          &OptimalValue::Point(Vec3::new(5.0, -100.0, 0.0)),
        ),
        Ok(Vec3::new(0.5, -circle_height_at_half, 0.0))
      );
    }

    #[test]
    fn projects_optimal_direction_with_no_constraints() {
      // Compute what the circle height should be at the 0.5 mark.
      let circle_height_at_half = (1.0f32 - 0.5 * 0.5).sqrt();

      let valid_line = Line {
        direction: Vec3::new(0.0, 0.0, 1.0),
        point: Vec3::new(0.5, 0.0, 0.0),
      };

      assert_eq!(
        solve_linear_program_along_line(
          &valid_line,
          1.0,
          Default::default(),
          &OptimalValue::Direction(Vec3::new(1.0, 0.0, 0.5).normalize()),
        ),
        Ok(Vec3::new(0.5, 0.0, circle_height_at_half))
      );

      assert_eq!(
        solve_linear_program_along_line(
          &valid_line,
          1.0,
          Default::default(),
          &OptimalValue::Direction(Vec3::new(1.0, 0.0, -0.5).normalize()),
        ),
        Ok(Vec3::new(0.5, 0.0, -circle_height_at_half))
      );
    }

    #[test]
    fn constraints_remove_valid_values() {
      let valid_line = Line {
        direction: Vec3::new(0.0, 0.0, 1.0),
        point: Vec3::new(0.0, 0.5, 0.0),
      };

      let constraints = [
        // This plane intersects `valid_line` at (0.5, 0.5).
        Plane {
          normal: Vec3::new(0.0, 0.0, -1.0),
          point: Vec3::new(0.0, -100.0, 0.5),
        }, /* This line intersects `valid_line` at (0.5, -0.75). */
        Plane {
          normal: Vec3::new(0.0, -1.0, 1.0).normalize(),
          point: Vec3::new(0.0, 0.25, -1.0),
        },
      ];

      // The middle value should be unchanged.
      assert_eq!(
        solve_linear_program_along_line(
          &valid_line,
          1.0,
          &constraints,
          &OptimalValue::Point(Vec3::new(0.0, -5.0, 0.25)),
        ),
        Ok(Vec3::new(0.0, 0.5, 0.25))
      );

      assert_eq!(
        solve_linear_program_along_line(
          &valid_line,
          1.0,
          &constraints,
          &OptimalValue::Point(Vec3::new(0.0, -5.0, 1.0)),
        ),
        Ok(Vec3::new(0.0, 0.5, 0.5))
      );

      assert_eq!(
        solve_linear_program_along_line(
          &valid_line,
          1.0,
          &constraints,
          &OptimalValue::Point(Vec3::new(0.0, -5.0, -1.0)),
        ),
        Ok(Vec3::new(0.0, 0.5, -0.75))
      );
    }

    #[test]
    fn constraints_are_infeasible() {
      let valid_line = Line {
        direction: Vec3::new(0.0, 1.0, 0.0),
        point: Vec3::new(0.5, 0.0, 0.0),
      };

      let constraints = [
        // This plane intersects `valid_line` at (0.5, 0.5), and invalidates
        // all points below.
        Plane {
          normal: Vec3::new(0.0, 1.0, 0.0),
          point: Vec3::new(-100.0, 0.5, 0.0),
        },
        // This plane intersects `valid_line` at (0.5, -0.5), and invalides all
        // points above.
        Plane {
          normal: Vec3::new(0.0, -1.0, 0.0),
          point: Vec3::new(-100.0, -0.5, 0.0),
        },
      ];

      assert_eq!(
        solve_linear_program_along_line(
          &valid_line,
          1.0,
          &constraints,
          &OptimalValue::Point(Vec3::ZERO),
        ),
        Err(())
      );
    }

    #[test]
    fn valid_line_outside_sphere() {
      let valid_line = Line {
        direction: Vec3::new(0.0, 1.0, 0.0),
        point: Vec3::new(2.0, 0.0, 0.0),
      };

      assert_eq!(
        solve_linear_program_along_line(
          &valid_line,
          1.0,
          Default::default(),
          &OptimalValue::Point(Vec3::ZERO),
        ),
        Err(())
      );
    }

    #[test]
    fn constraint_parallel_to_line() {
      let valid_line =
        Line { direction: Vec3::new(1.0, 0.0, 0.0), point: Vec3::ZERO };

      let feasible_constraint = [Plane {
        point: Vec3::new(0.0, -0.1, 0.0),
        normal: Vec3::new(0.0, 1.0, 0.0),
      }];

      assert_vec3_near!(
        solve_linear_program_along_line(
          &valid_line,
          1.0,
          &feasible_constraint,
          &OptimalValue::Direction(Vec3::X)
        )
        .unwrap(),
        Vec3::new(1.0, 0.0, 0.0)
      );

      let infeasible_constraint = [Plane {
        point: Vec3::new(0.0, 0.1, 0.0),
        normal: Vec3::new(0.0, 1.0, 0.0),
      }];
      assert_eq!(
        solve_linear_program_along_line(
          &valid_line,
          1.0,
          &infeasible_constraint,
          &OptimalValue::Direction(Vec3::X)
        ),
        Err(())
      );
    }
  }
}
