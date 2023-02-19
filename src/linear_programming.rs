use glam::Vec2;

// A half-plane to act as a constraint on the linear program. This is
// represented as a point and a direction, where the valid half-plane resides on
// the "clockwise" side of `direction` and `point`.
pub struct Line {
  pub point: Vec2,
  // Must always have length = 1
  pub direction: Vec2,
}

// Determines the optimal value of the linear program defined be `lines` and
// `preferred_value`. Optimality is defined as:
//  1) The nearest point to `preferred_value` subject to:
//      a) magnitude less than `max_length`
//      b) in the intersection of the half-planes defined by `lines`.
//  2) If the above does not exist, the half-planes are moved back at the same
//      speed until a single point exists that satisfies all constraints.
pub fn solve_linear_program(
  lines: &[Line],
  preferred_value: Vec2,
  max_length: f32,
) -> Vec2 {
  match solve_linear_program_2d(
    lines,
    max_length,
    &OptimalValue::Point(preferred_value),
  ) {
    LinearProgram2DResult::Feasible(optimal_value) => optimal_value,
    LinearProgram2DResult::Infeasible {
      index_of_failed_line,
      partial_value,
    } => solve_linear_program_3d(
      lines,
      index_of_failed_line,
      partial_value,
      max_length,
    ),
  }
}

const RVO_EPSILON: f32 = 0.00001;

// The definition of the optimal value ignoring all constraints.
enum OptimalValue {
  // The best value of the linear program should be the one nearest to this
  // point (that satisfies the constraints).
  Point(Vec2),
  // The best value of the linear program should be the one furthest in this
  // direction (that satisfies the constraints). This must be a unit vector.
  Direction(Vec2),
}

// Solves the linear program restricted to `line`, and within the circle defined
// by `radius`. In addition, all `constraints` are used to further restrict the
// resulting value. The best value is defined by `optimal_value`.
fn solve_linear_program_along_line(
  line: &Line,
  radius: f32,
  constraints: &[Line],
  optimal_value: &OptimalValue,
) -> Result<Vec2, ()> {
  // Find the intersecting "times" of the line between `line` and the circle
  // with `radius`. This is fairly straightforward by using the equation of ray
  // and a circle and solving. Note that `line.direction` is a unit vector. The
  // following is the result of expanding out the quadratic equation.

  let line_dot_product = line.point.dot(line.direction);
  let discriminant = line_dot_product * line_dot_product + radius * radius
    - line.point.dot(line.point);

  if radius < 0.0 {
    // `line` does not intersect the circle with `radius`, so the linear program
    // is infeasible.
    return Err(());
  }

  let discriminant = discriminant.sqrt();
  // The right time is the furthest distance in `line.direction` still in the
  // circle, and the left time is the furthest distance in the opposite
  // direction.
  let mut t_left = -line_dot_product - discriminant;
  let mut t_right = -line_dot_product + discriminant;

  for constraint in constraints {
    // Solve for the time of intersect for `line` between `line` and
    // `constraint`. This can be done by solving for when the lines intersect
    // (using some linear algebra), and the only computing the time value
    // corresponding to `line`.

    let direction_determinant =
      determinant(line.direction, constraint.direction);
    let numerator =
      determinant(constraint.direction, line.point - constraint.point);

    if direction_determinant.abs() <= RVO_EPSILON {
      // `line` and `constraint` are nearly parallel.

      if numerator < 0.0 {
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
    let t = numerator / direction_determinant;

    // Cut the remaining values along `line` based on how the half-plane of
    // `constraint` is oriented.
    if direction_determinant >= 0.0 {
      t_right = t_right.min(t);
    } else {
      t_left = t_left.max(t);
    }

    if t_left > t_right {
      // Since t_left > t_right, all values have been invalidated by performing
      // the last cut, so the problem is infeasible.
      return Err(());
    }
  }

  let t = match optimal_value {
    OptimalValue::Direction(direction) => {
      // If the optimal value is determined by a direction, just pick the most
      // extreme value in that direction. This will always either be t_right or
      // t_left.
      if direction.dot(line.direction) > 0.0 {
        t_right
      } else {
        t_left
      }
    }
    OptimalValue::Point(point) => {
      // If the optimal value is determined by a point, project that point onto
      // the line segment [t_left, t_right].

      // Project to the line unconstrained.
      let t = line.direction.dot(*point - line.point);

      // Clamp that point to the correct range.
      t.clamp(t_left, t_right)
    }
  };

  // Compute the actual optimal value using the time along `line`.
  Ok(line.point + t * line.direction)
}

// The result of the 2D linear program.
enum LinearProgram2DResult {
  // The linear program was feasible and holds the optimal value.
  Feasible(Vec2),
  // The linear program was infeasible.
  Infeasible {
    // The index of the line which caused the linear program to be invalid.
    index_of_failed_line: usize,
    // The value at the time that the linear program was determined to be
    // invalid. The value is "partial" in the sense that it is partially
    // constrained by the lines prior to `index_of_failed_line`.
    partial_value: Vec2,
  },
}

// Solves the 2D linear program, restricted to the circle defined by `radius`,
// and under `constraints`. The best value is defined by `optimal_value`.
fn solve_linear_program_2d(
  constraints: &[Line],
  radius: f32,
  optimal_value: &OptimalValue,
) -> LinearProgram2DResult {
  let mut best_value = match optimal_value {
    // If optimizing by a direction, the best value is just on the circle in
    // that direction.
    &OptimalValue::Direction(direction) => direction * radius,
    // If using a point and the point is outside the circle, clamp it back to
    // the circle.
    &OptimalValue::Point(point) if point.length_squared() > radius * radius => {
      point.normalize() * radius
    }
    // If using a point and the point is inside the circle, use it as is.
    &OptimalValue::Point(point) => point,
  };

  for (index, constraint) in constraints.iter().enumerate() {
    if determinant(constraint.direction, best_value - constraint.point) > 0.0 {
      // If the current best value is already on the value side of the
      // half-plane defined by `constraint`, there is nothing to do.
      continue;
    }

    // Since the current `best_value` violates `constraint`, the new best value
    // must reside somewhere on the line defined by `constraint`.
    match solve_linear_program_along_line(
      constraint,
      radius,
      constraints,
      optimal_value,
    ) {
      Ok(new_value) => best_value = new_value,
      Err(()) => {
        return LinearProgram2DResult::Infeasible {
          index_of_failed_line: index,
          partial_value: best_value,
        }
      }
    }
  }

  LinearProgram2DResult::Feasible(best_value)
}

// Solves the 3D linear program, after the 2D linear program was determined to
// be infeasible. This effectively finds the first valid value when moving all
// half-planes back at the same speed. `lines` are the half-plane constraints.
// `index_of_failed_line` and `partial_value` are the results from the
// infeasible 2D program. `max_length` limits the magnitude of the resulting
// value.
fn solve_linear_program_3d(
  lines: &[Line],
  index_of_failed_line: usize,
  partial_value: Vec2,
  max_length: f32,
) -> Vec2 {
  todo!()
}

fn determinant(a: Vec2, b: Vec2) -> f32 {
  a.x * b.y - a.y * b.x
}
