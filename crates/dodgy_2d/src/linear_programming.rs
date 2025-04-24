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

use crate::determinant;
use glam::Vec2;

/// A half-plane to act as a constraint on the linear program. This is
/// represented as a point and a direction, where the valid half-plane resides
/// on the counter-clockwise side of `direction` and `point`.
#[derive(Clone, Debug)]
pub struct Line {
  pub point: Vec2,
  /// Must always have length = 1
  pub direction: Vec2,
}

/// Solves the linear program defined as finding the value closest to
/// `preferred_value` under the constraints that the value has a length less
/// than `radius`, and is outside all half-planes defined by `constraints`. If
/// satisfying all constraints is infeasible, the non-rigid constraints (i.e.
/// `constraints[rigid_constraint_count..]`) are relaxed and the
/// least-penetrating value is returned. If the rigid constraints cannot be
/// satisfied, returns the best value prior to that determination.
pub fn solve_linear_program(
  constraints: &[Line],
  rigid_constraint_count: usize,
  radius: f32,
  preferred_value: Vec2,
) -> Result<Vec2, Vec2> {
  match solve_linear_program_2d(
    constraints,
    radius,
    &OptimalValue::Point(preferred_value),
  ) {
    LinearProgram2DResult::Feasible(optimal_value) => Ok(optimal_value),
    LinearProgram2DResult::Infeasible {
      index_of_failed_line,
      partial_value,
    } if index_of_failed_line < rigid_constraint_count => Err(partial_value),
    LinearProgram2DResult::Infeasible {
      index_of_failed_line,
      partial_value,
    } => Ok(solve_linear_program_3d(
      constraints,
      rigid_constraint_count,
      radius,
      index_of_failed_line,
      partial_value,
    )),
  }
}

const RVO_EPSILON: f32 = 0.00001;

/// The definition of the optimal value ignoring all constraints.
enum OptimalValue {
  /// The best value of the linear program should be the one nearest to this
  /// point (that satisfies the constraints).
  Point(Vec2),
  /// The best value of the linear program should be the one furthest in this
  /// direction (that satisfies the constraints). This must be a unit vector.
  Direction(Vec2),
}

/// Solves the linear program restricted to `line`, and within the circle
/// defined by `radius`. In addition, all `constraints` are used to further
/// restrict the resulting value. The best value is defined by `optimal_value`.
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

  if discriminant < 0.0 {
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

  let t = match *optimal_value {
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
      let t = line.direction.dot(point - line.point);

      // Clamp that point to the correct range.
      t.clamp(t_left, t_right)
    }
  };

  // Compute the actual optimal value using the time along `line`.
  Ok(line.point + t * line.direction)
}

/// The result of the 2D linear program.
#[derive(PartialEq, Debug)]
enum LinearProgram2DResult {
  /// The linear program was feasible and holds the optimal value.
  Feasible(Vec2),
  /// The linear program was infeasible.
  Infeasible {
    /// The index of the line which caused the linear program to be invalid.
    index_of_failed_line: usize,
    /// The value at the time that the linear program was determined to be
    /// invalid. The value is "partial" in the sense that it is partially
    /// constrained by the lines prior to `index_of_failed_line`.
    partial_value: Vec2,
  },
}

/// Solves the 2D linear program, restricted to the circle defined by `radius`,
/// and under `constraints`. The best value is defined by `optimal_value`.
fn solve_linear_program_2d(
  constraints: &[Line],
  radius: f32,
  optimal_value: &OptimalValue,
) -> LinearProgram2DResult {
  let mut best_value = match *optimal_value {
    // If optimizing by a direction, the best value is just on the circle in
    // that direction.
    OptimalValue::Direction(direction) => direction * radius,
    // If using a point and the point is outside the circle, clamp it back to
    // the circle.
    OptimalValue::Point(point) if point.length_squared() > radius * radius => {
      point.normalize() * radius
    }
    // If using a point and the point is inside the circle, use it as is.
    OptimalValue::Point(point) => point,
  };

  for (index, constraint) in constraints.iter().enumerate() {
    if determinant(constraint.direction, best_value - constraint.point) > 0.0 {
      // If the current best value is already on the valid side of the
      // half-plane defined by `constraint`, there is nothing to do.
      continue;
    }

    // Since the current `best_value` violates `constraint`, the new best value
    // must reside somewhere on the line defined by `constraint`.
    match solve_linear_program_along_line(
      constraint,
      radius,
      &constraints[0..index],
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

/// Solves the 3D linear program, after the 2D linear program was determined to
/// be infeasible. This effectively finds the first valid value when moving all
/// non-rigid half-planes back at the same speed. `radius` limits the magnitude
/// of the resulting value. `rigid_constraint_count` determines the constraints
/// that will not be moved. These are assumed to be trivially satisfiable (in
/// practice these correspond to obstacles in RVO, which can be satisfied by a
/// velocity of 0). `index_of_failed_line` and `partial_value` are the results
/// from the infeasible 2D program, where `partial_value` is assumed to satisfy
/// all `constraints[0..index_of_failed_line]`.
fn solve_linear_program_3d(
  constraints: &[Line],
  rigid_constraint_count: usize,
  radius: f32,
  index_of_failed_line: usize,
  partial_value: Vec2,
) -> Vec2 {
  debug_assert!(rigid_constraint_count <= index_of_failed_line);

  // The 2D linear program returned a partial value that is guaranteed to
  // satisfy all constraints up to `index_of_failed_line`. So the current best
  // value is `partial_value` and the deepest penetration into a constraint is 0
  // (since all previous constraints are satisfied).
  let mut penetration = 0.0;
  let mut best_value = partial_value;

  for (index, constraint) in
    constraints[index_of_failed_line..].iter().enumerate()
  {
    if determinant(constraint.direction, constraint.point - best_value)
      <= penetration
    {
      // `best_value` does not penetrate the constraint any more than other
      // constraints, so move on (this constraint will still be considered for
      // future constraints).
      continue;
    }

    let index = index + index_of_failed_line;

    // The goal is to find the value that penetrates all constraints the least.
    // While optimizing the value to penetrate `constraint` as little as
    // possible (in the direction of the valid `constraint` half-plane), if the
    // value is constrained such that all other constraints are not violated
    // more than `constraint`, the resulting value will penetrate all
    // constraints the least.

    // Start a new problem to find the least penetrating value for `constraint`.
    let mut penetration_constraints = Vec::with_capacity(index);
    // Copy over all the rigid constraints - these must always be satisfied
    // without modification.
    penetration_constraints
      .extend_from_slice(&constraints[0..rigid_constraint_count]);

    for previous_constraint in &constraints[rigid_constraint_count..index] {
      // The new constraint for `previous_constraint` is the half-plane such
      // that `previous_constraint` is violated more than `constraint`. This
      // half-plane is defined by the line through the intersection of both
      // constraint lines (where both constraints are 0), and in the direction
      // of equal violation. This direction is therefore the difference between
      // `previous_constraint` and `constraint`.

      let intersection_determinant =
        determinant(constraint.direction, previous_constraint.direction);

      let intersection_point = if intersection_determinant.abs() <= RVO_EPSILON
      {
        // The constraint lines are parallel.

        if constraint.direction.dot(previous_constraint.direction) > 0.0 {
          // The constraint lines point in the same direction, so optimizing
          // `constraint` will also satisfy `previous_constraint` just as well.
          continue;
        }

        // The constraint lines point in opposite directions, so the average of
        // the two lines is where the constraints are violated the same amount.
        (constraint.point + previous_constraint.point) * 0.5
      } else {
        // Use linear algebra to solve for the time of intersect between the
        // constraint lines (for the `constraint` line).
        let intersection_t = determinant(
          previous_constraint.direction,
          constraint.point - previous_constraint.point,
        ) / intersection_determinant;

        // Compute the actual intersection point.
        constraint.point + intersection_t * constraint.direction
      };

      penetration_constraints.push(Line {
        direction: (previous_constraint.direction - constraint.direction)
          .normalize(),
        point: intersection_point,
      });
    }

    // This should in principle not happen. The optimal value is by definition
    // already in the feasible region of the linear program. If it fails, it is
    // due to small floating point errors, and the current `best_value` is kept.
    if let LinearProgram2DResult::Feasible(result) = solve_linear_program_2d(
      &penetration_constraints,
      radius,
      // The optimal value is the furthest value in the direction of the valid
      // side of `constraint`'s half-plane.
      &OptimalValue::Direction(constraint.direction.perp()),
    ) {
      best_value = result;
      penetration =
        determinant(constraint.direction, constraint.point - best_value);
    }
  }

  best_value
}

#[cfg(test)]
#[path = "linear_programming_test.rs"]
mod test;
