// The contents of this file were primarily ported from Agent.cc from RVO2-3D
// with significant alterations. As per the Apache-2.0 license, the original
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
use glam::Vec3;

/// A half-space to act as a constraint on the linear program. This is
/// represented as a point and a normal, where the valid half-space resides in
/// the direction of the normal.
#[derive(Clone, Debug)]
pub struct Plane {
  pub point: Vec3,
  /// Must always have length = 1
  pub normal: Vec3,
}

impl Plane {
  pub fn signed_distance_to_plane(&self, point: Vec3) -> f32 {
    (point - self.point).dot(self.normal)
  }
}

/// Solves the linear program defined as finding the value closest to
/// `preferred_value` under the constraints that the value has a length less
/// than `radius`, and is outside all half-spaces defined by `constraints`. If
/// satisfying all constraints is infeasible, the constraints are relaxed and
/// the least-penetrating value is returned.
pub fn solve_linear_program(
  constraints: &[Plane],
  radius: f32,
  preferred_value: Vec3,
) -> Vec3 {
  match solve_linear_program_3d(
    constraints,
    radius,
    &OptimalValue::Point(preferred_value),
  ) {
    LinearProgram3DResult::Feasible(optimal_value) => optimal_value,
    LinearProgram3DResult::Infeasible {
      index_of_failed_line,
      partial_value,
    } => solve_linear_program_4d(
      constraints,
      radius,
      index_of_failed_line,
      partial_value,
    ),
  }
}

#[derive(Clone, Debug)]
struct Line {
  point: Vec3,
  /// Must always have length = 1
  direction: Vec3,
}

const RVO_EPSILON: f32 = 0.00001;

/// The definition of the optimal value ignoring all constraints.
enum OptimalValue {
  /// The best value of the linear program should be the one nearest to this
  /// point (that satisfies the constraints).
  Point(Vec3),
  /// The best value of the linear program should be the one furthest in this
  /// direction (that satisfies the constraints). This must be a unit vector.
  Direction(Vec3),
}

/// Solves the linear program restricted to `line`, and within the sphere
/// defined by `radius`. In addition, all `constraints` are used to further
/// restrict the resulting value. The best value is defined by `optimal_value`.
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

/// Solves the linear program restricted to `plane`, and within the sphere
/// defined by `radius`. In addition, all `constraints` are used to further
/// restrict the resulting value. The best value is defined by `optimal_value`.
fn solve_linear_program_along_plane(
  plane: &Plane,
  radius: f32,
  constraints: &[Plane],
  optimal_value: &OptimalValue,
) -> Result<Vec3, ()> {
  // We need to figure out the radius and center of the circle in `plane` that
  // intersects the sphere.
  let plane_distance_from_origin = plane.point.dot(plane.normal);
  let plane_distance_squared_from_origin =
    plane_distance_from_origin * plane_distance_from_origin;
  let radius_squared = radius * radius;

  if plane_distance_squared_from_origin > radius_squared {
    // The plane is too far away from the origin, so no values are valid.
    return Err(());
  }

  // Compute the radius and the center of the valid circle in `plane`.
  let squared_radius_in_valid_plane =
    radius_squared - plane_distance_squared_from_origin;
  let valid_plane_center = plane_distance_from_origin * plane.normal;

  let mut best_value = match *optimal_value {
    OptimalValue::Direction(direction) => {
      let projected_optimal_direction_in_plane =
        direction - direction.dot(plane.normal) * plane.normal;
      let squared_length_of_projection = projected_optimal_direction_in_plane
        .dot(projected_optimal_direction_in_plane);

      if squared_length_of_projection <= RVO_EPSILON {
        valid_plane_center
      } else {
        valid_plane_center
          + (squared_radius_in_valid_plane / squared_length_of_projection)
            .sqrt()
            * projected_optimal_direction_in_plane
      }
    }
    OptimalValue::Point(point) => {
      let distance_from_plane = plane.signed_distance_to_plane(point);
      let projected_point = point - distance_from_plane * plane.normal;

      if projected_point.dot(projected_point) > radius_squared {
        let relative_point = projected_point - valid_plane_center;
        let squared_distance_from_center = relative_point.dot(relative_point);

        valid_plane_center
          + (squared_radius_in_valid_plane / squared_distance_from_center)
            .sqrt()
            * relative_point
      } else {
        projected_point
      }
    }
  };

  for (index, constraint) in constraints.iter().enumerate() {
    let pen = constraint.signed_distance_to_plane(best_value);
    if pen >= 0.0 {
      // If the current best value is already on the valid side of the
      // half-space defined by `constraint`, there is nothing to do.
      continue;
    }
    let cross = constraint.normal.cross(plane.normal);
    if cross.dot(cross) <= RVO_EPSILON {
      // `plane` is parallel to `constraint`, but the current best value (which
      // satisfies `plane`) is on the wrong side of `constraint`. Therefore, we
      // can't satisfy both `plane` and `constraint`.
      return Err(());
    }

    let cross = cross.normalize();
    let line_normal = cross.cross(plane.normal);
    let line = Line {
      direction: cross,
      point: plane.point
        - constraint.signed_distance_to_plane(plane.point)
          / line_normal.dot(constraint.normal)
          * line_normal,
    };

    let Ok(new_value) = solve_linear_program_along_line(
      &line,
      radius,
      &constraints[0..index],
      optimal_value,
    ) else {
      return Err(());
    };

    best_value = new_value;
  }

  Ok(best_value)
}

/// The result of the 3D linear program.
#[derive(PartialEq, Debug)]
enum LinearProgram3DResult {
  /// The linear program was feasible and holds the optimal value.
  Feasible(Vec3),
  /// The linear program was infeasible.
  Infeasible {
    /// The index of the line which caused the linear program to be invalid.
    index_of_failed_line: usize,
    /// The value at the time that the linear program was determined to be
    /// invalid. The value is "partial" in the sense that it is partially
    /// constrained by the lines prior to `index_of_failed_line`.
    partial_value: Vec3,
  },
}

/// Solves the 3D linear program, restricted to the sphere defined by `radius`,
/// and under `constraints`. The best value is defined by `optimal_value`.
fn solve_linear_program_3d(
  constraints: &[Plane],
  radius: f32,
  optimal_value: &OptimalValue,
) -> LinearProgram3DResult {
  let mut best_value = match *optimal_value {
    // If optimizing by a direction, the best value is just on the sphere in
    // that direction.
    OptimalValue::Direction(direction) => direction * radius,
    // If using a point and the point is outside the sphere, clamp it back to
    // the sphere.
    OptimalValue::Point(point) if point.length_squared() > radius * radius => {
      point.normalize() * radius
    }
    // If using a point and the point is inside the sphere, use it as is.
    OptimalValue::Point(point) => point,
  };

  for (index, constraint) in constraints.iter().enumerate() {
    if constraint.signed_distance_to_plane(best_value) > 0.0 {
      // If the current best value is already on the valid side of the
      // half-plane defined by `constraint`, there is nothing to do.
      continue;
    }

    // Since the current `best_value` violates `constraint`, the new best value
    // must reside somewhere on the plane defined by `constraint`.
    match solve_linear_program_along_plane(
      constraint,
      radius,
      &constraints[0..index],
      optimal_value,
    ) {
      Ok(new_value) => best_value = new_value,
      Err(()) => {
        return LinearProgram3DResult::Infeasible {
          index_of_failed_line: index,
          partial_value: best_value,
        }
      }
    }
  }

  LinearProgram3DResult::Feasible(best_value)
}

/// Solves the 4D linear program, after the 3D linear program was determined to
/// be infeasible. This effectively finds the first valid value when moving all
/// non-rigid half-spaces back at the same speed. `radius` limits the magnitude
/// of the resulting value. `index_of_failed_plane` and `partial_value` are the
/// results from the infeasible 3D program, where `partial_value` is assumed to
/// satisfy all `constraints[0..index_of_failed_plane]`.
fn solve_linear_program_4d(
  constraints: &[Plane],
  radius: f32,
  index_of_failed_plane: usize,
  partial_value: Vec3,
) -> Vec3 {
  let mut penetration = 0.0;
  let mut best_value = partial_value;

  for (index, constraint) in
    constraints[index_of_failed_plane..].iter().enumerate()
  {
    if -constraint.signed_distance_to_plane(best_value) <= penetration {
      // `best_value` does not penetrate the constraint any more than other
      // constraints, so move on (this constraint will still be considered for
      // future constraints).
      continue;
    }

    let index = index + index_of_failed_plane;

    // The goal is to find the value that penetrates all constraints the least.
    // While optimizing the value to penetrate `constraint` as little as
    // possible (in the direction of the valid `constraint` half-plane), if the
    // value is constrained such that all other constraints are not violated
    // more than `constraint`, the resulting value will penetrate all
    // constraints the least.

    // Start a new problem to find the least penetrating value for `constraint`.
    let mut penetration_constraints = Vec::with_capacity(index);
    for previous_constraint in &constraints[0..index] {
      // The new constraint for `previous_constraint` is the half-space such
      // that `previous_constraint` is violated no more than `constraint`. This
      // half-space is defined by the plane through the intersection of both
      // constraint lines (where both constraints are 0), and in the direction
      // of equal violation. This direction is therefore the difference between
      // the normals of `previous_constraint` and `constraint`.

      let cross = previous_constraint.normal.cross(constraint.normal);

      let new_plane_point = if cross.dot(cross) <= RVO_EPSILON {
        // The constraint planes are parallel.

        if constraint.normal.dot(previous_constraint.normal) > 0.0 {
          // The constraint spaces point in the same direction, so optimizing
          // `constraint` will also satisfy `previous_constraint` just as well.
          continue;
        }

        // The constraint planes point in opposite directions, so the average of
        // the two planes is where the constraints are violated the same amount.
        (constraint.point + previous_constraint.point) * 0.5
      } else {
        let line_normal = cross.cross(constraint.normal);
        constraint.point
          - previous_constraint.signed_distance_to_plane(constraint.point)
            / line_normal.dot(previous_constraint.normal)
            * line_normal
      };
      penetration_constraints.push(Plane {
        point: new_plane_point,
        normal: (previous_constraint.normal - constraint.normal).normalize(),
      });
    }

    // This should in principle never be infeasible. The optimal value is by
    // definition already in the feasible region of the linear program. If
    // it fails, it is due to small floating point errors, and the current
    // `best_value` is kept.
    if let LinearProgram3DResult::Feasible(result) = solve_linear_program_3d(
      &penetration_constraints,
      radius,
      // The optimal value is the furthest value in the direction of the valid
      // side of `constraint`'s half-space.
      &OptimalValue::Direction(constraint.normal),
    ) {
      best_value = result;
      penetration = -constraint.signed_distance_to_plane(best_value);
    }
  }

  best_value
}

#[cfg(test)]
#[path = "linear_programming_test.rs"]
mod test;
