use glam::Vec2;

// A half-plane to act as a constraint on the linear program. This is
// represented as a point and a direction, where the valid half-plane resides on
// the "clockwise" side of `direction` and `point`.
pub struct Line {
  pub point: Vec2,
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
    preferred_value,
    max_length,
    /*direction_opt=*/ false,
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

// Solves the 2D linear program. This effectively finds nearest value in the
// intersection of the half-planes to `preferred_value`. `lines` define the
// half-plane constraints and `preferred_value` is the optimal value ignoring
// the constraints. `max_length` limits the magnitude of any valid value.
// TODO: Figure out what direction_opt does.
fn solve_linear_program_2d(
  lines: &[Line],
  preferred_value: Vec2,
  max_length: f32,
  direction_opt: bool,
) -> LinearProgram2DResult {
  todo!()
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
