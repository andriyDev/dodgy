use super::*;

macro_rules! assert_vec2_near {
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
  use glam::Vec2;

  use super::{solve_linear_program_along_line, Line, OptimalValue};

  #[test]
  fn projects_optimal_point_with_no_constraints() {
    // Compute what the circle height should be at the 0.5 mark.
    let circle_height_at_half = (1.0f32 - 0.5 * 0.5).sqrt();

    let valid_line =
      Line { direction: Vec2::new(0.0, 1.0), point: Vec2::new(0.5, 0.0) };

    assert_eq!(
      solve_linear_program_along_line(
        &valid_line,
        1.0,
        Default::default(),
        &OptimalValue::Point(Vec2::new(5.0, 0.25)),
      ),
      Ok(Vec2::new(0.5, 0.25))
    );

    assert_eq!(
      solve_linear_program_along_line(
        &valid_line,
        1.0,
        Default::default(),
        &OptimalValue::Point(Vec2::new(5.0, 2.0)),
      ),
      Ok(Vec2::new(0.5, circle_height_at_half))
    );

    assert_eq!(
      solve_linear_program_along_line(
        &valid_line,
        1.0,
        Default::default(),
        &OptimalValue::Point(Vec2::new(5.0, -100.0)),
      ),
      Ok(Vec2::new(0.5, -circle_height_at_half))
    );
  }

  #[test]
  fn projects_optimal_direction_with_no_constraints() {
    // Compute what the circle height should be at the 0.5 mark.
    let circle_height_at_half = (1.0f32 - 0.5 * 0.5).sqrt();

    let valid_line =
      Line { direction: Vec2::new(0.0, 1.0), point: Vec2::new(0.5, 0.0) };

    assert_eq!(
      solve_linear_program_along_line(
        &valid_line,
        1.0,
        Default::default(),
        &OptimalValue::Direction(Vec2::new(1.0, 0.5).normalize()),
      ),
      Ok(Vec2::new(0.5, circle_height_at_half))
    );

    assert_eq!(
      solve_linear_program_along_line(
        &valid_line,
        1.0,
        Default::default(),
        &OptimalValue::Direction(Vec2::new(1.0, -0.5).normalize()),
      ),
      Ok(Vec2::new(0.5, -circle_height_at_half))
    );
  }

  #[test]
  fn constraints_remove_valid_values() {
    let valid_line =
      Line { direction: Vec2::new(0.0, 1.0), point: Vec2::new(0.5, 0.0) };

    let constraints = [
      // This line intersects `valid_line` at (0.5, 0.5).
      Line { direction: Vec2::new(-1.0, 0.0), point: Vec2::new(-100.0, 0.5) },
      // This line intersects `valid_line` at (0.5, -0.75).
      Line {
        direction: Vec2::new(1.0, 1.0).normalize(),
        point: Vec2::new(0.25, -1.0),
      },
    ];

    // The middle value should be unchanged.
    assert_eq!(
      solve_linear_program_along_line(
        &valid_line,
        1.0,
        &constraints,
        &OptimalValue::Point(Vec2::new(-5.0, 0.25)),
      ),
      Ok(Vec2::new(0.5, 0.25))
    );

    assert_eq!(
      solve_linear_program_along_line(
        &valid_line,
        1.0,
        &constraints,
        &OptimalValue::Point(Vec2::new(-5.0, 1.0)),
      ),
      Ok(Vec2::new(0.5, 0.5))
    );

    assert_eq!(
      solve_linear_program_along_line(
        &valid_line,
        1.0,
        &constraints,
        &OptimalValue::Point(Vec2::new(-5.0, -1.0)),
      ),
      Ok(Vec2::new(0.5, -0.75))
    );
  }

  #[test]
  fn constraints_are_infeasible() {
    let valid_line =
      Line { direction: Vec2::new(0.0, 1.0), point: Vec2::new(0.5, 0.0) };

    let constraints = [
      // This line intersects `valid_line` at (0.5, 0.5), and invalidates all
      // points below.
      Line { direction: Vec2::new(1.0, 0.0), point: Vec2::new(-100.0, 0.5) },
      // This line intersects `valid_line` at (0.5, -0.5), and invalides all
      // points above.
      Line { direction: Vec2::new(-1.0, 0.0), point: Vec2::new(-100.0, -0.5) },
    ];

    // The middle value should be unchanged.
    assert_eq!(
      solve_linear_program_along_line(
        &valid_line,
        1.0,
        &constraints,
        &OptimalValue::Point(Vec2::ZERO),
      ),
      Err(())
    );
  }

  #[test]
  fn valid_line_outside_circle() {
    let valid_line =
      Line { direction: Vec2::new(0.0, 1.0), point: Vec2::new(2.0, 0.0) };

    assert_eq!(
      solve_linear_program_along_line(
        &valid_line,
        1.0,
        Default::default(),
        &OptimalValue::Point(Vec2::ZERO),
      ),
      Err(())
    );
  }
}

mod solve_linear_program_2d_tests {
  use glam::Vec2;

  use crate::linear_programming::LinearProgram2DResult;

  use super::{solve_linear_program_2d, Line, OptimalValue};

  #[test]
  fn uses_projected_optimal_point() {
    let one_over_root_2 = 1.0f32 / 2.0f32.sqrt();

    assert_eq!(
      solve_linear_program_2d(
        Default::default(),
        1.0,
        &OptimalValue::Point(Vec2::new(0.5, 0.25)),
      ),
      LinearProgram2DResult::Feasible(Vec2::new(0.5, 0.25))
    );

    assert_eq!(
      solve_linear_program_2d(
        Default::default(),
        1.0,
        &OptimalValue::Point(Vec2::new(1.0, 1.0)),
      ),
      LinearProgram2DResult::Feasible(Vec2::new(
        one_over_root_2,
        one_over_root_2
      ))
    );
  }

  #[test]
  fn uses_optimal_direction() {
    let one_over_root_2 = 1.0f32 / 2.0f32.sqrt();

    assert_eq!(
      solve_linear_program_2d(
        Default::default(),
        3.0,
        &OptimalValue::Direction(Vec2::new(one_over_root_2, one_over_root_2)),
      ),
      LinearProgram2DResult::Feasible(Vec2::new(
        one_over_root_2 * 3.0,
        one_over_root_2 * 3.0
      ))
    );

    assert_eq!(
      solve_linear_program_2d(
        Default::default(),
        5.0,
        &OptimalValue::Direction(Vec2::new(one_over_root_2, -one_over_root_2)),
      ),
      LinearProgram2DResult::Feasible(Vec2::new(
        one_over_root_2 * 5.0,
        one_over_root_2 * -5.0
      ))
    );
  }

  #[test]
  fn satisfies_constraints() {
    let one_over_root_2 = 1.0f32 / 2.0f32.sqrt();

    let constraints = [
      Line { direction: Vec2::new(0.0, 1.0), point: Vec2::new(0.5, 0.0) },
      Line { direction: Vec2::new(1.0, 0.0), point: Vec2::new(-0.5, -0.25) },
    ];

    // Same in, same out.
    assert_eq!(
      solve_linear_program_2d(
        &constraints,
        1.0,
        &OptimalValue::Point(Vec2::new(-0.1, 0.3))
      ),
      LinearProgram2DResult::Feasible(Vec2::new(-0.1, 0.3))
    );

    // Limited to radius.
    assert_eq!(
      solve_linear_program_2d(
        &constraints,
        1.0,
        &OptimalValue::Point(Vec2::new(-2.0, 2.0))
      ),
      LinearProgram2DResult::Feasible(Vec2::new(
        -one_over_root_2,
        one_over_root_2
      ))
    );

    // Restricted by `constraints[0]`.
    assert_eq!(
      solve_linear_program_2d(
        &constraints,
        1.0,
        &OptimalValue::Point(Vec2::new(2.0, 0.5))
      ),
      LinearProgram2DResult::Feasible(Vec2::new(0.5, 0.5))
    );

    // Restricted by `constraints[1]`.
    assert_eq!(
      solve_linear_program_2d(
        &constraints,
        1.0,
        &OptimalValue::Point(Vec2::new(0.0, -0.5))
      ),
      LinearProgram2DResult::Feasible(Vec2::new(0.0, -0.25))
    );

    // Restricted by both constraints.
    assert_eq!(
      solve_linear_program_2d(
        &constraints,
        1.0,
        &OptimalValue::Point(Vec2::new(1.0, -0.5))
      ),
      LinearProgram2DResult::Feasible(Vec2::new(0.5, -0.25))
    );
  }

  #[test]
  fn constraints_are_infeasible() {
    let constraints = [
      Line { direction: Vec2 { x: 0.0, y: 1.0 }, point: Vec2::ZERO },
      Line { direction: Vec2 { x: 1.0, y: 0.0 }, point: Vec2::ZERO },
      Line {
        direction: Vec2 { x: -1.0, y: -1.0 }.normalize(),
        point: Vec2::new(0.1, -0.1),
      },
    ];

    assert_eq!(
      solve_linear_program_2d(
        &constraints,
        1.0,
        &OptimalValue::Point(Vec2::ONE)
      ),
      LinearProgram2DResult::Infeasible {
        index_of_failed_line: 2,
        partial_value: Vec2::new(0.0, 1.0)
      }
    )
  }

  #[test]
  fn optimal_direction_uses_constraint_lines() {
    // Compute what the circle height should be at the 0.5 mark.
    let circle_height_at_half = (1.0f32 - 0.5 * 0.5).sqrt();

    let constraints = [
      Line { direction: Vec2 { x: 0.0, y: 1.0 }, point: Vec2::new(0.5, 0.0) },
      Line { direction: Vec2 { x: 1.0, y: 0.0 }, point: Vec2::new(-0.5, -0.3) },
      Line {
        direction: Vec2 { x: -1.0, y: -1.0 }.normalize(),
        point: Vec2::new(-0.6, 0.6),
      },
    ];

    // Direction points towards intersection of first two constraints.
    assert_eq!(
      solve_linear_program_2d(
        &constraints,
        1.0,
        &OptimalValue::Direction(Vec2::new(1.0, -1.0).normalize())
      ),
      LinearProgram2DResult::Feasible(Vec2::new(0.5, -0.3))
    );

    // Direction points (barely) towards intersection of first constraint and
    // circle radius.
    assert_eq!(
      solve_linear_program_2d(
        &constraints,
        1.0,
        &OptimalValue::Direction(Vec2::new(1.0, 0.5).normalize())
      ),
      LinearProgram2DResult::Feasible(Vec2::new(0.5, circle_height_at_half))
    );

    // Direction points towards top of circle (missing all constraints).
    assert_eq!(
      solve_linear_program_2d(
        &constraints,
        1.0,
        &OptimalValue::Direction(Vec2::new(0.0, 1.0))
      ),
      LinearProgram2DResult::Feasible(Vec2::new(0.0, 1.0))
    );
  }
}

mod solve_linear_program_3d_tests {
  use glam::Vec2;
  use std::f32::consts::PI;

  use super::{solve_linear_program_3d, Line};

  #[test]
  fn minimally_penetrates_constraints() {
    let constraints = [
      Line { direction: Vec2::new(1.0, 0.0), point: Vec2::new(-100.0, 0.0) },
      Line { direction: Vec2::new(0.0, -1.0), point: Vec2::new(0.0, 0.0) },
      Line {
        direction: Vec2::new(-1.0, 1.0).normalize(),
        point: Vec2::new(0.0, -1.0),
      },
    ];

    let root_2 = 2.0f32.sqrt();

    assert_vec2_near!(
      solve_linear_program_3d(
        &constraints,
        /* rigid_constraint_count= */ 0,
        /* radius= */ 2.0,
        /* index_of_failed_line= */ 0,
        Vec2::new(0.0, 0.0)
      ),
      // I had to do some math to solve this. This is the point equa-distant
      // from all three constraint lines.
      Vec2::new(-1.0, -1.0).normalize() * (root_2 / (2.0 + root_2))
    );
  }

  #[test]
  fn rigid_constraints_never_relaxed() {
    let constraints = [
      Line { direction: Vec2::new(1.0, 0.0), point: Vec2::new(-100.0, 0.0) },
      Line { direction: Vec2::new(0.0, -1.0), point: Vec2::new(0.0, 0.0) },
      Line {
        direction: Vec2::new(-1.0, 1.0).normalize(),
        point: Vec2::new(0.0, -1.0),
      },
    ];

    // The first two constraints cannot be relaxed, so (0, 0) is the best
    // value (nearest to satisfying the third constraint).
    assert_vec2_near!(
      solve_linear_program_3d(
        &constraints,
        /* rigid_constraint_count= */ 2,
        /* radius= */ 2.0,
        /* index_of_failed_line= */ 2,
        Vec2::new(0.0, 0.0)
      ),
      Vec2::new(0.0, 0.0)
    );

    // The first constraint cannot be relaxed, so find the best value between
    // the last two constraints. Constraint 2 and 3 are 45 degrees apart, so
    // find the intersection point of constraint 1 and the 22.5 degree line
    // between constraint 2 and 3. Turns out that is
    // $sin(pi / 8) / cos(pi / 8)$ (sin for the angle, cos to make sure the
    // adjacent side length is 1).
    assert_vec2_near!(
      solve_linear_program_3d(
        &constraints,
        /* rigid_constraint_count= */ 1,
        /* radius= */ 2.0,
        /* index_of_failed_line= */ 1,
        Vec2::new(0.0, 0.0)
      ),
      Vec2::new(-(PI / 8.0).tan(), 0.0)
    );
  }
}

mod solve_linear_program_tests {
  use glam::Vec2;
  use std::f32::consts::PI;

  use super::{solve_linear_program, Line};

  #[test]
  fn uses_projected_optimal_point() {
    let one_over_root_2 = 1.0f32 / 2.0f32.sqrt();

    assert_eq!(
      solve_linear_program(
        Default::default(),
        /* rigid_constraint_count= */ 0,
        /* radius= */ 1.0,
        Vec2::new(0.5, 0.25)
      ),
      Vec2::new(0.5, 0.25)
    );

    assert_eq!(
      solve_linear_program(
        Default::default(),
        /* rigid_constraint_count= */ 0,
        /* radius= */ 1.0,
        Vec2::new(1.0, 1.0)
      ),
      Vec2::new(one_over_root_2, one_over_root_2)
    );
  }

  #[test]
  fn satisfies_constraints() {
    let one_over_root_2 = 1.0f32 / 2.0f32.sqrt();

    let constraints = [
      Line { direction: Vec2::new(0.0, 1.0), point: Vec2::new(0.5, 0.0) },
      Line { direction: Vec2::new(1.0, 0.0), point: Vec2::new(-0.5, -0.25) },
    ];

    // Same in, same out.
    assert_eq!(
      solve_linear_program(
        &constraints,
        /* rigid_constraint_count= */ 0,
        /* radius= */ 1.0,
        Vec2::new(-0.1, 0.3)
      ),
      Vec2::new(-0.1, 0.3)
    );

    // Limited to radius.
    assert_eq!(
      solve_linear_program(
        &constraints,
        /* rigid_constraint_count= */ 0,
        /* radius= */ 1.0,
        Vec2::new(-2.0, 2.0)
      ),
      Vec2::new(-one_over_root_2, one_over_root_2)
    );

    // Restricted by `constraints[0]`.
    assert_eq!(
      solve_linear_program(
        &constraints,
        /* rigid_constraint_count= */ 0,
        /* radius= */ 1.0,
        Vec2::new(2.0, 0.5)
      ),
      Vec2::new(0.5, 0.5)
    );

    // Restricted by `constraints[1]`.
    assert_eq!(
      solve_linear_program(
        &constraints,
        /* rigid_constraint_count= */ 0,
        /* radius= */ 1.0,
        Vec2::new(0.0, -0.5)
      ),
      Vec2::new(0.0, -0.25)
    );

    // Restricted by both constraints.
    assert_eq!(
      solve_linear_program(
        &constraints,
        /* rigid_constraint_count= */ 0,
        /* radius= */ 1.0,
        Vec2::new(1.0, -0.5)
      ),
      Vec2::new(0.5, -0.25)
    );
  }

  #[test]
  fn infeasible_program_minimally_penetrates_constraints() {
    let constraints = [
      Line { direction: Vec2::new(1.0, 0.0), point: Vec2::new(-100.0, 0.0) },
      Line { direction: Vec2::new(0.0, -1.0), point: Vec2::new(0.0, 0.0) },
      Line {
        direction: Vec2::new(-1.0, 1.0).normalize(),
        point: Vec2::new(0.0, -1.0),
      },
    ];

    let root_2 = 2.0f32.sqrt();

    assert_vec2_near!(
      solve_linear_program(
        &constraints,
        /* rigid_constraint_count= */ 0,
        /* radius= */ 2.0,
        /* preferred_value= */ Vec2::new(1.0, 1.0)
      ),
      // I had to do some math to solve this. This is the point equa-distant
      // from all three constraint lines.
      Vec2::new(-1.0, -1.0).normalize() * (root_2 / (2.0 + root_2))
    );
  }

  #[test]
  fn rigid_constraints_never_relaxed_when_infeasible() {
    let constraints = [
      Line { direction: Vec2::new(1.0, 0.0), point: Vec2::new(-100.0, 0.0) },
      Line { direction: Vec2::new(0.0, -1.0), point: Vec2::new(0.0, 0.0) },
      Line {
        direction: Vec2::new(-1.0, 1.0).normalize(),
        point: Vec2::new(0.0, -1.0),
      },
    ];

    // The first two constraints cannot be relaxed, so (0, 0) is the best
    // value (nearest to satisfying the third constraint).
    assert_vec2_near!(
      solve_linear_program(
        &constraints,
        /* rigid_constraint_count= */ 2,
        /* radius= */ 2.0,
        /* preferred_value= */ Vec2::new(0.0, 0.0)
      ),
      Vec2::new(0.0, 0.0)
    );

    // The first constraint cannot be relaxed, so find the best value between
    // the last two constraints. Constraint 2 and 3 are 45 degrees apart, so
    // find the intersection point of constraint 1 and the 22.5 degree line
    // between constraint 2 and 3. Turns out that is
    // $sin(pi / 8) / cos(pi / 8)$ (sin for the angle, cos to make sure the
    // adjacent side length is 1).
    assert_vec2_near!(
      solve_linear_program(
        &constraints,
        /* rigid_constraint_count= */ 1,
        /* radius= */ 2.0,
        /* preferred_value= */ Vec2::new(0.0, 0.0)
      ),
      Vec2::new(-(PI / 8.0).tan(), 0.0)
    );
  }
}
