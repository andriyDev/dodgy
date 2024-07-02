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

mod solve_linear_program_along_plane_tests {
  use glam::Vec3;

  use crate::linear_programming::{
    solve_linear_program_along_plane, OptimalValue,
  };

  use super::Plane;

  #[test]
  fn uses_projected_optimal_point() {
    let value_plane = Plane {
      point: Vec3::new(2.0, 2.0, 2.0),
      normal: Vec3::new(1.0, 0.0, 1.0).normalize(),
    };

    assert_vec3_near!(
      solve_linear_program_along_plane(
        &value_plane,
        10.0,
        &[],
        &OptimalValue::Point(Vec3::new(0.0, 1.0, 0.0)),
      )
      .unwrap(),
      Vec3::new(2.0, 1.0, 2.0)
    );

    assert_vec3_near!(
      solve_linear_program_along_plane(
        &value_plane,
        10.0,
        &[],
        &OptimalValue::Point(Vec3::new(0.0, 15.0, 0.0)),
      )
      .unwrap(),
      // The radius of the sphere around the origin is 10.0. The distance of
      // the plane from the origin is 2*sqrt(2), so the radius of the
      // intersection is sqrt(92).
      Vec3::new(2.0, 92f32.sqrt(), 2.0)
    );
  }

  #[test]
  fn uses_projected_optimal_direction() {
    let value_plane = Plane {
      // This is equivalent to the point (2.0, 0.0, 2.0).
      point: Vec3::new(-10.0, 10.0, 14.0),
      normal: Vec3::new(1.0, 0.0, 1.0).normalize(),
    };

    assert_vec3_near!(
      solve_linear_program_along_plane(
        &value_plane,
        10.0,
        &[],
        &OptimalValue::Direction(Vec3::new(0.0, 1.0, 0.0)),
      )
      .unwrap(),
      // The radius of the sphere around the origin is 10.0. The distance of
      // the plane from the origin is 2*sqrt(2), so the radius of the
      // intersection is sqrt(92).
      Vec3::new(2.0, 92f32.sqrt(), 2.0)
    );

    // The direction is perpendicular to the surface, so we just pick the
    // center of the valid circle along the plane.
    assert_vec3_near!(
      solve_linear_program_along_plane(
        &value_plane,
        10.0,
        &[],
        &OptimalValue::Direction(Vec3::new(1.0, 0.0, 1.0).normalize()),
      )
      .unwrap(),
      Vec3::new(2.0, 0.0, 2.0)
    );
  }

  #[test]
  fn satisfies_constraints() {
    let value_plane =
      Plane { point: Vec3::ZERO, normal: Vec3::new(0.0, 0.0, 1.0) };

    let constraints = [
      Plane {
        point: Vec3::new(0.0, -0.1, 0.0),
        normal: Vec3::new(0.0, 1.0, 0.0),
      },
      Plane {
        point: Vec3::new(0.0, 0.5, 0.0),
        normal: Vec3::new(-1.0, -1.0, 0.0),
      },
    ];

    assert_vec3_near!(
      solve_linear_program_along_plane(
        &value_plane,
        1.0,
        &constraints,
        &OptimalValue::Direction(Vec3::new(1.0, 0.0, 0.0))
      )
      .unwrap(),
      Vec3::new(0.6, -0.1, 0.0)
    );
  }

  #[test]
  fn constraints_are_infeasible() {
    let value_plane =
      Plane { point: Vec3::ZERO, normal: Vec3::new(0.0, 0.0, 1.0) };

    let constraints = [
      Plane {
        point: Vec3::new(0.0, -0.1, 0.0),
        normal: Vec3::new(0.0, -1.0, 0.0),
      },
      Plane {
        point: Vec3::new(0.0, 0.1, 0.0),
        normal: Vec3::new(0.0, 1.0, 0.0),
      },
    ];

    assert_eq!(
      solve_linear_program_along_plane(
        &value_plane,
        1.0,
        &constraints,
        &OptimalValue::Point(Vec3::ZERO)
      ),
      Err(())
    );
  }

  #[test]
  fn value_plane_and_constraint_are_parallel() {
    let value_plane =
      Plane { point: Vec3::ZERO, normal: Vec3::new(0.0, 0.0, 1.0) };

    // This constraint invalidates the whole value plane.
    let constraints = [Plane {
      point: Vec3::new(0.0, 0.0, 0.1),
      normal: Vec3::new(0.0, 0.0, 1.0),
    }];

    assert_eq!(
      solve_linear_program_along_plane(
        &value_plane,
        1.0,
        &constraints,
        &OptimalValue::Point(Vec3::ZERO)
      ),
      Err(())
    );
  }

  #[test]
  fn value_plane_outside_radius() {
    // All these values have a distance larger than 1.0.
    let value_plane = Plane {
      point: Vec3::new(0.0, 0.0, 1.1),
      normal: Vec3::new(0.0, 0.0, 1.0),
    };

    assert_eq!(
      solve_linear_program_along_plane(
        &value_plane,
        1.0,
        &[],
        &OptimalValue::Point(Vec3::ZERO)
      ),
      Err(())
    );
  }
}

mod solve_linear_program_3d_tests {
  use glam::Vec3;

  use crate::linear_programming::{
    solve_linear_program_3d, LinearProgram3DResult, OptimalValue, Plane,
  };

  fn unwrap_feasible(result: LinearProgram3DResult) -> Vec3 {
    match result {
      LinearProgram3DResult::Feasible(result) => result,
      other => panic!("expected feasible, got {:?}", other),
    }
  }

  fn unwrap_infeasible(result: LinearProgram3DResult) -> (usize, Vec3) {
    match result {
      LinearProgram3DResult::Infeasible {
        index_of_failed_line,
        partial_value,
      } => (index_of_failed_line, partial_value),
      other => panic!("expected infeasible, got {:?}", other),
    }
  }

  #[test]
  fn uses_projected_optimal_point() {
    let one_over_root_2 = 1.0f32 / 2.0f32.sqrt();

    assert_vec3_near!(
      unwrap_feasible(solve_linear_program_3d(
        Default::default(),
        1.0,
        &OptimalValue::Point(Vec3::new(0.5, 0.25, 0.0)),
      )),
      Vec3::new(0.5, 0.25, 0.0)
    );

    assert_vec3_near!(
      unwrap_feasible(solve_linear_program_3d(
        Default::default(),
        1.0,
        &OptimalValue::Point(Vec3::new(1.0, 1.0, 0.0)),
      )),
      Vec3::new(one_over_root_2, one_over_root_2, 0.0)
    );
  }

  #[test]
  fn uses_optimal_direction() {
    let one_over_root_2 = 1.0f32 / 2.0f32.sqrt();

    assert_vec3_near!(
      unwrap_feasible(solve_linear_program_3d(
        Default::default(),
        3.0,
        &OptimalValue::Direction(Vec3::new(
          0.0,
          one_over_root_2,
          one_over_root_2
        )),
      )),
      Vec3::new(0.0, one_over_root_2 * 3.0, one_over_root_2 * 3.0)
    );

    assert_vec3_near!(
      unwrap_feasible(solve_linear_program_3d(
        Default::default(),
        5.0,
        &OptimalValue::Direction(Vec3::new(
          0.0,
          one_over_root_2,
          -one_over_root_2
        )),
      )),
      Vec3::new(0.0, one_over_root_2 * 5.0, one_over_root_2 * -5.0)
    );
  }

  #[test]
  fn projects_to_constraint() {
    let constraints = [Plane {
      point: Vec3::new(0.5, 0.5, 0.5),
      normal: Vec3::ONE.normalize(),
    }];

    assert_vec3_near!(
      unwrap_feasible(solve_linear_program_3d(
        &constraints,
        1.0,
        &OptimalValue::Point(Vec3::ZERO),
      )),
      Vec3::new(0.5, 0.5, 0.5)
    );
  }

  #[test]
  fn constraints_are_infeasible() {
    let constraints = [
      // This constraint is always satisfied.
      Plane {
        point: Vec3::new(0.5, 0.0, 0.0),
        normal: Vec3::new(-1.0, 0.0, 0.0),
      },
      Plane {
        point: Vec3::new(0.0, 0.1, 0.0),
        normal: Vec3::new(0.0, 1.0, 0.0),
      },
      Plane {
        point: Vec3::new(0.0, -0.1, 0.0),
        normal: Vec3::new(0.0, -1.0, 0.0),
      },
    ];

    let (index_of_failed_line, partial_value) =
      unwrap_infeasible(solve_linear_program_3d(
        &constraints,
        1.0,
        &OptimalValue::Point(Vec3::ZERO),
      ));

    assert_eq!(index_of_failed_line, 2);
    assert_vec3_near!(partial_value, Vec3::new(0.0, 0.1, 0.0));
  }
}

mod solve_linear_program_4d_tests {
  use glam::Vec3;

  use super::{solve_linear_program_4d, Plane};

  #[test]
  fn finds_least_penetrating_value() {
    let constraints = [
      Plane {
        point: Vec3::new(0.0, 1.0, 0.0),
        normal: Vec3::new(0.0, 1.0, 0.0),
      },
      // Redundant constraint to first constraint.
      Plane {
        point: Vec3::new(0.0, 1.0, 0.0),
        normal: Vec3::new(0.0, 1.0, 0.0),
      },
      Plane {
        point: Vec3::new(1.0, 0.0, 0.0),
        normal: Vec3::new(1.0, 0.0, 0.0),
      },
      Plane {
        point: Vec3::new(-2.0, -2.0, 0.0),
        normal: Vec3::new(-1.0, -1.0, 0.0).normalize(),
      },
    ];

    assert_vec3_near!(
      solve_linear_program_4d(
        &constraints,
        /* radius= */ 10.0,
        /* index_of_failed_line= */ 3,
        /* partial_value= */ Vec3::new(1.0, 1.0, 0.0)
      ),
      Vec3::new(-0.75736, -0.75736, 9.94248)
    );
  }
}

mod solve_linear_program_tests {
  use glam::Vec3;

  use super::{solve_linear_program, Plane};

  #[test]
  fn finds_valid_value_when_feasible() {
    let constraints = [
      Plane {
        point: Vec3::new(0.0, 1.0, 0.0),
        normal: Vec3::new(0.0, 1.0, 0.0),
      },
      Plane {
        point: Vec3::new(1.0, 0.0, 0.0),
        normal: Vec3::new(1.0, 0.0, 0.0),
      },
      Plane {
        point: Vec3::new(0.0, 0.0, 1.0),
        normal: Vec3::new(0.0, 0.0, 1.0),
      },
    ];

    assert_vec3_near!(
      solve_linear_program(
        &constraints,
        /* radius= */ 10.0,
        /* preferred_value= */ Vec3::ZERO,
      ),
      Vec3::new(1.0, 1.0, 1.0)
    );
  }

  #[test]
  fn finds_least_penetrating_value_when_infeasible() {
    let constraints = [
      Plane {
        point: Vec3::new(0.0, 1.0, 0.0),
        normal: Vec3::new(0.0, 1.0, 0.0),
      },
      // Redundant constraint to first constraint.
      Plane {
        point: Vec3::new(0.0, 1.0, 0.0),
        normal: Vec3::new(0.0, 1.0, 0.0),
      },
      Plane {
        point: Vec3::new(1.0, 0.0, 0.0),
        normal: Vec3::new(1.0, 0.0, 0.0),
      },
      Plane {
        point: Vec3::new(-2.0, -2.0, 0.0),
        normal: Vec3::new(-1.0, -1.0, 0.0).normalize(),
      },
    ];

    assert_vec3_near!(
      solve_linear_program(
        &constraints,
        /* radius= */ 10.0,
        /* preferred_value= */ Vec3::ZERO,
      ),
      Vec3::new(-0.75736, -0.75736, 9.94248)
    );
  }
}
