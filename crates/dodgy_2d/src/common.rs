use glam::Vec2;

/// Computes the 2D determinant of `a` and `b`, aka the 2D cross product.
pub fn determinant(a: Vec2, b: Vec2) -> f32 {
  a.x * b.y - a.y * b.x
}

/// Computes the "time" along both lines when the lines intersect. If the lines
/// are parallel, the result is None. The lines are not line segments; the time
/// is allowed to be any number, even negative or greater than one. The
/// resulting Vec2 contains the time for line 1 in the x component, and the time
/// for line 2 in the y component.
pub fn time_to_intersect_lines(
  line_1_start: Vec2,
  line_1_end: Vec2,
  line_2_start: Vec2,
  line_2_end: Vec2,
) -> Option<Vec2> {
  let relative_line_1_start = line_1_start - line_2_start;
  let line_1_delta = line_1_end - line_1_start;
  let line_2_delta = line_2_end - line_2_start;

  let matrix_determinant = determinant(line_2_delta, line_1_delta);
  if matrix_determinant == 0.0 {
    None
  } else {
    // Use some linear algebra to solve this (take the inverse of the line
    // equation matrix).
    Some(
      Vec2::new(
        determinant(relative_line_1_start, line_2_delta),
        determinant(relative_line_1_start, line_1_delta),
      ) / matrix_determinant,
    )
  }
}

#[cfg(test)]
mod tests {
  use glam::Vec2;

  use crate::common::{determinant, time_to_intersect_lines};

  #[test]
  fn determinant_correct() {
    assert_eq!(determinant(Vec2::new(1.0, 2.0), Vec2::new(3.0, 4.0)), -2.0);
  }

  #[test]
  fn intersecting_lines_get_correct_tti() {
    assert_eq!(
      time_to_intersect_lines(
        Vec2::new(0.0, 0.0),
        Vec2::new(4.0, 0.0),
        Vec2::new(1.0, 1.0),
        Vec2::new(1.0, 3.0)
      ),
      Some(Vec2::new(0.25, -0.5))
    );
  }

  #[test]
  fn parallel_lines_get_none_tti() {
    assert_eq!(
      time_to_intersect_lines(
        Vec2::new(0.0, 0.0),
        Vec2::new(4.0, 0.0),
        Vec2::new(1.0, 1.0),
        Vec2::new(5.0, 1.0)
      ),
      None
    );

    assert_eq!(
      time_to_intersect_lines(
        Vec2::new(0.0, 0.0),
        Vec2::new(4.0, 0.0),
        Vec2::new(0.0, 0.0),
        Vec2::new(4.0, 0.0)
      ),
      None
    );
  }
}
