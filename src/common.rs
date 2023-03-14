use glam::Vec2;

pub fn determinant(a: Vec2, b: Vec2) -> f32 {
  a.x * b.y - a.y * b.x
}

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
