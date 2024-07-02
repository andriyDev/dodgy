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
