use glam::Vec2;

use super::VisibilitySet;

#[test]
fn empty_always_visible() {
  let viz = VisibilitySet::new();

  assert!(viz.is_line_visible(Vec2::new(1.0, 0.0), Vec2::new(0.0, 1.0)));
  assert!(viz.is_line_visible(Vec2::new(-3.0, -2.0), Vec2::new(-3.001, -1.0)));
  assert!(viz.is_line_visible(Vec2::new(100.0, 0.1), Vec2::new(-100.0, 0.1)));
}

#[test]
fn one_obstruction() {
  let mut viz = VisibilitySet::new();
  viz.add_line(Vec2::new(1.0, 0.0), Vec2::new(0.0, 1.0));

  assert!(!viz.is_line_visible(Vec2::new(2.0, 1.0), Vec2::new(1.0, 2.0)));
  assert!(viz.is_line_visible(Vec2::new(1.0, 1.0), Vec2::new(-0.01, 2.0)));
  assert!(viz.is_line_visible(Vec2::new(-1.0, 0.0), Vec2::new(0.0, -1.0)));
}

#[test]
fn two_connected_obstructions() {
  let mut viz = VisibilitySet::new();
  viz.add_line(Vec2::new(1.0, 0.0), Vec2::new(0.0, 1.0));

  assert!(viz.is_line_visible(Vec2::new(4.0, 1.0), Vec2::new(4.0, -1.0)));

  viz.add_line(Vec2::new(3.0, 1.0), Vec2::new(3.0, -3.0));

  assert!(!viz.is_line_visible(Vec2::new(4.0, 1.0), Vec2::new(4.0, -1.0)));
  assert!(viz.is_line_visible(Vec2::new(1.0, 1.0), Vec2::new(-0.01, 2.0)));
  assert!(viz.is_line_visible(Vec2::new(-1.0, 0.0), Vec2::new(0.0, -1.0)));
}

#[test]
fn two_disjoint_obstructions() {
  let mut viz = VisibilitySet::new();
  viz.add_line(Vec2::new(1.0, 0.0), Vec2::new(0.0, 1.0));
  viz.add_line(Vec2::new(-1.0, 0.0), Vec2::new(0.0, -1.0));

  assert!(!viz.is_line_visible(Vec2::new(2.0, 1.0), Vec2::new(1.0, 2.0)));
  assert!(!viz.is_line_visible(Vec2::new(-2.0, -1.0), Vec2::new(-1.0, -2.0)));
  assert!(viz.is_line_visible(Vec2::new(-2.0, 1.0), Vec2::new(-1.0, 2.0)));
  assert!(viz.is_line_visible(Vec2::new(2.0, -1.0), Vec2::new(1.0, -2.0)));
}

#[test]
fn touching_edges() {
  let mut viz = VisibilitySet::new();
  viz.add_line(Vec2::new(1.0, 0.0), Vec2::new(0.0, 1.0));
  viz.add_line(Vec2::new(-1.0, 0.0), Vec2::new(0.0, 1.0));

  assert!(viz.is_line_visible(Vec2::new(-1.0, 0.5), Vec2::new(1.0, 0.5)));
  assert!(!viz.is_line_visible(Vec2::new(-1.0, 1.5), Vec2::new(1.0, 1.5)));
}

macro_rules! assert_sets_eq {
  ($actual: expr, { $($e:expr),* }) => {{
    let mut actual_sorted = $actual.iter().copied().collect::<Vec<u32>>();
    let mut expected_sorted = vec![$($e,)*];

    actual_sorted.sort();
    expected_sorted.sort();

    assert_eq!(actual_sorted, expected_sorted);
  }};
}

#[test]
fn layered_walls() {
  let mut viz = VisibilitySet::new();
  let line_0 = viz.add_line(Vec2::new(2.0, 0.0), Vec2::new(0.0, 2.0));
  let line_1 = viz.add_line(Vec2::new(0.75, 0.25), Vec2::new(0.25, 0.75));
  let line_2 = viz.add_line(Vec2::new(3.0, 1.0), Vec2::new(1.0, 3.0));

  assert!(line_2.is_none());

  assert_sets_eq!(viz.get_visible_line_ids(), {
    line_0.unwrap(),
    line_1.unwrap()
  });
}

#[test]
fn obscured_wall_sharing_vertex() {
  let mut viz = VisibilitySet::new();
  let line_0 = viz.add_line(Vec2::new(1.0, 0.0), Vec2::new(0.0, 1.0));
  // This wall is completely obscured.
  viz.add_line(Vec2::new(1.0, 1.0), Vec2::new(0.0, 1.0));

  assert_sets_eq!(viz.get_visible_line_ids(), { line_0.unwrap() });
}

#[test]
fn intersecting_obstructions() {
  let mut viz = VisibilitySet::new();
  let line_0 = viz.add_line(Vec2::new(1.0, 1.0), Vec2::new(-1.0, 1.0));
  // This wall intersects the previous wall.
  let line_1 = viz.add_line(Vec2::new(0.5, 0.9), Vec2::new(-0.5, 1.1));

  assert_sets_eq!(viz.get_visible_line_ids(), { line_0.unwrap(), line_1.unwrap() });

  // This line is in front of both obstructions.
  assert!(viz.is_line_visible(Vec2::new(0.5, 0.75), Vec2::new(-0.5, 0.75)));
  // This line is behind both obstructions.
  assert!(!viz.is_line_visible(Vec2::new(0.5, 1.25), Vec2::new(-0.5, 1.25)));
  // This line intersects the second obstruction, and will be in front of it
  // and the first obstruction on the right.
  assert!(viz.is_line_visible(Vec2::new(0.5, 0.95), Vec2::new(-0.5, 0.95)));
}
