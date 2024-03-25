use std::{collections::HashSet, f32::consts::PI, mem::swap};

use glam::Vec2;

use crate::common::time_to_intersect_lines;

pub struct VisibilitySet {
  cones: Vec<Cone>,
  next_id: u32,
}

impl VisibilitySet {
  pub fn new() -> VisibilitySet {
    VisibilitySet { cones: Vec::new(), next_id: 0 }
  }

  pub fn is_line_visible(&self, start_point: Vec2, end_point: Vec2) -> bool {
    if self.cones.len() == 0 {
      return true;
    }

    fn is_cone_visible(set: &VisibilitySet, check_cone: Cone) -> bool {
      for i in -1..set.cones.len() as isize {
        let free_cone = Cone {
          left_ray: ConeRay {
            angle: if i == -1 {
              0.0
            } else {
              set.cones[i as usize].right_ray.angle
            },
            distance: 0.0,
            point: Vec2::ZERO,
          },
          right_ray: ConeRay {
            angle: if i == set.cones.len() as isize - 1 {
              2.0 * PI
            } else {
              set.cones[(i + 1) as usize].left_ray.angle
            },
            distance: 0.0,
            point: Vec2::ZERO,
          },
          line_id: 0,
          related_angle: 0.0,
        };

        if free_cone.left_ray.angle == free_cone.right_ray.angle {
          continue;
        }

        if free_cone.overlaps_cone(check_cone) {
          return true;
        }
      }

      for blocking_cone in set.cones.iter() {
        let check_overlap_cone =
          match check_cone.overlapping_cone(*blocking_cone) {
            None => continue,
            Some(overlap_cone) => overlap_cone,
          };

        let blocking_overlap_cone = blocking_cone
          .overlapping_cone(check_cone)
          .expect("Cones definitely overlap.");

        if check_overlap_cone.left_ray.distance
          < blocking_overlap_cone.left_ray.distance
          || check_overlap_cone.right_ray.distance
            < blocking_overlap_cone.right_ray.distance
        {
          return true;
        }
      }

      false
    }

    match Cone::create_from_line(start_point, end_point, 0) {
      LineToCone::One(cone) => is_cone_visible(self, cone),
      LineToCone::Two(cone_1, cone_2) => {
        is_cone_visible(self, cone_1) || is_cone_visible(self, cone_2)
      }
    }
  }

  pub fn add_line(
    &mut self,
    start_point: Vec2,
    end_point: Vec2,
  ) -> Option<u32> {
    fn add_cone(set: &mut VisibilitySet, new_cone: Cone) -> bool {
      let mut visible = false;

      let mut old_cones = Vec::new();
      swap(&mut set.cones, &mut old_cones);

      for i in -1..old_cones.len() as isize {
        let free_cone = Cone {
          left_ray: ConeRay {
            angle: if i == -1 {
              0.0
            } else {
              old_cones[i as usize].right_ray.angle
            },
            distance: 0.0,
            point: Vec2::ZERO,
          },
          right_ray: ConeRay {
            angle: if i == old_cones.len() as isize - 1 {
              2.0 * PI
            } else {
              old_cones[(i + 1) as usize].left_ray.angle
            },
            distance: 0.0,
            point: Vec2::ZERO,
          },
          line_id: 0,
          related_angle: 0.0,
        };

        if free_cone.left_ray.angle == free_cone.right_ray.angle {
          continue;
        }

        if let Some(overlap_cone) = new_cone.overlapping_cone(free_cone) {
          set.cones.push(overlap_cone);
          visible = true;
        }
      }

      for blocking_cone in old_cones {
        let new_overlap_cone = match new_cone.overlapping_cone(blocking_cone) {
          None => {
            set.cones.push(blocking_cone);
            continue;
          }
          Some(overlap_cone) => overlap_cone,
        };

        let blocking_overlap_cone = blocking_cone
          .overlapping_cone(new_cone)
          .expect("Cones definitely overlap.");

        let left_in_front = new_overlap_cone.left_ray.distance
          < blocking_overlap_cone.left_ray.distance;
        let right_in_front = new_overlap_cone.right_ray.distance
          < blocking_overlap_cone.right_ray.distance;
        if !left_in_front && !right_in_front {
          set.cones.push(blocking_cone);
          continue;
        }

        if blocking_cone.left_ray.angle < blocking_overlap_cone.left_ray.angle {
          set.cones.push(Cone {
            left_ray: blocking_cone.left_ray,
            right_ray: blocking_overlap_cone.left_ray,
            line_id: blocking_cone.line_id,
            related_angle: blocking_cone.related_angle,
          });
        }
        if blocking_overlap_cone.right_ray.angle < blocking_cone.right_ray.angle
        {
          set.cones.push(Cone::create_from_rays(
            blocking_overlap_cone.right_ray,
            blocking_cone.right_ray,
            blocking_cone.line_id,
          ));
        }

        let intersect_times = match time_to_intersect_lines(
          new_overlap_cone.left_ray.point,
          new_overlap_cone.right_ray.point,
          blocking_overlap_cone.left_ray.point,
          blocking_overlap_cone.right_ray.point,
        ) {
          None => {
            set.cones.push(new_overlap_cone);
            visible = true;
            continue;
          }
          Some(intersect_times) => intersect_times,
        };

        if intersect_times.x <= 0.0
          || 1.0 <= intersect_times.x
          || intersect_times.y <= 0.0
          || 1.0 <= intersect_times.y
        {
          set.cones.push(new_overlap_cone);
          visible = true;
          continue;
        }

        let split_point = new_overlap_cone.left_ray.point
          + (new_overlap_cone.right_ray.point
            - new_overlap_cone.left_ray.point)
            * intersect_times.x;
        let split_ray = ConeRay::from_point(split_point);

        visible = true;

        if left_in_front {
          set.cones.push(Cone::create_from_rays(
            new_overlap_cone.left_ray,
            split_ray,
            new_overlap_cone.line_id,
          ));
          set.cones.push(Cone::create_from_rays(
            split_ray,
            blocking_overlap_cone.right_ray,
            blocking_overlap_cone.line_id,
          ));
        } else {
          set.cones.push(Cone::create_from_rays(
            blocking_overlap_cone.left_ray,
            split_ray,
            blocking_overlap_cone.line_id,
          ));
          set.cones.push(Cone::create_from_rays(
            split_ray,
            new_overlap_cone.right_ray,
            new_overlap_cone.line_id,
          ));
        }
      }

      set.cones.sort_by(|a, b| a.left_ray.angle.total_cmp(&b.left_ray.angle));

      visible
    }

    if start_point == end_point {
      return None;
    }

    let visible =
      match Cone::create_from_line(start_point, end_point, self.next_id) {
        LineToCone::One(cone) => add_cone(self, cone),
        LineToCone::Two(cone_1, cone_2) => {
          // Use | to perform an OR without short circuiting.
          add_cone(self, cone_1) | add_cone(self, cone_2)
        }
      };

    if !visible {
      None
    } else {
      let id = self.next_id;
      self.next_id += 1;
      Some(id)
    }
  }

  pub fn get_visible_line_ids(&self) -> HashSet<u32> {
    self.cones.iter().map(|cone| cone.line_id).collect()
  }
}

#[derive(Clone, Copy)]
struct Cone {
  left_ray: ConeRay,
  right_ray: ConeRay,
  line_id: u32,
  related_angle: f32,
}

enum LineToCone {
  One(Cone),
  Two(Cone, Cone),
}

impl Cone {
  fn create_from_line(
    start_point: Vec2,
    end_point: Vec2,
    line_id: u32,
  ) -> LineToCone {
    let mut partial_cone = Cone::create_from_rays(
      ConeRay::from_point(start_point),
      ConeRay::from_point(end_point),
      line_id,
    );

    if partial_cone.left_ray.angle > partial_cone.right_ray.angle {
      swap(&mut partial_cone.left_ray, &mut partial_cone.right_ray);
    }

    if partial_cone.right_ray.angle - partial_cone.left_ray.angle < PI {
      return LineToCone::One(partial_cone);
    }

    let zero_distance = partial_cone.distance_at_angle(0.0);
    let split_ray = ConeRay {
      angle: 0.0,
      distance: zero_distance,
      point: zero_distance * Vec2::X,
    };

    let split_max_ray = ConeRay {
      angle: 2.0 * PI,
      distance: split_ray.distance,
      point: split_ray.point,
    };

    LineToCone::Two(
      Cone::create_from_rays(split_ray, partial_cone.left_ray, line_id),
      Cone::create_from_rays(partial_cone.right_ray, split_max_ray, line_id),
    )
  }

  fn create_from_rays(
    left_ray: ConeRay,
    right_ray: ConeRay,
    line_id: u32,
  ) -> Cone {
    Cone {
      related_angle: Cone::derive_related_angle(
        left_ray.point,
        right_ray.point,
      ),
      left_ray,
      right_ray,
      line_id,
    }
  }

  fn derive_related_angle(left_point: Vec2, right_point: Vec2) -> f32 {
    Vec2::angle_between(left_point - right_point, left_point)
  }

  fn distance_at_angle(&self, angle: f32) -> f32 {
    let relative_angle = {
      let mut relative_angle = angle - self.left_ray.angle;
      if relative_angle < 0.0 {
        relative_angle += 2.0 * PI;
      }
      relative_angle
    };

    self.left_ray.distance * self.related_angle.sin()
      / (relative_angle + self.related_angle).sin()
  }

  fn ray_at_angle(&self, angle: f32) -> ConeRay {
    let distance = self.distance_at_angle(angle);
    ConeRay { angle, distance, point: distance * Vec2::from_angle(angle) }
  }

  fn overlaps_cone(&self, other: Cone) -> bool {
    other.left_ray.angle < self.right_ray.angle
      && self.left_ray.angle < other.right_ray.angle
  }

  fn overlapping_cone(&self, other: Cone) -> Option<Cone> {
    if !self.overlaps_cone(other) {
      return None;
    }

    let left_ray = if self.left_ray.angle < other.left_ray.angle {
      // `self` contains `other.left_ray`.
      self.ray_at_angle(other.left_ray.angle)
    } else {
      // `self` does not contain `other.left_ray`.
      self.left_ray
    };

    let right_ray = if other.right_ray.angle < self.right_ray.angle {
      // `self` contains `other.right_ray`.
      self.ray_at_angle(other.right_ray.angle)
    } else {
      // `self` does not contain `other.right_ray`.
      self.right_ray
    };

    Some(Cone::create_from_rays(left_ray, right_ray, self.line_id))
  }
}

#[derive(Clone, Copy)]
struct ConeRay {
  angle: f32,
  distance: f32,
  point: Vec2,
}

impl ConeRay {
  fn from_point(point: Vec2) -> ConeRay {
    let angle = point.y.atan2(point.x);
    ConeRay {
      angle: if angle >= 0.0 { angle } else { 2.0 * PI + angle },
      distance: point.length(),
      point,
    }
  }
}

#[cfg(test)]
mod tests {
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
}
