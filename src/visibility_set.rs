use glam::Vec2;

pub struct VisibilitySet;

impl VisibilitySet {
  pub fn new() -> VisibilitySet {
    VisibilitySet
  }

  pub fn is_line_visible(&self, start_point: Vec2, end_point: Vec2) -> bool {
    todo!()
  }

  pub fn add_line(&mut self, start_point: Vec2, end_point: Vec2) -> u32 {
    todo!()
  }

  pub fn get_visible_line_ids(&self) -> Vec<u32> {
    todo!()
  }
}
