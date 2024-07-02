use super::*;

mod get_plane_for_neighbour_tests {
  use glam::Vec3;

  use super::*;

  macro_rules! assert_plane_eq {
    ($a: expr, $b: expr) => {{
      let a = $a;
      let b = $b;

      assert!(
        a.point.distance_squared(b.point) < 1e-5,
        "\n  left: {:?}\n right: {:?}",
        a,
        b
      );
      assert!(
        a.normal.distance_squared(b.normal) < 1e-5,
        "\n  left: {:?}\n right: {:?}",
        a,
        b
      );
    }};
  }

  #[test]
  fn velocity_projects_on_cutoff_sphere() {
    let position = Vec3::new(1.0, 2.0, 0.0);
    let radius = 2.0;

    let agent = Agent {
      position: Vec3::ZERO,
      velocity: Vec3::ZERO,
      radius: radius - 1.0,
      avoidance_responsibility: 1.0,
    };

    let neighbour = Agent {
      position: position,
      velocity: Vec3::ZERO,
      radius: 1.0,
      avoidance_responsibility: 1.0,
    };

    let actual_plane = agent.get_plane_for_neighbour(
      &neighbour, /* time_horizon= */ 1.0, /* time_step= */ 1.0,
    );
    // The agent's velocity projects directly onto the cut-off sphere.
    assert_plane_eq!(
      actual_plane,
      Plane {
        point: position.normalize() * (position.length() - radius),
        normal: -position.normalize(),
      }
    );
  }

  #[test]
  fn velocity_projects_to_shadow() {
    let mut agent = Agent {
      position: Vec3::ZERO,
      velocity: Vec3::new(1.0, 3.0, 0.0),
      radius: 1.0,
      avoidance_responsibility: 1.0,
    };

    let neighbour = Agent {
      position: Vec3::new(2.0, 2.0, 0.0),
      velocity: Vec3::ZERO,
      radius: 1.0,
      avoidance_responsibility: 1.0,
    };

    let inside_shadow_plane = agent.get_plane_for_neighbour(
      &neighbour, /* time_horizon= */ 1.0, /* time_step= */ 1.0,
    );
    assert_plane_eq!(
      inside_shadow_plane,
      Plane {
        point: Vec3::new(0.5, 3.0, 0.0),
        normal: Vec3::new(-1.0, 0.0, 0.0),
      }
    );

    agent.velocity = Vec3::new(10.0, -1.0, 0.0);

    let outside_shadow_plane = agent.get_plane_for_neighbour(
      &neighbour, /* time_horizon= */ 1.0, /* time_step= */ 1.0,
    );
    assert_plane_eq!(
      outside_shadow_plane,
      Plane {
        point: Vec3::new(10.0, 0.0, 0.0),
        normal: Vec3::new(0.0, -1.0, 0.0),
      }
    );
  }

  #[test]
  fn collision_uses_time_step() {
    let agent = Agent {
      position: Vec3::ZERO,
      velocity: Vec3::ZERO,
      radius: 2.0,
      avoidance_responsibility: 1.0,
    };

    let neighbour = Agent {
      position: Vec3::new(2.0, 2.0, 0.0),
      velocity: Vec3::ZERO,
      radius: 2.0,
      avoidance_responsibility: 1.0,
    };

    let collision_plane = agent.get_plane_for_neighbour(
      &neighbour, /* time_horizon= */ 1.0, /* time_step= */ 0.5,
    );
    assert_plane_eq!(
      collision_plane,
      Plane {
        point: (Vec3::new(1.0, 1.0, 0.0).normalize() * -8.0
          + Vec3::new(4.0, 4.0, 0.0))
          * 0.5,
        normal: Vec3::new(-1.0, -1.0, 0.0).normalize(),
      }
    );
  }

  #[test]
  fn no_collision_uses_time_horizon() {
    let agent = Agent {
      position: Vec3::ZERO,
      velocity: Vec3::ZERO,
      radius: 1.0,
      avoidance_responsibility: 1.0,
    };

    let neighbour = Agent {
      position: Vec3::new(2.0, 2.0, 0.0),
      velocity: Vec3::ZERO,
      radius: 1.0,
      avoidance_responsibility: 1.0,
    };

    let collision_plane = agent.get_plane_for_neighbour(
      &neighbour, /* time_horizon= */ 2.0, /* time_step= */ 0.5,
    );
    assert_plane_eq!(
      collision_plane,
      Plane {
        point: -Vec3::new(1.0, 1.0, 0.0).normalize() + Vec3::new(1.0, 1.0, 0.0),
        normal: Vec3::new(-1.0, -1.0, 0.0).normalize(),
      }
    );
  }

  #[test]
  fn uses_avoidance_responsibility() {
    let agent = Agent {
      position: Vec3::ZERO,
      velocity: Vec3::new(1.5, 0.0, 0.0),
      radius: 1.0,
      avoidance_responsibility: 1.0,
    };

    let neighbour = Agent {
      position: Vec3::new(4.0, 0.0, 0.0),
      velocity: Vec3::ZERO,
      radius: 1.0,
      avoidance_responsibility: 3.0,
    };

    let actual_plane = agent.get_plane_for_neighbour(
      &neighbour, /* time_horizon= */ 2.0, /* time_step= */ 0.5,
    );
    assert_plane_eq!(
      actual_plane,
      Plane {
        point: Vec3::new(1.375, 0.0, 0.0),
        normal: Vec3::new(-1.0, 0.0, 0.0),
      }
    );
  }

  #[test]
  fn uses_avoidance_responsibility_only_when_inside_vo() {
    let agent = Agent {
      position: Vec3::ZERO,
      velocity: Vec3::new(0.5, 0.0, 0.0),
      radius: 1.0,
      avoidance_responsibility: 1.0,
    };

    let neighbour = Agent {
      position: Vec3::new(4.0, 0.0, 0.0),
      velocity: Vec3::ZERO,
      radius: 1.0,
      avoidance_responsibility: 3.0,
    };

    let actual_plane = agent.get_plane_for_neighbour(
      &neighbour, /* time_horizon= */ 2.0, /* time_step= */ 0.5,
    );
    assert_plane_eq!(
      actual_plane,
      Plane {
        point: Vec3::new(1.0, 0.0, 0.0),
        normal: Vec3::new(-1.0, 0.0, 0.0),
      }
    );
  }
}
