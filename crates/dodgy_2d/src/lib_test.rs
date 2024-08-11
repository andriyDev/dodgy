use super::*;

mod get_line_for_neighbour_tests {
  use glam::Vec2;

  use super::{Agent, Line};

  macro_rules! assert_line_eq {
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
        a.direction.distance_squared(b.direction) < 1e-5,
        "\n  left: {:?}\n right: {:?}",
        a,
        b
      );
    }};
  }

  #[test]
  fn velocity_projects_on_cutoff_circle() {
    let position = Vec2::new(1.0, 2.0);
    let radius = 2.0;

    let agent = Agent {
      position: Vec2::ZERO,
      velocity: Vec2::ZERO,
      radius: radius - 1.0,
      avoidance_responsibility: 1.0,
    };

    let neighbour = Agent {
      position: position,
      velocity: Vec2::ZERO,
      radius: 1.0,
      avoidance_responsibility: 1.0,
    };

    let actual_line = agent.get_line_for_neighbour(
      &neighbour, /* time_horizon= */ 1.0, /* time_step= */ 1.0,
    );
    // The agent's velocity projects directly onto the cut-off circle.
    assert_line_eq!(
      actual_line,
      Line {
        point: position.normalize() * (position.length() - radius),
        direction: position.perp().normalize(),
      }
    );
  }

  #[test]
  fn velocity_projects_to_shadow() {
    let mut agent = Agent {
      position: Vec2::ZERO,
      velocity: Vec2::new(1.0, 3.0),
      radius: 1.0,
      avoidance_responsibility: 1.0,
    };

    let neighbour = Agent {
      position: Vec2::new(2.0, 2.0),
      velocity: Vec2::ZERO,
      radius: 1.0,
      avoidance_responsibility: 1.0,
    };

    let inside_shadow_line = agent.get_line_for_neighbour(
      &neighbour, /* time_horizon= */ 1.0, /* time_step= */ 1.0,
    );
    assert_line_eq!(
      inside_shadow_line,
      Line { point: Vec2::new(0.5, 3.0), direction: Vec2::new(0.0, 1.0) }
    );

    agent.velocity = Vec2::new(10.0, -1.0);

    let outside_shadow_line = agent.get_line_for_neighbour(
      &neighbour, /* time_horizon= */ 1.0, /* time_step= */ 1.0,
    );
    assert_line_eq!(
      outside_shadow_line,
      Line { point: Vec2::new(10.0, 0.0), direction: Vec2::new(-1.0, 0.0) }
    );
  }

  #[test]
  fn collision_uses_time_step() {
    let agent = Agent {
      position: Vec2::ZERO,
      velocity: Vec2::new(0.0, 0.0),
      radius: 2.0,
      avoidance_responsibility: 1.0,
    };

    let neighbour = Agent {
      position: Vec2::new(2.0, 2.0),
      velocity: Vec2::ZERO,
      radius: 2.0,
      avoidance_responsibility: 1.0,
    };

    let collision_line = agent.get_line_for_neighbour(
      &neighbour, /* time_horizon= */ 1.0, /* time_step= */ 0.5,
    );
    assert_line_eq!(
      collision_line,
      Line {
        point: (Vec2::ONE.normalize() * -8.0 + Vec2::new(4.0, 4.0)) * 0.5,
        direction: Vec2::new(-1.0, 1.0).normalize(),
      }
    );
  }

  #[test]
  fn no_collision_uses_time_horizon() {
    let agent = Agent {
      position: Vec2::ZERO,
      velocity: Vec2::new(0.0, 0.0),
      radius: 1.0,
      avoidance_responsibility: 1.0,
    };

    let neighbour = Agent {
      position: Vec2::new(2.0, 2.0),
      velocity: Vec2::ZERO,
      radius: 1.0,
      avoidance_responsibility: 1.0,
    };

    let collision_line = agent.get_line_for_neighbour(
      &neighbour, /* time_horizon= */ 2.0, /* time_step= */ 0.5,
    );
    assert_line_eq!(
      collision_line,
      Line {
        point: -Vec2::ONE.normalize() + Vec2::new(1.0, 1.0),
        direction: Vec2::new(-1.0, 1.0).normalize(),
      }
    );
  }

  #[test]
  fn uses_avoidance_responsibility() {
    let agent = Agent {
      position: Vec2::ZERO,
      velocity: Vec2::new(1.5, 0.0),
      radius: 1.0,
      avoidance_responsibility: 1.0,
    };

    let neighbour = Agent {
      position: Vec2::new(4.0, 0.0),
      velocity: Vec2::ZERO,
      radius: 1.0,
      avoidance_responsibility: 3.0,
    };

    let actual_line = agent.get_line_for_neighbour(
      &neighbour, /* time_horizon= */ 2.0, /* time_step= */ 0.5,
    );
    assert_line_eq!(
      actual_line,
      Line { point: Vec2::new(1.375, 0.0), direction: Vec2::new(0.0, 1.0) }
    );
  }

  #[test]
  fn uses_avoidance_responsibility_only_when_inside_vo() {
    let agent = Agent {
      position: Vec2::ZERO,
      velocity: Vec2::new(0.5, 0.0),
      radius: 1.0,
      avoidance_responsibility: 1.0,
    };

    let neighbour = Agent {
      position: Vec2::new(4.0, 0.0),
      velocity: Vec2::ZERO,
      radius: 1.0,
      avoidance_responsibility: 3.0,
    };

    let actual_line = agent.get_line_for_neighbour(
      &neighbour, /* time_horizon= */ 2.0, /* time_step= */ 0.5,
    );
    assert_line_eq!(
      actual_line,
      Line { point: Vec2::new(1.0, 0.0), direction: Vec2::new(0.0, 1.0) }
    );
  }
}

mod compute_avoiding_velocity_tests {
  use super::*;

  #[test]
  fn invalidating_obstacles_falls_back_to_zero_velocity() {
    let agent = Agent {
      position: Vec2::ZERO,
      velocity: Vec2::new(2.0, 0.0),
      radius: 0.5,
      avoidance_responsibility: 1.0,
    };

    let preferred_velocity = Vec2::new(2.0, 0.0);
    let time_step = 0.01;

    let obstacles: Vec<Cow<Obstacle>> = vec![
      Cow::Owned(Obstacle::Closed {
        vertices: vec![
          Vec2::new(1.0, 10.0),
          Vec2::new(1.0, 0.0),
          Vec2::new(2.0, 10.0),
        ],
      }),
      Cow::Owned(Obstacle::Closed {
        vertices: vec![
          Vec2::new(1.0, 1e-6),
          Vec2::new(1.0, -10.0),
          Vec2::new(2.0, -10.0),
        ],
      }),
    ];

    // Just check that this does not panic.
    agent.compute_avoiding_velocity(
      &[],
      &obstacles,
      preferred_velocity,
      /* max_speed= */ 2.0,
      time_step,
      &AvoidanceOptions {
        obstacle_margin: 0.0,
        obstacle_time_horizon: 1.0,
        time_horizon: 1.0,
      },
    );
  }

  #[test]
  fn moves_apart_if_directly_on_top_of_each_other() {
    let agent = Agent {
      position: Vec2::ZERO,
      velocity: Vec2::ZERO,
      radius: 0.5,
      avoidance_responsibility: 1.0,
    };

    let avoiding_velocity = agent.compute_avoiding_velocity(
      &[Cow::Owned(agent.clone())],
      &[],
      /* preferred_velocity= */ Vec2::ZERO,
      /* max_speed= */ 2.0,
      /* time_step= */ 0.01,
      &AvoidanceOptions {
        obstacle_margin: 0.0,
        obstacle_time_horizon: 1.0,
        time_horizon: 1.0,
      },
    );

    // Agents will move in a random direction if they are perfectly on top of
    // one another.
    assert_ne!(avoiding_velocity, Vec2::ZERO);
  }
}
