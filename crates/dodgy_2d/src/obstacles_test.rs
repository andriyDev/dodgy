use super::*;

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
fn covered_edge_is_skipped() {
  let agent = Agent {
    position: Vec2::new(0.0, -1.0),
    velocity: Vec2::new(0.0, 0.0),
    radius: 1.0,
    avoidance_responsibility: 1.0,
  };

  let line = get_line_for_agent_to_edge(
    &agent,
    EdgeVertex { point: Vec2::new(-1.0, 0.0), convex: true },
    EdgeVertex { point: Vec2::new(1.0, 0.0), convex: true },
    None,
    None,
    /* edge_margin= */ agent.radius,
    /* time_horizoon= */ 0.5,
    &[Line { point: Vec2::new(-2.5, 1.5), direction: Vec2::new(-1.0, 1.0) }],
  );

  assert!(line.is_some(), "Line should have been Some since the existing line did not fully cover the edge.");

  let line = get_line_for_agent_to_edge(
    &agent,
    EdgeVertex { point: Vec2::new(-1.0, 0.0), convex: true },
    EdgeVertex { point: Vec2::new(1.0, 0.0), convex: true },
    None,
    None,
    /* edge_margin= */ agent.radius,
    /* time_horizoon= */ 1.0,
    &[Line { point: Vec2::new(-3.0, 1.0), direction: Vec2::new(-1.0, 1.0) }],
  );
  assert!(line.is_none(), "Line should have been None due to an existing line covering the edge. Line was {:?}", line);
}

#[test]
fn agent_collides_with_edge() {
  let agent = Agent {
    position: Vec2::new(0.0, -0.1),
    velocity: Vec2::new(0.0, -1.0),
    radius: 1.0,
    avoidance_responsibility: 1.0,
  };

  let vertices =
    vec![Vec2::new(-1.0, 0.0), Vec2::new(1.0, 0.0), Vec2::new(0.0, 1.0)];

  let line = get_line_for_agent_to_edge(
    &agent,
    EdgeVertex { point: vertices[0], convex: true },
    EdgeVertex { point: vertices[1], convex: true },
    Some(vertices[2]),
    Some(vertices[2]),
    /* edge_margin= */ agent.radius,
    /* time_horizoon= */ 1.0,
    &[],
  );
  assert_line_eq!(
    line.expect("Line should be Some."),
    Line { direction: Vec2::new(-1.0, 0.0), point: Vec2::ZERO }
  );
}

#[test]
fn agent_collides_with_left_vertex() {
  let agent = Agent {
    position: Vec2::new(-1.1, -0.1),
    velocity: Vec2::new(0.0, -1.0),
    radius: 1.0,
    avoidance_responsibility: 1.0,
  };

  let vertices =
    vec![Vec2::new(-1.0, 0.0), Vec2::new(1.0, 0.0), Vec2::new(0.0, 1.0)];

  let line = get_line_for_agent_to_edge(
    &agent,
    EdgeVertex { point: vertices[0], convex: true },
    EdgeVertex { point: vertices[1], convex: true },
    Some(vertices[2]),
    Some(vertices[2]),
    /* edge_margin= */ agent.radius,
    /* time_horizoon= */ 1.0,
    &[],
  );
  assert_line_eq!(
    line.expect("Line should be Some, but was None."),
    Line { direction: Vec2::new(-1.0, 1.0).normalize(), point: Vec2::ZERO }
  );
}

#[test]
fn agent_collides_with_right_vertex_with_line() {
  let agent = Agent {
    position: Vec2::new(1.1, -0.1),
    velocity: Vec2::new(0.0, -1.0),
    radius: 1.0,
    avoidance_responsibility: 1.0,
  };

  let vertices = vec![Vec2::new(-1.0, 0.0), Vec2::new(1.0, 0.0)];

  let line = get_line_for_agent_to_edge(
    &agent,
    EdgeVertex { point: vertices[0], convex: true },
    EdgeVertex { point: vertices[1], convex: true },
    None,
    None,
    /* edge_margin= */ agent.radius,
    /* time_horizoon= */ 1.0,
    &[],
  );
  assert_line_eq!(
    line.expect("Line should be Some, but was None."),
    Line { direction: Vec2::new(-1.0, -1.0).normalize(), point: Vec2::ZERO }
  );
}

#[test]
fn agent_collides_with_right_vertex_handled_by_next_edge() {
  let agent = Agent {
    position: Vec2::new(1.1, -0.1),
    velocity: Vec2::new(0.0, -1.0),
    radius: 1.0,
    avoidance_responsibility: 1.0,
  };

  let vertices =
    vec![Vec2::new(-1.0, 0.0), Vec2::new(1.0, 0.0), Vec2::new(0.0, 1.0)];

  let line = get_line_for_agent_to_edge(
    &agent,
    EdgeVertex { point: vertices[0], convex: true },
    EdgeVertex { point: vertices[1], convex: true },
    Some(vertices[2]),
    Some(vertices[2]),
    /* edge_margin= */ agent.radius,
    /* time_horizoon= */ 1.0,
    &[],
  );
  assert!(line.is_none(), "The right vertex should be handled by the next edge. The generated line was {:?}", line.unwrap());

  let line = get_line_for_agent_to_edge(
    &agent,
    EdgeVertex { point: vertices[1], convex: true },
    EdgeVertex { point: vertices[2], convex: true },
    Some(vertices[0]),
    Some(vertices[0]),
    /* edge_margin= */ agent.radius,
    /* time_horizoon= */ 1.0,
    &[],
  );
  assert!(line.is_some(), "The right vertex should be handled by this edge.");
}

#[test]
fn agent_velocity_projects_to_cutoff_line() {
  let agent = Agent {
    position: Vec2::new(0.0, -2.0),
    velocity: Vec2::new(0.5, -1.0),
    radius: 1.0,
    avoidance_responsibility: 1.0,
  };

  let vertices =
    vec![Vec2::new(-1.0, 0.0), Vec2::new(1.0, 0.0), Vec2::new(0.0, 1.0)];

  let line = get_line_for_agent_to_edge(
    &agent,
    EdgeVertex { point: vertices[0], convex: true },
    EdgeVertex { point: vertices[1], convex: true },
    Some(vertices[2]),
    Some(vertices[2]),
    /* edge_margin= */ agent.radius,
    /* time_horizoon= */ 0.5,
    &[],
  );
  assert_line_eq!(
    line.expect("Line should be Some."),
    Line { direction: Vec2::new(-1.0, 0.0), point: Vec2::new(-2.0, 2.0) }
  );
}

#[test]
fn agent_velocity_projects_to_shadows() {
  let mut agent = Agent {
    position: Vec2::new(0.0, -2.0),
    velocity: Vec2::new(3.0, 3.0),
    radius: 1.0,
    avoidance_responsibility: 1.0,
  };

  let vertices =
    vec![Vec2::new(-1.0, 0.0), Vec2::new(1.0, 0.0), Vec2::new(0.0, 1.0)];

  let line = get_line_for_agent_to_edge(
    &agent,
    EdgeVertex { point: vertices[0], convex: true },
    EdgeVertex { point: vertices[1], convex: true },
    Some(vertices[2]),
    Some(vertices[2]),
    /* edge_margin= */ agent.radius,
    /* time_horizoon= */ 0.5,
    &[],
  );
  assert_line_eq!(
    line.expect("Line should be Some."),
    Line { direction: Vec2::new(-0.8, -0.6), point: Vec2::new(3.2, 2.4) }
  );

  agent.velocity = Vec2::new(-3.0, 3.0);

  let line = get_line_for_agent_to_edge(
    &agent,
    EdgeVertex { point: vertices[0], convex: true },
    EdgeVertex { point: vertices[1], convex: true },
    Some(vertices[2]),
    Some(vertices[2]),
    /* edge_margin= */ agent.radius,
    /* time_horizoon= */ 0.5,
    &[],
  );
  assert_line_eq!(
    line.expect("Line should be Some."),
    Line { direction: Vec2::new(-0.8, 0.6), point: Vec2::new(-3.2, 2.4) }
  );
}

#[test]
fn agent_velocity_projects_to_covered_shadows_creates_no_lines() {
  let mut agent = Agent {
    position: Vec2::new(0.0, -2.0),
    velocity: Vec2::new(-10.0, 0.0),
    radius: 1.0,
    avoidance_responsibility: 1.0,
  };

  let vertices = vec![
    Vec2::new(-2.0, 0.5),
    Vec2::new(-1.0, 0.0),
    Vec2::new(1.0, 0.0),
    Vec2::new(2.0, 0.5),
  ];

  let line = get_line_for_agent_to_edge(
    &agent,
    EdgeVertex { point: vertices[1], convex: true },
    EdgeVertex { point: vertices[2], convex: true },
    Some(vertices[0]),
    Some(vertices[3]),
    /* edge_margin= */ agent.radius,
    /* time_horizoon= */ 0.5,
    &[],
  );
  assert!(line.is_none(), "Left shadow was covered, so the left edge should have taken care of creating the line.");

  let line = get_line_for_agent_to_edge(
    &agent,
    EdgeVertex { point: vertices[0], convex: true },
    EdgeVertex { point: vertices[1], convex: true },
    None,
    Some(vertices[2]),
    /* edge_margin= */ agent.radius,
    /* time_horizoon= */ 0.5,
    &[],
  );
  assert!(line.is_some(), "Left shadow was covered for the right edge, so this edge should create a line.");

  agent.velocity = Vec2::new(10.0, 0.0);

  let line = get_line_for_agent_to_edge(
    &agent,
    EdgeVertex { point: vertices[1], convex: true },
    EdgeVertex { point: vertices[2], convex: true },
    Some(vertices[0]),
    Some(vertices[3]),
    /* edge_margin= */ agent.radius,
    /* time_horizoon= */ 0.5,
    &[],
  );
  assert!(line.is_none(), "Right shadow was covered, so the right edge should have taken care of creating the line.");

  let line = get_line_for_agent_to_edge(
    &agent,
    EdgeVertex { point: vertices[2], convex: true },
    EdgeVertex { point: vertices[3], convex: true },
    Some(vertices[1]),
    None,
    /* edge_margin= */ agent.radius,
    /* time_horizoon= */ 0.5,
    &[],
  );
  assert!(line.is_some(), "Right shadow was covered for the left edge, so this edge should create a line.");
}

#[test]
fn backwards_edges_are_ignored() {
  let agent = Agent {
    position: Vec2::new(0.0, 0.0),
    velocity: Vec2::new(0.0, 0.0),
    radius: 1.0,
    avoidance_responsibility: 1.0,
  };

  let vertices = vec![Vec2::new(-1.0, -1.0), Vec2::new(-1.0, 1.0)];

  let line = get_line_for_agent_to_edge(
    &agent,
    EdgeVertex { point: vertices[0], convex: true },
    EdgeVertex { point: vertices[1], convex: true },
    None,
    None,
    /* edge_margin= */ agent.radius,
    /* time_horizoon= */ 1.0,
    &[],
  );
  assert!(line.is_some(), "Forward edges should not be ignored.");

  let line = get_line_for_agent_to_edge(
    &agent,
    EdgeVertex { point: vertices[1], convex: true },
    EdgeVertex { point: vertices[0], convex: true },
    None,
    None,
    /* edge_margin= */ agent.radius,
    /* time_horizoon= */ 1.0,
    &[],
  );
  assert!(
    line.is_none(),
    "Backward edges should be ignored. {:?}",
    line.unwrap()
  );
}

#[test]
fn velocity_projects_to_cutoff_endpoints() {
  let mut agent = Agent {
    position: Vec2::ZERO,
    velocity: Vec2::new(3.0, 0.0),
    radius: 0.0,
    avoidance_responsibility: 1.0,
  };

  assert_line_eq!(
    get_line_for_agent_to_edge(
      &agent,
      EdgeVertex { point: Vec2::new(-1.0, 2.0), convex: true },
      EdgeVertex { point: Vec2::new(1.0, 2.0), convex: true },
      None,
      None,
      /* edge_margin= */ agent.radius,
      /* time_horizoon= */ 1.0,
      &[],
    )
    .unwrap(),
    Line {
      direction: Vec2::new(-1.0, -1.0).normalize(),
      point: Vec2::new(1.0, 2.0)
    }
  );

  agent.velocity = Vec2::new(-3.0, 0.0);

  assert_line_eq!(
    get_line_for_agent_to_edge(
      &agent,
      EdgeVertex { point: Vec2::new(-1.0, 2.0), convex: true },
      EdgeVertex { point: Vec2::new(1.0, 2.0), convex: true },
      None,
      None,
      /* edge_margin= */ agent.radius,
      /* time_horizoon= */ 1.0,
      &[],
    )
    .unwrap(),
    Line {
      direction: Vec2::new(-1.0, 1.0).normalize(),
      point: Vec2::new(-1.0, 2.0)
    }
  );
}

#[test]
fn velocity_projects_to_degenerate_edge() {
  let agent = Agent {
    position: Vec2::ZERO,
    velocity: Vec2::ZERO,
    radius: 0.0,
    avoidance_responsibility: 1.0,
  };

  assert_line_eq!(
    get_line_for_agent_to_edge(
      &agent,
      EdgeVertex { point: Vec2::new(0.0, 2.0), convex: true },
      EdgeVertex { point: Vec2::new(0.0, 2.0), convex: true },
      None,
      None,
      /* edge_margin= */ agent.radius,
      /* time_horizoon= */ 1.0,
      &[],
    )
    .unwrap(),
    Line { direction: Vec2::new(-1.0, 0.0), point: Vec2::new(0.0, 2.0) }
  );
}

#[test]
fn shadow_of_endpoint_covers_edge() {
  let mut agent = Agent {
    position: Vec2::ZERO,
    velocity: Vec2::new(-0.5, 3.0),
    radius: 1.0,
    avoidance_responsibility: 1.0,
  };

  // Right endpoint shadow covers edge.
  assert_line_eq!(
    get_line_for_agent_to_edge(
      &agent,
      EdgeVertex { point: Vec2::new(-1.0, 4.0), convex: true },
      EdgeVertex { point: Vec2::new(0.0, 2.0f32.sqrt()), convex: true },
      None,
      None,
      /* edge_margin= */ agent.radius,
      /* time_horizoon= */ 1.0,
      &[],
    )
    .unwrap(),
    // The line follows the shadow, rather than the edge.
    Line {
      direction: Vec2::new(-1.0, 1.0).normalize(),
      point: Vec2::new(-1.0, 1.0).normalize()
    }
  );

  agent.velocity = Vec2::new(0.5, 3.0);

  // Left endpoint shadow covers edge.
  assert_line_eq!(
    get_line_for_agent_to_edge(
      &agent,
      EdgeVertex { point: Vec2::new(0.0, 2.0f32.sqrt()), convex: true },
      EdgeVertex { point: Vec2::new(1.0, 4.0), convex: true },
      None,
      None,
      /* edge_margin= */ agent.radius,
      /* time_horizoon= */ 1.0,
      &[],
    )
    .unwrap(),
    // The line follows the shadow, rather than the edge.
    Line {
      direction: Vec2::new(-1.0, -1.0).normalize(),
      point: Vec2::new(1.0, 1.0).normalize()
    }
  );
}

macro_rules! assert_lines_eq_unordered {
  ($left: expr, $right: expr) => {{
    let left = $left;
    let right = $right;
    let mut left_not_found = Vec::new();
    let mut right_not_found = right.iter().cloned().collect::<Vec<Line>>();

    for left_line in left.iter() {
      let mut found_index = None;
      for (right_index, right_line) in right_not_found.iter().enumerate() {
        if left_line.point.distance_squared(right_line.point) < 1e-5
          && left_line.direction.distance_squared(right_line.direction) < 1e-5
        {
          found_index = Some(right_index);
          break;
        }
      }
      if let Some(found_index) = found_index {
        right_not_found.remove(found_index);
      } else {
        left_not_found.push(left_line);
      }
    }

    if !left_not_found.is_empty() || !right_not_found.is_empty() {
      panic!("Left lines did not match right lines.\n\nleft={:?}\nright={:?}\n\nunmatched left={:?}\nunmatched right={:?}", left, right, left_not_found, right_not_found);
    }
  }};
}

#[test]
fn lines_generated_for_closed_convex_obstacle() {
  let mut agent = Agent {
    position: Vec2::ZERO,
    velocity: Vec2::new(0.5, 3.0),
    radius: 0.0,
    avoidance_responsibility: 1.0,
  };

  let obstacle = Obstacle::Closed {
    vertices: vec![
      Vec2::new(4.0, 4.0),
      Vec2::new(-4.0, 4.0),
      Vec2::new(0.0, 2.0),
    ],
  };

  // Velocity projects to one of the obstacle's edges.
  assert_lines_eq_unordered!(
    get_lines_for_agent_to_obstacle(
      &agent,
      &obstacle,
      /* obstacle_margin= */ agent.radius,
      /* time_horizon= */ 1.0,
    ),
    [Line {
      direction: Vec2::new(-2.0, -1.0).normalize(),
      point: Vec2::new(0.0, 2.0),
    }]
  );

  agent.velocity = Vec2::new(-0.5, 3.0);

  // Velocity projects across looping vertices.
  assert_lines_eq_unordered!(
    get_lines_for_agent_to_obstacle(
      &agent,
      &obstacle,
      /* obstacle_margin= */ agent.radius,
      /* time_horizon= */ 1.0,
    ),
    [Line {
      direction: Vec2::new(-2.0, 1.0).normalize(),
      point: Vec2::new(-4.0, 4.0),
    }]
  );

  agent.velocity = Vec2::new(2.0, 7.0);

  // Velocity projects to obstacle's shadow.
  assert_lines_eq_unordered!(
    get_lines_for_agent_to_obstacle(
      &agent,
      &obstacle,
      /* obstacle_margin= */ agent.radius,
      /* time_horizon= */ 1.0,
    ),
    [Line {
      direction: Vec2::new(-1.0, -1.0).normalize(),
      point: Vec2::new(4.0, 4.0),
    }]
  );
}

#[test]
fn lines_generated_for_open_convex_obstacle() {
  let mut agent = Agent {
    position: Vec2::ZERO,
    velocity: Vec2::new(0.5, 3.0),
    radius: 0.0,
    avoidance_responsibility: 1.0,
  };

  let obstacle = Obstacle::Open {
    vertices: vec![
      Vec2::new(-4.0, 4.0),
      Vec2::new(0.0, 2.0),
      Vec2::new(4.0, 4.0),
    ],
  };

  // Velocity projects to one of the obstacle's edges.
  assert_lines_eq_unordered!(
    get_lines_for_agent_to_obstacle(
      &agent,
      &obstacle,
      /* obstacle_margin= */ agent.radius,
      /* time_horizon= */ 1.0,
    ),
    [Line {
      direction: Vec2::new(-2.0, -1.0).normalize(),
      point: Vec2::new(0.0, 2.0),
    }]
  );

  agent.velocity = Vec2::new(-0.5, 3.0);

  // Velocity projects to another of the obstacle's edges.
  assert_lines_eq_unordered!(
    get_lines_for_agent_to_obstacle(
      &agent,
      &obstacle,
      /* obstacle_margin= */ agent.radius,
      /* time_horizon= */ 1.0,
    ),
    [Line {
      direction: Vec2::new(-2.0, 1.0).normalize(),
      point: Vec2::new(-4.0, 4.0),
    }]
  );

  agent.velocity = Vec2::new(2.0, 7.0);

  // Velocity projects to obstacle's shadow.
  assert_lines_eq_unordered!(
    get_lines_for_agent_to_obstacle(
      &agent,
      &obstacle,
      /* obstacle_margin= */ agent.radius,
      /* time_horizon= */ 1.0,
    ),
    [Line {
      direction: Vec2::new(-1.0, -1.0).normalize(),
      point: Vec2::new(4.0, 4.0),
    }]
  );
}

#[test]
fn velocity_projects_to_concave_corner() {
  let agent = Agent {
    position: Vec2::ZERO,
    velocity: Vec2::new(0.0, 3.0),
    radius: 0.0,
    avoidance_responsibility: 1.0,
  };

  let obstacle = Obstacle::Open {
    vertices: vec![
      Vec2::new(-1.0, 1.0),
      Vec2::new(0.0, 2.0),
      Vec2::new(1.0, 1.0),
    ],
  };

  assert_lines_eq_unordered!(
    get_lines_for_agent_to_obstacle(
      &agent,
      &obstacle,
      /* obstacle_margin= */ agent.radius,
      /* time_horizon= */ 1.0,
    ),
    [
      Line {
        direction: Vec2::new(-1.0, -1.0).normalize(),
        point: Vec2::new(0.0, 2.0),
      },
      Line {
        direction: Vec2::new(-1.0, 1.0).normalize(),
        point: Vec2::new(0.0, 2.0),
      },
    ]
  );
}

#[test]
fn no_line_for_projecting_to_concave_endpoint_covered_by_shadow() {
  let agent = Agent {
    position: Vec2::ZERO,
    velocity: Vec2::ZERO,
    radius: 0.0,
    avoidance_responsibility: 1.0,
  };

  // Use the looping part of the obstacle to prevent the edge (0,2)-to-(0,4)
  // being ignored.
  let obstacle = Obstacle::Closed {
    vertices: vec![
      Vec2::new(1.0, 1.0),
      Vec2::new(0.0, 2.0),
      Vec2::new(0.0, 4.0),
      Vec2::new(-1.0, 1.0),
    ],
  };

  // The (0,2)-to-(0,4) edge does not generate a constraint.
  assert_lines_eq_unordered!(
    get_lines_for_agent_to_obstacle(
      &agent,
      &obstacle,
      /* obstacle_margin= */ agent.radius,
      /* time_horizon= */ 1.0,
    ),
    [Line { direction: Vec2::new(-1.0, 0.0), point: Vec2::new(-1.0, 1.0) }]
  );

  // Repeat to use the other endpoint.
  let obstacle = Obstacle::Closed {
    vertices: vec![
      Vec2::new(1.0, 1.0),
      Vec2::new(0.0, 4.0),
      Vec2::new(0.0, 2.0),
      Vec2::new(-1.0, 1.0),
    ],
  };

  // The (0,4)-to-(0,2) edge does not generate a constraint.
  assert_lines_eq_unordered!(
    get_lines_for_agent_to_obstacle(
      &agent,
      &obstacle,
      /* obstacle_margin= */ agent.radius,
      /* time_horizon= */ 1.0,
    ),
    [Line { direction: Vec2::new(-1.0, 0.0), point: Vec2::new(-1.0, 1.0) }]
  );
}

#[test]
fn collision_with_non_back_face_culled_edge_ignored() {
  let mut agent = Agent {
    position: Vec2::new(0.0, -0.5),
    velocity: Vec2::ZERO,
    radius: 1.0,
    avoidance_responsibility: 1.0,
  };

  let obstacle = Obstacle::Open {
    vertices: vec![
      Vec2::new(-1.0, 1.0),
      Vec2::new(0.0, 0.0),
      Vec2::new(3.0, 0.0),
      Vec2::new(2.0, 1.0),
    ],
  };

  // Neither of the first two edges will be back-face culled, so only the
  // right edge will generate the constraint.
  assert_lines_eq_unordered!(
    get_lines_for_agent_to_obstacle(
      &agent,
      &obstacle,
      /* obstacle_margin= */ agent.radius,
      /* time_horizon= */ 1.0,
    ),
    [Line { direction: Vec2::new(-1.0, 0.0), point: Vec2::new(0.0, 0.0) }]
  );

  agent.position = Vec2::new(3.1, -0.5);

  // The right edge is back-face culled (so doesn't generate a constraint), so
  // only the left edge will generate the constraint.
  assert_lines_eq_unordered!(
    get_lines_for_agent_to_obstacle(
      &agent,
      &obstacle,
      /* obstacle_margin= */ agent.radius,
      /* time_horizon= */ 1.0,
    ),
    [Line {
      direction: Vec2::new(-0.1, 0.5).normalize().perp(),
      point: Vec2::new(0.0, 0.0)
    }]
  );
}

#[test]
fn collision_with_convex_vertex() {
  let mut agent = Agent {
    position: Vec2::new(0.1, -0.1),
    velocity: Vec2::ZERO,
    radius: 1.0,
    avoidance_responsibility: 1.0,
  };

  let obstacle = Obstacle::Open {
    vertices: vec![
      Vec2::new(-2.0, -1.0),
      Vec2::new(-1.0, 0.0),
      Vec2::new(0.0, 0.0),
      Vec2::new(1.0, -1.0),
    ],
  };

  assert_lines_eq_unordered!(
    get_lines_for_agent_to_obstacle(
      &agent,
      &obstacle,
      /* obstacle_margin= */ agent.radius,
      /* time_horizon= */ 1.0,
    ),
    [
      Line { direction: Vec2::new(-1.0, 1.0).normalize(), point: Vec2::ZERO },
      Line { direction: Vec2::new(-1.0, 0.0), point: Vec2::ZERO },
    ]
  );

  agent.position = Vec2::new(-1.1, -0.1);

  assert_lines_eq_unordered!(
    get_lines_for_agent_to_obstacle(
      &agent,
      &obstacle,
      /* obstacle_margin= */ agent.radius,
      /* time_horizon= */ 1.0,
    ),
    [
      Line { direction: Vec2::new(-1.0, -1.0).normalize(), point: Vec2::ZERO },
      Line { direction: Vec2::new(-1.0, 0.0), point: Vec2::ZERO },
    ]
  );
}
