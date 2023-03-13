use glam::Vec2;

use crate::{common::determinant, Agent, Line};

// A single obstacle in the simulation.
pub enum Obstacle {
  // A closed obstacle. The obstacle is closed in that the last vertex will have
  // an edge connecting it to the first vertex. The edges cannot cross, and the
  // interior of the obstacle is to the "left" of the edges. In other words,
  // obstacles with vertices going counter-clockwise will prevent objects from
  // getting into the loop, and obstacles with vertices going clockwise will
  // prevent objects from leaving the loop.
  Closed { vertices: Vec<Vec2> },
  // An open obstacle. The vertices are assumed to be a part of some closed
  // obstacle, so the left of the edge is solid, and the right is clear.
  Open { vertices: Vec<Vec2> },
}

// Computes the lines describing the half-planes of valid velocities for `agent`
// induced by `obstacle`. `time_horizon` determines how much time in the future
// should collisions be considered for this obstacle.
pub fn get_lines_for_agent_to_obstacle(
  agent: &Agent,
  obstacle: &Obstacle,
  time_horizon: f32,
) -> Vec<Line> {
  match obstacle {
    Obstacle::Closed { vertices } => get_lines_for_agent_to_obstacle_const::<
      true,
    >(agent, vertices, time_horizon),
    Obstacle::Open { vertices } => {
      get_lines_for_agent_to_obstacle_const::<false>(
        agent,
        vertices,
        time_horizon,
      )
    }
  }
}

fn get_lines_for_agent_to_obstacle_const<const CLOSED: bool>(
  agent: &Agent,
  vertices: &[Vec2],
  time_horizon: f32,
) -> Vec<Line> {
  let convexity = {
    let mut convexity = Vec::with_capacity(vertices.len());
    for i in 0..vertices.len() {
      // An edge is convex if the right edge is on the right side of the left
      // edge.
      if CLOSED {
        let left = vertices[if i == 0 { vertices.len() - 1 } else { i - 1 }];
        let center = vertices[i];
        let right = vertices[if i == vertices.len() - 1 { 0 } else { i + 1 }];
        convexity.push(determinant(right - center, left - center) >= 0.0);
      } else {
        if i == 0 || i == vertices.len() - 1 {
          convexity.push(true);
        } else {
          let left = vertices[i - 1];
          let center = vertices[i];
          let right = vertices[i + 1];
          convexity.push(determinant(right - center, left - center) >= 0.0);
        }
      }
    }
    convexity
  };

  let mut lines = Vec::new();
  let edge_vertex_for_index = |index: usize| EdgeVertex {
    point: vertices[index],
    convex: convexity[index],
  };
  if CLOSED {
    for left_index in 0..vertices.len() {
      let right_index =
        if left_index == vertices.len() - 1 { 0 } else { left_index + 1 };
      let left_left_index =
        if left_index == 0 { vertices.len() - 1 } else { left_index - 1 };
      let right_right_index =
        if right_index == vertices.len() - 1 { 0 } else { right_index + 1 };

      let left_vertex = edge_vertex_for_index(left_index);
      let right_vertex = edge_vertex_for_index(right_index);

      if let Some(line) = get_line_for_agent_to_edge(
        agent,
        left_vertex,
        right_vertex,
        Some(vertices[left_left_index]),
        Some(vertices[right_right_index]),
        time_horizon,
        &lines,
      ) {
        lines.push(line);
      }
    }
  } else {
    for left_index in 0..(vertices.len() - 1) {
      let right_index = left_index + 1;

      let left_vertex = edge_vertex_for_index(left_index);
      let right_vertex = edge_vertex_for_index(right_index);

      if let Some(line) = get_line_for_agent_to_edge(
        agent,
        left_vertex,
        right_vertex,
        if left_index == 0 { None } else { Some(vertices[left_index - 1]) },
        if right_index == vertices.len() - 1 {
          None
        } else {
          Some(vertices[right_index + 1])
        },
        time_horizon,
        &lines,
      ) {
        lines.push(line);
      }
    }
  }

  lines
}

#[derive(Clone, Copy)]
struct EdgeVertex {
  point: Vec2,
  convex: bool,
}

fn get_line_for_agent_to_edge(
  agent: &Agent,
  mut left_vertex: EdgeVertex,
  mut right_vertex: EdgeVertex,
  mut left_left_vertex: Option<Vec2>,
  mut right_right_vertex: Option<Vec2>,
  time_horizon: f32,
  existing_lines: &[Line],
) -> Option<Line> {
  let relative_left_vertex = left_vertex.point - agent.position;
  let relative_right_vertex = right_vertex.point - agent.position;

  let edge_vector = right_vertex.point - left_vertex.point;
  let agent_from_left_vertex = -relative_left_vertex;

  // If the agent is already on the wrong side of the edge, then either the
  // agent is outside the obstacle and there is a "front side" to the obstacle
  // that will add necessary constraints, or the agent is already inside the
  // obstacle. In either case, adding constraints will block the agent from
  // moving.
  if determinant(agent_from_left_vertex, edge_vector) < 0.0 {
    return None;
  }

  fn is_edge_covered(
    left_vertex: Vec2,
    right_vertex: Vec2,
    time_horizon: f32,
    radius: f32,
    existing_lines: &[Line],
  ) -> bool {
    const EDGE_COVER_EPSILON: f32 = 1e-5;

    for line in existing_lines {
      // If both vertices are "deep" enough into the invalid section of a
      // pre-existing line, this edge cannot be collided with already, so it is
      // covered.
      if determinant(left_vertex / time_horizon - line.point, line.direction)
        >= radius / time_horizon - EDGE_COVER_EPSILON
        && determinant(right_vertex / time_horizon - line.point, line.direction)
          >= radius / time_horizon - EDGE_COVER_EPSILON
      {
        return true;
      }
    }

    false
  }

  // If the edge is already covered by existing lines, there's no reason to
  // generate another constraint - the agent already will not be able to collide
  // with this edge.
  if is_edge_covered(
    relative_left_vertex,
    relative_right_vertex,
    time_horizon,
    agent.radius,
    existing_lines,
  ) {
    return None;
  }

  let edge_unit_vector = edge_vector.normalize();

  let dist_left_squared = relative_left_vertex.length_squared();
  let dist_right_squared = relative_right_vertex.length_squared();
  let squared_radius = agent.radius * agent.radius;

  // Compute the time along the edge that the agent's position projects to.
  let edge_t =
    agent_from_left_vertex.dot(edge_vector) / edge_vector.length_squared();
  let dist_to_edge_line_squared =
    (edge_t * edge_vector).distance_squared(agent_from_left_vertex);

  // If the distance to the edge is less than the radius, the agent is
  // colliding with the edge. Cut off any velocities going into the edge
  // (those that increase the penetration of the edge). Note the endpoints of
  // the edge must be handled separately to ensure the correct direction is
  // used.

  if edge_t < 0.0 && dist_left_squared <= squared_radius {
    // TODO: Figure out why not convex is skipped here.
    if !left_vertex.convex {
      return None;
    }

    return Some(Line {
      point: Vec2::ZERO,
      direction: relative_left_vertex.perp().normalize(),
    });
  } else if edge_t > 1.0 && dist_right_squared <= squared_radius {
    // TODO: Figure out why not convex is skipped here and explain why the
    // determinant means the vertex will be taken care by the neighbouring
    // obstacle.
    if !right_vertex.convex {
      return None;
    }
    if let Some(right_right_vertex) = right_right_vertex {
      if determinant(
        relative_right_vertex,
        right_right_vertex - right_vertex.point,
      ) < 0.0
      {
        return None;
      }
    }

    return Some(Line {
      point: Vec2::ZERO,
      direction: relative_right_vertex.perp().normalize(),
    });
  } else if 0.0 <= edge_t
    && edge_t <= 1.0
    && dist_to_edge_line_squared <= squared_radius
  {
    return Some(Line { point: Vec2::ZERO, direction: -edge_unit_vector });
  }

  // The agent is not colliding with the edge. We now must find a constraint
  // that covers the edge's velocity obstacle.

  // First, we must compute the direction of the shadow of the edge in
  // velocity space. Note that the `time_horizon` is not needed here, since
  // scaling the obstacle will not change the direction of the shadow.

  let mut left_shadow_direction;
  let mut right_shadow_direction;

  if edge_t < 0.0 && dist_to_edge_line_squared <= squared_radius {
    // The right vertex is covered by the shadow of the left, so the shadow is
    // only defined by the left vertex.

    // TODO: Figure out why.
    if !left_vertex.convex {
      return None;
    }

    right_right_vertex = Some(right_vertex.point);
    right_vertex = left_vertex;

    // See the shadow-handling for agent-agent constraint lines for how this is
    // computed.
    let tangent_triangle_leg = (dist_left_squared - squared_radius).sqrt();

    left_shadow_direction = (relative_left_vertex * tangent_triangle_leg
      + relative_left_vertex.perp() * agent.radius)
      / dist_left_squared;
    right_shadow_direction = (relative_left_vertex * tangent_triangle_leg
      - relative_left_vertex.perp() * agent.radius)
      / dist_left_squared;
  } else if edge_t > 1.0 && dist_to_edge_line_squared <= squared_radius {
    // The left vertex is covered by the shadow of the right, so the shadow is
    // only defined by the right vertex.

    // TODO: Figure out why.
    if !right_vertex.convex {
      return None;
    }

    left_left_vertex = Some(left_vertex.point);
    left_vertex = right_vertex;

    // See the shadow-handling for agent-agent constraint lines for how this is
    // computed.
    let tangent_triangle_leg = (dist_right_squared - squared_radius).sqrt();
    left_shadow_direction = (relative_right_vertex * tangent_triangle_leg
      + relative_right_vertex.perp() * agent.radius)
      / dist_right_squared;
    right_shadow_direction = (relative_right_vertex * tangent_triangle_leg
      - relative_right_vertex.perp() * agent.radius)
      / dist_right_squared;
  } else {
    if left_vertex.convex {
      // The left of the shadow is defined by the left vertex, since the vertex
      // is convex (and therefore the connecting edge "curves" right if
      // present).
      let left_tangent_triangle_leg =
        (dist_left_squared - squared_radius).sqrt();
      left_shadow_direction = (relative_left_vertex
        * left_tangent_triangle_leg
        + relative_left_vertex.perp() * agent.radius)
        / dist_left_squared;
    } else {
      // The left of the shadow is covered by the edge leading out of the left
      // vertex. This technically means there is no shadow. Consider a concave
      // vertex. If the velocity is inside the "corner" part of the concave
      // edges, the velocity should project to the corner of the edges. By
      // setting the shadow direction to extend the edge, this tricky case is
      // handled gracefully, since both edges will extend the shadow this way.
      left_shadow_direction = -edge_unit_vector;
    }

    // The same reasoning as the left shadow applies for the right shadow.
    if right_vertex.convex {
      let right_tangent_triangle_leg =
        (dist_right_squared - squared_radius).sqrt();
      right_shadow_direction = (relative_right_vertex
        * right_tangent_triangle_leg
        - relative_right_vertex.perp() * agent.radius)
        / dist_right_squared;
    } else {
      right_shadow_direction = edge_unit_vector;
    }
  }

  // The shadow of each vertex can either cover the adjacent edges or be covered
  // by the adjacent edges. If the shadow is covered by an adjacent edge and the
  // velocity were to project to that edge, a constraint should not be
  // generated, since that edge will already generate the necessary constraint
  // (no need to double up).

  let mut is_left_shadow_covered = false;
  let mut is_right_shadow_covered = false;

  let left_edge_direction =
    left_left_vertex.and_then(|v| Some(v - left_vertex.point));
  if left_vertex.convex
    && left_edge_direction.is_some()
    && determinant(left_shadow_direction, left_edge_direction.unwrap()) >= 0.0
  {
    left_shadow_direction = left_edge_direction.unwrap().normalize();
    is_left_shadow_covered = true;
  }

  let right_edge_direction =
    right_right_vertex.and_then(|v| Some(v - right_vertex.point));
  if right_vertex.convex
    && right_edge_direction.is_some()
    && determinant(right_shadow_direction, right_edge_direction.unwrap()) <= 0.0
  {
    right_shadow_direction = right_edge_direction.unwrap().normalize();
    is_right_shadow_covered = true;
  }

  // The previously computed relative positions are no longer valid since it is
  // possible the vertices were swapped around when computing the shadow
  // directions.
  let left_cutoff = (left_vertex.point - agent.position) / time_horizon;
  let right_cutoff = (right_vertex.point - agent.position) / time_horizon;
  let cutoff_vector = right_cutoff - left_cutoff;

  let is_degenerate_edge = left_vertex.point == right_vertex.point;

  // In the original paper, the constraint is the tangent line of the velocity
  // obstacle at the closest point to the zero velocity. However, in the
  // implementation of RVO2, the tangent line is the one at the closest point to
  // the agent's current velocity.
  // TODO: Figure out if using the zero velocity makes things noticably worse.
  let velocity = agent.velocity;

  // Compute the time along the cutoff edge where the velocity projects to.
  // Note if the edge is degenerate, just pretend the velocity is halfway on the
  // degenerate edge.
  let t_cutoff_edge = if is_degenerate_edge {
    0.5
  } else {
    (velocity - left_cutoff).dot(cutoff_vector) / cutoff_vector.length_squared()
  };

  // Compute the time along the shadow directions that the velocity projects to.
  // Since the shadow directions are tangent to the cutoff circles, the time is
  // positive only if the velocity is actually along the shadow line (since the
  // shadow only starts after the tangent point).
  let t_left_shadow = (velocity - left_cutoff).dot(left_shadow_direction);
  let t_right_shadow = (velocity - right_cutoff).dot(right_shadow_direction);

  // If the velocity is to the left of the edge, and does not project to the
  // left shadow, project to the left cutoff. We also handle the degenerate edge
  // case here as well. If the edge is degenerate and the velocity does not
  // project to either shadow, projecting to either cutoff is fine (so also
  // project to the left).
  if t_cutoff_edge < 0.0 && t_left_shadow < 0.0
    || is_degenerate_edge && t_left_shadow < 0.0 && t_right_shadow < 0.0
  {
    let velocity_from_left_cutoff = (velocity - left_cutoff).normalize();

    return Some(Line {
      direction: -velocity_from_left_cutoff.perp(),
      point: left_cutoff
        + agent.radius / time_horizon * velocity_from_left_cutoff,
    });
  }

  // Repeat for the right cutoff (but ignore the degenerate edge case).
  if t_cutoff_edge > 1.0 && t_right_shadow < 0.0 {
    let velocity_from_right_cutoff = (velocity - right_cutoff).normalize();

    return Some(Line {
      direction: -velocity_from_right_cutoff.perp(),
      point: right_cutoff
        + agent.radius / time_horizon * velocity_from_right_cutoff,
    });
  }

  // Finally, project to the left shadow, the right shadow, or the cutoff line,
  // whichever is closest to the velocity. Note also if the shadow was covered,
  // and the velocity projects to that shadow, the constraint should NOT be
  // generated. The constraint will already be created by the edge that covered
  // the shadow.

  let cutoff_edge_distance_squared =
    if t_cutoff_edge < 0.0 || t_cutoff_edge > 1.0 || is_degenerate_edge {
      f32::INFINITY
    } else {
      (velocity - left_cutoff + t_cutoff_edge * cutoff_vector).length_squared()
    };
  let left_shadow_distance_squared = if t_left_shadow < 0.0 {
    f32::INFINITY
  } else {
    (velocity - left_cutoff + t_left_shadow * left_shadow_direction)
      .length_squared()
  };
  let right_shadow_distance_squared = if t_right_shadow < 0.0 {
    f32::INFINITY
  } else {
    (velocity - right_cutoff + t_right_shadow * right_shadow_direction)
      .length_squared()
  };

  if cutoff_edge_distance_squared <= left_shadow_distance_squared
    && cutoff_edge_distance_squared <= right_shadow_distance_squared
  {
    let line_direction = -cutoff_vector.normalize();
    Some(Line {
      direction: line_direction,
      point: left_cutoff + agent.radius / time_horizon * line_direction.perp(),
    })
  } else if left_shadow_distance_squared <= right_shadow_distance_squared {
    if is_left_shadow_covered {
      None
    } else {
      Some(Line {
        direction: left_shadow_direction,
        point: left_cutoff
          + agent.radius / time_horizon * left_shadow_direction.perp(),
      })
    }
  } else {
    if is_right_shadow_covered {
      None
    } else {
      Some(Line {
        direction: -right_shadow_direction,
        point: right_cutoff
          - agent.radius / time_horizon * right_shadow_direction.perp(),
      })
    }
  }
}

#[cfg(test)]
mod tests {
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
      max_velocity: 0.0,
    };

    let line = get_line_for_agent_to_edge(
      &agent,
      EdgeVertex { point: Vec2::new(-1.0, 0.0), convex: true },
      EdgeVertex { point: Vec2::new(1.0, 0.0), convex: true },
      None,
      None,
      0.5,
      &[Line { point: Vec2::new(-2.5, 1.5), direction: Vec2::new(-1.0, 1.0) }],
    );

    assert!(line.is_some(), "Line should have been Some since the existing line did not fully cover the edge.");

    let line = get_line_for_agent_to_edge(
      &agent,
      EdgeVertex { point: Vec2::new(-1.0, 0.0), convex: true },
      EdgeVertex { point: Vec2::new(1.0, 0.0), convex: true },
      None,
      None,
      1.0,
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
      max_velocity: 0.0,
    };

    let vertices =
      vec![Vec2::new(-1.0, 0.0), Vec2::new(1.0, 0.0), Vec2::new(0.0, 1.0)];

    let line = get_line_for_agent_to_edge(
      &agent,
      EdgeVertex { point: vertices[0], convex: true },
      EdgeVertex { point: vertices[1], convex: true },
      Some(vertices[2]),
      Some(vertices[2]),
      1.0,
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
      max_velocity: 0.0,
    };

    let vertices =
      vec![Vec2::new(-1.0, 0.0), Vec2::new(1.0, 0.0), Vec2::new(0.0, 1.0)];

    let line = get_line_for_agent_to_edge(
      &agent,
      EdgeVertex { point: vertices[0], convex: true },
      EdgeVertex { point: vertices[1], convex: true },
      Some(vertices[2]),
      Some(vertices[2]),
      1.0,
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
      max_velocity: 0.0,
    };

    let vertices = vec![Vec2::new(-1.0, 0.0), Vec2::new(1.0, 0.0)];

    let line = get_line_for_agent_to_edge(
      &agent,
      EdgeVertex { point: vertices[0], convex: true },
      EdgeVertex { point: vertices[1], convex: true },
      None,
      None,
      1.0,
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
      max_velocity: 0.0,
    };

    let vertices =
      vec![Vec2::new(-1.0, 0.0), Vec2::new(1.0, 0.0), Vec2::new(0.0, 1.0)];

    let line = get_line_for_agent_to_edge(
      &agent,
      EdgeVertex { point: vertices[0], convex: true },
      EdgeVertex { point: vertices[1], convex: true },
      Some(vertices[2]),
      Some(vertices[2]),
      1.0,
      &[],
    );
    assert!(line.is_none(), "The right vertex should be handled by the next edge. The generated line was {:?}", line.unwrap());

    let line = get_line_for_agent_to_edge(
      &agent,
      EdgeVertex { point: vertices[1], convex: true },
      EdgeVertex { point: vertices[2], convex: true },
      Some(vertices[0]),
      Some(vertices[0]),
      1.0,
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
      max_velocity: 0.0,
    };

    let vertices =
      vec![Vec2::new(-1.0, 0.0), Vec2::new(1.0, 0.0), Vec2::new(0.0, 1.0)];

    let line = get_line_for_agent_to_edge(
      &agent,
      EdgeVertex { point: vertices[0], convex: true },
      EdgeVertex { point: vertices[1], convex: true },
      Some(vertices[2]),
      Some(vertices[2]),
      0.5,
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
      max_velocity: 0.0,
    };

    let vertices =
      vec![Vec2::new(-1.0, 0.0), Vec2::new(1.0, 0.0), Vec2::new(0.0, 1.0)];

    let line = get_line_for_agent_to_edge(
      &agent,
      EdgeVertex { point: vertices[0], convex: true },
      EdgeVertex { point: vertices[1], convex: true },
      Some(vertices[2]),
      Some(vertices[2]),
      0.5,
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
      0.5,
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
      max_velocity: 0.0,
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
      0.5,
      &[],
    );
    assert!(line.is_none(), "Left shadow was covered, so the left edge should have taken care of creating the line.");

    let line = get_line_for_agent_to_edge(
      &agent,
      EdgeVertex { point: vertices[0], convex: true },
      EdgeVertex { point: vertices[1], convex: true },
      None,
      Some(vertices[2]),
      0.5,
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
      0.5,
      &[],
    );
    assert!(line.is_none(), "Right shadow was covered, so the right edge should have taken care of creating the line.");

    let line = get_line_for_agent_to_edge(
      &agent,
      EdgeVertex { point: vertices[2], convex: true },
      EdgeVertex { point: vertices[3], convex: true },
      Some(vertices[1]),
      None,
      0.5,
      &[],
    );
    assert!(line.is_some(), "Right shadow was covered for the left edge, so this edge should create a line.");
  }
}
