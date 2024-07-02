use glam::Vec3;

use crate::{
  simulator::{AgentParameters, Simulator, SimulatorMargin},
  Agent,
};

macro_rules! assert_vec_near {
  ($left: expr, $right: expr, $eps: expr) => {{
    let left = $left;
    let right = $right;
    let eps = $eps;
    assert!(
      left.distance(right) < eps,
      "left: {}, right: {}, epsilon: {}",
      left,
      right,
      eps
    );
  }};
}

#[test]
fn two_agent_simulation() {
  let mut simulator = Simulator::new();

  simulator.add_agent(
    Agent {
      // Perturb one agent slightly so they can find a reasonable solution.
      position: Vec3::new(10.0, 0.0, 0.01),
      velocity: Vec3::ZERO,
      radius: 1.0,
      avoidance_responsibility: 1.0,
    },
    AgentParameters {
      goal_point: Vec3::new(-10.0, 0.0, 0.0),
      max_speed: 2.0,
      obstacle_margin: SimulatorMargin::AgentRadius,
      time_horizon: 2.0,
      obstacle_time_horizon: 1.0,
    },
  );

  simulator.add_agent(
    Agent {
      position: Vec3::new(-10.0, 0.0, 0.0),
      velocity: Vec3::ZERO,
      radius: 1.0,
      avoidance_responsibility: 1.0,
    },
    AgentParameters {
      goal_point: Vec3::new(10.0, 0.0, 0.0),
      max_speed: 2.0,
      obstacle_margin: SimulatorMargin::AgentRadius,
      time_horizon: 2.0,
      obstacle_time_horizon: 1.0,
    },
  );

  // Test accessors.
  assert_eq!(simulator.get_agent_count(), 2);

  assert_eq!(
    simulator.get_agent_parameters(0).goal_point,
    Vec3::new(-10.0, 0.0, 0.0)
  );
  assert_eq!(
    simulator.get_agent_parameters(1).goal_point,
    Vec3::new(10.0, 0.0, 0.0)
  );

  simulator.get_agent_mut(1).position = Vec3::new(-5.0, 0.0, 0.0);
  simulator.get_agent_parameters_mut(1).time_horizon = 3.0;

  for _ in 0..200 {
    simulator.step(0.1);
  }

  assert_vec_near!(
    simulator.get_agent(0).position,
    Vec3::new(-10.0, 0.0, 0.0),
    1e-4
  );
  assert_vec_near!(
    simulator.get_agent(1).position,
    Vec3::new(10.0, 0.0, 0.0),
    1e-4
  );

  simulator.remove_agent(0);

  // Agent 1 should now have "moved" into Agent 0.
  assert_eq!(simulator.get_agent_count(), 1);
  assert_vec_near!(
    simulator.get_agent(0).position,
    Vec3::new(10.0, 0.0, 0.0),
    1e-4
  );
}
