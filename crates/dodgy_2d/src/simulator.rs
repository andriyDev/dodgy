use std::collections::HashMap;

use glam::Vec2;

use crate::{Agent, AvoidanceOptions, Obstacle};

pub struct Simulator {
  agents: Vec<Agent>,
  agent_parameters: Vec<AgentParameters>,
  obstacles: Vec<Obstacle>,
}

pub struct AgentParameters {
  pub goal_point: Vec2,
  pub max_speed: f32,
  pub obstacle_margin: SimulatorMargin,
  pub time_horizon: f32,
  pub obstacle_time_horizon: f32,
}

pub enum SimulatorMargin {
  AgentRadius,
  Distance(f32),
}

impl Simulator {
  pub fn new() -> Simulator {
    Self {
      agents: Vec::new(),
      agent_parameters: Vec::new(),
      obstacles: Vec::new(),
    }
  }

  pub fn add_agent(&mut self, agent: Agent, agent_parameters: AgentParameters) {
    self.agents.push(agent);
    self.agent_parameters.push(agent_parameters);
  }

  pub fn add_obstacle(&mut self, obstacle: Obstacle) {
    assert!(if let Obstacle::Closed { vertices: _ } = &obstacle {
      true
    } else {
      false
    });

    self.obstacles.push(obstacle);
  }

  pub fn remove_agent(&mut self, agent_index: usize) {
    self.agents.remove(agent_index);
  }

  pub fn remove_obstacle(&mut self, obstacle_index: usize) {
    self.obstacles.remove(obstacle_index);
  }

  pub fn get_agent(&self, agent_index: usize) -> &Agent {
    &self.agents[agent_index]
  }

  pub fn get_agent_mut(&mut self, agent_index: usize) -> &mut Agent {
    &mut self.agents[agent_index]
  }

  pub fn get_agent_count(&self) -> usize {
    self.agents.len()
  }

  pub fn get_obstacle_count(&self) -> usize {
    self.obstacles.len()
  }

  pub fn get_agent_parameters(&self, agent_index: usize) -> &AgentParameters {
    &self.agent_parameters[agent_index]
  }

  pub fn get_agent_parameters_mut(
    &mut self,
    agent_index: usize,
  ) -> &mut AgentParameters {
    &mut self.agent_parameters[agent_index]
  }

  pub fn step(&mut self, time_step: f32) {
    let mut agent_pair_to_distance_squared = HashMap::new();
    // TODO: Make this fast.
    for i in 0..self.agents.len() {
      for j in (i + 1)..self.agents.len() {
        let distance_squared =
          self.agents[i].position.distance_squared(self.agents[j].position);
        agent_pair_to_distance_squared.insert((i, j), distance_squared);
        agent_pair_to_distance_squared.insert((j, i), distance_squared);
      }
    }

    let mut new_velocities = Vec::with_capacity(self.agents.len());
    for (index, (agent, parameters)) in
      self.agents.iter().zip(self.agent_parameters.iter()).enumerate()
    {
      let mut neighbours = Vec::new();
      for other_index in 0..self.agents.len() {
        if index == other_index {
          continue;
        }

        let query_distance =
          parameters.max_speed * parameters.time_horizon + agent.radius * 2.0;
        if agent_pair_to_distance_squared[&(index, other_index)]
          <= query_distance * query_distance
        {
          continue;
        }

        neighbours.push(&self.agents[other_index]);
      }

      let near_obstacles = Vec::new();

      new_velocities.push(agent.compute_avoiding_velocity(
        &neighbours,
        &near_obstacles,
        parameters.goal_point - agent.position,
        parameters.max_speed,
        time_step,
        &AvoidanceOptions {
          obstacle_margin: match parameters.obstacle_margin {
            SimulatorMargin::AgentRadius => agent.radius,
            SimulatorMargin::Distance(v) => v,
          },
          time_horizon: parameters.time_horizon,
          obstacle_time_horizon: parameters.obstacle_time_horizon,
        },
      ));
    }

    for (agent, new_velocity) in self.agents.iter_mut().zip(new_velocities) {
      agent.velocity = new_velocity;
      agent.position += new_velocity * time_step;
    }
  }
}

#[cfg(test)]
mod tests {
  use glam::Vec2;

  use crate::{Agent, AgentParameters, Obstacle, Simulator, SimulatorMargin};

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
  fn two_agent_one_obstacle_simulation() {
    let mut simulator = Simulator::new();

    simulator.add_agent(
      Agent {
        position: Vec2::new(10.0, 0.0),
        velocity: Vec2::ZERO,
        radius: 1.0,
        avoidance_responsibility: 1.0,
      },
      AgentParameters {
        goal_point: Vec2::new(-10.0, 0.0),
        max_speed: 2.0,
        obstacle_margin: SimulatorMargin::AgentRadius,
        time_horizon: 2.0,
        obstacle_time_horizon: 1.0,
      },
    );

    simulator.add_agent(
      Agent {
        position: Vec2::new(-10.0, 0.0),
        velocity: Vec2::ZERO,
        radius: 1.0,
        avoidance_responsibility: 1.0,
      },
      AgentParameters {
        goal_point: Vec2::new(10.0, 0.0),
        max_speed: 2.0,
        obstacle_margin: SimulatorMargin::AgentRadius,
        time_horizon: 2.0,
        obstacle_time_horizon: 1.0,
      },
    );

    simulator.add_obstacle(Obstacle::Closed {
      vertices: vec![
        Vec2::new(-2.0, -2.0),
        Vec2::new(-2.0, -1.0),
        Vec2::new(0.0, 1.0),
        Vec2::new(2.0, -2.0),
        Vec2::new(2.0, -1.0),
      ],
    });

    // Test accessors.
    assert_eq!(simulator.get_agent_count(), 2);
    assert_eq!(simulator.get_obstacle_count(), 1);

    assert_eq!(
      simulator.get_agent_parameters(0).goal_point,
      Vec2::new(-10.0, 0.0)
    );
    assert_eq!(
      simulator.get_agent_parameters(1).goal_point,
      Vec2::new(10.0, 0.0)
    );

    simulator.get_agent_mut(1).position = Vec2::new(-5.0, 0.0);
    simulator.get_agent_parameters_mut(1).time_horizon = 3.0;

    for _ in 0..200 {
      simulator.step(0.1);
    }

    assert_vec_near!(
      simulator.get_agent(0).position,
      Vec2::new(-10.0, 0.0),
      1e-4
    );
    assert_vec_near!(
      simulator.get_agent(1).position,
      Vec2::new(10.0, 0.0),
      1e-4
    );

    simulator.remove_agent(0);

    // Agent 1 should now have "moved" into Agent 0.
    assert_eq!(simulator.get_agent_count(), 1);
    assert_vec_near!(
      simulator.get_agent(0).position,
      Vec2::new(10.0, 0.0),
      1e-4
    );

    simulator.remove_obstacle(0);
    assert_eq!(simulator.get_obstacle_count(), 0);
  }
}
