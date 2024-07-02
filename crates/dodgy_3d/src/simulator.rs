use std::{borrow::Cow, collections::HashMap};

use glam::Vec3;

use crate::{Agent, AvoidanceOptions};

pub struct Simulator {
  agents: Vec<Agent>,
  agent_parameters: Vec<AgentParameters>,
}

pub struct AgentParameters {
  pub goal_point: Vec3,
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
    Self { agents: Vec::new(), agent_parameters: Vec::new() }
  }

  pub fn add_agent(&mut self, agent: Agent, agent_parameters: AgentParameters) {
    self.agents.push(agent);
    self.agent_parameters.push(agent_parameters);
  }

  pub fn remove_agent(&mut self, agent_index: usize) {
    self.agents.remove(agent_index);
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

        neighbours.push(Cow::Borrowed(&self.agents[other_index]));
      }

      new_velocities.push(agent.compute_avoiding_velocity(
        &neighbours,
        parameters.goal_point - agent.position,
        parameters.max_speed,
        time_step,
        &AvoidanceOptions { time_horizon: parameters.time_horizon },
      ));
    }

    for (agent, new_velocity) in self.agents.iter_mut().zip(new_velocities) {
      agent.velocity = new_velocity;
      agent.position += new_velocity * time_step;
    }
  }
}

impl Default for Simulator {
  fn default() -> Self {
    Self::new()
  }
}

#[cfg(test)]
#[path = "simulator_test.rs"]
mod test;
