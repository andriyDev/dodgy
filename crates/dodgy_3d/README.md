# dodgy_3d

A Rust crate to compute local collision avoidance (specifically ORCA) for agents.

## Why local collision avoidance?

Characters in video games generally need to find paths to navigate around the
game world. Once this is done, the path needs to be followed. The trouble occurs
when characters start getting in the way of each other. As paths are not
generally regenerated every game frame, other characters cannot be taken into
account. Local collision avoidance provides cheap avoidance for characters even
in high-density situations.

## Which local collision avoidance?

There are several algorithms for local collision avoidance. This crate
implements [ORCA](https://gamma.cs.unc.edu/ORCA/).

This crate is essentially a port of [RVO2-3D](https://gamma.cs.unc.edu/RVO2/) to
Rust. Several changes have been made: tests have been written, code more
commented, and the public API made more flexible.

## Example

This example uses the "raw" API.

```rust
use std::borrow::Cow;

use dodgy_3d::{Agent, AvoidanceOptions, Vec3};

let mut agents: Vec<Cow<'static, Agent>> = vec![
  Cow::Owned(Agent {
    position: Vec3::ZERO,
    velocity: Vec3::ZERO,
    radius: 1.0,
    avoidance_responsibility: 1.0,
  }),
  // Add more agents here.
];

let goal_points = vec![
  Vec3::new(50.0, 0.0, 0.0),
  // Add goal points for every agent.
];

let time_horizon = 3.0;

fn get_delta_seconds() -> f32 {
  // Use something that actually gets the time between frames.
  return 0.01;
}

for i in 0..100 {
  let delta_seconds = get_delta_seconds();
  if delta_seconds == 0.0 {
    // Skip frames where agents can't move anyway.
    continue;
  }

  let mut new_velocities = Vec::with_capacity(agents.len());

  for i in 0..agents.len() {
    let neighbours = agents[..i]
      .iter()
      .chain(agents[(i + 1)..].iter())
      .map(|agent| agent.clone())
      .collect::<Vec<Cow<'_, Agent>>>();

    let agent_max_speed = 5.0;
    let preferred_velocity = (goal_points[i] - agents[i].position)
      .normalize_or_zero() * agent_max_speed;

    let avoidance_velocity = agents[i].compute_avoiding_velocity(
      &neighbours,
      preferred_velocity,
      agent_max_speed,
      delta_seconds,
      &AvoidanceOptions { time_horizon },
    );
    new_velocities.push(avoidance_velocity);
  }

  for (i, agent) in agents.iter_mut().map(Cow::to_mut).enumerate() {
    agent.velocity = new_velocities[i];
    agent.position += agent.velocity * delta_seconds;
  }

  // Update rendering using new agent positions.
}
```

This is the preferred API to use, since finding neighbours is essentially just a
spatial query. Often, finding related objects within some radius is performed
anyway in most game engines. Using this API, neighbours can be found through
your regular spatial queries, and exposes just the avoidance part.

However, an alternative using the `Simulator` struct:

```rust
use dodgy_3d::{
  Agent, AvoidanceOptions, AgentParameters, Simulator, SimulatorMargin, Vec3
};

let mut simulator = Simulator::new();
simulator.add_agent(Agent {
  position: Vec3::ZERO,
  velocity: Vec3::ZERO,
  radius: 1.0,
  avoidance_responsibility: 1.0,
}, AgentParameters {
  goal_point: Vec3::new(50.0, 0.0, 0.0),
  max_speed: 5.0,
  obstacle_margin: SimulatorMargin::Distance(0.1),
  time_horizon: 3.0,
  obstacle_time_horizon: 1.0,
});
// Add more agents.

fn get_delta_seconds() -> f32 {
  // Use something that actually gets the time between frames.
  return 0.01;
}

for i in 0..100 {
  let delta_seconds = get_delta_seconds();
  simulator.step(delta_seconds);

  // Update rendering using new agent positions (using simulator.get_agent).
}
```

Again, this is **not the preferred method**! It is just a simpler way to get up
and running for a small group of users. The other API is more flexible and
preferred.

## License

License under either of

* Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
* MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall
be dual licensed as above, without any additional terms or conditions.

## Attribution

dodgy_3d contains code ported from RVO2. See
[original_license.txt](original_license.txt).
