# dodgy

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

This crate is essentially a port of [RVO2](https://gamma.cs.unc.edu/RVO2/) to
Rust. Several changes have been made: tests have been written, code more
commented, and the public API made more flexible.

## Example

This example uses the "raw" API.

```rust
let agents = vec![
  Agent {
    position: Vec2::ZERO,
    velocity: Vec2::ZERO,
    radius: 1.0,
    avoidance_responsibility: 1.0,
    max_velocity: 5.0,
  },
  ...
];

let goal_points = vec![
  Vec2::new(50.0, 0.0),
  ...
];

let obstacles = vec![
  Obstacle::Closed{
    vertices: vec![
      Vec2::new(-1000.0, -1000.0),
      Vec2::new(-1000.0, 1000.0),
      Vec2::new(1000.0, 1000.0),
      Vec2::new(1000.0, -1000.0),
    ],
  },
  ...
];

let time_horizon = 3.0;
let obstacle_time_horizon = 1.0;

loop {
  let delta_seconds = ...;
  if delta_seconds == 0.0 {
    // Skip frames where agents can't move anyway.
    continue;
  }

  let new_velocities = Vec::with_capacity(agents.len());

  for i in 0..agents.len() {
    let neighbours = agents[..i]
      .iter()
      .chain(agents[(i + 1)..].iter())
      .collect::<Vec<&Agent>>();
    let nearby_obstacles = obstacles
      .iter()
      .map(|obstacle| obstacle)
      .collect::<Vec<&Obstacle>>();

    let preferred_velocity = (goal_points[i] - agents[i].position)
      .normalize_or_zero() * agents[i].max_velocity;

    let avoidance_velocity = agents[i].compute_avoidance_velocity(
      &neighbours,
      &nearby_obstacles,
      preferred_velocity,
      time_horizon,
      obstacle_time_horizon,
      delta_seconds);
    new_velocities.push(avoidance_velocity);
  }

  for (i, agent) in agents.iter_mut().enumerate() {
    agent.velocity = new_velocities[i];
    agent.position += agent.velocity * delta_seconds;
  }

  // Update rendering using new agent positions.
}
```

This is the preferred API to use, since finding neighbours is essentially just a
spatial query. Often, finding related objects within some radius is performed
anyway in most game engines. Using this API, neighbours can be find through your
regular spatial queries, and exposes just the avoidance part.

However, an alternative using the `Simulator` struct:

```rust
let mut simulator = Simulator::new();
simulator.add_agent(Agent {
  position: Vec2::ZERO,
  velocity: Vec2::ZERO,
  radius: 1.0,
  avoidance_responsibility: 1.0,
  max_velocity: 5.0,
}, AgentParameters {
  goal_point: Vec2::new(50.0, 0.0),
  time_horizon: 3.0,
  obstacle_time_horizon: 1.0,
});
...

simulator.add_obstacle(
  Obstacle::Closed{
    vertices: vec![
      Vec2::new(-1000.0, -1000.0),
      Vec2::new(-1000.0, 1000.0),
      Vec2::new(1000.0, 1000.0),
      Vec2::new(1000.0, -1000.0),
    ],
  }
);
...

loop {
  let delta_seconds = ...;
  simulator.step(delta_seconds);

  // Update rendering using new agent positions (using simulator.get_agent).
}
```

Again, this is **not the preferred method**! It is just a simpler way to get up
and running for a small group of users. The other API is more flexible and
preferred.

## License

Licensed under the [MIT license](LICENSE).

## Attribution

dodgy contains code ported from RVO2. See
[original_license.txt](original_license.txt).
