// Re-export Line so we can use it to provide debug data.
pub use crate::linear_programming::Line;

/// Internal data that is used to generate the final suggested velocity.
#[derive(Debug, Clone)]
pub enum DebugData {
  /// The original problem (where the agent uses its current velocity) was
  /// solved.
  Satisfied {
    /// The constraints that needed to be satisfied.
    constraints: Vec<Line>,
  },
  /// The original problem (where the agent uses its current velocity) was
  /// invalid, so the algorithm fell back to pretending the agent has a
  /// zero-velocity, which is trivially satisfiable.
  Fallback {
    /// The constraints for the original problem.
    original_constraints: Vec<Line>,
    /// The constraints after falling back (pretending the agent has zero
    /// velocity).
    fallback_constraints: Vec<Line>,
  },
}
