import numpy as np


def reynolds(positions, velocities=None, separation_gain=1.0, cohesion_gain=1.0,
             alignment_gain=1.0, perception_radius=None, max_agents=None):
    """Reynolds flocking for a single agent.

    Args:
        positions: List of relative positions to other agents.
        velocities: List of velocities of other agents (optional).
        separation_gain: Scalar gain for separation component.
        cohesion_gain: Scalar gain for cohesion component.
        alignment_gain: Scalar gain for alignment component.
        perception_radius: Scalar metric distance.
        max_agents: Maximum number of agents to include.

    Returns:
        command: Velocity command.

    """

    positions = np.array(positions)
    if velocities is None:
        velocities = np.zeros_like(positions)
    else:
        velocities = np.array(velocities)
    num_agents, dims = positions.shape

    indices = np.arange(num_agents)
    # Filter agents by metric distance
    distances = np.linalg.norm(positions, axis=1)
    if perception_radius is not None:
        indices = distances < perception_radius
        distances = distances[indices]

    # Filter agents by topological distance
    if max_agents is not None:
        indices = distances.argsort()[:max_agents]
        distances = distances[indices]

    # Return early if no agents
    if len(distances) == 0:
        return np.zeros(dims)

    # Compute Reynolds flocking only if there is an agent in range
    positions = positions[indices]
    velocities = velocities[indices]
    dist_inv = positions / distances[:, np.newaxis] ** 2

    # Renolds command computations
    separation = -separation_gain * dist_inv.mean(axis=0)  # Sum may lead to instabilities
    cohesion = cohesion_gain * positions.mean(axis=0)
    alignment = alignment_gain * velocities.mean(axis=0)

    return separation + cohesion + alignment
