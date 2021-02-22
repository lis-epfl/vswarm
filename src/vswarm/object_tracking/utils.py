import numpy as np


def cartesian2spherical(cartesian):
    """Convert cartesian to spherical coordinates

    Args:
        cartesian (ndarray): Vector (x, y, z)

    Returns:
        spherical (ndarray): Vector (distance, azimuth, elevation)
    """
    cartesian = np.array(cartesian).squeeze()
    x, y, z = cartesian
    distance = np.linalg.norm(cartesian)
    azimuth = np.arccos(z / distance)
    elevation = np.arctan2(y, x)  # Use arctan2 instead of arctan to get proper sign!
    return np.array([distance, azimuth, elevation])


def spherical2cartesian(spherical):
    """Convert spherical to cartesian coordinates

    Args:
        spherical (ndarray): Vector (distance, azimuth, elevation)

    Returns:
        cartesian (ndarray): Vector (x, y, z)
    """
    spherical = np.array(spherical).squeeze()
    distance, azimuth, elevation = spherical
    x = distance * np.sin(azimuth) * np.cos(elevation)
    y = distance * np.sin(azimuth) * np.sin(elevation)
    z = distance * np.cos(azimuth)
    return np.array([x, y, z])


def cartesian2polar(cartesian):
    """Converts cartesian to polar coordinates

    Args:
        cartesian (ndarray): Vector (x, y)

    Returns:
        polar (ndarray): Vector (range, azimuth)
    """
    cartesian = np.array(cartesian).squeeze()
    x, y = cartesian
    r = np.linalg.norm([x, y])
    azimuth = np.arctan2(y, x)
    return np.array([r, azimuth])


def polar2cartesian(polar):
    """Converts polar to cartesian coordinates

    Args:
        polar (ndarray): Vector (range, azimuth)

    Returns:
        cartesian (ndarray): Vector (x, y)
    """
    polar = np.array(polar).squeeze()
    r, azimuth = polar
    x = r * np.cos(azimuth)
    y = r * np.sin(azimuth)
    return np.array([x, y])


def random_uniform_within_circle():
    """Sample random point within a unit circle

    Returns:
        sample (ndarray): Vector (x, y)

    """
    rho = np.sqrt(np.random.uniform(0, 1))
    phi = np.random.uniform(0, 2 * np.pi)
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return np.array([x, y])
