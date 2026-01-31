#!/usr/bin/env python3
import numpy as np
import math

"""Example usage of the script."""
# Example vectors (not normalized)
initial_vec = [0.0, 0.0, 1.0]  # Pointing along z-axis
output_vec = [-3.0, -1.2, -10]  # Custom vector pointing diagonally


def normalize_vector(v):
    """Normalize a vector to unit length."""
    v = np.array(v, dtype=float)
    norm = np.linalg.norm(v)
    if norm == 0:
        raise ValueError("Cannot normalize zero vector")
    return v / norm


def calculate_rpy_from_vectors(initial_vector, output_vector):
    """
    Calculate RPY angles to transform initial_vector to output_vector.

    Args:
        initial_vector: Initial 3D vector [x, y, z] (not normalized)
        output_vector: Target 3D vector [x, y, z] (not normalized)

    Returns:
        tuple: (roll, pitch, yaw) in radians
    """
    # Normalize both vectors
    v1 = normalize_vector(initial_vector)
    v2 = normalize_vector(output_vector)

    # Calculate rotation axis using cross product
    rotation_axis = np.cross(v1, v2)
    axis_norm = np.linalg.norm(rotation_axis)

    # Check if vectors are already aligned
    if axis_norm < 1e-10:
        # Vectors are parallel or anti-parallel
        dot_product = np.dot(v1, v2)
        if dot_product > 0:
            # Vectors are already aligned
            return (0.0, 0.0, 0.0)
        else:
            # Vectors are anti-parallel, rotate 180 degrees around any perpendicular axis
            # Choose rotation around x-axis for simplicity
            return (math.pi, 0.0, 0.0)

    # Normalize rotation axis
    rotation_axis = rotation_axis / axis_norm

    # Calculate rotation angle using dot product
    cos_angle = np.dot(v1, v2)
    cos_angle = np.clip(cos_angle, -1.0, 1.0)  # Ensure valid range
    angle = math.acos(cos_angle)

    # Convert axis-angle to rotation matrix
    # Using Rodrigues' rotation formula
    K = np.array(
        [
            [0, -rotation_axis[2], rotation_axis[1]],
            [rotation_axis[2], 0, -rotation_axis[0]],
            [-rotation_axis[1], rotation_axis[0], 0],
        ]
    )

    R = np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * np.dot(K, K)

    # Extract RPY angles from rotation matrix
    # Using standard aerospace sequence: roll -> pitch -> yaw
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0

    return (roll, pitch, yaw)


def main():

    try:
        roll, pitch, yaw = calculate_rpy_from_vectors(initial_vec, output_vec)

        print(f"Initial vector: {initial_vec}")
        print(f"Output vector: {output_vec}")
        print(
            f"RPY angles (radians): roll={roll:.6f}, pitch={pitch:.6f}, yaw={yaw:.6f}"
        )
        print(
            f"RPY angles (degrees): roll={math.degrees(roll):.2f}°, pitch={math.degrees(pitch):.2f}°, yaw={math.degrees(yaw):.2f}°"
        )

        # Verify the transformation
        print("\nVerification:")
        # Create rotation matrix from RPY angles
        Rx = np.array(
            [
                [1, 0, 0],
                [0, math.cos(roll), -math.sin(roll)],
                [0, math.sin(roll), math.cos(roll)],
            ]
        )

        Ry = np.array(
            [
                [math.cos(pitch), 0, math.sin(pitch)],
                [0, 1, 0],
                [-math.sin(pitch), 0, math.cos(pitch)],
            ]
        )

        Rz = np.array(
            [
                [math.cos(yaw), -math.sin(yaw), 0],
                [math.sin(yaw), math.cos(yaw), 0],
                [0, 0, 1],
            ]
        )

        R_total = np.dot(Rz, np.dot(Ry, Rx))

        # Apply rotation to initial vector
        initial_normalized = normalize_vector(initial_vec)
        transformed = np.dot(R_total, initial_normalized)
        output_normalized = normalize_vector(output_vec)

        print(f"Transformed vector: {transformed}")
        print(f"Target vector (normalized): {output_normalized}")
        print(f"Difference: {np.linalg.norm(transformed - output_normalized):.10f}")

    except ValueError as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    main()
