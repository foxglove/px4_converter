import { Time } from "@foxglove/schemas";

import type { Quaternion } from "./types";

// Convert PX4 timestamp (microseconds) to Foxglove Time (sec, nsec)
// Handles both number and BigInt (uint64) types
export function convertTimestamp(timestampUs: number | bigint): Time {
  // Convert BigInt to number if needed
  const timestampNum = typeof timestampUs === "bigint" ? Number(timestampUs) : timestampUs;
  const sec = Math.floor(timestampNum / 1e6);
  const nsec = Math.floor((timestampNum % 1e6) * 1e3);
  return { sec, nsec };
}

// Build position covariance matrix from eph and epv
// Position covariance is in ENU (East, North, Up) frame, row-major order
export function buildPositionCovariance(
  eph?: number,
  epv?: number,
): [number, number, number, number, number, number, number, number, number] {
  if (eph == undefined && epv == undefined) {
    return [0, 0, 0, 0, 0, 0, 0, 0, 0];
  }

  // Convert standard deviation to variance (square it)
  const ephVar = eph != undefined ? eph * eph : 0;
  const epvVar = epv != undefined ? epv * epv : 0;

  // Position covariance matrix in ENU frame (row-major):
  // [E_E  E_N  E_U]
  // [N_E  N_N  N_U]
  // [U_E  U_N  U_U]
  // For diagonal covariance: E_E = N_N = eph^2, U_U = epv^2
  return [
    ephVar,
    0,
    0, // East-East, East-North, East-Up
    0,
    ephVar,
    0, // North-East, North-North, North-Up
    0,
    0,
    epvVar, // Up-East, Up-North, Up-Up
  ];
}

// Convert position from NED to ENU frame
// NED: X=North, Y=East, Z=Down
// ENU: X=East, Y=North, Z=Up
export function nedToEnuPosition(
  x: number,
  y: number,
  z: number,
): {
  x: number;
  y: number;
  z: number;
} {
  return {
    x: y, // East = East
    y: x, // North = North
    z: -z, // Up = -Down
  };
}

// Convert NED heading to ENU yaw and then to quaternion
// NED heading: 0 = North, positive = clockwise
// ENU yaw: 0 = East, positive = counter-clockwise
// Conversion: ENU_yaw = π/2 - NED_heading
export function nedHeadingToEnuQuaternion(nedHeading: number): {
  w: number;
  x: number;
  y: number;
  z: number;
} {
  // Convert NED heading to ENU yaw
  const enuYaw = Math.PI / 2 - nedHeading;
  const halfYaw = enuYaw / 2;
  return {
    w: Math.cos(halfYaw),
    x: 0,
    y: 0,
    z: Math.sin(halfYaw),
  };
}

// Convert quaternion from NED to ENU frame
// The quaternion represents rotation from body (FRD) to earth (NED)
// We need to convert it to body (FLU) to earth (ENU)
// Transformation: q_enu = q_ned_to_enu * q_ned * q_ned_to_enu^-1
// Where q_ned_to_enu rotates NED frame to ENU frame (90° around Z, then 180° around X)
export function nedQuaternionToEnu(q: [number, number, number, number]): Quaternion {
  const [w, x, y, z] = q;

  // Simplified transformation for quaternion components:
  // When transforming a quaternion from NED to ENU, the effect on the quaternion
  // representing body-to-earth rotation is:
  return {
    w,
    x: y, // Swap x and y
    y: x, // Swap x and y
    z: -z, // Negate z (accounting for Z-axis flip)
  };
}

// Extract yaw (rotation around Z-axis) from quaternion
export function extractYaw(q: Quaternion): number {
  const yaw = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
  return yaw;
}

// Create quaternion from yaw angle (rotation around Z-axis only)
export function yawToQuaternion(yaw: number): Quaternion {
  const halfYaw = yaw / 2;
  return {
    w: Math.cos(halfYaw),
    x: 0,
    y: 0,
    z: Math.sin(halfYaw),
  };
}

export function multiplyQuaternions(q1: Quaternion, q2: Quaternion): Quaternion {
  return {
    w: q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
    x: q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
    y: q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,
    z: q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w,
  };
}

export function quaternionConjugate(q: Quaternion): Quaternion {
  return { w: q.w, x: -q.x, y: -q.y, z: -q.z };
}

// Extract pitch and roll only (remove yaw) from quaternion
// This gives the rotation from base_intermediate (yaw-only) to base_link (full attitude)
export function extractPitchAndRoll(q: Quaternion): Quaternion {
  // Extract yaw
  const yaw = extractYaw(q);

  // Create yaw-only quaternion
  const yawQ = yawToQuaternion(yaw);

  // Remove yaw: q_pitch_roll = q * yawQ^-1
  // This gives us the rotation that, when combined with yaw, gives the full attitude
  const yawQInv = quaternionConjugate(yawQ);
  return multiplyQuaternions(q, yawQInv);
}
