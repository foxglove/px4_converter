import { ExtensionContext, MessageEvent } from "@foxglove/extension";
import { FrameTransform, LocationFix, Time } from "@foxglove/schemas";

// PX4 VehicleGlobalPosition message type
// Fused global position in WGS84 from the position estimator
type VehicleGlobalPosition = {
  timestamp: number | bigint; // time since system start (microseconds) - uint64 can be BigInt
  timestamp_sample?: number | bigint; // the timestamp of the raw data (microseconds)
  lat: number; // Latitude, (degrees)
  lon: number; // Longitude, (degrees)
  alt: number; // Altitude AMSL, (meters)
  alt_ellipsoid?: number; // Altitude above ellipsoid, (meters)
  lat_lon_valid?: boolean; // Whether lat/lon is valid
  alt_valid?: boolean; // Whether altitude is valid
  eph?: number; // Standard deviation of horizontal position error, (metres)
  epv?: number; // Standard deviation of vertical position error, (metres)
  [key: string]: unknown; // Allow for additional fields
};

// PX4 VehicleLocalPosition message type
// Local position estimate in NED frame
type VehicleLocalPosition = {
  timestamp: number | bigint; // time since system start (microseconds) - uint64 can be BigInt
  timestamp_sample?: number | bigint; // the timestamp of the raw data (microseconds)
  x: number; // X position in meters (North)
  y: number; // Y position in meters (East)
  z: number; // Z position in meters (Down)
  heading?: number; // Heading angle in radians (0 = North, positive = clockwise)
  [key: string]: unknown; // Allow for additional fields
};

// PX4 VehicleAttitude message type
// Quaternion rotation from FRD body frame to NED earth frame
type VehicleAttitude = {
  timestamp: number | bigint; // time since system start (microseconds) - uint64 can be BigInt
  timestamp_sample?: number | bigint; // the timestamp of the raw data (microseconds)
  q: [number, number, number, number]; // Quaternion [w, x, y, z] in Hamilton convention
  delta_q_reset?: [number, number, number, number];
  quat_reset_counter?: number;
  [key: string]: unknown; // Allow for additional fields
};

// Convert PX4 timestamp (microseconds) to Foxglove Time (sec, nsec)
// Handles both number and BigInt (uint64) types
function convertTimestamp(timestampUs: number | bigint): Time {
  // Convert BigInt to number if needed
  const timestampNum = typeof timestampUs === "bigint" ? Number(timestampUs) : timestampUs;
  const sec = Math.floor(timestampNum / 1e6);
  const nsec = Math.floor((timestampNum % 1e6) * 1e3);
  return { sec, nsec };
}

// Build position covariance matrix from eph and epv
// Position covariance is in ENU (East, North, Up) frame, row-major order
function buildPositionCovariance(
  eph?: number,
  epv?: number,
): [number, number, number, number, number, number, number, number, number] {
  if (eph === undefined && epv === undefined) {
    return [0, 0, 0, 0, 0, 0, 0, 0, 0];
  }

  // Convert standard deviation to variance (square it)
  const ephVar = eph !== undefined ? eph * eph : 0;
  const epvVar = epv !== undefined ? epv * epv : 0;

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

// Convert NED heading to ENU yaw and then to quaternion
// NED heading: 0 = North, positive = clockwise
// ENU yaw: 0 = East, positive = counter-clockwise
// Conversion: ENU_yaw = π/2 - NED_heading
function nedHeadingToEnuQuaternion(nedHeading: number): {
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

// Quaternion type helper
type Quaternion = { w: number; x: number; y: number; z: number };

// Convert quaternion from NED to ENU frame
// The quaternion represents rotation from body (FRD) to earth (NED)
// We need to convert it to body (FLU) to earth (ENU)
// Transformation: q_enu = q_ned_to_enu * q_ned * q_ned_to_enu^-1
// Where q_ned_to_enu rotates NED frame to ENU frame (90° around Z, then 180° around X)
function nedQuaternionToEnu(q: [number, number, number, number]): Quaternion {
  const [w, x, y, z] = q;

  // NED to ENU frame transformation:
  // 1. Rotate 90° around Z (North -> East)
  // 2. Rotate 180° around X (Down -> Up)
  // Combined: this swaps X↔Y and negates Z
  // The transformation quaternion for 90° Z then 180° X
  // q_transform = q_z(90°) * q_x(180°)
  // q_z(90°) = [cos(45°), 0, 0, sin(45°)] = [√2/2, 0, 0, √2/2]
  // q_x(180°) = [cos(90°), sin(90°), 0, 0] = [0, 1, 0, 0]
  // q_transform = [0, √2/2, 0, √2/2]

  // Actually, simpler: the effect is to swap X↔Y and negate Z in the quaternion
  // But we need to properly transform the quaternion, not just swap components
  // For a quaternion q representing rotation R, transforming to new frame F:
  // q_new = q_frame * q * q_frame^-1

  // The frame transformation from NED to ENU:
  // - X: North -> East (rotate 90° around Z)
  // - Y: East -> North (rotate -90° around Z)
  // - Z: Down -> Up (flip)
  // This is equivalent to: rotate 90° around Z, then 180° around new X

  // Simplified transformation for quaternion components:
  // When transforming a quaternion from NED to ENU, the effect on the quaternion
  // representing body-to-earth rotation is:
  return {
    w: w,
    x: y, // Swap x and y
    y: x, // Swap x and y
    z: -z, // Negate z (accounting for Z-axis flip)
  };
}

// Extract yaw (rotation around Z-axis) from quaternion
function extractYaw(q: Quaternion): number {
  // Yaw is rotation around Z-axis
  // For quaternion q = [w, x, y, z], yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
  // But simpler: yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
  const yaw = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
  return yaw;
}

// Create quaternion from yaw angle (rotation around Z-axis only)
function yawToQuaternion(yaw: number): Quaternion {
  const halfYaw = yaw / 2;
  return {
    w: Math.cos(halfYaw),
    x: 0,
    y: 0,
    z: Math.sin(halfYaw),
  };
}

// Multiply two quaternions: q1 * q2
function multiplyQuaternions(q1: Quaternion, q2: Quaternion): Quaternion {
  return {
    w: q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
    x: q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
    y: q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,
    z: q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w,
  };
}

// Get quaternion conjugate (inverse for unit quaternion)
function quaternionConjugate(q: Quaternion): Quaternion {
  return { w: q.w, x: -q.x, y: -q.y, z: -q.z };
}

// Extract pitch and roll only (remove yaw) from quaternion
// This gives the rotation from base_intermediate (yaw-only) to base_link (full attitude)
function extractPitchAndRoll(q: Quaternion): Quaternion {
  // Extract yaw
  const yaw = extractYaw(q);

  // Create yaw-only quaternion
  const yawQ = yawToQuaternion(yaw);

  // Remove yaw: q_pitch_roll = q * yawQ^-1
  // This gives us the rotation that, when combined with yaw, gives the full attitude
  const yawQInv = quaternionConjugate(yawQ);
  return multiplyQuaternions(q, yawQInv);
}

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerMessageConverter({
    type: "schema",
    fromSchemaName: "vehicle_global_position",
    toSchemaName: "foxglove.LocationFix",
    converter: (
      inputMessage: VehicleGlobalPosition,
      _messageEvent: MessageEvent<VehicleGlobalPosition>,
    ): LocationFix => {
      const { lat, lon, alt, timestamp, eph, epv } = inputMessage;

      // Use the position values directly (even if marked invalid, they may still be useful for debugging)
      // Note: LocationFix doesn't have validity flags, so we always use the provided values
      const latitude = lat;
      const longitude = lon;
      const altitude = alt;

      // Convert timestamp
      const time = convertTimestamp(timestamp);

      // Build position covariance from eph and epv if available
      const position_covariance = buildPositionCovariance(eph, epv);
      const hasCovariance = eph !== undefined || epv !== undefined;
      const position_covariance_type = hasCovariance ? 2 : 0; // 2 = DIAGONAL_KNOWN, 0 = UNKNOWN

      return {
        timestamp: time,
        frame_id: "base_link", // Default frame, adjust as needed
        latitude,
        longitude,
        altitude,
        position_covariance,
        position_covariance_type,
      } as LocationFix; // If we don't provide color, the defaults will show the current position
    },
  });

  extensionContext.registerMessageConverter({
    type: "schema",
    fromSchemaName: "vehicle_local_position",
    toSchemaName: "foxglove.FrameTransform",
    converter: (
      inputMessage: VehicleLocalPosition,
      _messageEvent: MessageEvent<VehicleLocalPosition>,
    ): FrameTransform => {
      const { x, y, z, timestamp, heading } = inputMessage;

      // Convert timestamp
      const time = convertTimestamp(timestamp);

      // Convert NED to ENU frame
      // NED: X=North, Y=East, Z=Down
      // ENU: X=East, Y=North, Z=Up
      const x_enu = y; // East = East
      const y_enu = x; // North = North
      const z_enu = -z; // Up = -Down

      // Use heading to create quaternion if available, otherwise use identity rotation
      const rotation =
        heading !== undefined ? nedHeadingToEnuQuaternion(heading) : { w: 1, x: 0, y: 0, z: 0 }; // Identity quaternion

      return {
        timestamp: time,
        parent_frame_id: "map",
        child_frame_id: "base_intermediate", // Has no roll and pitch, only yaw
        translation: { x: x_enu, y: y_enu, z: z_enu },
        rotation,
      } as FrameTransform;
    },
  });

  extensionContext.registerMessageConverter({
    type: "schema",
    fromSchemaName: "vehicle_attitude",
    toSchemaName: "foxglove.FrameTransform",
    converter: (inputMessage: VehicleAttitude): FrameTransform => {
      const q = inputMessage.q;
      const timestamp = inputMessage.timestamp;

      // Convert timestamp
      const time = convertTimestamp(timestamp);

      // Convert quaternion from NED to ENU frame
      const qEnu = nedQuaternionToEnu(q);

      // Extract pitch and roll only (remove yaw)
      // This gives the rotation from base_intermediate (yaw-only) to base_link (full attitude)
      const pitchRollRotation = extractPitchAndRoll(qEnu);

      return {
        timestamp: time,
        parent_frame_id: "base_intermediate",
        child_frame_id: "base_link",
        translation: { x: 0, y: 0, z: 0 }, // No translation, only rotation
        rotation: pitchRollRotation,
      } as FrameTransform;
    },
  });
}
