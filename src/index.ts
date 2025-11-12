import { ExtensionContext } from "@foxglove/extension";
import { FrameTransform, LocationFix, PosesInFrame, Time } from "@foxglove/schemas";

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
function extractYaw(q: Quaternion): number {
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

function multiplyQuaternions(q1: Quaternion, q2: Quaternion): Quaternion {
  return {
    w: q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
    x: q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
    y: q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,
    z: q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w,
  };
}

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
  extensionContext.registerMessageConverter<VehicleGlobalPosition>({
    type: "schema",
    fromSchemaName: "vehicle_global_position",
    toSchemaName: "foxglove.LocationFix",
    converter: (inputMessage, _messageEvent): LocationFix => {
      const { lat, lon, alt, timestamp, eph, epv } = inputMessage;

      // Use the position values directly (even if marked invalid, they may still be useful for debugging)
      // Note: LocationFix doesn't have validity flags, so we always use the provided values
      const latitude = lat;
      const longitude = lon;
      const altitude = alt;

      const time = convertTimestamp(timestamp);

      const position_covariance = buildPositionCovariance(eph, epv);
      const hasCovariance = eph != undefined || epv != undefined;
      const position_covariance_type = hasCovariance ? 2 : 0; // 2 = DIAGONAL_KNOWN, 0 = UNKNOWN

      return {
        timestamp: time,
        frame_id: "base_link",
        latitude,
        longitude,
        altitude,
        position_covariance,
        position_covariance_type,
      } as LocationFix;
    },
  });

  extensionContext.registerMessageConverter<VehicleLocalPosition>({
    type: "schema",
    fromSchemaName: "vehicle_local_position",
    toSchemaName: "foxglove.FrameTransform",
    converter: (inputMessage, _messageEvent): FrameTransform => {
      const { x, y, z, timestamp, heading } = inputMessage;

      const time = convertTimestamp(timestamp);

      // Convert NED to ENU frame
      // NED: X=North, Y=East, Z=Down
      // ENU: X=East, Y=North, Z=Up
      const x_enu = y; // East = East
      const y_enu = x; // North = North
      const z_enu = -z; // Up = -Down

      // Use heading to create quaternion if available, otherwise use identity rotation
      const rotation =
        heading != undefined ? nedHeadingToEnuQuaternion(heading) : { w: 1, x: 0, y: 0, z: 0 }; // Identity quaternion

      return {
        timestamp: time,
        parent_frame_id: "map",
        child_frame_id: "base_intermediate", // Has no roll and pitch, only yaw
        translation: { x: x_enu, y: y_enu, z: z_enu },
        rotation,
      } as FrameTransform;
    },
  });

  extensionContext.registerMessageConverter<VehicleAttitude>({
    type: "schema",
    fromSchemaName: "vehicle_attitude",
    toSchemaName: "foxglove.FrameTransform",
    converter: (inputMessage, _messageEvent): FrameTransform => {
      const q = inputMessage.q;
      const timestamp = inputMessage.timestamp;

      const time = convertTimestamp(timestamp);

      const qEnu = nedQuaternionToEnu(q);

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

  type PoseWithTimestamp = {
    position: { x: number; y: number; z: number };
    orientation: { x: number; y: number; z: number; w: number };
    timestamp: Time;
  };

  const accumulatedPoses: PoseWithTimestamp[] = [];
  const MAX_POSES = 10000; // Maximum number of poses to keep (sliding window)
  const MIN_DISTANCE_M = 0.03; // Minimum distance in meters before adding a new pose

  extensionContext.registerMessageConverter<VehicleLocalPosition>({
    type: "schema",
    fromSchemaName: "vehicle_local_position",
    toSchemaName: "foxglove.PosesInFrame",
    converter: (inputMessage, _messageEvent): PosesInFrame => {
      const { x, y, z, timestamp, heading } = inputMessage;

      const time = convertTimestamp(timestamp);

      // Convert NED to ENU frame
      // NED: X=North, Y=East, Z=Down
      // ENU: X=East, Y=North, Z=Up
      const x_enu = y; // East = East
      const y_enu = x; // North = North
      const z_enu = -z; // Up = -Down

      // Create orientation from heading if available, otherwise use identity
      const orientation =
        heading != undefined ? nedHeadingToEnuQuaternion(heading) : { w: 1, x: 0, y: 0, z: 0 };

      // Only add pose if it's required distance away from the last pose
      let shouldAddPose = true;
      if (accumulatedPoses.length > 0) {
        const lastPose = accumulatedPoses[accumulatedPoses.length - 1]!; // Safe: we checked length > 0
        const dx = x_enu - lastPose.position.x;
        const dy = y_enu - lastPose.position.y;
        const dz = z_enu - lastPose.position.z;
        const distance = Math.sqrt(dx * dx + dy * dy + dz * dz);
        shouldAddPose = distance >= MIN_DISTANCE_M;
      }

      // Add new pose to accumulated array if needed
      if (shouldAddPose) {
        accumulatedPoses.push({
          position: { x: x_enu, y: y_enu, z: z_enu },
          orientation,
          timestamp: time,
        });

        // Keep fixed sliding window of poses
        if (accumulatedPoses.length > MAX_POSES) {
          accumulatedPoses.shift(); // Remove oldest pose
        }
      }

      const poses = accumulatedPoses.map((p) => ({
        position: p.position,
        orientation: p.orientation,
      }));

      // Create PosesInFrame message
      return {
        timestamp: time,
        frame_id: "map",
        poses, // All accumulated poses
      } as PosesInFrame;
    },
  });
}
