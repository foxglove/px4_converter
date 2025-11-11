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
function buildPositionCovariance(eph?: number, epv?: number): [
  number,
  number,
  number,
  number,
  number,
  number,
  number,
  number,
  number,
] {
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
    ephVar, 0, 0, // East-East, East-North, East-Up
    0, ephVar, 0, // North-East, North-North, North-Up
    0, 0, epvVar, // Up-East, Up-North, Up-Up
  ];
}

// Convert NED heading to ENU yaw and then to quaternion
// NED heading: 0 = North, positive = clockwise
// ENU yaw: 0 = East, positive = counter-clockwise
// Conversion: ENU_yaw = π/2 - NED_heading
function nedHeadingToEnuQuaternion(nedHeading: number): { w: number; x: number; y: number; z: number } {
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
      const x_enu = y;      // East = East
      const y_enu = x;      // North = North
      const z_enu = -z;     // Up = -Down

      // Use heading to create quaternion if available, otherwise use identity rotation
      const rotation = heading !== undefined
        ? nedHeadingToEnuQuaternion(heading)
        : { w: 1, x: 0, y: 0, z: 0 }; // Identity quaternion

      return {
        timestamp: time,
        parent_frame_id: "map",
        child_frame_id: "base_intermediate", // Has no roll and pitch, only yaw
        translation: { x: x_enu, y: y_enu, z: z_enu },
        rotation,
      } as FrameTransform;
    },
  });
}
