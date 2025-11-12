// PX4 VehicleGlobalPosition message type
// Fused global position in WGS84 from the position estimator
export type VehicleGlobalPosition = {
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
export type VehicleLocalPosition = {
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
export type VehicleAttitude = {
  timestamp: number | bigint; // time since system start (microseconds) - uint64 can be BigInt
  timestamp_sample?: number | bigint; // the timestamp of the raw data (microseconds)
  q: [number, number, number, number]; // Quaternion [w, x, y, z] in Hamilton convention
  delta_q_reset?: [number, number, number, number];
  quat_reset_counter?: number;
  [key: string]: unknown; // Allow for additional fields
};

// Quaternion type helper
export type Quaternion = { w: number; x: number; y: number; z: number };
