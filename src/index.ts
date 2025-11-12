import { ExtensionContext } from "@foxglove/extension";
import { FrameTransform, LocationFix, PosesInFrame, Time } from "@foxglove/schemas";

import type { VehicleAttitude, VehicleGlobalPosition, VehicleLocalPosition } from "./types";
import {
  buildPositionCovariance,
  convertTimestamp,
  extractPitchAndRoll,
  nedHeadingToEnuQuaternion,
  nedQuaternionToEnu,
  nedToEnuPosition,
} from "./utils";

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

      const { x: x_enu, y: y_enu, z: z_enu } = nedToEnuPosition(x, y, z);

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

      const { x: x_enu, y: y_enu, z: z_enu } = nedToEnuPosition(x, y, z);

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
