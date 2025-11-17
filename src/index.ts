import { ExtensionContext } from "@foxglove/extension";
import { FrameTransform, LocationFix, PosesInFrame } from "@foxglove/schemas";

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
        parent_frame_id: "local_origin",
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

      // Create PosesInFrame message with a single pose
      return {
        timestamp: time,
        frame_id: "local_origin",
        poses: [
          {
            position: { x: x_enu, y: y_enu, z: z_enu },
            orientation,
          },
        ],
      } as PosesInFrame;
    },
  });
}
