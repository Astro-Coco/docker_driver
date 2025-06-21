from pymavlink import mavutil
import time
import math

def send_odometry(master,
                  frame_id: int,
                  child_frame_id: int,
                  position: tuple[float, float, float],
                  orientation: tuple[float, float, float, float],
                  velocity: tuple[float, float, float],
                  angular_velocity: tuple[float, float, float],
                  pos_covariance: list[float] | None = None,
                  velocity_covariance: list[float] | None = None,
                  reset_counter: int = 0,
                  estimator_type: int = 0,
                  quality: int = 100):
    """
    Send a MAVLink ODOMETRY message over an existing pymavlink connection.

    Fields (per MAVLink/common.xml and ArduPilot docs):
      time_usec           uint64 Timestamp (µs since system boot)
      frame_id            uint8  MAV_FRAME for pose data
      child_frame_id      uint8  MAV_FRAME for velocity frame
      x, y, z             float  Position in meters (Z positive down)
      q                   float[4] Quaternion (w, x, y, z)
      vx, vy, vz          float  Linear velocity in m/s (body or local FRD)
      rollspeed           float  Roll angular speed in rad/s
      pitchspeed          float  Pitch angular speed in rad/s
      yawspeed            float  Yaw angular speed in rad/s
      pos_covariance      float[21]  Position & angle covariances
      velocity_covariance float[21]  (ignored if NaN)
      reset_counter       uint8      Estimator reset counter
      estimator_type      uint8      (unused)
      quality             uint8      0–100% quality metric

    Example usage:
      master = mavutil.mavlink_connection('udpout:localhost:14550')
      send_odometry(
          master,
          frame_id=mavutil.mavlink.MAV_FRAME_LOCAL_FRD,
          child_frame_id=mavutil.mavlink.MAV_FRAME_BODY_FRD,
          position=(1.2, 0.0, -0.5),
          orientation=(1.0, 0.0, 0.0, 0.0),
          velocity=(0.1, 0.0, 0.0),
          angular_velocity=(0.0, 0.0, math.radians(10)),
      )
    """

    # Timestamp in microseconds
    time_usec = int(time.time() * 1e6)

    # Default covariances to NaN if not provided
    if pos_covariance is None:
        pos_covariance = [math.nan] * 21
    if velocity_covariance is None:
        velocity_covariance = [math.nan] * 21

    master.mav.odometry_send(
        time_usec,
        frame_id,
        child_frame_id,
        # position
        position[0], position[1], position[2],
        # quaternion (w, x, y, z)
        orientation[0], orientation[1], orientation[2], orientation[3],
        # linear velocity
        velocity[0], velocity[1], velocity[2],
        # angular velocity
        angular_velocity[0], angular_velocity[1], angular_velocity[2],
        # covariance arrays
        pos_covariance,
        velocity_covariance,
        # extra fields
        reset_counter,
        estimator_type,
        quality
    )
