# FAST-LIO to ArduPilot Integration Guide

This guide explains how to achieve stable drone flight using FAST-LIO SLAM with ArduPilot.

## System Architecture

```
┌──────────────┐     ┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│  Livox MID360 │────▶│   FAST-LIO   │────▶│  MAVROS      │────▶│  ArduPilot   │
│   (LiDAR)     │     │   (SLAM)     │     │  Bridge      │     │   (FCU)      │
└──────────────┘     └──────────────┘     └──────────────┘     └──────────────┘
       │                       │                    │                    │
   Livox/Imu            /Odometry topic     /mavros/vision*    Attitude/PWM
   topic               (10Hz)              topics            to motors
```

## Coordinate Frames

**CRITICAL**: The bridge now outputs NED (North-East-Down) directly to MAVROS:

- `map_ned` - ArduPilot's earth-fixed frame (NED) **MUST BE USED**
- `odom_ned` - Odometry origin in NED
- `odom` - FAST-LIO's origin (ENU - converted to NED by bridge)
- `camera_init` - FAST-LIO's world frame (aligned to `odom`)
- `body` - FAST-LIO's robot frame (IMU/LiDAR frame)
- `base_link` - ArduPilot standard robot frame

**WARNING**: Using `frame_id = "odom"` will cause drift because MAVROS won't convert to NED.
The bridge must use `frame_id = "map_ned"` for ArduPilot to correctly interpret the data.

## Critical Fixes Applied

### 1. Message Type Fix
**OLD (BROKEN)**: Using `PoseStamped` on `/mavros/vision_pose/pose`
**NEW (FIXED)**: Using `PoseWithCovarianceStamped` on `/mavros/vision_pose/pose_cov`

ArduPilot's EKF **requires** covariance information to properly fuse vision data.

### 2. Covariance Pass-Through
FAST-LIO outputs covariance in its Odometry message. The bridge now passes this through to MAVROS.
The covariance tells the EKF how much to trust the measurement.

### 3. Timestamp Handling
- Using message timestamp from FAST-LIO (not ROS time) for synchronization
- Added validity checks to detect time jumps

### 4. Removed Artificial Noise
**OLD**: Added random noise "for ArduPilot"
**NEW**: Clean position data

Artificial noise causes the drone to hunt/jitter trying to follow a noisy target.

### 5. TF Tree Alignment
Static transforms align FAST-LIO's `camera_init` frame with ArduPilot's `odom` frame.

## ArduPilot Parameter Configuration

### Critical Parameters (MUST SET)

```
# Use EKF3 (required for external nav)
AHRS_EKF_TYPE = 3

# Enable EKF3, disable EKF2
EK2_ENABLE = 0
EK3_ENABLE = 1

# Enable visual odometry
VISO_TYPE = 1

# EKF3 Source Selection (Primary = External Navigation)
EK3_SRC1_POSXY = 6    # External Navigation
EK3_SRC1_VELXY = 0    # DISABLED - FAST-LIO velocity from position diff has 500ms lag
EK3_SRC1_POSZ  = 6    # External Navigation
EK3_SRC1_VELZ  = 0    # DISABLED - Use barometer/IMU for Z velocity (more reliable)
EK3_SRC1_YAW   = 6    # External Navigation

# GPS as fallback (Source 2)
EK3_SRC2_POSXY = 3    # GPS
EK3_SRC2_VELXY = 3    # GPS
EK3_SRC2_POSZ  = 3    # GPS
EK3_SRC2_VELZ  = 3    # GPS
EK3_SRC2_YAW   = 3    # GPS with compass
```

### Tuning Parameters

```
# Vision measurement noise (adjust based on your SLAM accuracy)
# Lower = trust vision more, Higher = trust IMU/barometer more
EK3_POSNE_M_NSE = 0.1    # Horizontal position noise (meters)
EK3_ALT_M_NSE   = 0.1    # Altitude noise (meters)
EK3_VELNE_M_NSE = 0.1    # Horizontal velocity noise (m/s)
EK3_VELD_M_NSE  = 0.1    # Vertical velocity noise (m/s)

# Vision delay compensation
VISO_DELAY_MS = 50      # FAST-LIO processing delay (milliseconds)
```

### Velocity Configuration (IMPORTANT)

**RECOMMENDATION**: Keep velocity publishing DISABLED

```python
# In fastlio_mavros_integration.launch:
<param name="publish_velocity" value="false"/>
```

**Why?**
- FAST-LIO outputs at 10 Hz with position-based velocity
- 5-sample moving average creates 500ms lag
- This confuses ArduPilot's EKF and causes drift
- Barometer/IMU provide better Z velocity

If you must enable velocity:
```python
EK3_SRC1_VELXY = 6    # External Navigation (use with caution)
EK3_SRC1_VELZ  = 0    # Keep disabled - barometer is better
```

## Usage

### 1. Terminal 1 - Launch LiDAR Driver
```bash
cd ~/gps/ws_livox
source devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360.launch
```

### 2. Terminal 2 - Launch FAST-LIO
```bash
cd ~/gps/fastlio
source devel/setup.bash
roslaunch fast_lio mapping_mid360.launch
```

### 3. Terminal 3 - Launch MAVROS Bridge
```bash
cd ~/gps
source devel/setup.bash
roslaunch fastlio_mavros_bridge fastlio_mavros_bridge.launch
```

### 4. Verify Data Flow
```bash
# Check FAST-LIO output
rostopic hz /Odometry

# Check MAVROS input
rostopic hz /mavros/vision_pose/pose_cov

# Check FCU reception
rostopic echo /mavros/ekf2/odometry
```

## Troubleshooting

### Drone drifts/wobbles

1. **Check TF alignment**: Run `rosrun tf2_tools view_frames.py` and verify the tree is:
   ```
   map -> odom -> camera_init -> body -> base_link
   ```

2. **Check covariance values**:
   ```bash
   rostopic echo /mavros/vision_pose/pose_cov/pose/covariance[0]
   ```
   Should be small (0.001 - 0.1), not large (>1.0).

3. **Check EKF status**:
   ```bash
   rostopic echo /mavros/ekf2/odometry
   ```
   Should show valid position/velocity data.

### Drone doesn't take off

1. **Check ArduPilot mode**: Must be in GUIDED or LOITER, not STABILIZE.

2. **Check GPS/position lock**: ArduPilot requires position lock before arming.

3. **Check VISO health**:
   ```bash
   rostopic echo /mavros/vision_pose/pose_cov/header/stamp
   ```
   Should be recent (within 0.5 seconds).

### Erratic movement on takeoff

1. **Check frame alignment**: Ensure SLAM started with drone level.

2. **Check initial yaw**: The drone may be interpreting yaw incorrectly.

3. **Reduce EKF noise values**: Increase `EK3_POSNE_M_NSE` if SLAM is noisy.

### Position jumps

If the SLAM resets (position jumps to origin), the safety monitor will detect this and:
- Log an error message
- Set health to False
- You should land and restart SLAM

## Monitoring

### Key Topics

| Topic | Description | Expected Rate |
|-------|-------------|---------------|
| `/Odometry` | FAST-LIO output | 10 Hz |
| `/mavros/vision_pose/pose_cov` | MAVROS input | 10 Hz |
| `/mavros/ekf2/odometry` | EKF output | 10 Hz |
| `/slam_bridge/healthy` | Bridge health | 1 Hz |
| `/slam_safety/status` | Safety status | 10 Hz |

### Diagnostic Commands

```bash
# Full system check
rosrun fastlio_mavros_bridge check_fastlio.py

# Configure ArduPilot (view only)
rosrun fastlio_mavros_bridge configure_ardupilot.py

# Apply configuration (DANGER - modifies FCU params)
rosrun fastlio_mavros_bridge configure_ardupilot.py --apply
```

## Advanced Configuration

### Indoor Flight (No GPS)

```
# Disable GPS checks
GPS_TYPE = 0
AHRS_GPS_USE = 0

# Set optical flow limit (if using)
VISO_HGT_LIM = 10    # Max height for vision-only operation
```

### Outdoor Flight (GPS Backup)

```
# Keep GPS for fallback
GPS_TYPE = 1
AHRS_GPS_USE = 1

# Switch to GPS if vision fails
EK3_SRC_OPTIONS = 1    # Enable fallback on sensor timeout
```

## Hardware-Specific Notes

### Livox MID360

- Default IP: 192.168.1.100
- Connect via Livox Converter
- Publish rate: 10 Hz (point cloud + IMU)

### ArduPilot FCU

- Connect via USB (ttyACM0) or Telemetry
- Baud rate: 921600 (USB) or 57600 (telemetry)
- Requires ArduPilot 4.1+ for external navigation

### Mounting

- **CRITICAL**: Align IMU frame with FCU frame
- LiDAR can be at any angle (handled by FAST-LIO extrinsics)
- Ensure rigid mounting - vibration causes SLAM drift

## Safety Checklist

Before flight:

- [ ] FAST-LIO initialized and mapping
- [ ] /Odometry topic publishing at 10 Hz
- [ ] /mavros/vision_pose/pose_cov publishing
- [ ] EKF3 enabled (AHRS_EKF_TYPE = 3)
- [ ] VISO_TYPE = 1
- [ ] EK3_SRC1_* = 6 (External Navigation)
- [ ] FCU has position lock
- [ ] Safety monitor showing healthy = True
- [ ] RC transmitter has mode switch configured
- [ ] Test in GUIDED mode first

## Support

If issues persist after these fixes:

1. Check `rostopic echo /mavros/vision_pose/pose_cov` has valid data
2. Check ArduPilot logs for EKF errors (`EKF_PRIMARY` messages)
3. Verify FCU firmware supports external navigation (4.1+)
4. Test with drone on ground first - move around and verify position tracks
