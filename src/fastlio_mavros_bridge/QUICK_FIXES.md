# Quick Fixes for Common Issues

## Issues You Reported - Fixes Applied

### 1. TF Check Bug Fixed
**Error**: `'Buffer' object has no attribute '_frame_exists'`

**Fix**: Changed to use `all_frames_as_string()` and `lookup_transform()`

### 2. Added base_link_frd Transform
**Issue**: MAVROS expects `base_link_frd` frame for ArduPilot body frame

**Fix**: Added static transform:
```xml
<node pkg="tf2_ros" type="static_transform_publisher" name="base_link_to_frd"
      args="0 0 0 0 0 0 base_link base_link_frd"/>
```

### 3. Added Debug Logging to Bridge
**Issue**: Bridge says "no pose data received" but data exists

**Fix**: Added startup debug messages to show:
- What topic is being subscribed to
- Whether messages are being received
- Published message count

## Commands to Test

### Step 1: Check What Topics Are Publishing
```bash
# List all odometry topics
rostopic list | grep -i odom

# Check FAST-LIO output rate
rostopic hz /Odometry

# Check MAVROS input
rostopic hz /mavros/vision_pose/pose_cov

# Check if FCU is receiving
rostopic hz /mavros/odometry/in
```

### Step 2: Run Topic Debugger (New Tool)
```bash
rosrun fastlio_mavros_bridge topic_debugger.py
```

This will show:
- Which topics are actually receiving data
- What frame IDs are being used
- If data is flowing from FAST-LIO → MAVROS → FCU

### Step 3: Run Pre-Flight Check
```bash
rosrun fastlio_mavros_bridge preflight_check.py
```

Now fixed to properly check TF transforms.

### Step 4: Run Diagnostic Monitor
```bash
rosrun fastlio_mavros_bridge diagnostic_monitor.py
```

Now fixed to show all TF frames correctly.

## If Bridge Says "No Data Received"

Check these:

1. **Is FAST-LIO publishing to the right topic?**
   ```bash
   rostopic echo /Odometry/header/frame_id
   ```
   Should show something like `camera_init`

2. **Is the bridge subscribed to the right topic?**
   Look in the bridge output - it should say:
   ```
   Subscribed to: /Odometry
   ```

3. **Is there a namespace issue?**
   ```bash
   rostopic list | grep -i odom
   ```
   Check if FAST-LIO is publishing to `/fast_lio/odom` or something else

4. **Case sensitivity**: `/Odometry` vs `/odometry` - these are different!

## If EKF Is Not Using Vision Data

Check ArduPilot parameters in Mission Planner:

```
EK3_SRC1_POSXY = 6    # Must be 6 (External Navigation)
EK3_SRC1_POSZ  = 6    # Must be 6
EK3_SRC1_YAW   = 6    # Must be 6
VISO_TYPE      = 1    # Must be 1 (Visual Odometry)
AHRS_EKF_TYPE  = 3    # Must be 3 (EKF3)
```

Check in MAVLink Inspector:
- Look for `VISION_POSITION_DELTA` or `VISION_POSITION_ESTIMATE` messages
- If not present, EKF is not receiving vision data

## TF Tree Should Look Like This

```
map_ned
└── odom_ned
    └── odom
        └── camera_init
            └── ... (FAST-LIO frames)

base_link
└── base_link_frd
```

## Test Sequence

1. Start everything:
   ```bash
   roslaunch fastlio_mavros_bridge fastlio_mavros_integration.launch
   ```

2. In another terminal, run topic debugger:
   ```bash
   rosrun fastlio_mavros_bridge topic_debugger.py
   ```

3. Watch the output - you should see:
   - ✓ FAST-LIO topic receiving data
   - ✓ MAVROS vision_pose/pose_cov receiving data
   - ✓ MAVROS odometry/in receiving data (EKF fusing)
   - ✓ MAVROS state connected

4. If any are missing, the output will tell you what to fix.

## Common Fixes

### "No FAST-LIO data on any topic!"
```bash
# Check if FAST-LIO is running
rosnode list | grep fast

# Check what topic it's publishing to
rostopic list | grep -i odom

# If it's /fast_lio/odom, edit launch file:
<param name="odom_topic" value="/fast_lio/odom"/>
```

### "No MAVROS vision data"
```bash
# Check bridge is running
rosnode list | grep fastlio

# Check what frame_id it's using
rostopic echo /mavros/vision_pose/pose_cov/header/frame_id

# Should be: map_ned
# If it's odom, edit launch file:
<param name="frame_id" value="map_ned"/>
```

### "No FCU odometry"
```bash
# Check EKF parameters
rosparam get /mavros/vision_pose/frame_id

# Should be: map_ned
# Check ArduPilot params in Mission Planner (see above)
```

## Still Not Working?

Send me the output of:
```bash
# Terminal 1
roslaunch fastlio_mavros_bridge fastlio_mavros_integration.launch

# Terminal 2 (wait 10 seconds, then run)
rosrun fastlio_mavros_bridge topic_debugger.py

# Terminal 3
rostopic echo /mavros/vision_pose/pose_cov/header/frame_id
```

This will show exactly what's happening.
