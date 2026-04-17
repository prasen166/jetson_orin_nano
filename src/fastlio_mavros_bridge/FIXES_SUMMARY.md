# FAST-LIO to MAVROS Fixes Summary

## What Was Fixed

### 1. ENU→NED Conversion (CRITICAL)
**File**: `scripts/fastlio_to_mavros_improved.py`

**Problem**: The original conversion was incomplete - it swapped X/Y but didn't properly handle Z sign and quaternion rotation.

**Fix Applied**:
```python
# Rotation matrix for ENU to NED conversion
R = [[0, 1, 0],     # X_ned = Y_enu (North)
     [1, 0, 0],     # Y_ned = X_enu (East)
     [0, 0, -1]]    # Z_ned = -Z_enu (Down)

# Position: ned_pos = R * enu_pos
# Quaternion: R_ned = R * R_enu * R.T (similarity transform)
# Covariance: C_ned = R * C_enu * R.T
```

**Important**: In NED coordinates:
- Z = 0 is ground level
- Z = -5 means 5 meters above ground (negative!)
- Z should be **negative** when the drone is airborne

### 2. Frame ID Fixed (CRITICAL)
**File**: `launch/fastlio_mavros_integration.launch`

**Problem**: Frame was set to "odom" but ArduPilot requires "map_ned"

**Fix Applied**:
```xml
<param name="frame_id" value="map_ned"/>
```

### 3. Static TF Tree Fixed
**File**: `launch/fastlio_mavros_integration.launch`

**Problem**: Missing transform from map_ned to odom_ned

**Fix Applied**:
```xml
<!-- ArduPilot expects map_ned frame -->
<node pkg="tf2_ros" type="static_transform_publisher" name="map_ned_to_odom_ned"
      args="0 0 0 0 0 0 map_ned odom_ned"/>

<node pkg="tf2_ros" type="static_transform_publisher" name="odom_ned_to_odom"
      args="0 0 0 0 0 0 odom_ned odom"/>
```

### 4. Velocity Disabled by Default
**File**: `launch/fastlio_mavros_integration.launch`

**Problem**: FAST-LIO velocity from position differentiation has 500ms lag at 10Hz

**Fix Applied**:
```xml
<param name="publish_velocity" value="false"/>
```

**ArduPilot Parameters**:
```
EK3_SRC1_VELXY = 0    # Disabled - use 0, not 6
EK3_SRC1_VELZ = 0     # Disabled - barometer is better
```

### 5. Documentation Fixed
**File**: `ARDUPILOT_INTEGRATION.md`

**Problem**: Wrong parameter recommendations

**Fix Applied**:
- Changed `EK3_SRC1_VELXY` from 6 to 0
- Changed `EK3_SRC1_VELZ` from 6 to 0
- Added warnings about velocity issues

## New Diagnostic Tools

### 1. diagnostic_monitor.py
Real-time console output showing:
- FAST-LIO ENU output
- MAVROS NED input
- Conversion errors (expected vs actual)
- FCU output comparison
- TF tree status
- IMU data

**Usage**:
```bash
rosrun fastlio_mavros_bridge diagnostic_monitor.py
```

### 2. csv_logger.py
Logs all data to CSV for post-flight analysis.

**Output**: `~/fastlio_mavros_log_YYYYMMDD_HHMMSS.csv`

**Columns**:
- FAST-LIO ENU position/velocity/quaternion
- MAVROS NED position/quaternion
- Expected NED values
- Conversion errors
- FCU output
- Covariance values

### 3. analyze_log.py
Analyzes CSV log and reports issues:
```bash
python3 analyze_log.py ~/fastlio_mavros_log_*.csv
```

Checks:
- Conversion accuracy
- Z sign correctness
- Frame ID
- FCU tracking
- Covariance values
- FAST-LIO resets

## Quick Test Procedure

### Before Flight - Ground Test

1. **Launch everything**:
   ```bash
   roslaunch fastlio_mavros_bridge fastlio_mavros_integration.launch
   ```

2. **Monitor in real-time**:
   ```bash
   rosrun fastlio_mavros_bridge diagnostic_monitor.py
   ```

3. **Move drone by hand** and check:
   - **North movement**: FAST-LIO Y increases, MAVROS X increases
   - **East movement**: FAST-LIO X increases, MAVROS Y increases
   - **Up movement**: FAST-LIO Z increases, MAVROS Z **decreases** (more negative)
   - **Frame**: Should show "map_ned"
   - **Conversion error**: Should be < 0.01m

4. **Log data**:
   ```bash
   rosrun fastlio_mavros_bridge csv_logger.py
   ```

5. **Analyze after test**:
   ```bash
   python3 analyze_log.py ~/fastlio_mavros_log_*.csv
   ```

## Expected Behavior

### Coordinate Conversion
| Action | FAST-LIO (ENU) | MAVROS (NED) |
|--------|----------------|--------------|
| Move North 1m | X=0, Y=1, Z=0 | X=1, Y=0, Z=0 |
| Move East 1m | X=1, Y=0, Z=0 | X=0, Y=1, Z=0 |
| Move Up 1m | X=0, Y=0, Z=1 | X=0, Y=0, Z=-1 |

### Yaw Convention
- ENU Yaw: 0° = East, 90° = North, 180° = West, 270° = South
- NED Yaw: 0° = North, 90° = East, 180° = South, 270° = West

## If Drone Still Drifts

1. **Check diagnostic_monitor output**:
   - Look for "ERROR" messages
   - Verify Z sign is negative when airborne
   - Verify frame_id is "map_ned"

2. **Check ArduPilot parameters**:
   ```bash
   param show EK3_SRC1_POSXY  # Must be 6
   param show EK3_SRC1_POSZ   # Must be 6
   param show EK3_SRC1_YAW    # Must be 6
   param show EK3_SRC1_VELXY  # Must be 0
   param show EK3_SRC1_VELZ   # Must be 0
   param show VISO_TYPE       # Must be 1
   param show AHRS_EKF_TYPE   # Must be 3
   ```

3. **Send me the CSV log** from a ground test:
   ```bash
   # Find latest log
   ls -la ~/fastlio_mavros_log_*.csv | tail -1
   ```

4. **Try flying in different modes**:
   - STABILIZE: Manual control (no SLAM)
   - LOITER: GPS-based
   - GUIDED: Vision-based
   - GUIDED_NOGPS: Vision-only (no GPS fallback)

## Files Modified/Created

### Modified Files:
1. `scripts/fastlio_to_mavros_improved.py` - Fixed ENU→NED conversion
2. `launch/fastlio_mavros_integration.launch` - Fixed frame_id and TF
3. `ARDUPILOT_INTEGRATION.md` - Fixed parameter documentation

### New Files:
1. `scripts/diagnostic_monitor.py` - Real-time diagnostics
2. `scripts/csv_logger.py` - Data logging
3. `scripts/analyze_log.py` - Log analysis
4. `DEBUGGING_GUIDE.md` - Comprehensive debugging guide
5. `FIXES_SUMMARY.md` - This file

## Critical Checks Before Flight

- [ ] `frame_id` is "map_ned" (not "odom")
- [ ] Z coordinate is negative when drone is above ground
- [ ] Conversion error < 0.01m in diagnostic_monitor
- [ ] TF tree has map_ned → odom_ned → odom → camera_init
- [ ] EK3_SRC1_VELXY = 0 (disabled)
- [ ] EK3_SRC1_VELZ = 0 (disabled)
- [ ] VISO_TYPE = 1
- [ ] AHRS_EKF_TYPE = 3
