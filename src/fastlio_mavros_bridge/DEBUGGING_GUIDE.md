# FAST-LIO to MAVROS Debugging Guide

This guide helps you diagnose drift issues when using FAST-LIO with ArduPilot.

## Quick Start - Debugging Drift

When your drone drifts, run these diagnostic tools:

### 1. Real-Time Diagnostics (Terminal 1)
```bash
rosrun fastlio_mavros_bridge diagnostic_monitor.py
```

This prints:
- FAST-LIO ENU output (what SLAM sees)
- MAVROS NED input (what bridge sends)
- Conversion errors (difference between expected and actual)
- FCU output (what ArduPilot thinks)
- TF tree status
- IMU data

### 2. CSV Logging (Terminal 2)
```bash
rosrun fastlio_mavros_bridge csv_logger.py
```

Logs to `~/fastlio_mavros_log_YYYYMMDD_HHMMSS.csv` for analysis.

## Understanding the Data Flow

```
FAST-LIO (ENU)  →  Bridge (convert to NED)  →  MAVROS  →  ArduPilot EKF
    ↓                    ↓                        ↓              ↓
[Odometry]        [vision_pose/pose_cov]    [frame check]   [motor output]
   X=E                    X=N                    map_ned      Drift!
   Y=N                    Y=E
   Z=U                    Z=D (negative!)
```

**Critical**: In NED, Z should be **NEGATIVE** when the drone is above ground!

## Common Drift Causes & Solutions

### 1. Z-Coordinate Sign Error (Most Common)

**Symptom**: Drone thinks it's underground or keeps climbing

**Check in diagnostic_monitor**:
```
[1] FAST-LIO OUTPUT: Z=+2.5 (drone 2.5m above ground)
[2] MAVROS INPUT:    Z=-2.5 (CORRECT - negative in NED)
```

If MAVROS Z is **positive** when drone is airborne → **BUG!**

**Fix**: Check `enu_to_ned()` function has `ned_z = -enu_z`

### 2. Frame ID Mismatch

**Symptom**: ArduPilot ignores vision data completely

**Check in diagnostic_monitor**:
```
[2] MAVROS INPUT: Frame: odom  ← WRONG!
[3] ERROR: Frame is 'odom' but should be 'map_ned'!
```

**Fix**: In launch file, ensure:
```xml
<param name="frame_id" value="map_ned"/>
```

### 3. ENU→NED Conversion Error

**Symptom**: Drone moves in wrong direction (East instead of North)

**Check in diagnostic_monitor**:
```
[3] CONVERSION CHECK:
    Expected:  X=5.0  Y=3.0  Z=-2.0
    Actual:    X=3.0  Y=5.0  Z=-2.0  ← X and Y swapped!
```

**Fix**: Check rotation matrix in `enu_to_ned()`:
```python
R = [[0, 1, 0],   # X_ned = Y_enu
     [1, 0, 0],   # Y_ned = X_enu
     [0, 0, -1]]  # Z_ned = -Z_enu
```

### 4. FCU Not Using Vision Data

**Symptom**: No drift initially, then sudden jumps

**Check in diagnostic_monitor**:
```
[4] FCU OUTPUT: Position: X=0.1 Y=0.1 Z=0.0
    vs MAVROS:  dX=5.2 dY=3.1 dZ=-2.0 (error=6.1)
    WARNING: FCU position diverged from MAVROS input!
```

**Fix**: Check ArduPilot parameters:
```bash
EK3_SRC1_POSXY = 6    # External Navigation
EK3_SRC1_POSZ  = 6    # External Navigation
EK3_SRC1_YAW   = 6    # External Navigation
EK3_SRC1_VELXY = 0    # DISABLED (use 0 not 6)
EK3_SRC1_VELZ  = 0    # DISABLED
VISO_TYPE = 1         # Visual odometry enabled
```

### 5. FAST-LIO Reset/Jump

**Symptom**: Position suddenly jumps to origin

**Check in diagnostic_monitor**:
```
[1] FAST-LIO OUTPUT: Position: X=0.0 Y=0.0 Z=0.0
    ISSUES: Position at origin - possible SLAM reset
```

**Fix**: Land immediately, restart FAST-LIO

### 6. TF Tree Broken

**Check in diagnostic_monitor**:
```
[5] TF TREE CHECK:
    map_ned: MISSING
    odom_ned: MISSING
```

**Fix**: Check static transforms in launch file

## How to Use CSV Log for Analysis

After flight, open `~/fastlio_mavros_log_*.csv` in Excel/Python:

### Check 1: Conversion Accuracy
Plot `mv_ned_x` vs `exp_ned_x` - they should be identical
If different, conversion is wrong

### Check 2: Z Sign
Plot `fl_enu_z` and `mv_ned_z` together
- When drone climbs (ENU Z increases), NED Z should decrease (more negative)
- If both go up together → BUG!

### Check 3: FCU Tracking
Plot `mv_ned_x` and `fcu_x` together
If they diverge, EKF isn't fusing vision properly

### Check 4: Error Over Time
Plot `err_pos` column
Should stay near 0. If it grows, something is wrong

## Parameter Checklist

Before flight, verify:

```bash
# Check these with MAVProxy or QGroundControl

# EKF Source
param show EK3_SRC1_POSXY  # Should be 6
param show EK3_SRC1_POSZ   # Should be 6
param show EK3_SRC1_YAW    # Should be 6
param show EK3_SRC1_VELXY  # Should be 0
param show EK3_SRC1_VELZ   # Should be 0

# Vision settings
param show VISO_TYPE       # Should be 1
param show AHRS_EKF_TYPE   # Should be 3

# Noise settings (adjust based on your SLAM quality)
param show EK3_POSNE_M_NSE  # Start with 0.1
param show EK3_ALT_M_NSE    # Start with 0.1
```

## Testing Procedure

1. **Ground Test** (props off!):
   ```bash
   # Terminal 1: Launch everything
   roslaunch fastlio_mavros_bridge fastlio_mavros_integration.launch

   # Terminal 2: Monitor
   rosrun fastlio_mavros_bridge diagnostic_monitor.py

   # Terminal 3: Log
   rosrun fastlio_mavros_bridge csv_logger.py
   ```

2. **Move drone around by hand** while watching diagnostic output
   - X should increase when moving North
   - Y should increase when moving East
   - Z should decrease (more negative) when lifting up

3. **Check FCU tracking**:
   - MAVROS position and FCU position should match
   - If FCU lags behind or doesn't follow, check EKF parameters

4. **Only then** attempt flight

## Emergency: Drone is Drifting!

1. **Switch to STABILIZE mode immediately** - takes control from EKF
2. **Land manually**
3. **Check logs**:
   ```bash
   # Find latest log
   ls -la ~/fastlio_mavros_log_*.csv | tail -1

   # Analyze
   python3 analyze_log.py ~/fastlio_mavros_log_*.csv
   ```

## Quick Diagnostic Commands

```bash
# Check topic rates
rostopic hz /Odometry
rostopic hz /mavros/vision_pose/pose_cov

# Check frame_id
rostopic echo /mavros/vision_pose/pose_cov/header/frame_id

# Check covariance (should be small)
rostopic echo /mavros/vision_pose/pose_cov/pose/covariance[0]

# Check TF tree
rosrun tf2_tools view_frames.py

# Check EKF status
rostopic echo /mavros/ekf2/odometry

# Monitor FCU position vs vision
rostopic echo /mavros/global_position/local
```

## Still Drifting?

If you've checked everything and it still drifts:

1. **Send me the CSV log** from a ground test
2. **Screenshot the diagnostic_monitor output**
3. **Check if drift happens in LOITER mode** (GPS) vs GUIDED mode (vision only)
4. **Try flying in GUIDED_NOGPS** to isolate if it's vision or GPS interference

Most drift issues are:
- 40% Z sign error (NED vs ENU confusion)
- 30% Frame ID wrong (odom vs map_ned)
- 20% EKF not using vision (wrong parameters)
- 10% Actual SLAM drift (FAST-LIO losing track)
