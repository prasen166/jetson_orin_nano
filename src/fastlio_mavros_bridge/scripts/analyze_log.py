#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Analyze FAST-LIO to MAVROS CSV log file
Usage: python3 analyze_log.py ~/fastlio_mavros_log_*.csv
"""

import sys
import pandas as pd
import numpy as np

def analyze_log(filename):
    print("="*70)
    print("FAST-LIO to MAVROS Log Analysis")
    print("="*70)
    print("\nFile: %s\n" % filename)

    df = pd.read_csv(filename)

    # 1. Check conversion accuracy
    print("[1] CONVERSION ACCURACY:")
    pos_errors = df['err_pos']
    print("    Mean position error: %.4f m" % pos_errors.mean())
    print("    Max position error:  %.4f m" % pos_errors.max())

    if pos_errors.max() > 0.01:
        print("    ⚠ WARNING: Conversion errors detected!")
        print("       Check enu_to_ned() function")
    else:
        print("    ✓ Conversion is accurate")

    # 2. Check Z sign
    print("\n[2] Z COORDINATE SIGN CHECK:")
    fl_z_mean = df['fl_enu_z'].mean()
    mv_z_mean = df['mv_ned_z'].mean()

    print("    FAST-LIO Z (ENU):  avg = %.2f" % fl_z_mean)
    print("    MAVROS Z (NED):    avg = %.2f" % mv_z_mean)

    if fl_z_mean > 0 and mv_z_mean > 0:
        print("    ✗ CRITICAL: Z sign is wrong! Both are positive!")
        print("       NED Z should be negative when drone is above ground")
        print("       Fix: ned_z = -enu_z in enu_to_ned()")
    elif fl_z_mean > 0 and mv_z_mean < 0:
        print("    ✓ Z sign is correct (ENU up+, NED down-)")
    else:
        print("    ? Unexpected Z values - verify drone was airborne")

    # 3. Check frame_id
    print("\n[3] FRAME ID CHECK:")
    frames = df['frame_id'].unique()
    print("    Frames used: %s" % frames)

    if 'map_ned' not in frames:
        print("    ✗ CRITICAL: frame_id is not 'map_ned'!")
        print("       ArduPilot expects map_ned frame")
    else:
        print("    ✓ Frame is map_ned")

    # 4. Check FCU tracking
    print("\n[4] FCU TRACKING:")
    if df['fcu_x'].abs().max() > 0.01:
        fcu_errors = np.sqrt(
            (df['mv_ned_x'] - df['fcu_x'])**2 +
            (df['mv_ned_y'] - df['fcu_y'])**2 +
            (df['mv_ned_z'] - df['fcu_z'])**2
        )
        print("    Mean FCU error: %.3f m" % fcu_errors.mean())
        print("    Max FCU error:  %.3f m" % fcu_errors.max())

        if fcu_errors.max() > 1.0:
            print("    ⚠ WARNING: FCU position diverges from vision!")
            print("       Check EKF parameters:")
            print("       - EK3_SRC1_POSXY = 6")
            print("       - EK3_SRC1_POSZ = 6")
            print("       - VISO_TYPE = 1")
        else:
            print("    ✓ FCU is tracking vision correctly")
    else:
        print("    ⚠ No FCU data available")

    # 5. Check covariance
    print("\n[5] COVARIANCE CHECK:")
    cov_xx = df['cov_xx'].mean()
    cov_yy = df['cov_yy'].mean()
    cov_zz = df['cov_zz'].mean()
    print("    Position covariance: XX=%.4f YY=%.4f ZZ=%.4f" % (cov_xx, cov_yy, cov_zz))

    if cov_xx > 1.0 or cov_yy > 1.0 or cov_zz > 1.0:
        print("    ⚠ WARNING: High covariance - EKF will trust vision less")
    else:
        print("    ✓ Covariance values are reasonable")

    # 6. Check for FAST-LIO resets
    print("\n[6] FAST-LIO RESETS:")
    jumps = 0
    for i in range(1, len(df)):
        dist = np.sqrt(
            (df['fl_enu_x'].iloc[i] - df['fl_enu_x'].iloc[i-1])**2 +
            (df['fl_enu_y'].iloc[i] - df['fl_enu_y'].iloc[i-1])**2 +
            (df['fl_enu_z'].iloc[i] - df['fl_enu_z'].iloc[i-1])**2
        )
        if dist > 5.0:  # 5m jump
            jumps += 1

    if jumps > 0:
        print("    ⚠ WARNING: %d position jumps detected (possible SLAM resets)" % jumps)
    else:
        print("    ✓ No SLAM resets detected")

    # 7. Summary
    print("\n" + "="*70)
    print("SUMMARY:")

    issues = []
    if pos_errors.max() > 0.01:
        issues.append("Conversion errors")
    if fl_z_mean > 0 and mv_z_mean > 0:
        issues.append("Z sign wrong")
    if 'map_ned' not in frames:
        issues.append("Frame ID wrong")

    if issues:
        print("Issues found: %s" % ", ".join(issues))
        print("\nRECOMMENDATIONS:")
        if "Z sign wrong" in issues:
            print("- Fix Z coordinate: ned_z = -enu_z")
        if "Frame ID wrong" in issues:
            print("- Set frame_id='map_ned' in launch file")
        if "Conversion errors" in issues:
            print("- Check enu_to_ned() rotation matrix")
    else:
        print("✓ All checks passed! Data flow looks correct.")
        print("\nIf still drifting, check:")
        print("- ArduPilot EKF parameters (EK3_SRC1_*)")
        print("- Mechanical vibration affecting SLAM")
        print("- Initial alignment (was drone level at startup?)")

    print("="*70)

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 analyze_log.py <csv_file>")
        sys.exit(1)

    analyze_log(sys.argv[1])
