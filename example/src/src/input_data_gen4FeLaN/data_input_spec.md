# Data Input Spec

| Name | Desirable Convention | keys name | Source |
|---|---|---|---|
| **Joints** | | | |
| pos (q) | | q | oK |
| vel (qd) | | qd | OK |
| acc (qdd) ⚠️ | | qdd | FINITE DIFFERENCE + LOW PASS |
| Torque Read | | tau_act | Ok |
| Torque Cmd ⚠️ | | | |
| **Base** | | | |
| linear pos ⚠️ | Base frame | q | `SportModeState.position` (`/sportmodestate`, `/lf/sportmodestate`) — documented Odometry/world frame (`read_motion_state.cpp`); not independently re-verified |
| linear vel ⚠️ | Base frame | qd | `SportModeState.velocity` (`/sportmodestate`, `/lf/sportmodestate`) — **empirically confirmed Body frame already** (`plot/contact_estimation/debug/check_velocity_frame.py`, consistent across 7 bags); contradicts the Odometry/world-frame comment in `read_motion_state.cpp`. No rotation needed to reach the desired Base-frame convention |
| linear acc ⚠️ | Base frame | qdd | FINITE DIFFERENCE + LOW PASS |
| ang pos | Quaternion | q | `imu_state.quaternion` (`/lowstate`, `/sportmodestate`), wxyz order — base orientation in world frame (standard convention) |
| ang vel | Base frame | qd | `imu_state.gyroscope` (`/lowstate`) — Body frame by sensor convention (gyroscope measures angular rate about sensor/body axes) |
| ang acc ⚠️ | Base frame | qdd | FINITE DIFFERENCE + LOW PASS |
| **Forces** | | | |
| Contact Force 🔴 | Base frame | | |
| Contact State | | contact_state | Ok |
| Total Ext Torque (tau fb) ⚠️ | Base frame | | |
| Robot Model | urdf or xml file | | |

⚠️ = orange highlight in source sheet (needs attention / TBD)
🔴 = red highlight in source sheet (blocking / not yet available)

## Sampling

| Field | Value |
|---|---|
| Sample Frequency | 500 Hz |
| Dataset Duration | around 30s per segmentation |

## Extra fields

| Field | Value |
|---|---|
| t | |
| labels | `["run_40", "run_41", "run_51"]` |

## Notes

`q : [base_linear_pos, base_ang_pos, joint_pos]`
`qd : [base_linear_vel, base_ang_vel, joint_vel]`
`qdd : [base_linear_acc, base_ang_acc, joint_acc]`
