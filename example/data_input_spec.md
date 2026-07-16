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
| linear pos ⚠️ | Base frame | q | ok |
| linear vel | Base frame | qd | Ok (spotmodestate + rotation) |
| linear acc ⚠️ | Base frame | qdd | FINITE DIFFERENCE + LOW PASS |
| ang pos | Quaternion | q | ok |
| ang vel | Base frame | qd | ok (imu_state/gyroscope) |
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
| Sample Frequency | > 100 Hz |
| Dataset Duration | 10 minutes |

## Extra fields

| Field | Value |
|---|---|
| t | |
| labels | `["run_40", "run_41", "run_51"]` |

## Notes

`q : [base_linear_pos, base_ang_pos, joint_pos]`
`qd : [base_linear_vel, base_ang_vel, joint_vel]`
`qdd : [base_linear_acc, base_ang_acc, joint_acc]`
