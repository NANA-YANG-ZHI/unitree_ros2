# Contact Estimation Theory (Section III-A)

Source: *"Robust Localization, Mapping, and Navigation for Quadruped Robots"*, Section III-A "Contact estimation".

## 1. Motivation

Low-cost quadruped robots (e.g. Dingo, RealAnt) are often **not equipped with
contact/force sensors**. In the absence of external force sensing, the paper
estimates contact state from a **Generalized Momentum (GM) disturbance
observer**, using only joint torque measurements (no dedicated foot-force
sensors).

## 2. Classic GM disturbance observer

The generalized momentum is defined as

$$p = M(x) v$$

where `M(x)` is the joint-space inertia matrix and `v` is the generalized
velocity. The disturbance observer estimates `p` and the external contact
force `f` jointly:

$$
\begin{bmatrix}\dot{\hat p}\\ \dot{\hat f}\end{bmatrix}
=
\begin{bmatrix}0 & -J^T\\ 0 & 0\end{bmatrix}
\begin{bmatrix}\hat p\\ \hat f\end{bmatrix}
+
\begin{bmatrix}\bar\tau\\ 0\end{bmatrix}
+
\begin{bmatrix}L_1(p-\hat p)\\ L_2(p-\hat p)\end{bmatrix}
$$

with:
- `p̂` the estimated generalized momentum
- `f̂ ∈ R^12` the estimated contact force for the 4 legs (3 axes each)
- `J` the contact Jacobian
- `τ̄ = τ_m + C^T v − g`, where `τ_m` are the motor torques, `C` the Coriolis
  matrix, `g` the gravity vector
- `L1`, `L2` the observer gains

## 3. Restricting to the z-axis + mixed-mode (high-gain / sliding-mode) observer

Since the goal is only to **detect contact**, not reconstruct the full
3D force vector, two simplifications are imposed:

1. Only the **z-axis** (normal) force is estimated: `f̂ = [0 0 f̂_z]`.
2. The estimated force is **clipped to be non-negative** (a foot can only push,
   not pull).

To reduce the phase lag inherent to high-gain observers, the paper blends a
**high-gain** and a **sliding-mode** observer into a single "mixed-mode"
observer:

$$
\begin{bmatrix}\dot{\hat p}\\ \dot{\hat f}\end{bmatrix}
=
\begin{bmatrix}0 & -J^T\\ 0 & 0\end{bmatrix}
\begin{bmatrix}\hat p\\ \hat f\end{bmatrix}
+
\begin{bmatrix}\bar\tau\\ 0\end{bmatrix}
+
\begin{bmatrix}L\,k_1(p-\hat p)\\ L^2\,k_2(p-\hat p)\end{bmatrix}
$$

with the nonlinear injection terms

$$
k_1(s) := q(s), \qquad k_2(s) := \operatorname{sign}(s) + q(s)
$$

$$
q(s) := \operatorname{sign}(s)\,|s|^{1/2} + s
$$

`sign(s) = 1` if `s > 0`, `-1` if `s < 0`, and `L` is the (single) gain
parameter of the mixed observer. This design only requires a global
incremental affine bound on the force-signal nonlinearities, a **strictly
weaker assumption** than that required by pure high-gain observers.

## 4. From estimated force to contact state

To obtain discrete contact states from the (noisy) estimated `f̂_z`:

1. **Low-pass filter** the estimated z-axis force per foot to suppress
   steady-state oscillations.
2. Compare the filtered force against a **foot-specific threshold**: if the
   filtered force exceeds the threshold, the foot is declared "in contact".

---

## 5. How contact is estimated in `contact_detection.py`

The code implements the theory above almost verbatim:

| Theory symbol | Code |
|---|---|
| `p = M(x)v` | `gm_gt = M @ v` in `apply_contact_force_estimation` (`M = pino.crba(...)`) |
| `p̂` | `self.gm_est` / `gm` argument of `dis_ob_tau_zaxis` |
| `J^T` (contact Jacobian, combined feet) | `compute_jacobian_feet_combined_T` (built via Pinocchio frame Jacobians + spatial-force frame transforms `trans_foot_world_aligned_to_trunk*`) |
| `τ̄ = τ_m + C^T v − g` | `C = pino_data.C` (`computeCoriolisMatrix`), `g = pino_data.g` (`computeGeneralizedGravity`), combined in `dgm = (tau + Fxy_gt) + est_F_z + C.T @ v - g + ...` |
| `f̂_z` restricted to z-axis, xy fixed to 0 | `idx = [2, 5, 8, 11]`; `Js_z_T`/`Js_xy_T` split; `f_xy_gt = np.zeros(8)` |
| `err = p - p̂` | `err = gm_measured - gm` |
| `k1(s), k2(s)` (hg / sliding / mixing) | `err_mapping_func(err, alg=...)`, with `q_func` implementing `q(s) = k_sliding·sign(s)\|s\|^{1/2} + k_hg·s` and `alg="mixing"` returning `(q(s), sign(s)+q(s))` exactly matching `k1`, `k2` |
| `L1(p−p̂)`, `L2(p−p̂)` update | `dgm = ... + L1 @ err1`; `dest_Fz = L2 @ err2` |
| Euler integration of the observer | `gm += dgm * dt`; `delta_F_z = dest_Fz * dt` |
| clip force to be non-negative | `est_f_z_new = np.clip(est_f_z_new, [0,0,0,0], [100,100,100,100])` |
| low-pass filter per foot | `apply_contact_detection`: `est_f_filtered[idx] = (1-alpha_foot)*prev + alpha_foot*est_f[idx]` |
| foot-specific threshold → contact state | `contact_state = est_f_filtered[idx] > self.threshold[foot_name]` |

`L1 = diag(2·bandwidth)` and `L2 = diag(bandwidth²)` in `ContactDetector.__init__`
follow the standard critically-damped second-order gain parametrization from a
single `bandwidth` hyperparameter (loaded from `contact_param.yaml`).

The file also contains `apply_contact_detection_gt`, a **ground-truth contact
estimator** (used in simulation, per the paper's IV-A: *"For simulated
environments, we do not use the contact estimation module ... we use the
ground-truth contact instead"*). It thresholds the foot's forward-kinematics
**z-position** (filtered) instead of the estimated force — a foot below a
height threshold is considered in contact.

### Estimation pipeline, step by step (`ContactDetector.apply_contact_detection`)

1. Compute the true generalized momentum `p = M(q)·v` via Pinocchio's
   composite rigid body algorithm (CRBA).
2. Run one Euler step of the mixed-mode GM observer
   (`dis_ob_tau_zaxis`) to update `p̂` and `f̂_z` for all 4 feet.
3. Low-pass filter each foot's estimated `f̂_z`.
4. Threshold the filtered force per foot → boolean contact state.

## 6. Inputs expected

**Per-call inputs to `apply_contact_detection(q, v, tau)` / `dis_ob_tau_zaxis`:**

- `q` — generalized joint positions (Pinocchio configuration vector, includes
  floating base pose + joint angles), typically read from joint encoders (+
  base pose/orientation).
- `v` — generalized velocity (floating base twist + joint velocities), from
  joint encoders (velocity) and IMU (base angular/linear rates).
- `tau` — measured joint/motor torques `τ_m` (from joint encoders' torque
  sensing / motor current estimate). **No dedicated foot-force or contact
  sensor is required.**

**Construction-time inputs to `ContactDetector.__init__`:**

- `pino_model`, `pino_data` — Pinocchio robot model/data built from the
  robot's URDF, used to compute `M`, `C`, `g`, and the foot Jacobians/frame
  placements for feet `fl_foot`, `fr_foot`, `rl_foot`, `rr_foot` and the
  `body` frame.
- `bandwidth`, `nv`, `freq`, `alg` — observer bandwidth, number of velocity
  DoFs, control frequency (→ `dt = 1/freq`), and observer mode
  (`"hg"`, `"sliding"`, or `"mixing"`).
- `contact_param.yaml` (loaded from the `quadstack_contact` package share
  directory) — provides per-foot `alpha` (force low-pass filter coefficient),
  `threshold` (force contact threshold), and, for the ground-truth variant,
  `foot_pos_z_threshold` / `foot_pos_z_alpha`.

No external contact/force sensors are required at any point — only
proprioceptive data (encoders + IMU) and a kinematic/dynamic model of the
robot.
