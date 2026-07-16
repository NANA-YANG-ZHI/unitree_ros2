"""
Tutorial: compare the three disturbance-observer injection strategies used in
example/src/src/contact_estimation/contact_detection.py ("hg", "sliding",
"mixing") on a minimal toy system.

This does NOT need ROS2/Pinocchio. It reduces the real generalized-momentum
observer to a scalar 1-DoF version:

    p_true_dot = tau_bar + f_true(t)      ("true" system, tau_bar = 0 here)
    p_hat_dot  = f_hat + L1 * err1        (momentum estimate, observer)
    f_hat_dot  =         L2 * err2        (force estimate, observer)
    err        = p_meas - p_hat

where (err1, err2) are produced by the exact same soft_sign/alpha_func/
q_func/err_mapping_func math as the real contact_detection.py, just copied
here so the script is standalone. f_true(t) is a step disturbance (simulated
foot touchdown at t=1s, liftoff at t=2.5s) -- this plays the role of the
unknown contact force f_ext.

Run:
    python compare_observer_strategies.py

Produces observer_strategies_comparison.png in this directory.
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


# --- exact copies of the injection-shaping functions from contact_detection.py ---

def soft_sign(x, k=100):
    return np.tanh(k * x)


def sign(x, use_soft_sign=True):
    return soft_sign(x) if use_soft_sign else np.sign(x)


def alpha_func(err, alpha, use_soft_sign=True):
    return sign(err, use_soft_sign) * (np.abs(err) ** alpha)


def q_func(err, k_hg, k_sliding, use_soft_sign=True):
    return k_sliding * alpha_func(err, 1 / 2, use_soft_sign) + k_hg * err


def err_mapping_func(err, alg="hg", k_hg=1.0, k_sliding=1.0, use_soft_sign=True):
    if alg == "hg":
        return k_hg * err, k_hg * err
    elif alg == "sliding":
        return alpha_func(k_sliding * err, 1 / 2, use_soft_sign), sign(err, use_soft_sign)
    elif alg == "mixing":
        return (q_func(err, k_hg, k_sliding, use_soft_sign),
                sign(err, use_soft_sign) + q_func(err, k_hg, k_sliding, use_soft_sign))
    else:
        raise NotImplementedError(alg)


# --- toy scalar observer simulation ---

def f_true_profile(t):
    return np.where((t >= 1.0) & (t < 2.5), 15.0, 0.0)


def simulate(alg, bandwidth=6.0, dt=1 / 200, t_end=4.0, noise_std=0.0, seed=0,
             k_hg=1.0, k_sliding=1.0, use_soft_sign=True):
    rng = np.random.default_rng(seed)
    n_steps = int(t_end / dt)
    t = np.arange(n_steps) * dt
    f_true = f_true_profile(t)

    L1 = 2 * bandwidth
    L2 = bandwidth ** 2

    p_true = 0.0
    p_hat = 0.0
    f_hat = 0.0
    f_hat_hist = np.zeros(n_steps)

    for i in range(n_steps):
        p_true += dt * f_true[i]
        p_meas = p_true + (rng.normal(0, noise_std) if noise_std > 0 else 0.0)

        err = p_meas - p_hat
        err1, err2 = err_mapping_func(err, alg=alg, k_hg=k_hg, k_sliding=k_sliding,
                                       use_soft_sign=use_soft_sign)

        p_hat += dt * (f_hat + L1 * err1)
        f_hat += dt * (L2 * err2)

        f_hat_hist[i] = f_hat

    return t, f_true, f_hat_hist


def main():
    algs = ["hg", "sliding", "mixing"]
    colors = {"hg": "tab:blue", "sliding": "tab:red", "mixing": "tab:green"}

    fig, axes = plt.subplots(2, 2, figsize=(13, 8), sharex="col")

    for alg in algs:
        t, f_true, f_hat = simulate(alg, noise_std=0.0)
        axes[0, 0].plot(t, f_hat, color=colors[alg], label=alg)
    axes[0, 0].plot(t, f_true, "k--", label="true", linewidth=1)
    axes[0, 0].set_title("Clean measurement -- full trajectory")
    axes[0, 0].set_ylabel("force estimate")
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)

    for alg in algs:
        t, f_true, f_hat = simulate(alg, noise_std=0.0)
        axes[1, 0].plot(t, f_hat, color=colors[alg], label=alg)
    axes[1, 0].plot(t, f_true, "k--", linewidth=1)
    axes[1, 0].set_xlim(0.95, 1.5)
    axes[1, 0].set_title("Clean measurement -- zoom on touchdown transient")
    axes[1, 0].set_xlabel("time [s]")
    axes[1, 0].set_ylabel("force estimate")
    axes[1, 0].grid(True, alpha=0.3)

    for alg in algs:
        t, f_true, f_hat = simulate(alg, noise_std=0.15)
        axes[0, 1].plot(t, f_hat, color=colors[alg], label=alg, linewidth=0.8)
    axes[0, 1].plot(t, f_true, "k--", label="true", linewidth=1)
    axes[0, 1].set_title("Noisy measurement -- full trajectory")
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)

    for alg in algs:
        t, f_true, f_hat = simulate(alg, noise_std=0.15)
        axes[1, 1].plot(t, f_hat, color=colors[alg], linewidth=0.8)
    axes[1, 1].plot(t, f_true, "k--", linewidth=1)
    axes[1, 1].set_xlim(1.6, 2.4)
    axes[1, 1].set_title("Noisy measurement -- zoom on steady contact (chattering)")
    axes[1, 1].set_xlabel("time [s]")
    axes[1, 1].grid(True, alpha=0.3)

    fig.suptitle("Disturbance-observer strategies: hg (linear) vs sliding vs mixing\n"
                 "toy scalar version of the contact_detection.py momentum observer")
    fig.tight_layout()

    out_path = "observer_strategies_comparison.png"
    fig.savefig(out_path, dpi=150)
    print(f"Saved {out_path}")


if __name__ == "__main__":
    main()
