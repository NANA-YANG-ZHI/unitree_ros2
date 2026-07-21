"""
Finite-difference + low-pass filtering utilities.

Go2 does not report reliable joint/base acceleration directly (LowState's
motor_state[i].ddq is typically unpopulated/noisy on real hardware, and
SportModeState has no acceleration field at all) -- see the qdd rows in
example/data_input_spec.md, all marked "FINITE DIFFERENCE + LOW PASS". This
module derives qdd from qd instead of trusting a sensor field.
"""

import numpy as np
from scipy.signal import butter, filtfilt

# data_input_spec.md's qdd rows call for a cutoff "around 20-25 Hz".
DEFAULT_LOWPASS_FREQ = 22.5


def finite_diff(signal, dt, extend_last=True):
    """First-order forward difference along axis 0.

    signal: (N, D). Returns (N, D) if extend_last (last row repeats the
    final difference so the output stays aligned sample-for-sample with the
    input), else (N-1, D).
    """
    fd_signal = (signal[1:, :] - signal[:-1, :]) / dt
    if extend_last:
        fd_signal = np.append(fd_signal, fd_signal[-1:, :], axis=0)
    return fd_signal


def lowpass_filter(data, cutoff_freq, fs, order=4):
    """Zero-phase Butterworth low-pass filter, applied along axis 0."""
    nyquist = 0.5 * fs
    normal_cutoff = cutoff_freq / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return filtfilt(b, a, data, axis=0)


def compute_qdd(qd, fs_freq, lowpass_freq=DEFAULT_LOWPASS_FREQ):
    """qd (N, D) -> qdd (N, D): finite-difference then low-pass filter.

    Used for all three qdd blocks in data_input_spec.md's Notes
    (base_linear_acc, base_ang_acc, joint_acc) in one call, since they're all
    just derivatives of the corresponding qd columns.
    """
    raw_qdd = finite_diff(qd.copy(), 1.0 / fs_freq)
    return lowpass_filter(raw_qdd, lowpass_freq, fs_freq)
