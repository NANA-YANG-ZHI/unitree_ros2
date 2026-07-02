#ifndef _EXCITATION_HPP_
#define _EXCITATION_HPP_

#include <vector>
#include <random>
#include "nlohmann/json.hpp"

// C++ port of example/src/src/excitation/excitation_ref.py (JAX reference).
//
// Generates a constrained random Fourier series excitation trajectory:
//     q(t)  =  q0 + sum_k  A_k/(2*wf*k) * sin(2*wf*k*t + phi0)
//                        -  B_k/(2*wf*k) * cos(2*wf*k*t + phi0)
// with wf = 2*pi/duration, subject to q_rand(0) = qd_rand(0) = qdd_rand(0) = 0
// so the trajectory blends smoothly out of / into a hold at q0.
//
// Tunable parameters (see order/njoints/param_range/seed/duration/q0 below):
//   order       - number of Fourier harmonics. Higher = richer/higher-frequency
//                 excitation but more constrained coefficients to solve for
//                 (order >= 2 required by the constraint reduction below).
//   njoints     - number of independent channels excited at once; each gets
//                 its own random A, B, phi0 satisfying the constraint on its own.
//   param_range - per-joint (length njoints) raw amplitude scale applied to the
//                 randomly drawn coefficients *before* the zero-initial-condition
//                 constraint is enforced. Not a hard bound on q(t) -- tune
//                 empirically per axis against hardware limits by checking the plot.
//   seed        - C++ analog of JAX's `key`. Selects which concrete random draw
//                 of A/B/phi0 is used; changing it gives a different trajectory
//                 shape at the same statistical amplitude ("a different tryout").
//   duration    - period of the fundamental frequency (wf = 2*pi/duration);
//                 shorter duration => faster oscillation for the same param_range.
//   q0          - constant offset added on top of the (zero-mean) random Fourier
//                 component; the operating point the excitation oscillates around.
class FourierExcitation
{
public:
    FourierExcitation(int order, int njoints, std::vector<double> param_range);

    // Draws A, B, phi0 satisfying q(0)=qd(0)=qdd(0)=0 for the random component.
    void generate_random_param(unsigned seed);

    void set_duration(double duration) { duration_ = duration; }
    void set_offset(const std::vector<double> &q0) { q0_ = q0; }

    // q(t), size njoints.
    std::vector<double> eval(double t) const;

    // q(t) and qd(t). Note: unlike excitation_ref.py's return_vel branch (which
    // reuses the sin/cos terms left over from the position loop's last harmonic
    // for every k -- an apparent bug), this recomputes sin/cos per-harmonic.
    void eval(double t, std::vector<double> &q, std::vector<double> &qd) const;

    int order() const { return order_; }
    int njoints() const { return njoints_; }
    double duration() const { return duration_; }

    // Dumps order, njoints, param_range, duration, q0, A, B, phi0 so a Python
    // script can reconstruct eval() exactly (see plot/excitation_*.py).
    nlohmann::json to_json() const;

private:
    void generate_constraints();

    int order_;
    int njoints_;
    std::vector<double> param_range_;
    double duration_;
    std::vector<double> q0_;

    std::vector<std::vector<double>> A_;   // order_ x njoints_
    std::vector<std::vector<double>> B_;   // order_ x njoints_
    std::vector<double> phi0_;             // njoints_

    std::vector<double> cA_;               // order_ (all ones)
    std::vector<std::vector<double>> cB_;  // 2 x order_, reduced echelon form
};

#endif
