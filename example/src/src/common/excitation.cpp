#include "excitation.hpp"
#include <cmath>

FourierExcitation::FourierExcitation(int order, int njoints, std::vector<double> param_range)
    : order_(order), njoints_(njoints), param_range_(std::move(param_range)),
      duration_(1.0), q0_(njoints, 0.0)
{
}

void FourierExcitation::generate_constraints()
{
    cA_.assign(order_, 1.0);

    std::vector<double> row0(order_), row1(order_);
    for (int i = 0; i < order_; ++i) {
        row0[i] = 1.0 / static_cast<double>(i + 1);
        row1[i] = static_cast<double>(i + 1);
    }

    std::vector<double> diff(order_);
    for (int i = 0; i < order_; ++i) diff[i] = row0[i] - row1[i];
    const double pivot = diff[1];  // matches excitation_ref.py: (cB[0]-cB[1])[1]

    cB_.assign(2, std::vector<double>(order_, 0.0));
    for (int i = 0; i < order_; ++i) cB_[0][i] = diff[i] / pivot;
    cB_[1] = row1;
}

void FourierExcitation::generate_random_param(unsigned seed)
{
    generate_constraints();

    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> unit(-1.0, 1.0);
    std::uniform_real_distribution<double> phase_dist(0.0, 2.0 * M_PI);

    A_.assign(order_, std::vector<double>(njoints_, 0.0));
    B_.assign(order_, std::vector<double>(njoints_, 0.0));
    phi0_.assign(njoints_, 0.0);

    for (int j = 0; j < njoints_; ++j) {
        const double param_norm = param_range_[j] * 0.5 / static_cast<double>(order_);
        for (int k = 0; k < order_; ++k) {
            A_[k][j] = unit(rng) * param_norm;
            B_[k][j] = unit(rng) * param_norm;
        }
    }

    // Enforce q_rand(0) = qd_rand(0) = qdd_rand(0) = 0, per-joint, independently.
    // Order matters: B[1] is fixed first (from the original, un-modified B[2:]),
    // then B[0] is fixed using the already-updated B[1:] -- matches
    // excitation_ref.py:69-71 exactly.
    for (int j = 0; j < njoints_; ++j) {
        double sumA = 0.0;
        for (int k = 0; k < order_ - 1; ++k) sumA += cA_[k] * A_[k][j];
        A_[order_ - 1][j] = -sumA;

        double sumB1 = 0.0;
        for (int k = 2; k < order_; ++k) sumB1 += cB_[0][k] * B_[k][j];
        B_[1][j] = -sumB1;

        double sumB0 = 0.0;
        for (int k = 1; k < order_; ++k) sumB0 += cB_[1][k] * B_[k][j];
        B_[0][j] = -sumB0;
    }

    for (int j = 0; j < njoints_; ++j) phi0_[j] = phase_dist(rng);
}

std::vector<double> FourierExcitation::eval(double t) const
{
    const double omega_f = 2.0 * M_PI / duration_;
    std::vector<double> q = q0_;

    for (int k = 1; k <= order_; ++k) {
        const double denom = 2.0 * omega_f * k;
        for (int j = 0; j < njoints_; ++j) {
            const double phase = 2.0 * omega_f * k * t + phi0_[j];
            const double s = std::sin(phase);
            const double c = std::cos(phase);
            q[j] += s * A_[k - 1][j] / denom - c * B_[k - 1][j] / denom;
        }
    }
    return q;
}

void FourierExcitation::eval(double t, std::vector<double> &q, std::vector<double> &qd) const
{
    const double omega_f = 2.0 * M_PI / duration_;
    q = q0_;
    qd.assign(njoints_, 0.0);

    for (int k = 1; k <= order_; ++k) {
        const double denom = 2.0 * omega_f * k;
        for (int j = 0; j < njoints_; ++j) {
            const double phase = 2.0 * omega_f * k * t + phi0_[j];
            const double s = std::sin(phase);
            const double c = std::cos(phase);
            q[j]  += s * A_[k - 1][j] / denom - c * B_[k - 1][j] / denom;
            qd[j] += c * A_[k - 1][j] - s * B_[k - 1][j];
        }
    }
}

nlohmann::json FourierExcitation::to_json() const
{
    nlohmann::json j;
    j["order"] = order_;
    j["njoints"] = njoints_;
    j["param_range"] = param_range_;
    j["duration"] = duration_;
    j["q0"] = q0_;
    j["A"] = A_;
    j["B"] = B_;
    j["phi0"] = phi0_;
    return j;
}
