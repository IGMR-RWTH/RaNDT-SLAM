#include "ndt_registration/ceres_loss_functions.h"

#include <cmath>
#include <limits>


namespace ceres
{

void WelschLoss::Evaluate(double s, double rho[3]) const
{
  const double exp = std::exp(s * c_);

  rho[0] = b_ * (1 - exp);
  rho[1] = exp;
  rho[2] = c_ * exp;
}

void BarronLoss::Evaluate(double s, double rho[3]) const
{
  if (a_ >= 2.0) {
    rho[0] = s;
    rho[1] = 1;
    rho[2] = 0;
  }
  else if (std::abs(a_) <= 0.05) {
    const double sum = 1.0 + s * c_;
    const double inv = 1.0 / sum;
    rho[0] = b_ * std::log(sum);
    rho[1] = std::max(std::numeric_limits<double>::min(), inv);
    rho[2] = -c_ * (inv * inv);
  }
  else {
    const double to_exp = s * times_s_ + 1.0;
    rho[0] = pre_factor_ * (std::pow(to_exp, exponent_) - 1.);
    rho[1] = pre_factor_ * exponent_ * std::pow(to_exp, exponent_-1.) * times_s_;
    rho[2] = pre_factor_ * exponent_ * (exponent_-1) * std::pow(to_exp, exponent_-2.) * times_s_ * times_s_;
  }
}

}  // namespace ceres