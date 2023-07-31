
#ifndef ADAPTIVE_LOSS_FUNCTION_H
#define ADAPTIVE_LOSS_FUNCTION_H

#include <ceres/loss_function.h>

namespace ceres
{

class WelschLoss : public ceres::LossFunction
{
public:
  explicit WelschLoss(const double a) : b_(a * a), c_(-1.0 / b_)
  {
  }
  explicit WelschLoss(const double a, const double mu) : b_(mu * a * a), c_(-1.0 / b_)
  {
  }

  void Evaluate(double, double*) const override;

private:
  const double b_;
  const double c_;
};

class BarronLoss : public ceres::LossFunction
{
public:
  explicit BarronLoss(const double a, const double alpha, const double mu) : b_(mu * a * a), c_(1/b_), a_(alpha), factor_(std::abs(a_-2.0)), exponent_(0.5 * a_), pre_factor_(b_ * factor_/a_), times_s_(2 * c_ / factor_) 
  {
  }
public:
  explicit BarronLoss(const double a, const double alpha) : b_(a * a), c_(1/b_), a_(alpha), factor_(std::abs(a_-2.0)), exponent_(0.5 * a_), pre_factor_(b_ * factor_/a_), times_s_(2 * c_ / factor_)
  {
  }

  void Evaluate(double, double*) const override;

private:
  const double a_;
  const double b_;
  const double c_;
  const double factor_;
  const double exponent_;
  const double pre_factor_;
  const double times_s_;
};

}  // namespace ceres

#endif