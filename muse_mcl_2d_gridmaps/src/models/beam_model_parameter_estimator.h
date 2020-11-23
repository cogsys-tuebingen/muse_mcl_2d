#ifndef BEAM_MODEL_PARAMETER_ESTIMATOR_H
#define BEAM_MODEL_PARAMETER_ESTIMATOR_H

#include <atomic>
#include <cmath>
#include <cslibs_utility/logger/csv_logger.hpp>
#include <iostream>
#include <thread>
#include <vector>

namespace muse_mcl_2d_gridmaps {
class BeamModelParameterEstimator {
 public:
  using Ptr = std::shared_ptr<BeamModelParameterEstimator>;
  using ParameterLogger =
      cslibs_utility::logger::CSVLogger<double, double, double, double, double,
                                        double>;

  struct Parameters {
    double z_hit;
    double z_max;
    double z_short;
    double z_rand;
    double sigma_hit;
    double denominator_exponent_hit;
    double denominator_hit;
    double lambda_short;

    bool compare(const Parameters &other, const double epsilon = 1e-3) {
      auto eps = [epsilon](const double a, const double b) {
        return (std::abs(a - b)) >= epsilon;
      };

      return !(eps(z_hit, other.z_hit) || eps(z_max, other.z_max) ||
                   eps(z_short, other.z_short) || eps(z_rand, other.z_rand) ||
                   eps(sigma_hit, other.sigma_hit),
               eps(lambda_short, other.lambda_short));
    }

    bool isNormal() const {
      return std::isnormal(z_hit) && z_hit > 0.0 && std::isnormal(z_max) &&
             z_max > 0.0 && std::isnormal(z_short) && z_short > 0.0 &&
             std::isnormal(z_rand) && z_rand > 0.0 &&
             std::isnormal(sigma_hit) && sigma_hit > 0.0 &&
             std::isnormal(lambda_short) && lambda_short > 0.0;
    }

    void print() const {
      std::cerr << "BeamModelParameters: z_hit - " << z_hit << "\n";
      std::cerr << "BeamModelParameters: z_max - " << z_max << "\n";
      std::cerr << "BeamModelParameters: z_short - " << z_short << "\n";
      std::cerr << "BeamModelParameters: z_rand - " << z_rand << "\n";
      std::cerr << "BeamModelParameters: sigma_hit - " << sigma_hit << "\n";
      std::cerr << "BeamModelParameters: lambda_short - " << lambda_short
                << "\n";
    }

    void setSigmaHit(const double s) {
      sigma_hit = s;
      denominator_exponent_hit = 0.5 / (sigma_hit * sigma_hit);
      denominator_hit = 1.0 / std::sqrt(2.0 * M_PI * sigma_hit * sigma_hit);
    }
  };

  BeamModelParameterEstimator(const Parameters &parameters,
                              const std::size_t max_iterations = 0);

  void setMeasurements(const std::vector<double> &z,
                       const std::vector<double> &z_bar,
                       const double max_range);

  void setMeasurements(const std::vector<double> &z,
                       const std::vector<double> &z_bar,
                       const std::vector<double> &prior,
                       const double max_range);

  void getParameters(Parameters &parameters) const;

 private:
  std::atomic_bool running_;
  std::atomic_bool stop_;
  std::thread worker_thread_;
  std::size_t max_iterations_;

  std::vector<double> z_;
  std::vector<double> z_bar_;
  std::vector<double> prior_;
  double range_max_;

  Parameters parameters_;
  Parameters parameters_working_copy_;

  ParameterLogger::Ptr logger_;

  void run();
};
}  // namespace muse_mcl_2d_gridmaps

#endif  // BEAM_MODEL_PARAMETER_ESTIMATOR_H
