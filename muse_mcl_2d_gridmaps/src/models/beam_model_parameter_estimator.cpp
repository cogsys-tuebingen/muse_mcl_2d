#include "beam_model_parameter_estimator.h"

#include <cmath>
#include <assert.h>

namespace muse_mcl_2d_gridmaps {
BeamModelParameterEstimator::BeamModelParameterEstimator(const Parameters &parameters,
                                                         const std::size_t max_iterations) :
    running_(false),
    stop_(false),
    max_iterations_(max_iterations),
    parameters_(parameters),
    parameters_working_copy_(parameters)
{
    ParameterLogger::header_t header = {{"z_hit, z_max, z_short, z_rand, sigma_hit, lambda_short"}};
    logger_.reset(new ParameterLogger(header,  "/tmp/beam_model.csv"));
    logger_->log(parameters_working_copy_.z_hit,
                 parameters_working_copy_.z_short,
                 parameters_working_copy_.z_max,
                 parameters_working_copy_.z_rand,
                 parameters_working_copy_.sigma_hit,
                 parameters_working_copy_.lambda_short);
}

void BeamModelParameterEstimator::setMeasurements(const std::vector<double> &z,
                                                  const std::vector<double> &z_bar,
                                                  const double range_max)
{
    if(running_)
        return;

    std::vector<double> prior(z.size(), 1.0);
    setMeasurements(z, z_bar, prior, range_max);
}

void BeamModelParameterEstimator::setMeasurements(const std::vector<double> &z,
                                                  const std::vector<double> &z_bar,
                                                  const std::vector<double> &prior,
                                                  const double range_max)
{
    if(running_)
        return;

    z_          = z;
    z_bar_      = z_bar;
    prior_      = prior;

    range_max_  = range_max;

    /// kick off the thread here
    if(worker_thread_.joinable())
       worker_thread_.join();
   worker_thread_ = std::thread([this](){run();});

}

void BeamModelParameterEstimator::getParameters(Parameters &parameters) const
{
    parameters = parameters_;
}


void BeamModelParameterEstimator::run()
{
    running_ = true;
    const double p_rand = parameters_working_copy_.z_rand * 1.0 / range_max_;

    /// Mixture distribution
    auto p_hit = [this](const double ray_range, const double map_range) {
        const double dz = ray_range - map_range;
        return parameters_working_copy_.z_hit * std::exp(-dz * dz * parameters_working_copy_.denominator_exponent_hit);
    };
    auto p_short = [this](const double ray_range, const double map_range) {
        if(ray_range < map_range) {
            return parameters_working_copy_.z_short *
                    (1.0 / (1.0 - std::exp(-parameters_working_copy_.lambda_short  * map_range))) *
                    parameters_working_copy_.lambda_short * exp(-parameters_working_copy_.lambda_short * ray_range);
        }
        return 0.0;
    };
    auto p_max = [this](const double ray_range)
    {
        if(ray_range >= range_max_)
            return parameters_working_copy_.z_max * 1.0;
        return 0.0;
    };
    auto p_random = [this, p_rand](const double ray_range)
    {
        if(ray_range < range_max_)
            return p_rand;
        return 0.0;
    };

    auto sq = [] (const double a) {return a * a;};

    running_ = true;
    auto terminate = [this] (std::size_t iteration) {
        return stop_ || (max_iterations_ > 0 && iteration > max_iterations_);
    };

    /// beam model functions

    /// here the estimation goes
    const std::size_t measurements_size = z_.size();
    std::size_t iteration = 0;
    Parameters parameters_previous = parameters_working_copy_;
    while(!terminate(iteration)) {
        double e_hit    = 0.0;
        double e_short  = 0.0;
        double e_rand   = 0.0;
        double e_max    = 0.0;
        double e_sigma  = 0.0;
        double e_lambda = 0.0;
        double norm     = 0.0;
        for(std::size_t i = 0 ; i < measurements_size ; ++i) {
            const double z          = z_[i];
            const double z_bar      = z_bar_[i];
            const double p_hit_i    = p_hit(z, z_bar);
            const double p_short_i  = p_short(z, z_bar);
            const double p_max_i    = p_max(z);
            const double p_rand_i   = p_random(z);
            const double nu = 1.0 / (p_hit_i + p_short_i + p_max_i + p_rand_i) * prior_[i];
            norm     += prior_[i];
            e_hit    += nu * p_hit_i;
            e_short  += nu * p_short_i;
            e_rand   += nu * p_rand_i;
            e_max    += nu * p_max_i;
            e_sigma  += nu * p_hit_i * sq(z - z_bar);
            e_lambda += nu * p_short_i * z;
        }

        norm = 1.0 / norm;
        parameters_working_copy_.z_hit        = std::max(norm * e_hit,   1e-6);
        parameters_working_copy_.z_short      = std::max(norm * e_short, 1e-6);
        parameters_working_copy_.z_max        = std::max(norm * e_max,   1e-6);
        parameters_working_copy_.z_rand       = std::max(norm * e_rand,  1e-6);
        parameters_working_copy_.setSigmaHit(std::sqrt(1.0 / e_hit * e_sigma));
        parameters_working_copy_.lambda_short = e_short / e_lambda;

        if(parameters_previous.compare(parameters_working_copy_))
            break;

        if(!parameters_working_copy_.isNormal())
            break;

        parameters_previous = parameters_working_copy_;

        ++iteration;
    }

    parameters_ = parameters_previous;

    logger_->log(parameters_working_copy_.z_hit,
                 parameters_working_copy_.z_max,
                 parameters_working_copy_.z_short,
                 parameters_working_copy_.z_rand,
                 parameters_working_copy_.sigma_hit,
                 parameters_working_copy_.lambda_short);

    running_ = false;
}
}
