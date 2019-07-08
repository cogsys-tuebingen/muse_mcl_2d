#pragma once

#include <muse_mcl_2d/scheduling/scheduler_2d.hpp>

#include <cslibs_plugins/plugin.hpp>

#include <unordered_map>

#include <fstream>

namespace muse_mcl_2d {
class RateDropStatistic : public muse_mcl_2d::Scheduler2D
{
public:
    using Ptr                 = std::shared_ptr<RateDropStatistic>;
    using rate_t              = cslibs_time::Rate;
    using update_t            = muse_smc::Update<Sample2D>;
    using time_priority_map_t = std::unordered_map<id_t, double>;
    using resampling_t        = muse_smc::Resampling<Sample2D>;
    using sample_set_t        = muse_smc::SampleSet<Sample2D>;
    using count_map_t         = std::unordered_map<id_t, std::size_t>;
    using name_map_t          = std::unordered_map<id_t, std::string>;
    using time_t              = cslibs_time::Time;
    using duration_t          = cslibs_time::Duration;
    using update_model_map_t  = std::map<std::string, UpdateModel2D::Ptr>;

    RateDropStatistic();
    virtual ~RateDropStatistic();

    inline void print();

    inline void setup(const update_model_map_t &update_models,
                      ros::NodeHandle &nh) override;
    virtual bool apply(typename update_t::Ptr     &u,
                       typename sample_set_t::Ptr &s) override;
    virtual bool apply(typename resampling_t::Ptr &r,
                       typename sample_set_t::Ptr &s) override;

protected:
    time_t              next_update_time_;
    time_t              resampling_time_;
    duration_t          resampling_period_;
    bool                may_resample_;

    /// drop statistic stuff
    std::string         output_path_;
    count_map_t         drops_;
    count_map_t         processed_;
    name_map_t          names_;
};
}
