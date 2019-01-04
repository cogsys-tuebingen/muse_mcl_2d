#include <muse_mcl_2d/scheduling/scheduler_2d.hpp>
#include <unordered_map>

namespace muse_mcl_2d {
class Dummy : public muse_mcl_2d::Scheduler2D
{
public:
    using Ptr                 = std::shared_ptr<Dummy>;
    using rate_t              = cslibs_time::Rate;
    using update_t            = muse_smc::Update<StateSpaceDescription2D, cslibs_plugins_data::Data>;
    using resampling_t        = muse_smc::Resampling<StateSpaceDescription2D>;
    using sample_set_t        = muse_smc::SampleSet<StateSpaceDescription2D>;
    using time_t              = cslibs_time::Time;
    using duration_t          = cslibs_time::Duration;
    using update_model_map_t  = std::map<std::string, UpdateModel2D::Ptr>;

    Dummy() :
        may_resample_(false)
    {
    }

    inline void setup(const update_model_map_t &,
                      ros::NodeHandle &nh) override
    {
    }

    virtual bool apply(typename update_t::Ptr     &u,
                       typename sample_set_t::Ptr &s) override
    {
        u->apply(s->getWeightIterator());

        may_resample_ = true;
        return true;
    }

    virtual bool apply(typename resampling_t::Ptr &r,
                       typename sample_set_t::Ptr &s) override
    {
        auto do_apply = [ &r, &s, this] () {
            r->apply(*s);

            may_resample_ = false;
            return true;
        };
        return may_resample_ ? do_apply() : false;
    }

protected:
    time_t next_update_time_;
    bool   may_resample_;
};
}

CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d::Dummy, muse_mcl_2d::Scheduler2D)
