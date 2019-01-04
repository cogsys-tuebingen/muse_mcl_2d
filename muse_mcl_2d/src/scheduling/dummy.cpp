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

    inline void setup(const update_model_map_t &update_models,
                      ros::NodeHandle &nh) override
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};

        for (const auto &um : update_models) {
            const UpdateModel2D::Ptr &u = um.second;
            const std::size_t id = u->getId();

            applied_[id] = false;
        }
        update_all_ = nh.param<bool>(param_name("update_all"), false);
    }

    virtual bool apply(typename update_t::Ptr     &u,
                       typename sample_set_t::Ptr &s) override
    {
        u->apply(s->getWeightIterator());
        applied_[u->getModelId()] = true;

        may_resample_ = true;
        if (update_all_)
            for (auto &entry : applied_)
                may_resample_ &= entry.second;
        return true;
    }

    virtual bool apply(typename resampling_t::Ptr &r,
                       typename sample_set_t::Ptr &s) override
    {
        auto do_apply = [ &r, &s, this] () {
            r->apply(*s);

            may_resample_ = false;
            for (auto &entry : applied_)
                entry.second = false;
            return true;
        };
        return may_resample_ ? do_apply() : false;
    }

protected:
    time_t next_update_time_;
    bool   may_resample_;

    std::map<std::size_t, bool> applied_;
    bool update_all_;
};
}

CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d::Dummy, muse_mcl_2d::Scheduler2D)
