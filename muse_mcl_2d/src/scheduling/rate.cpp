#include <muse_mcl_2d/impl/scheduling/rate.hpp>

namespace muse_mcl_2d {

Rate::Rate() :
        may_resample_(false)
{
}

void Rate::setup(const Rate::update_model_map_t&, ros::NodeHandle& nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};
    double preferred_rate = nh.param<double>(param_name("preferred_rate"), 5.0);
    resampling_period_ = duration_t(preferred_rate > 0.0 ? 1.0 / preferred_rate : 0.0);
}

bool Rate::apply(update_t::Ptr& u, sample_set_t::Ptr& s)
{
    auto now = []() {
        return time_t(ros::Time::now().toNSec());
    };

    const time_t time_now = now();
    if (next_update_time_.isZero())
        next_update_time_ = time_now;

    const time_t stamp = u->getStamp();
    if (stamp >= next_update_time_) {
        u->apply(s->getWeightIterator());
        next_update_time_ = time_now;

        may_resample_ = true;
        return true;
    }
    return false;
}

bool Rate::apply(resampling_t::Ptr& r, sample_set_t::Ptr& s)
{
    const cslibs_time::Time &stamp = s->getStamp();

    auto now = []() {
        return time_t(ros::Time::now().toNSec());
    };

    const time_t time_now = now();

    if (resampling_time_.isZero())
        resampling_time_ = time_now;

    auto do_apply = [&stamp, &r, &s, &time_now, this] () {
        r->apply(*s);

        resampling_time_ = time_now + resampling_period_;
        may_resample_ = false;
        return true;
    };
    auto do_not_apply = [] () {
        return false;
    };
    return (may_resample_ && resampling_time_ < stamp) ? do_apply() : do_not_apply();
}
}

CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d::Rate, muse_mcl_2d::Scheduler2D)
