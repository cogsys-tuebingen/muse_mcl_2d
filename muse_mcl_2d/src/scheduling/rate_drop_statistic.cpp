#include <muse_mcl_2d/impl/scheduling/rate_drop_statistic.hpp>

namespace muse_mcl_2d {

RateDropStatistic::RateDropStatistic() : may_resample_(false) {}

RateDropStatistic::~RateDropStatistic() { print(); }

void RateDropStatistic::print() {
  std::ofstream out(output_path_);
  for (auto& name : names_) {
    out << name.second << ": \n";
    out << "\t dropped   : " << drops_[name.first] << "\n";
    out << "\t processed : " << processed_[name.first] << "\n";
    out << "\t all-in-all: " << drops_[name.first] + processed_[name.first]
        << "\n";
    out << "\t drop rate : "
        << static_cast<double>(100 * drops_[name.first]) /
               static_cast<double>(drops_[name.first] + processed_[name.first])
        << "\n";
    out << "\n";
  }
  out.flush();
  out.close();
}

void RateDropStatistic::setup(const update_model_map_t& update_models,
                              ros::NodeHandle& nh) {
  auto param_name = [this](const std::string& name) {
    return name_ + "/" + name;
  };
  double preferred_rate = nh.param<double>(param_name("preferred_rate"), 5.0);
  resampling_period_ =
      duration_t(preferred_rate > 0.0 ? 1.0 / preferred_rate : 0.0);

  output_path_ =
      nh.param<std::string>(param_name("output_path"), "/tmp/drop_statistic");

  for (const auto& um : update_models) {
    const UpdateModel2D::Ptr& u = um.second;
    const std::size_t id = u->getId();
    const std::string name = u->getName();
    drops_[id] = 0;
    processed_[id] = 0;
    names_[id] = name;
  }
}

bool RateDropStatistic::apply(update_t::Ptr& u, sample_set_t::Ptr& s) {
  auto now = []() { return time_t(ros::Time::now().toNSec()); };

  const time_t time_now = now();
  if (next_update_time_.isZero()) next_update_time_ = time_now;

  const time_t stamp = u->getStamp();
  if (stamp >= next_update_time_) {
    // const time_t start = now();
    u->apply(s->getWeightIterator());
    // const duration_t dur = (now() - start);

    next_update_time_ = time_now;

    ++processed_[u->getModelId()];
    may_resample_ = true;
    return true;
  }

  ++drops_[u->getModelId()];
  print();
  return false;
}

bool RateDropStatistic::apply(resampling_t::Ptr& r, sample_set_t::Ptr& s) {
  const cslibs_time::Time& stamp = s->getStamp();

  auto now = []() { return time_t(ros::Time::now().toNSec()); };

  const time_t time_now = now();

  if (resampling_time_.isZero()) resampling_time_ = time_now;

  auto do_apply = [&stamp, &r, &s, &time_now, this]() {
    r->apply(*s);

    resampling_time_ = time_now + resampling_period_;
    may_resample_ = false;
    return true;
  };
  auto do_not_apply = []() { return false; };
  return (may_resample_ && resampling_time_ < stamp) ? do_apply()
                                                     : do_not_apply();
}
}  // namespace muse_mcl_2d

CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d::RateDropStatistic,
                            muse_mcl_2d::Scheduler2D)
