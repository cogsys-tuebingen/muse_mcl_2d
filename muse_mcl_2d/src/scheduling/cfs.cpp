#include <muse_mcl_2d/impl/scheduling/cfs.hpp>

namespace muse_mcl_2d {

CFS::CFS() : may_resample_(false) {}

void CFS::setup(const CFS::update_model_map_t& update_models,
                ros::NodeHandle& nh) {
  auto param_name = [this](const std::string& name) {
    return name_ + "/" + name;
  };

  std::map<std::string, double> nice_values;
  nh.getParam(param_name("nice_values"), nice_values);

  for (const auto& um : update_models) {
    const UpdateModel2D::Ptr& u = um.second;
    const std::size_t id = u->getId();
    const std::string name = u->getName();
    double nice = 1.0;

    if (nice_values.find(u->getName()) == nice_values.end())
      ROS_WARN_STREAM("Did not find nice value for update model '"
                      << u->getName() << "', setting to 1.0.");
    else
      nice = nice_values[name];

    nice_values_[id] = std::min(1.0, std::max(0.0, nice));
    q_.push(Entry(id));
  }
  double preferred_rate = nh.param<double>(param_name("preferred_rate"), 5.0);
  resampling_period_ =
      duration_t(preferred_rate > 0.0 ? 1.0 / preferred_rate : 0.0);
}

bool CFS::apply(std::shared_ptr<update_t>& u, std::shared_ptr<sample_set_t>& s) {
  auto now = []() { return time_t(ros::Time::now().toNSec()); };

  const time_t time_now = now();
  const time_t stamp = u->getStamp();
  if (next_update_time_.isZero()) next_update_time_ = time_now;

  const id_t id = u->getModelId();
  if (id == q_.top().id && stamp >= next_update_time_) {
    Entry entry = q_.top();
    q_.pop();

    const time_t start = now();
    u->apply(s->getWeightIterator());
    const duration_t dur = (now() - start);

    entry.vtime += static_cast<int64_t>(static_cast<double>(dur.nanoseconds()) *
                                        nice_values_[id]);
    next_update_time_ = time_now;

    q_.push(entry);
    may_resample_ = true;
    return true;
  }
  return false;
}

bool CFS::apply(std::shared_ptr<resampling_t>& r, std::shared_ptr<sample_set_t>& s) {
  const cslibs_time::Time& stamp = s->getStamp();

  if (resampling_time_.isZero()) resampling_time_ = stamp;

  auto do_apply = [&stamp, &r, &s, this]() {
    r->apply(*s);

    resampling_time_ = stamp + resampling_period_;

    int64_t min_vtime = q_.top().vtime;
    queue_t q;
    for (auto e : q_) {
      e.vtime -= min_vtime;
      q.push(e);
    }
    std::swap(q, q_);
    may_resample_ = false;
    return true;
  };
  auto do_not_apply = []() { return false; };
  return (may_resample_ && resampling_time_ < stamp) ? do_apply()
                                                     : do_not_apply();
}
}  // namespace muse_mcl_2d

CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d::CFS, muse_mcl_2d::Scheduler2D)
