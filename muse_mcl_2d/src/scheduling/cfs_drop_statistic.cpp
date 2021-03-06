#include <muse_mcl_2d/impl/scheduling/cfs_drop_statistic.hpp>

namespace muse_mcl_2d {

CFSDropStatistic::CFSDropStatistic() : may_resample_(false) {}

CFSDropStatistic::~CFSDropStatistic() { print(); }

void CFSDropStatistic::print() {
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

void CFSDropStatistic::setup(
    const CFSDropStatistic::update_model_map_t& update_models,
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
    drops_[id] = 0;
    processed_[id] = 0;
    names_[id] = name;
    q_.push(Entry(id));
  }
  double preferred_rate = nh.param<double>(param_name("preferred_rate"), 5.0);
  resampling_period_ =
      duration_t(preferred_rate > 0.0 ? 1.0 / preferred_rate : 0.0);

  output_path_ =
      nh.param<std::string>(param_name("output_path"), "/tmp/drop_statistic");
}

bool CFSDropStatistic::apply(std::shared_ptr<update_t>& u, std::shared_ptr<sample_set_t>& s) {
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
    ++processed_[id];
    may_resample_ = true;
    return true;
  }
  ++drops_[id];
  print();
  return false;
}

bool CFSDropStatistic::apply(std::shared_ptr<resampling_t>& r, std::shared_ptr<sample_set_t>& s) {
  const cslibs_time::Time& stamp = s->getStamp();

  auto now = []() { return time_t(ros::Time::now().toNSec()); };

  const time_t time_now = now();

  if (resampling_time_.isZero()) resampling_time_ = time_now;

  auto do_apply = [&r, &s, &time_now, this]() {
    r->apply(*s);

    resampling_time_ = time_now + resampling_period_;

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

CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d::CFSDropStatistic,
                            muse_mcl_2d::Scheduler2D)
