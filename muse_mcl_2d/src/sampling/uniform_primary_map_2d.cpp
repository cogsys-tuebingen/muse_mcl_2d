#include <class_loader/register_macro.hpp>
#include <muse_mcl_2d/impl/sampling/uniform_primary_map_2d.hpp>

namespace muse_mcl_2d {

bool UniformPrimaryMap2D::update(const std::string& frame) {
  const ros::Time now = ros::Time::now();

  primary_map_provider_->waitForStateSpace();
  primary_map_ = primary_map_provider_->getStateSpace();
  if (!primary_map_) {
    throw std::runtime_error("[UniformPrimaryMap2D] : primary map was null!");
    return false;
  }

  if (!tf_->lookupTransform(frame, primary_map_->getFrame(), now, w_T_primary_,
                            tf_timeout_)) {
    return false;
  }

  const std::size_t map_provider_count = map_providers_.size();
  for (std::size_t i = 0; i < map_provider_count; ++i) {
    const MapProvider2D::Ptr m = map_providers_[i];
    m->waitForStateSpace();
    const auto map = m->getStateSpace();
    if (!map) {
      throw std::runtime_error(
          "[UniformPrimaryMap2D] : a secondary map was null!");
    }

    transform_t secondary_map_T_w;
    if (tf_->lookupTransform(map->getFrame(), frame, now, secondary_map_T_w,
                             tf_timeout_)) {
      secondary_maps_T_w_[i] = secondary_map_T_w;
      secondary_maps_[i] = map;
    } else {
      return false;
    }
  }
  /// particles are generated in the primary map frame, since formulation has
  /// to be axis-aligned, relative to the map origin
  /// but internal frames are already within calculation

  cslibs_math_2d::Point2d min = primary_map_->getMin();
  cslibs_math_2d::Point2d max = primary_map_->getMax();
  rng_.reset(new rng_t({min(0), min(1), -M_PI}, {max(0), max(1), M_PI}));
  if (random_seed_ >= 0)
    rng_.reset(new rng_t({min(0), min(1), -M_PI}, {max(0), max(1), M_PI},
                         random_seed_));

  return true;
}

bool UniformPrimaryMap2D::apply(sample_set_t& sample_set) {
  if (sample_size_ < sample_set.getMinimumSampleSize() ||
      sample_size_ > sample_set.getMaximumSampleSize())
    throw std::runtime_error("Initialization sample size invalid!");

  if (!update(sample_set.getFrame())) return false;

  sample_set_t::sample_insertion_t insertion = sample_set.getInsertion();
  const std::size_t secondary_maps_count = secondary_maps_.size();
  const ros::Time sampling_start = ros::Time::now();
  Sample2D sample;
  sample.weight() = 1.0 / static_cast<double>(sample_size_);
  for (std::size_t i = 0; i < sample_size_; ++i) {
    bool valid = false;
    while (!valid) {
      ros::Time now = ros::Time::now();
      if (sampling_start + sampling_timeout_ < now) return false;

      sample.state().setFrom(rng_->get());
      valid = primary_map_->validate(sample.state());
      if (valid) {
        auto pose = w_T_primary_ * sample.state();
        for (std::size_t i = 0; i < secondary_maps_count; ++i)
          valid &= secondary_maps_[i]->validate(secondary_maps_T_w_[i] * pose);
      }
    }
    insertion.insert(sample);
  }
  return true;
}

void UniformPrimaryMap2D::apply(Sample2D& sample) {
  const std::size_t secondary_maps_count = secondary_maps_.size();
  const ros::Time sampling_start = ros::Time::now();
  bool valid = false;
  while (!valid) {
    ros::Time now = ros::Time::now();
    if (sampling_start + sampling_timeout_ < now) return;

    sample.state().setFrom(rng_->get());
    valid = primary_map_->validate(sample.state());
    if (valid) {
      transform_t pose = w_T_primary_ * sample.state();
      for (std::size_t i = 0; i < secondary_maps_count; ++i)
        valid &= secondary_maps_[i]->validate(secondary_maps_T_w_[i] * pose);
    }
  }
}

void UniformPrimaryMap2D::doSetup(
    const std::map<std::string, MapProvider2D::Ptr>& map_providers,
    ros::NodeHandle& nh) {
  auto param_name = [this](const std::string& name) {
    return name_ + "/" + name;
  };
  random_seed_ = nh.param(param_name("seed"), -1);

  std::string primary_map_provider =
      nh.param(param_name("primary_map"), std::string(""));
  std::vector<std::string> secondary_map_providers;
  nh.getParam(param_name("secondary_maps"), secondary_map_providers);

  if (primary_map_provider == "")
    throw std::runtime_error(
        "[UniformPrimaryMap2D]: Primary map provider must be set!");

  primary_map_provider_ = map_providers.at(primary_map_provider);
  std::string ms = "[";
  for (auto m : secondary_map_providers) {
    map_providers_.emplace_back(map_providers.at(m));
    ms += m + ",";
  }
  ms.back() = ']';

  secondary_maps_T_w_.resize(secondary_map_providers.size());
  secondary_maps_.resize(secondary_map_providers.size());
}
}  // namespace muse_mcl_2d
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d::UniformPrimaryMap2D,
                            muse_mcl_2d::UniformSampling2D)
