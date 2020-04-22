#include <muse_mcl_2d/impl/sampling/normal_sampling_2d.hpp>

#include <class_loader/register_macro.hpp>

namespace muse_mcl_2d {

bool Normal2D::update(const std::string& frame)
{
    maps_.clear();
    const ros::Time now = ros::Time::now();

    cslibs_math_2d::Point2d min(std::numeric_limits<double>::max(),
                std::numeric_limits<double>::max());
    cslibs_math_2d::Point2d max(std::numeric_limits<double>::lowest(),
                std::numeric_limits<double>::lowest());

    for (auto &m : map_providers_) {
        transform_t map_t_w;
        m->waitForStateSpace();
        Map2D::ConstPtr map = m->getStateSpace();
        if (!map)
            throw std::runtime_error("[Normal2D] : map was null!");

        if (tf_->lookupTransform(map->getFrame(), frame, now, map_t_w, tf_timeout_)) {
            maps_.emplace_back(map);
            maps_T_w_.emplace_back(map_t_w);

            transform_t w_T_map = map_t_w.inverse();
            min = cslibs_math::linear::min(w_T_map * map->getMin(), min);
            max = cslibs_math::linear::max(w_T_map * map->getMax(), max);
        }
    }
    return true;
}

bool Normal2D::apply(const transform_t& pose, const covariance_t& covariance, sample_set_t& sample_set)
{
    if (!update(sample_set.getFrame()))
        return false;

    rng_t::Ptr rng(new rng_t(pose.toEigen(), covariance));
    if (random_seed_ >= 0)
        rng.reset(new rng_t(pose.toEigen(), covariance, random_seed_));

    if (sample_size_ < sample_set.getMinimumSampleSize() &&
        sample_size_ > sample_set.getMaximumSampleSize())
        throw std::runtime_error("Initialization sample size invalid!");

    sample_set_t::sample_insertion_t insertion = sample_set.getInsertion();

    const ros::Time sampling_start = ros::Time::now();
    const std::size_t map_count = maps_.size();

    Sample2D sample;
    sample.weight = 1.0 / static_cast<double>(sample_size_);
    for (std::size_t i = 0 ; i < sample_size_ ; ++i) {
        bool valid = false;
        while (!valid) {
            ros::Time now = ros::Time::now();
            if (sampling_start + sampling_timeout_ < now)
                return false;

            sample.state.setFrom(rng->get());
            valid = true;
            for (std::size_t i = 0 ; i < map_count ; ++i)
                valid &= maps_[i]->validate(maps_T_w_[i] * sample.state);
        }
        insertion.insert(sample);
    }

    ++random_seed_;

    return true;
}

void Normal2D::doSetup(const std::map<std::string, MapProvider2D::Ptr>& map_providers, ros::NodeHandle& nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};
    random_seed_ = nh.param(param_name("seed"), -1);

    std::vector<std::string> map_provider_ids;
    nh.getParam(param_name("maps"), map_provider_ids);

    if(map_provider_ids.size() == 0) {
        throw std::runtime_error("[NormalSampling]: No map providers were found!");
    }

    std::string ms ="[";
    for(auto m : map_provider_ids) {

        if(map_providers.find(m) == map_providers.end())
            throw std::runtime_error("[NormalSampling]: Cannot find map provider '" + m + "'!");

        map_providers_.emplace_back(map_providers.at(m));
        ms += m + ",";
    }
    ms.back() = ']';
}
}

CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d::Normal2D, muse_mcl_2d::NormalSampling2D)
