#include <class_loader/class_loader_register_macro.h>

#include <ros/time.h>

#include <muse_mcl_2d/sampling/uniform_2d.hpp>


#include <cslibs_math/sampling/uniform.hpp>
#include <cslibs_math_ros/tf/conversion_2d.hpp>

namespace muse_mcl_2d {
using Metric = cslibs_math::sampling::Metric;
using Radian = cslibs_math::sampling::Radian;
using rng_t  = cslibs_math::sampling::Uniform<Metric, Metric, Radian>;

class UniformPrimaryMap2D : public UniformSampling2D
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<UniformPrimaryMap2D>;

    UniformPrimaryMap2D() = default;

    virtual bool update(const std::string &frame) override
    {
        const ros::Time   now = ros::Time::now();

        primary_map_provider_->waitForStateSpace();
        primary_map_  = primary_map_provider_->getStateSpace();
        if(!primary_map_) {
            throw std::runtime_error("[UniformPrimaryMap2D] : primary map was null!");
            return false;
        }

        tf::Transform tf_w_T_primary;
        if(!tf_->lookupTransform(frame, primary_map_->getFrame(), now, tf_w_T_primary, tf_timeout_)) {
            return false;
        }

        w_T_primary_ = cslibs_math_ros::tf::conversion_2d::from(tf_w_T_primary);

        const std::size_t map_provider_count = map_providers_.size();
        for(std::size_t i = 0 ; i < map_provider_count ; ++i) {
            const MapProvider2D::Ptr m = map_providers_[i];
            m->waitForStateSpace();
            Map2D::ConstPtr map = m->getStateSpace();
            if(!map) {
                throw std::runtime_error("[UniformPrimaryMap2D] : a secondary map was null!");
            }

            tf::Transform tf_secondary_map_T_w;
            if(tf_->lookupTransform(map->getFrame(), frame, now, tf_secondary_map_T_w, tf_timeout_)) {
                secondary_maps_T_w_[i] = cslibs_math_ros::tf::conversion_2d::from(tf_secondary_map_T_w);
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
            rng_.reset(new rng_t({min(0), min(1), -M_PI}, {max(0), max(1), M_PI}, random_seed_));

        return true;
    }

    virtual bool apply(sample_set_t &sample_set) override
    {
        if (sample_size_ < sample_set.getMinimumSampleSize() ||
                sample_size_ > sample_set.getMaximumSampleSize())
            throw std::runtime_error("Initialization sample size invalid!");

        if (!update(sample_set.getFrame()))
            return false;

        sample_set_t::sample_insertion_t insertion = sample_set.getInsertion();
        const std::size_t          secondary_maps_count = secondary_maps_.size();
        const ros::Time sampling_start = ros::Time::now();
        Sample2D sample;
        sample.weight = 1.0 / static_cast<double>(sample_size_);
        for (std::size_t i = 0 ; i < sample_size_; ++i) {
            bool valid = false;
            while (!valid) {
                ros::Time now = ros::Time::now();
                if (sampling_start + sampling_timeout_ < now)
                    return false;

                sample.state.setFrom(rng_->get());
                valid = primary_map_->validate(sample.state);
                if (valid) {
                    auto pose  = w_T_primary_ * sample.state;
                    for (std::size_t i = 0 ; i < secondary_maps_count ; ++i)
                        valid &= secondary_maps_[i]->validate(secondary_maps_T_w_[i] * pose);
                }
            }
            insertion.insert(sample);
        }
        return true;
    }

    virtual void apply(Sample2D &sample) override
    {
        const std::size_t secondary_maps_count = secondary_maps_.size();
        const ros::Time   sampling_start = ros::Time::now();
        bool valid = false;
        while (!valid) {
            ros::Time now = ros::Time::now();
            if (sampling_start + sampling_timeout_ < now)
                return;

            sample.state.setFrom(rng_->get());
            valid = primary_map_->validate(sample.state);
            if (valid) {
                cslibs_math_2d::Transform2d pose  = w_T_primary_ * sample.state;
                for (std::size_t i = 0 ; i < secondary_maps_count ; ++i)
                    valid &= secondary_maps_[i]->validate(secondary_maps_T_w_[i] * pose);
            }
        }
    }

protected:
    std::vector<MapProvider2D::Ptr> map_providers_;
    int                             random_seed_;

    rng_t::Ptr                      rng_;
    cslibs_math_2d::Transform2d     w_T_primary_;
    Map2D::ConstPtr                 primary_map_;
    MapProvider2D::Ptr              primary_map_provider_;
    std::vector<Map2D::ConstPtr>    secondary_maps_;
    std::vector<cslibs_math_2d::Transform2d>  secondary_maps_T_w_;


    virtual void doSetup(const std::map<std::string, MapProvider2D::Ptr> &map_providers,
                         ros::NodeHandle &nh) override
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};
        random_seed_ = nh.param(param_name("seed"), -1);

        std::string primary_map_provider = nh.param(param_name("primary_map"), std::string(""));
        std::vector<std::string> secondary_map_providers;
        nh.getParam(param_name("secondary_maps"), secondary_map_providers);

        if(primary_map_provider == "")
            throw std::runtime_error("[UniformPrimaryMap2D]: Primary map provider must be set!");

        primary_map_provider_ = map_providers.at(primary_map_provider);
        std::string ms ="[";
        for(auto m : secondary_map_providers) {
            map_providers_.emplace_back(map_providers.at(m));
            ms += m + ",";
        }
        ms.back() = ']';

        secondary_maps_T_w_.resize(secondary_map_providers.size());
        secondary_maps_.resize(secondary_map_providers.size());
    }


};
}
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d::UniformPrimaryMap2D, muse_mcl_2d::UniformSampling2D)

