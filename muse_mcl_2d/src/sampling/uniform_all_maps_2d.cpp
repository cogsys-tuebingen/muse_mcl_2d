#include <class_loader/class_loader_register_macro.h>

#include <ros/time.h>

#include <cslibs_math/sampling/uniform.hpp>

#include <muse_mcl_2d/sampling/uniform_2d.hpp>
#include <cslibs_math_ros/tf/conversion_2d.hpp>

namespace muse_mcl_2d {
using Metric = cslibs_math::sampling::Metric;
using Radian = cslibs_math::sampling::Radian;
using rng_t  = cslibs_math::sampling::Uniform<Metric, Metric, Radian>;

class EIGEN_ALIGN16 UniformAllMaps2D : public UniformSampling2D
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<UniformAllMaps2D>;

    virtual bool update(const std::string &frame) override
    {
        const ros::Time now = ros::Time::now();

        cslibs_math_2d::Point2d min(std::numeric_limits<double>::max(),
                          std::numeric_limits<double>::max());
        cslibs_math_2d::Point2d max(std::numeric_limits<double>::lowest(),
                          std::numeric_limits<double>::lowest());

        const std::size_t map_provider_count = map_providers_.size();
        maps_T_w_.resize(map_provider_count);
        maps_.resize(map_provider_count);

        for(std::size_t i = 0 ; i < map_provider_count ; ++i) {
            const MapProvider2D::Ptr m = map_providers_[i];

            m->waitForStateSpace();
            Map2D::ConstPtr map = m->getStateSpace();
            if(!map)
                throw std::runtime_error("[UniformAllMaps2D] : map was null!");

            tf::Transform tf_map_T_w;
            if (tf_->lookupTransform(map->getFrame(), frame, now, tf_map_T_w, tf_timeout_)) {
                cslibs_math_2d::Transform2d map_T_w =  cslibs_math_ros::tf::conversion_2d::from(tf_map_T_w);
                maps_[i] = map;
                maps_T_w_[i] = map_T_w;

                cslibs_math_2d::Transform2d w_T_map = map_T_w.inverse();
                min = cslibs_math::linear::min(w_T_map * map->getMin(), min);
                max = cslibs_math::linear::max(w_T_map * map->getMax(), max);
            } else {
                std::cout << "[UniformAllMaps2D] : tf not found!" << std::endl;
                return false;
            }
        }

        rng_.reset(new rng_t({min(0), min(1), -M_PI}, {max(0), max(1), M_PI}));
        if (random_seed_ >= 0)
            rng_.reset(new rng_t({min(0), min(1), -M_PI}, {max(0), max(1), M_PI}, 0));

        return true;
    }

    virtual bool apply(sample_set_t &particle_set) override
    {
        if (sample_size_ < particle_set.getMinimumSampleSize() ||
                sample_size_ > particle_set.getMaximumSampleSize())
            throw std::runtime_error("Initialization sample size invalid!");

        if (!update(particle_set.getFrame()))
            return false;

        sample_set_t::sample_insertion_t insertion = particle_set.getInsertion();
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

                sample.state.setFrom(rng_->get());
                valid = true;
                for (std::size_t i = 0 ; i < map_count ; ++i)
                    valid &= maps_[i]->validate(maps_T_w_[i] * sample.state);
            }
            insertion.insert(sample);
        }
        return true;
    }

    virtual void apply(Sample2D &sample) override
    {
        if (!rng_)
            return;

        const ros::Time sampling_start = ros::Time::now();
        const std::size_t map_count = maps_.size();
        bool valid = false;
        while (!valid) {
            ros::Time now = ros::Time::now();
            if (sampling_start + sampling_timeout_ < now)
                break;

            sample.state.setFrom(rng_->get());
            valid = true;
            for (std::size_t i = 0 ; i < map_count ; ++i)
                valid &= maps_[i]->validate(maps_T_w_[i] * sample.state);
        }
    }

protected:
    int random_seed_;
    std::vector<Map2D::ConstPtr>                                                       maps_;
    std::vector<cslibs_math_2d::Transform2d, cslibs_math_2d::Transform2d::allocator_t> maps_T_w_;
    std::vector<MapProvider2D::Ptr>                                                    map_providers_;
    rng_t::Ptr                                                                         rng_;

    virtual void doSetup(const std::map<std::string, MapProvider2D::Ptr> &map_providers,
                         ros::NodeHandle &nh) override
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};
        random_seed_ = nh.param(param_name("seed"), -1);

        std::vector<std::string> map_provider_ids;
        nh.getParam(param_name("maps"), map_provider_ids);

        if(map_provider_ids.size() == 0) {
            throw std::runtime_error("[UniformSampling]: No map providers were found!");
        }

        std::string ms ="[";
        for(auto m : map_provider_ids) {

            if(map_providers.find(m) == map_providers.end())
                throw std::runtime_error("[UniformSampling]: Cannot find map provider '" + m + "'!");

            map_providers_.emplace_back(map_providers.at(m));
            ms += m + ",";
        }
        ms.back() = ']';
    }
};
}

CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d::UniformAllMaps2D, muse_mcl_2d::UniformSampling2D)

