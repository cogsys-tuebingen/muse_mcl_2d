#include <muse_mcl_2d/scheduling/scheduler_2d.hpp>

#include <cslibs_plugins/plugin.hpp>

#include <ext/pb_ds/priority_queue.hpp>

#include <unordered_map>

namespace muse_mcl_2d {
class CFS : public muse_mcl_2d::Scheduler2D
{
public:
    struct Entry {
        int64_t     vtime;
        std::size_t id;

        inline explicit Entry(const std::size_t id) :
            vtime(0),
            id(id)
        {
        }

        inline explicit Entry(const int64_t     vtime,
                              const std::size_t id) :
            vtime(vtime),
            id(id)
        {
        }

        inline Entry(const Entry &other) :
            vtime(other.vtime),
            id(other.id)

        {
        }

        inline Entry & operator = (const Entry &other)
        {
            vtime = other.vtime;
            id = other.id;
            return *this;
        }

        struct Greater {
            inline bool operator()( const Entry& lhs,
                                    const Entry& rhs ) const
            {
                return (lhs.vtime == rhs.vtime) ? (lhs.id > rhs.id) : (lhs.vtime > rhs.vtime);
            }
        };
    };

    using Ptr                 = std::shared_ptr<CFS>;
    using rate_t              = cslibs_time::Rate;
    using update_t            = muse_smc::Update<StateSpaceDescription2D, cslibs_plugins_data::Data>;
    using queue_t             = __gnu_pbds::priority_queue<Entry, typename Entry::Greater, __gnu_pbds::rc_binomial_heap_tag>;
    using time_priority_map_t = std::unordered_map<id_t, double>;
    using resampling_t        = muse_smc::Resampling<StateSpaceDescription2D, cslibs_plugins_data::Data>;
    using sample_set_t        = muse_smc::SampleSet<StateSpaceDescription2D>;
    using nice_map_t          = std::unordered_map<id_t, double>;
    using time_t              = cslibs_time::Time;
    using duration_t          = cslibs_time::Duration;
    using update_model_map_t  = std::map<std::string, UpdateModel2D::Ptr>;


    inline void setup(const update_model_map_t &update_models,
                      ros::NodeHandle &nh) override
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};

        std::map<std::string, double> nice_values;
        nh.getParam(param_name("nice_values"), nice_values);

        for(const auto &um : update_models) {
            const UpdateModel2D::Ptr &u = um.second;
            const std::size_t id = u->getId();
            const std::string name = u->getName();
            double nice = 1.0;

            if(nice_values.find(u->getName()) == nice_values.end()) {
              ROS_WARN_STREAM("Did not find nice value for update model '" << u->getName() << "', setting to 1.0.");
            } else {
              nice = nice_values[name];
            }
            nice_values_[id] = std::min(1.0, std::max(0.0, nice));
            q_.push(Entry(id));
        }
        double preferred_rate = nh.param<double>(param_name("preferred_rate"), 5.0);
        resampling_period_ = duration_t(preferred_rate > 0.0 ? 1.0 / preferred_rate : 0.0);
    }

    virtual bool apply(typename update_t::Ptr     &u,
                       typename sample_set_t::Ptr &s) override
    {
        auto now = []()
        {
            return time_t(ros::Time::now().toNSec());
        };

        if(last_update_time_.isZero())
            last_update_time_ = now();

        const id_t   id    = u->getModelId();
        const time_t stamp = u->getStamp();

        if(id == q_.top().id && stamp >= next_update_time_) {
            Entry entry = q_.top();
            q_.pop();

            const time_t start = now();
            u->apply(s->getWeightIterator());
            const duration_t dur = (now() - start);
            const duration_t dur_prediction_resampling = (now() - last_update_time_);
           //  std::cerr << dur_prediction_resampling << "\n";

            entry.vtime += static_cast<int64_t>(static_cast<double>(dur.nanoseconds()) * nice_values_[id]);
            next_update_time_ = stamp + dur ; // ; dur_prediction_resampling;

            q_.push(entry);
            last_update_time_ = now();
            return true;
        }
        return false;
    }


    virtual bool apply(typename resampling_t::Ptr &r,
                       typename sample_set_t::Ptr &s) override
    {
        const cslibs_time::Time &stamp = s->getStamp();

        auto now = []()
        {
            return time_t(ros::Time::now().toNSec());
        };

        if(resampling_time_.isZero())
            resampling_time_ = stamp;

        auto do_apply = [&stamp, &r, &s, &now, this] () {
            r->apply(*s);

            resampling_time_   = stamp + resampling_period_;

            int64_t min_vtime = q_.top().vtime;
            queue_t q;
            for(auto e : q_) {
                e.vtime -= min_vtime;
                q.push(e);
            }
            std::swap(q, q_);
            return true;
        };
        auto do_not_apply = [] () {
            return false;
        };
        return resampling_time_ < stamp ? do_apply() : do_not_apply();
    }

protected:
    time_t              last_update_time_;
    time_t              next_update_time_;
    time_t              resampling_time_;
    duration_t          resampling_period_;
    nice_map_t          nice_values_;
    queue_t             q_;
};
}

CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d::CFS, muse_mcl_2d::Scheduler2D)
