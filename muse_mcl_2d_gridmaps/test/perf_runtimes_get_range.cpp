#include <tf/tf.h>

#include <cslibs_math/random/random.hpp>
#include <muse_smc/samples/sample_set.hpp>

#include <muse_mcl_2d_gridmaps/static_maps/conversion/convert_binary_gridmap.hpp>
#include <muse_mcl_2d_gridmaps/static_maps/binary_gridmap.h>
#include <muse_mcl_2d/samples/sample_density_2d.hpp>
#include <muse_mcl_2d/samples/sample_2d.hpp>


void mapOnly()
{

    nav_msgs::OccupancyGrid grid;
    grid.header.stamp = ros::Time::now();
    grid.info.height = 2500;
    grid.info.width = 2500;
    grid.info.resolution = 0.05;
    grid.data.resize(2500 * 2500, 0);
    grid.info.origin.orientation = tf::createQuaternionMsgFromYaw(0.0);

    muse_mcl_2d_gridmaps::static_maps::BinaryGridMap::Ptr binary;
    muse_mcl_2d_gridmaps::static_maps::conversion::from(grid, binary);

    cslibs_math_2d::Point2d start(62.5, 62.5);

    const double angle_incr = M_PI / 100.0;
    const double radius = 10.0;
    double angle = 0.0;
    double range = 0.0;
    cslibs_time::Time now = cslibs_time::Time::now();
    const std::size_t iterations = 100000;
    for(std::size_t i = 0 ; i < iterations ; ++i) {
        cslibs_math_2d::Point2d end = start + cslibs_math_2d::Vector2d(std::cos(angle) * radius,
                                                                             std::sin(angle) * radius);
        range = binary->getRange(start, end);
        angle += angle_incr;
    }
    std::cout << "took : " << (cslibs_time::Time::now() - now).milliseconds() / iterations << "\n";

    cslibs_math::random::Uniform<1>uniform(-10.0, 10.0);
    now = cslibs_time::Time::now();
    double val = 0.0;
    for(std::size_t i = 9 ; i < iterations ; ++i) {
        val = tfSqrt(uniform.get());
    }
    std::cout << "took : " << (cslibs_time::Time::now() - now).milliseconds() / iterations << "\n";

    now = cslibs_time::Time::now();
    for(std::size_t i = 9 ; i < iterations ; ++i) {
        val = std::sqrt(uniform.get());
    }
    std::cout << "took : " << (cslibs_time::Time::now() - now).milliseconds() / iterations << "\n";

    cslibs_math_2d::Vector2d v1(0.0,0.0);
    cslibs_math_2d::Vector2d v2(1.0,2.0);
    now = cslibs_time::Time::now();
    double l = 0.0;
    for(std::size_t i = 0 ; i < iterations * iterations * iterations ; ++i) {
        l = (v1 - v2).length();
    }
    std::cout << "(v1 - v2).length() took :             " << (cslibs_time::Time::now() - now).milliseconds() << "\n";

    now = cslibs_time::Time::now();
    l = 0.0;
    for(std::size_t i = 0 ; i < iterations * iterations * iterations ; ++i) {
        l = std::sqrt((v1 - v2).length2());
    }
    std::cout << "std::sqrt((v1 - v2).length2()) took : " << (cslibs_time::Time::now() - now).milliseconds() << "\n";


    now = cslibs_time::Time::now();
    l = 0.0;
    for(std::size_t i = 0 ; i < iterations * iterations * iterations ; ++i) {
        l = v1.distance(v2);
    }
    std::cout << "v1.distance(v2) took :                " << (cslibs_time::Time::now() - now).milliseconds() << "\n";


    tf::Vector3 v1_tf(0.0,0.0,0.0);
    tf::Vector3 v2_tf(1.0,2.0,0.0);
    now = cslibs_time::Time::now();
    l = 0.0;
    for(std::size_t i = 0 ; i < iterations * iterations * iterations ; ++i) {
        l = v1_tf.distance(v2_tf);
    }
    std::cout << "v1_tf.distance(v2_tf) took :          " << (cslibs_time::Time::now() - now).milliseconds() << "\n";
}

void withParticles()
{
    nav_msgs::OccupancyGrid grid;
    grid.header.stamp = ros::Time::now();
    grid.info.height = 2500;
    grid.info.width = 2500;
    grid.info.resolution = 0.05;
    grid.data.resize(2500 * 2500, 0);
    grid.info.origin.orientation = tf::createQuaternionMsgFromYaw(0.0);

    muse_mcl_2d_gridmaps::static_maps::BinaryGridMap::Ptr binary;
    muse_mcl_2d_gridmaps::static_maps::conversion::from(grid, binary);


    cslibs_math_2d::Point2d start(62.5, 62.5);

    muse_mcl_2d::SampleIndexation2D  indexation({0.5, M_PI / 180.0 * 10.0});
    muse_mcl_2d::SampleDensity2D::Ptr density (new muse_mcl_2d::SampleDensity2D(indexation, 500000));
    muse_smc::SampleSet<muse_mcl_2d::StateSpaceDescription2D> set("frame",
                                                                   cslibs_time::Time::now(),
                                                                   100,
                                                                   500000,
                                                                   density);


    cslibs_math::random::Uniform<1> rng_l(-100.0, 100.0);
    cslibs_math::random::Uniform<1> rng_a(-M_PI, M_PI);

    const std::size_t size = set.getMaximumSampleSize();
    std::vector<muse_mcl_2d::Sample2D> buff;
    for(std::size_t i = 0 ; i < size; ++i) {
        muse_mcl_2d::Sample2D s;
        s.state.tx() = rng_l.get();
        s.state.ty() = rng_l.get();
        s.state.setYaw(cslibs_math::common::angle::normalize(rng_a.get()));
        buff.emplace_back(s);
    }



    auto start_time = cslibs_time::Time::now();
    auto i = set.getInsertion();
    std::size_t  inserted = 0ul;
    while(i.canInsert()) {
        i.insert(buff[inserted]);
        ++inserted;
    }

    std::cout << "inserted : " << inserted << "\n";
    std::cout << "took time: " << (cslibs_time::Time::now() - start_time).milliseconds() << "ms" << "\n";

    auto iteration = set.getWeightIterator();
    auto end = iteration.end();
    cslibs_math_2d::Transform2d m_T_l;
    cslibs_math_2d::Point2d p;
    const double radius = 10.0;
    double angle = 0.0;
    double range = 0.0;
    cslibs_math_2d::Point2d ray_end = start + cslibs_math_2d::Vector2d(std::cos(angle) * radius,
                                                             std::sin(angle) * radius);
    start_time = cslibs_time::Time::now();
    for(auto it = iteration.begin() ; it != end ; ++it) {
        cslibs_math_2d::Point2d   ray_end_point = m_T_l * ray_end;
        range = binary->getRange(start, ray_end_point);
    }

    std::cout << "took : " << (cslibs_time::Time::now() - start_time).milliseconds() << "\n";
}

int main(int argc, char *argv[])
{
    ros::Time::init();
    mapOnly();
    withParticles();

    return 0;
}
