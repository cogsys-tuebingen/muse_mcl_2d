#include <tf/tf.h>

#include <cslibs_time/time.hpp>
#include <cslibs_math/random/random.hpp>
#include <iomanip>

#include <cslibs_math_2d/linear/transform.hpp>
#include <cslibs_math_2d/linear/vector.hpp>
#include <cslibs_time/stamped.hpp>
#include <cslibs_math/common/angle.hpp>

const std::size_t ITERATIONS = 1000000;

void constructors()
{
    cslibs_math::random::Uniform<double,1> rng(-10.0, 10.0);

    cslibs_time::Time start = cslibs_time::Time::now();
    double yaw = 0.0;
    for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
        cslibs_math_2d::Transform2d t;
        yaw = t.yaw();
    }
    std::cout << "empty:" << "\n";
    std::cout << "took time: " << (cslibs_time::Time::now() - start).milliseconds() << "ms" << "\n";

    start = cslibs_time::Time::now();
    for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
        cslibs_math_2d::Transform2d t(i, i);
        yaw = t.yaw();
    }
    std::cout << "x y:" << "\n";
    std::cout << "took time: " << (cslibs_time::Time::now() - start).milliseconds() << "ms" << "\n";

    start = cslibs_time::Time::now();
    cslibs_math_2d::Vector2d v(rng.get(), rng.get());
    for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
        cslibs_math_2d::Transform2d t(v);
        yaw = t.yaw();
        v(0) += i;
    }
    std::cout << "v:" << "\n";
    std::cout << "took time: " << (cslibs_time::Time::now() - start).milliseconds() << "ms" << "\n";

    start = cslibs_time::Time::now();
    for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
        cslibs_math_2d::Transform2d t(i,i,i);
        yaw = t.yaw();
    }
    std::cout << "x y yaw:" << "\n";
    std::cout << "took time: " << (cslibs_time::Time::now() - start).milliseconds() << "ms" << "\n";

    start = cslibs_time::Time::now();
    for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
        cslibs_math_2d::Transform2d t(v,i);
        yaw = t.yaw();
    }
    std::cout << "v yaw:" << "\n";
    std::cout << "took time: " << (cslibs_time::Time::now() - start).milliseconds() << "ms" << "\n";

    start = cslibs_time::Time::now();
    cslibs_math_2d::Transform2d t(rng.get(), rng.get());
    for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
        cslibs_math_2d::Transform2d t_(t);
        yaw = t_.yaw();
    }
    std::cout << "t:" << "\n";
    std::cout << "took time: " << (cslibs_time::Time::now() - start).milliseconds() << "ms" << "\n";
}

void multiplyVector()
{
    cslibs_math::random::Uniform<double,1> rng(-10.0, 10.0);
    double mean_ms_t = 0.0;
    double mean_ms_tf= 0.0;

    for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
        cslibs_time::Time start = cslibs_time::Time::now();
        cslibs_math_2d::Transform2d t(rng.get(), rng.get(), cslibs_math::common::angle::normalize(rng.get()));
        cslibs_math_2d::Vector2d v(rng.get(), rng.get());
        for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
            v = t * v;
        }
        mean_ms_t += (cslibs_time::Time::now() - start).milliseconds();

        start = cslibs_time::Time::now();
        tf::Transform tf_t(tf::createQuaternionFromYaw(cslibs_math::common::angle::normalize(rng.get())),
                           tf::Vector3(rng.get(), rng.get(), 0.0));
        tf::Vector3   tf_v (rng.get(), rng.get(), 0.0);
        for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
            tf_v = tf_t * tf_v;
        }
        mean_ms_tf += (cslibs_time::Time::now() - start).milliseconds();
    }

    std::cout << "vector:" << "\n";
    std::cout << "took time: " << std::fixed << std::setprecision(10) << mean_ms_t / ITERATIONS << "ms" << "\n";
    std::cout << "tf vector:"  << "\n";
    std::cout << "took time: " << std::fixed << std::setprecision(10) << mean_ms_tf/ ITERATIONS << "ms" << "\n";
}

void multiplyTransform()
{
    cslibs_math::random::Uniform<double,1> rng(-10.0, 10.0);
    double mean_ms_t = 0.0;
    double mean_ms_tf= 0.0;

    cslibs_math_2d::Transform2d t;
    tf::Transform tf;
    for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
        cslibs_time::Time start = cslibs_time::Time::now();
        cslibs_math_2d::Transform2d ta(rng.get(), rng.get(), cslibs_math::common::angle::normalize(rng.get()));
        cslibs_math_2d::Transform2d tb(rng.get(), rng.get(), cslibs_math::common::angle::normalize(rng.get()));
        for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
            tb = ta * tb;
        }
        mean_ms_t += (cslibs_time::Time::now() - start).milliseconds();
        t = tb;

        start = cslibs_time::Time::now();
        tf::Transform tf_ta(tf::createQuaternionFromYaw(cslibs_math::common::angle::normalize(rng.get())),
                            tf::Vector3(rng.get(), rng.get(), 0.0));
        tf::Transform tf_tb(tf::createQuaternionFromYaw(cslibs_math::common::angle::normalize(rng.get())),
                            tf::Vector3(rng.get(), rng.get(), 0.0));
        for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
            tf_tb = tf_ta * tf_tb;
        }
        mean_ms_tf += (cslibs_time::Time::now() - start).milliseconds();
        tf = tf_tb;
    }

    std::cout << "transform multiply:" << "\n";
    std::cout << "took time: " << std::fixed << std::setprecision(10) << mean_ms_t / ITERATIONS << "ms" << "\n";
    std::cout << "tf transform multiply:"  << "\n";
    std::cout << "took time: " << std::fixed << std::setprecision(10) << mean_ms_tf/ ITERATIONS << "ms" << "\n";
}

void multiplyAssignTransform()
{
    cslibs_math::random::Uniform<double,1> rng(-10.0, 10.0);
    double mean_ms_t = 0.0;
    double mean_ms_tf= 0.0;

    cslibs_math_2d::Transform2d t;
    tf::Transform tf;
    for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
        cslibs_time::Time start = cslibs_time::Time::now();
        cslibs_math_2d::Transform2d ta(rng.get(), rng.get(), cslibs_math::common::angle::normalize(rng.get()));
        cslibs_math_2d::Transform2d tb(rng.get(), rng.get(), cslibs_math::common::angle::normalize(rng.get()));
        for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
            tb *= ta;
        }
        mean_ms_t += (cslibs_time::Time::now() - start).milliseconds();
        t = tb;

        start = cslibs_time::Time::now();
        tf::Transform tf_ta(tf::createQuaternionFromYaw(cslibs_math::common::angle::normalize(rng.get())),
                            tf::Vector3(rng.get(), rng.get(), 0.0));
        tf::Transform tf_tb(tf::createQuaternionFromYaw(cslibs_math::common::angle::normalize(rng.get())),
                            tf::Vector3(rng.get(), rng.get(), 0.0));
        for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
            tf_tb *= tf_ta;
        }
        mean_ms_tf += (cslibs_time::Time::now() - start).milliseconds();
        tf = tf_tb;
    }

    std::cout << "transform multiply assign:" << "\n";
    std::cout << "took time: " << std::fixed << std::setprecision(10) << mean_ms_t / ITERATIONS << "ms" << "\n";
    std::cout << "tf transform multiply assign:"  << "\n";
    std::cout << "took time: " << std::fixed << std::setprecision(10) << mean_ms_tf/ ITERATIONS << "ms" << "\n";
}

void assign()
{
    cslibs_math::random::Uniform<double,1> rng(-10.0, 10.0);
    double mean_ms_t = 0.0;
    double mean_ms_tl= 0.0;
    double mean_ms_tf= 0.0;

    cslibs_math_2d::Transform2d t;
    tf::Transform tf;
    for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
        cslibs_time::Time start = cslibs_time::Time::now();
        cslibs_math_2d::Transform2d ta(rng.get(), rng.get(), cslibs_math::common::angle::normalize(rng.get()));
        cslibs_math_2d::Transform2d tb(rng.get(), rng.get(), cslibs_math::common::angle::normalize(rng.get()));
        for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
            tb = ta;
        }
        mean_ms_t += (cslibs_time::Time::now() - start).milliseconds();
        t = tb;

        start = cslibs_time::Time::now();
        tf::Transform tf_ta(tf::createQuaternionFromYaw(cslibs_math::common::angle::normalize(rng.get())),
                            tf::Vector3(rng.get(), rng.get(), 0.0));
        tf::Transform tf_tb(tf::createQuaternionFromYaw(cslibs_math::common::angle::normalize(rng.get())),
                            tf::Vector3(rng.get(), rng.get(), 0.0));
        for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
            tf_tb = tf_ta;
        }
        mean_ms_tf += (cslibs_time::Time::now() - start).milliseconds();
        tf = tf_tb;
    }

    std::cout << "transform assign:" << "\n";
    std::cout << "took time: " << std::fixed << std::setprecision(10) << mean_ms_t / ITERATIONS << "ms" << "\n";
    std::cout << "tf transform assign:"  << "\n";
    std::cout << "took time: " << std::fixed << std::setprecision(10) << mean_ms_tf/ ITERATIONS << "ms" << "\n";
}

void inverse()
{
    cslibs_math::random::Uniform<double,1> rng(-10.0, 10.0);
    double mean_ms_t = 0.0;
    double mean_ms_tf= 0.0;

    cslibs_math_2d::Transform2d t;
    tf::Transform tf;
    for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
        cslibs_time::Time start = cslibs_time::Time::now();
        cslibs_math_2d::Transform2d ta(rng.get(), rng.get(), cslibs_math::common::angle::normalize(rng.get()));
        for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
            t = ta.inverse() * t;
        }
        mean_ms_t += (cslibs_time::Time::now() - start).milliseconds();

        start = cslibs_time::Time::now();
        tf::Transform tf_ta(tf::createQuaternionFromYaw(cslibs_math::common::angle::normalize(rng.get())),
                            tf::Vector3(rng.get(), rng.get(), 0.0));
        for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
            tf = tf_ta.inverse() * tf;
        }
        mean_ms_tf += (cslibs_time::Time::now() - start).milliseconds();
    }

    std::cout << "transform inverse:" << "\n";
    std::cout << "took time: " << std::fixed << std::setprecision(10) << mean_ms_t / ITERATIONS << "ms" << "\n";
    std::cout << "tf transform inverse:"  << "\n";
    std::cout << "took time: " << std::fixed << std::setprecision(10) << mean_ms_tf/ ITERATIONS << "ms" << "\n";
}

void length()
{
    cslibs_math::random::Uniform<double,1> rng(-10.0, 10.0);
    double mean_ms_t = 0.0;
    double mean_ms_tf= 0.0;

    for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
        cslibs_time::Time start = cslibs_time::Time::now();
        cslibs_math_2d::Vector2d tv(rng.get(), rng.get());
        double length = 0.0;
        for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
            length = tv.length();
        }
        mean_ms_t += (cslibs_time::Time::now() - start).milliseconds();

        start = cslibs_time::Time::now();
        tf::Vector3   tf_v (rng.get(), rng.get(), 0.0);
        for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
           length = tf_v.length();
        }
        mean_ms_tf += (cslibs_time::Time::now() - start).milliseconds();
    }

    std::cout << "vector length:" << "\n";
    std::cout << "took time: " << std::fixed << std::setprecision(10) << mean_ms_t / ITERATIONS << "ms" << "\n";
    std::cout << "tf vector:"  << "\n";
    std::cout << "took time: " << std::fixed << std::setprecision(10) << mean_ms_tf/ ITERATIONS << "ms" << "\n";
}

void distance()
{

}



int main(int argc, char *argv[])
{
    constructors();
    multiplyVector();
    multiplyTransform();
    multiplyAssignTransform();
    assign();
    inverse();
    length();

    return 0;
}
