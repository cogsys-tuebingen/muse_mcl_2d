#include <cslibs_time/time.hpp>
#include <cslibs_math/random/random.hpp>

#include <iostream>

static const int ITERATIONS = 1000000;

void std_exp()
{
    cslibs_math::random::Uniform<double,1> rng(-100.0, +100.0);

    double ms = 0;
    double y = 0.0;
    static_cast<void>(ms);
    static_cast<void>(y);
    for(int i = 0 ; i < ITERATIONS ; ++i) {
        double x = rng.get();

        cslibs_time::Time start = cslibs_time::Time::now();
        y = std::exp(x);
        ms += (cslibs_time::Time::now() - start).milliseconds();
    }
    std::cout << "std exp took: " << ms / ITERATIONS << "ms" << std::endl;
}

void c_exp()
{
    cslibs_math::random::Uniform<double,1> rng(-100.0, +100.0);

    double ms = 0;
    double y = 0.0;
    static_cast<void>(ms);
    static_cast<void>(y);
    for(int i = 0 ; i < ITERATIONS ; ++i) {
        double x = rng.get();

        cslibs_time::Time start = cslibs_time::Time::now();
        y = exp(x);
        ms += (cslibs_time::Time::now() - start).milliseconds();
    }
    std::cout << "c   exp took: " << ms / ITERATIONS << "ms" << std::endl;
}

void std_log()
{
    cslibs_math::random::Uniform<double,1> rng(-100.0, +100.0);

    double ms = 0;
    double y = 0.0;
    static_cast<void>(ms);
    static_cast<void>(y);
    for(int i = 0 ; i < ITERATIONS ; ++i) {
        double x = rng.get();

        cslibs_time::Time start = cslibs_time::Time::now();
        y = std::log(x);
        ms += (cslibs_time::Time::now() - start).milliseconds();
    }
    std::cout << "std  log took: " << ms / ITERATIONS << "ms" << std::endl;
}

void c_log()
{
    cslibs_math::random::Uniform<double,1> rng(-100.0, +100.0);

    double ms = 0;
    double y = 0.0;
    static_cast<void>(ms);
    static_cast<void>(y);
    for(int i = 0 ; i < ITERATIONS ; ++i) {
        double x = rng.get();

        cslibs_time::Time start = cslibs_time::Time::now();
        y = log(x);
        ms += (cslibs_time::Time::now() - start).milliseconds();
    }
    std::cout << "c   log took: " << ms / ITERATIONS << "ms" << std::endl;
}


int main(int argc, char *argv[])
{
    for(std::size_t i = 0 ; i < 10 ; ++i) {
        std_exp();
        c_exp();
        std_log();
        c_log();
        std::cout << "--------------------------------" << std::endl;
    }
    return 0;
}
