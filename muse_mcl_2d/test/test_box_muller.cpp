#include <cslibs_math/random/random.hpp>
#include <cslibs_math/statistics/distribution.hpp>
#include <cslibs_time/duration.hpp>
#include <cslibs_time/time.hpp>

double pf_ran_gaussian(double sigma)
{
  double x1, x2, w, r;

  do
  {
    do { r = drand48(); } while (r==0.0);
    x1 = 2.0 * r - 1.0;
    do { r = drand48(); } while (r==0.0);
    x2 = 2.0 * r - 1.0;
    w = x1*x1 + x2*x2;
  } while(w > 1.0 || w==0.0);

  return(sigma * x2 * sqrt(-2.0*log(w)/w));
}


int main(int argc, char *argv[])
{
    std::size_t samples = 1000000;
    std::vector<double> sigmas = {{0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0}};


    for(const double sigma : sigmas) {
        std::cout << "Running with sigma '" << sigma << "'" << std::endl;
        cslibs_time::Duration duration_box_muller;
        cslibs_math::statistics::Distribution<1> distribution_box_muller;
        for(std::size_t i = 0 ; i < samples ; ++i) {
            double r;
            cslibs_time::Time t = cslibs_time::Time::now();
            r = pf_ran_gaussian(sigma);
            duration_box_muller += (cslibs_time::Time::now() - t);
            distribution_box_muller.add(r);
        }
        std::cout << "Box Muller : " << std::endl;
        std::cout << "Mean       : " << distribution_box_muller.getMean() << std::endl;
        std::cout << "Sigma      : " << distribution_box_muller.getStandardDeviation() << std::endl;
        std::cout << "Time       : " << duration_box_muller.milliseconds() / static_cast<double>(samples) << std::endl;

        std::cout << "----------------------" << std::endl;

        cslibs_time::Duration duration_normal;
        cslibs_math::statistics::Distribution<1,0> distribution_normal;
        cslibs_math::random::Normal<1> rng(0.0, sigma);
        for(std::size_t i = 0 ; i < samples ; ++i) {
            double r;
            cslibs_time::Time t = cslibs_time::Time::now();
            r = rng.get();
            duration_normal += (cslibs_time::Time::now() - t);
            distribution_normal.add(r);
        }
        std::cout << "Normal     : " << std::endl;
        std::cout << "Mean       : " << distribution_normal.getMean() << std::endl;
        std::cout << "Sigma      : " << distribution_normal.getStandardDeviation() << std::endl;
        std::cout << "Time       : " << duration_normal.milliseconds() / static_cast<double>(samples) << std::endl;
        std::cout << "----------------------" << std::endl;
    }

    return 0;
}
