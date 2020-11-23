#ifndef PLOT_WEIGHTS_HPP
#define PLOT_WEIGHTS_HPP

#include <opencv2/opencv.hpp>

#include <muse_smc/samples/sample_set.hpp>

#include <atomic>

namespace muse_mcl_2d {
class PlotWeights
{
public:
    using sample_set_t = muse_smc::SampleSet<Sample2D>;
    using Ptr = std::shared_ptr<PlotWeights>;

    PlotWeights(const std::size_t height,
                const std::size_t width,
                const std::string &window_name) :
        window_name_(window_name),
        canvas_(height, width, CV_8UC3, cv::Scalar::all(255))
    {
    }

    virtual ~PlotWeights()
    {
    }

    void plot(const std::vector<double> &weights)
    {
        /// wipe that
        canvas_.setTo(255);
        double max = std::numeric_limits<double>::lowest();
        double mean = 0.0;
        for(auto w : weights) {
            if(w > max)
                max = w;
            mean += w;
        }
        mean /= static_cast<double>(weights.size());

        auto sq = [](const double d) {return d*d;};
        double std_dev = 0.0;
        for(auto w : weights) {
            std_dev += sq(mean - w);
        }
        std_dev /= static_cast<double>(weights.size());
        std_dev = std::sqrt(std_dev);

        if(max == 0.0) {
            std::cerr << "Warning : all weights are zero!" << std::endl;
            return;
        }

        const std::size_t size = weights.size();
        const double      height_scale = static_cast<double>(canvas_.rows) / max;
        const double      width_scale  = static_cast<double>(canvas_.cols) / static_cast<double>(size);

        for(std::size_t i = 0 ; i < weights.size() ; ++i) {
            cv::line(canvas_, cv::Point(i * width_scale, 0), cv::Point(i * width_scale,  weights[i] * height_scale), cv::Scalar(255), 1);
        }

        std::cout << mean << " " << std_dev << std::endl;
        std::cout << std_dev / mean << std::endl;

        int y_mean = mean * height_scale;
        int y_std_dev_up = y_mean + std_dev * height_scale;
        int y_std_dev_do = y_mean - std_dev * height_scale;
        cv::line(canvas_, cv::Point(0,y_mean), cv::Point(canvas_.cols - 1,y_mean), cv::Scalar(0,255));
        cv::line(canvas_, cv::Point(0,y_std_dev_up), cv::Point(canvas_.cols - 1,y_std_dev_up), cv::Scalar(0,127,255));
        cv::line(canvas_, cv::Point(0,y_std_dev_do), cv::Point(canvas_.cols - 1,y_std_dev_do), cv::Scalar(0,127,255));

        cv::flip(canvas_, canvas_, 0);
        cv::copyMakeBorder(canvas_, display_, 5, 5, 5, 5, cv::BORDER_CONSTANT, cv::Scalar(127,127,127));
        cv::imshow(window_name_, display_);
    }

private:
    std::string         window_name_;
    cv::Mat             canvas_;
    cv::Mat             display_;
};
}

#endif // PLOT_WEIGHTS_HPP
