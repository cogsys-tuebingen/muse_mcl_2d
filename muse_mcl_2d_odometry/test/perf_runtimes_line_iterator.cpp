#include <opencv2/opencv.hpp>
#include <muse_mcl_2d_gridmaps/static_maps/algorithms/bresenham.hpp>
#include <muse_mcl_2d_gridmaps/static_maps/binary_gridmap.h>
#include <nav_msgs/OccupancyGrid.h>

int main(int argc, char *argv[])
{
    cv::Mat display(200, 200, CV_8UC3, cv::Scalar());
    cv::Mat mask(200, 200, CV_8UC1, cv::Scalar());



    {

        cv::Point start(100,100);
        std::vector<cv::Point> ends;
        double angle = -M_PI;
        double angle_incr = M_PI / 30.0;
        double radius = 50.0;
        while(angle < M_PI) {
            ends.emplace_back(start +
                              cv::Point(std::cos(angle) * radius,
                                        std::sin(angle) * radius));
            angle += angle_incr;
        }


        display.setTo(cv::Scalar());
        for(std::size_t i = 0 ; i < ends.size() ; ++i) {
            const cv::Point &end = ends[i];
            cv::circle(display, start, 2, cv::Scalar(255),CV_FILLED, CV_AA);
            cv::circle(display, end, 2, cv::Scalar(255,255),CV_FILLED, CV_AA);
            mask.at<uchar>(end.y, end.x) = 255;

            muse_mcl_2d_gridmaps::static_maps::algorithms::Bresenham<uchar> it({start.x, start.y},
            {end.x,end.y},
            {mask.cols, mask.rows},
                                                                               mask.ptr<uchar>() );

            muse_mcl_2d_gridmaps::static_maps::algorithms::Bresenham<uchar> const_it({start.x, start.y},
            {end.x,end.y},
            {mask.cols, mask.rows},
                                                                                     mask.ptr<uchar>() );
            while(!it.done()) {
                display.at<cv::Vec3b>(it.y(), it.x()) = cv::Vec3b(255,255,255);
                std::cout << "before " << (int) *const_it << "\n";
                *it = 255;
                std::cout << "after " << (int) *const_it << "\n";
                ++it;
            }

            std::size_t iterations = 0ul;
            cslibs_time::Time start = cslibs_time::Time::now();
            while(!const_it.done()) {
                ++const_it;
                ++iterations;
            }
            std::cout << "took " << (cslibs_time::Time::now() - start).milliseconds() << "ms for " << iterations << " iterations" << "\n";

            cv::imshow("display", display);
            cv::imshow("mask", mask);
            int key = cv::waitKey(0) & 0xFF;
            if(key == 27)
                break;

        }
    }

    return 0;
}
