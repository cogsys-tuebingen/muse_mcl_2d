#include <gtest/gtest.h>

#include <cslibs_math_2d/types/box.hpp>
#include <cslibs_math_2d/types/transform.hpp>
#include <cslibs_math/random/random.hpp>

using rng_t = cslibs_math::random::Uniform<1>;

TEST(Test_muse_mcl_2d, testBoxConstructors)
{
    rng_t rng(-10.0, 10.0);

    cslibs_math_2d::Box2d b0;
    EXPECT_EQ(b0.getMin().x(), std::numeric_limits<double>::lowest());
    EXPECT_EQ(b0.getMin().y(), std::numeric_limits<double>::lowest());
    EXPECT_EQ(b0.getMax().x(), std::numeric_limits<double>::max());
    EXPECT_EQ(b0.getMax().y(), std::numeric_limits<double>::max());

    double x0 = rng.get();
    double y0 = rng.get();
    double x1 = rng.get();
    double y1 = rng.get();
    if(x0 > x1)
        std::swap(x0, x1);
    if(y0 > y1)
        std::swap(y0, y1);
    cslibs_math_2d::Box2d b1(x0,y0,x1,y1);
    EXPECT_EQ(b1.getMin().x(), x0);
    EXPECT_EQ(b1.getMin().y(), y0);
    EXPECT_EQ(b1.getMax().x(), x1);
    EXPECT_EQ(b1.getMax().y(), y1);

    cslibs_math_2d::Box2d b2({x0,y0},{x1,y1});
    EXPECT_EQ(b1.getMin().x(), x0);
    EXPECT_EQ(b1.getMin().y(), y0);
    EXPECT_EQ(b1.getMax().x(), x1);
    EXPECT_EQ(b1.getMax().y(), y1);
}

TEST(Test_muse_mcl_2d, testBoxIntersects)
{
    rng_t rng_out_sign(0.0, 1.0);
    rng_t rng_out(1.1, 10.0);
    rng_t rng_in(-1.0, 1.0);

    auto generate_outer = [&rng_out, &rng_out_sign]()
    {
        const double sign = rng_out_sign.get() > 0.5 ? 1. : -1.;
        return cslibs_math_2d::Point2d(rng_out.get(), rng_out.get()) * sign;
    };
    auto generate_inner = [&rng_in] ()
    {
        return cslibs_math_2d::Point2d(rng_in.get(), rng_in.get());
    };


    const     cslibs_math_2d::Box2d bb(-1.0, -1.0, 1.0, 1.0);
    for(std::size_t i = 0 ; i < 1000 ; ++i) {
        EXPECT_TRUE(bb.intersects({generate_inner(), generate_outer()}));
    }

    rng_out.set(10.0, 20.0);
    for(std::size_t i = 0 ; i < 1000 ; ++i) {
        const auto start = cslibs_math_2d::Point2d(rng_out.get(), rng_out.get());
        const auto end   = cslibs_math_2d::Point2d(rng_out.get(), rng_out.get());
        const double angle_incr = M_PI / 18.0;
        double angle = 0.0;

        for(std::size_t i = 0 ; i < 36 ; ++i, angle+=angle_incr) {
            cslibs_math_2d::Transform2d rot(angle);
            EXPECT_FALSE(bb.intersects({rot * start, rot * end}));
        }
    }

}

TEST(Test_muse_mcl_2d, testBoxIntersection)
{

}


int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
