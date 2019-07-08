#ifndef MAP_2D_HPP
#define MAP_2D_HPP

#include <muse_mcl_2d/samples/sample_2d.hpp>
#include <muse_smc/state_space/state_space.hpp>

namespace muse_mcl_2d {
using Map2D = muse_smc::StateSpace<Sample2D>;
}

#endif // MAP_2D_HPP
