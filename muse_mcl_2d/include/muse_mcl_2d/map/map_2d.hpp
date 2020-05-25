#ifndef MUSE_MCL_2D_MAP_2D_HPP
#define MUSE_MCL_2D_MAP_2D_HPP

#include <muse_mcl_2d/instance/sample_2d.hpp>
#include <muse_smc/smc/traits/state_space.hpp>

namespace muse_mcl_2d {
    using Map2D = muse_smc::traits::StateSpace<Sample2D>::type;
}

#endif // MUSE_MCL_2D_MAP_2D_HPP
