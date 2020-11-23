#include <gtest/gtest.h>

#include <cslibs_plugins/plugin_loader.hpp>
#include <cslibs_plugins_data/data_provider.hpp>
#include <muse_mcl_2d/map/map_provider_2d.hpp>
#include <muse_mcl_2d/update/update_model_2d.hpp>

TEST(Test_muse_mcl_2d_ndt, testLoadProviders) {
  ros::NodeHandle nh{"~"};

  const std::string package_name = "muse_mcl_2d";

  cslibs_plugins::PluginManager<muse_mcl_2d::MapProvider2D> manager(
      muse_mcl_2d::MapProvider2D::Type(), package_name);
  manager.load();
  EXPECT_TRUE(manager.pluginsLoaded());

  std::vector<std::string> class_names = {
      "muse_mcl_2d_ndt::NDTFlatGridmap2DProvider",
      "muse_mcl_2d_ndt::NDTGridmap2dProvider",
      "muse_mcl_2d_ndt::NDTGridmap2dServiceProvider",
      "muse_mcl_2d_ndt::NDTOccupancyGridmap2dProvider",
      "muse_mcl_2d_ndt::NDTOccupancyGridmap2dServiceProvider",
      "muse_mcl_2d_ndt::NDTGridmap3dProvider",
      "muse_mcl_2d_ndt::NDTGridmap3dServiceProvider",
      "muse_mcl_2d_ndt::NDTOccupancyGridmap3dProvider",
      "muse_mcl_2d_ndt::NDTOccupancyGridmap3dServiceProvider",
      "muse_mcl_2d_ndt::ProbabilityGridmapProvider",
      "muse_mcl_2d_ndt::ProbabilityOccupancyGridmapProvider"};

  for (const auto &class_name : class_names) {
    auto constructor = manager.getConstructor(class_name);
    muse_mcl_2d::MapProvider2D::Ptr plugin;

    if (constructor) {
      try {
        plugin = constructor();
      } catch (const std::exception &e) {
        std::cerr << "[muse_mcl_2d_ndt]: " << e.what() << std::endl;
      }
    }

    EXPECT_TRUE(static_cast<bool>(constructor));
    EXPECT_TRUE(plugin.get() != nullptr);
  }
}

TEST(Test_muse_mcl_2d_ndt, testLoadModels) {
  ros::NodeHandle nh{"~"};

  const std::string package_name = "muse_mcl_2d";

  cslibs_plugins::PluginManager<muse_mcl_2d::UpdateModel2D> manager(
      muse_mcl_2d::UpdateModel2D::Type(), package_name);
  manager.load();
  EXPECT_TRUE(manager.pluginsLoaded());

  std::vector<std::string> class_names = {
      "muse_mcl_2d_ndt::Gridmap2dLikelihoodFieldModel",
      "muse_mcl_2d_ndt::OccupancyGridmap2dLikelihoodFieldModel",
      "muse_mcl_2d_ndt::Gridmap3dLikelihoodFieldModel",
      "muse_mcl_2d_ndt::OccupancyGridmap3dLikelihoodFieldModel"};

  for (const auto &class_name : class_names) {
    auto constructor = manager.getConstructor(class_name);
    muse_mcl_2d::UpdateModel2D::Ptr plugin;
    if (constructor) {
      try {
        plugin = constructor();
      } catch (const std::exception &e) {
        std::cerr << "[muse_mcl_2d_ndt]: " << e.what() << std::endl;
      }
    } else {
      std::cerr << class_name << std::endl;
    }

    EXPECT_TRUE(static_cast<bool>(constructor));
    EXPECT_TRUE(plugin.get() != nullptr);
  }
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "muse_mcl_2d_ndt_integration_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
