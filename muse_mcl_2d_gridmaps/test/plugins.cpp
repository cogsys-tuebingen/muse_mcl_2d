#include <gtest/gtest.h>

#include <cslibs_plugins/plugin_loader.hpp>
#include <cslibs_plugins_data/data_provider.hpp>
#include <muse_mcl_2d/map/map_provider_2d.hpp>
#include <muse_mcl_2d/update/update_model_2d.hpp>

TEST(Test_muse_mcl_2d_gridmaps, testLoadProviders) {
  ros::NodeHandle nh{"~"};

  const std::string package_name = "muse_mcl_2d";

  cslibs_plugins::PluginManager<muse_mcl_2d::MapProvider2D> manager(
      muse_mcl_2d::MapProvider2D::Type(), package_name);
  manager.load();
  EXPECT_TRUE(manager.pluginsLoaded());

  std::vector<std::string> class_names = {
      "muse_mcl_2d_gridmaps::BinaryGridmapProvider",
      "muse_mcl_2d_gridmaps::BinaryGridmapServiceProvider",
      "muse_mcl_2d_gridmaps::BinaryGridmapLoadProvider",
      "muse_mcl_2d_gridmaps::DistanceGridmapProvider",
      "muse_mcl_2d_gridmaps::DistanceGridmapServiceProvider",
      "muse_mcl_2d_gridmaps::DistanceGridmapLoadProvider",
      "muse_mcl_2d_gridmaps::ProbabilityGridmapProvider",
      "muse_mcl_2d_gridmaps::ProbabilityGridmapServiceProvider",
      "muse_mcl_2d_gridmaps::ProbabilityGridmapLoadProvider",
      "muse_mcl_2d_gridmaps::LikelihoodFieldGridmapProvider",
      "muse_mcl_2d_gridmaps::LikelihoodFieldGridmapServiceProvider",
      "muse_mcl_2d_gridmaps::LikelihoodFieldGridmapLoadProvider"};

  for (const auto &class_name : class_names) {
    auto constructor = manager.getConstructor(class_name);
    muse_mcl_2d::MapProvider2D::Ptr plugin;
    if (constructor) {
      try {
        plugin = constructor();
      } catch (const std::exception &e) {
        std::cerr << "[muse_mcl_2d_gridmaps]: " << e.what() << std::endl;
      }
    }
    EXPECT_TRUE(static_cast<bool>(constructor));
    EXPECT_TRUE(plugin.get() != nullptr);
  }
}

TEST(Test_muse_mcl_2d_gridmaps, testLoadModels) {
  ros::NodeHandle nh{"~"};

  const std::string package_name = "muse_mcl_2d";

  cslibs_plugins::PluginManager<muse_mcl_2d::UpdateModel2D> manager(
      muse_mcl_2d::UpdateModel2D::Type(), package_name);
  manager.load();
  EXPECT_TRUE(manager.pluginsLoaded());

  std::vector<std::string> class_names = {
      "muse_mcl_2d_gridmaps::BeamModel",
      "muse_mcl_2d_gridmaps::BeamModelAMCL",
      "muse_mcl_2d_gridmaps::BeamModelMLE",
      "muse_mcl_2d_gridmaps::BeamModelLog",
      "muse_mcl_2d_gridmaps::BeamModelAMCLNormalized",
      "muse_mcl_2d_gridmaps::LikelihoodFieldModel",
      "muse_mcl_2d_gridmaps::LikelihoodFieldModelAMCL",
      "muse_mcl_2d_gridmaps::LikelihoodFieldProbModelAMCL",
      "muse_mcl_2d_gridmaps::LikelihoodFieldModelLog",
      "muse_mcl_2d_gridmaps::LikelihoodFieldModelAMCLNormalized",
      "muse_mcl_2d_gridmaps::LikelihoodFieldModelPC",
      "muse_mcl_2d_gridmaps::LikelihoodFieldModelPCLog"};

  for (const auto &class_name : class_names) {
    auto constructor = manager.getConstructor(class_name);
    muse_mcl_2d::UpdateModel2D::Ptr plugin;
    if (constructor) {
      try {
        plugin = constructor();
      } catch (const std::exception &e) {
        std::cerr << "[muse_mcl_2d_gridmaps]: " << e.what() << std::endl;
      }
    }
    EXPECT_TRUE(static_cast<bool>(constructor));
    EXPECT_TRUE(plugin.get() != nullptr);
  }
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "muse_mcl_2d_gridmaps_integration_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
