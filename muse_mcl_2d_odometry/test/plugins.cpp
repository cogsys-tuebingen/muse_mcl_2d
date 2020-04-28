#include <gtest/gtest.h>

#include <cslibs_plugins/plugin_loader.hpp>
#include <cslibs_plugins_data/data_provider.hpp>
#include <muse_mcl_2d/prediction/prediction_model_2d.hpp>

TEST(Test_muse_mcl_2d_odometry, testLoadModels) {
  ros::NodeHandle nh{"~"};

  const std::string package_name = "muse_mcl_2d";

  cslibs_plugins::PluginManager<muse_mcl_2d::PredictionModel2D> manager(
      muse_mcl_2d::PredictionModel2D::Type(), package_name);
  manager.load();
  EXPECT_TRUE(manager.pluginsLoaded());

  std::vector<std::string> class_names = {
      "muse_mcl_2d_odometry::DifferentialDrive",
      "muse_mcl_2d_odometry::DifferentialDriveUncorrected",
      "muse_mcl_2d_odometry::DifferentialDriveBoxMuller",
      "muse_mcl_2d_odometry::DifferentialDriveAbs",
      "muse_mcl_2d_odometry::OmniDrive"};

  for (const auto &class_name : class_names) {
    auto constructor = manager.getConstructor(class_name);
    muse_mcl_2d::PredictionModel2D::Ptr plugin;
    if (constructor) {
      try {
        plugin = constructor();
      } catch (const std::exception &e) {
        std::cerr << "[muse_mcl_odometry_2d]: " << e.what() << std::endl;
      }
    }
    EXPECT_TRUE(static_cast<bool>(constructor));
    EXPECT_TRUE(plugin.get() != nullptr);
  }
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "muse_mcl_odometry_2d_integration_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
