#include <gtest/gtest.h>

#include <cslibs_plugins/plugin_loader.hpp>
#include <cslibs_plugins_data/data_provider.hpp>

#include <muse_mcl_2d/resampling/resampling_2d.hpp>
#include <muse_mcl_2d/sampling/uniform_sampling_2d.hpp>
#include <muse_mcl_2d/sampling/normal_sampling_2d.hpp>
#include <muse_mcl_2d/density/sample_density_2d.hpp>
#include <muse_mcl_2d/scheduling/scheduler_2d.hpp>

TEST(Test_muse_mcl_2d, testLoadResampling) {
  ros::NodeHandle nh{"~"};

  const std::string package_name = "muse_mcl_2d";

  cslibs_plugins::PluginManager<muse_mcl_2d::Resampling2D> manager(
      muse_mcl_2d::Resampling2D::Type(), package_name);
  manager.load();
  EXPECT_TRUE(manager.pluginsLoaded());

  std::vector<std::string> class_names = {
      "muse_mcl_2d::KLD2D",
      "muse_mcl_2d::KLDAugmented2D",
      "muse_mcl_2d::Multinomial",
      "muse_mcl_2d::Residual",
      "muse_mcl_2d::Stratified",
      "muse_mcl_2d::Systematic",
      "muse_mcl_2d::WheelOfFortune",
      "muse_mcl_2d::LocalRegenerationKLD2D"};

  for (const auto &class_name : class_names) {
    auto constructor = manager.getConstructor(class_name);
    muse_mcl_2d::Resampling2D::Ptr plugin;
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

TEST(Test_muse_mcl_2d, testLoadUniformSampling) {
  ros::NodeHandle nh{"~"};

  const std::string package_name = "muse_mcl_2d";

  cslibs_plugins::PluginManager<muse_mcl_2d::UniformSampling2D> manager(
      muse_mcl_2d::UniformSampling2D::Type(), package_name);
  manager.load();
  EXPECT_TRUE(manager.pluginsLoaded());

  std::vector<std::string> class_names = {
      "muse_mcl_2d::UniformAllMaps2D",
      "muse_mcl_2d::UniformPrimaryMap2D"};

  for (const auto &class_name : class_names) {
    auto constructor = manager.getConstructor(class_name);
    muse_mcl_2d::UniformSampling2D::Ptr plugin;
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


TEST(Test_muse_mcl_2d, testLoadNormalSampling) {
  ros::NodeHandle nh{"~"};

  const std::string package_name = "muse_mcl_2d";

  cslibs_plugins::PluginManager<muse_mcl_2d::NormalSampling2D> manager(
      muse_mcl_2d::NormalSampling2D::Type(), package_name);
  manager.load();
  EXPECT_TRUE(manager.pluginsLoaded());

  std::vector<std::string> class_names = {
      "muse_mcl_2d::Normal2D"};

  for (const auto &class_name : class_names) {
    auto constructor = manager.getConstructor(class_name);
    muse_mcl_2d::NormalSampling2D::Ptr plugin;
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

TEST(Test_muse_mcl_2d, testLoadSampleDensity) {
  ros::NodeHandle nh{"~"};

  const std::string package_name = "muse_mcl_2d";

  cslibs_plugins::PluginManager<muse_mcl_2d::SampleDensity2D> manager(
      muse_mcl_2d::SampleDensity2D::Type(), package_name);
  manager.load();
  EXPECT_TRUE(manager.pluginsLoaded());

  std::vector<std::string> class_names = {
      "muse_mcl_2d::SimpleSampleDensity2D",
      "muse_mcl_2d::MCSampleDensity2D"};

  for (const auto &class_name : class_names) {
    auto constructor = manager.getConstructor(class_name);
    muse_mcl_2d::SampleDensity2D::Ptr plugin;
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

TEST(Test_muse_mcl_2d, testLoadScheduler) {
  ros::NodeHandle nh{"~"};

  const std::string package_name = "muse_mcl_2d";

  cslibs_plugins::PluginManager<muse_mcl_2d::Scheduler2D> manager(
      muse_mcl_2d::Scheduler2D::Type(), package_name);
  manager.load();
  EXPECT_TRUE(manager.pluginsLoaded());

  std::vector<std::string> class_names = {
      "muse_mcl_2d::CFS",
      "muse_mcl_2d::CFSLaggy",
      "muse_mcl_2d::Rate",
      "muse_mcl_2d::CFSDropStatistic",
      "muse_mcl_2d::RateDropStatistic",
      "muse_mcl_2d::Dummy"};

  for (const auto &class_name : class_names) {
    auto constructor = manager.getConstructor(class_name);
    muse_mcl_2d::Scheduler2D::Ptr plugin;
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
  ros::init(argc, argv, "muse_mcl_2d_integration_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
