#include "ds_base/ds_process.h"

#include <gtest/gtest.h>

using namespace ds_base;

#if 0
class SurfaceJoystickControllerTest: public SurfaceJoystickController
{
 public:
  SurfaceJoystickControllerTest(): SurfaceJoystickController(){}
  ~SurfaceJoystickControllerTest() override = default;

  uint64_t type() const noexcept override {
    return 0;
  }

 protected:
  void setupTransferFunctions() override {

  }
};
#endif

class ProcessTest : public ::testing::Test
{
 protected:
  void SetUp() override
  {
    process_ = std::unique_ptr<DsProcess>(new DsProcess);
  }

  std::unique_ptr<DsProcess> process_;
};

TEST_F(ProcessTest, clean_exit)
{
  process_->setup();
  process_.reset();
}

void empty_callback(ds_core_msgs::RawData) {}

TEST_F(ProcessTest, multiple_asio_connections)
{
  process_->setup();

  auto str = ros::names::resolve(ros::this_node::getName(), std::string{"connection1"});
  auto con1 = process_->addConnection(str, &empty_callback);

  str = ros::names::resolve(ros::this_node::getName(), std::string{"connection2"});
  auto con2 = process_->addConnection(str, &empty_callback);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  ros::init(argc, argv, "ds_process");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  testing::InitGoogleTest(&argc, argv);
  auto ret = RUN_ALL_TESTS();
  // spinner.stop();
  ros::shutdown();
  ros::waitForShutdown();
  return ret;
};
