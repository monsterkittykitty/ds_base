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
