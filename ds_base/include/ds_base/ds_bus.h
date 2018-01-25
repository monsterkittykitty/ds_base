//
// Created by ivaughn on 1/15/18.
//

#ifndef PROJECT_DS_IOSM_PROCESS_H
#define PROJECT_DS_IOSM_PROCESS_H

#include <string>
#include <memory>
#include "ds_base/ds_process.h"

namespace ds_base
{

struct DsBusPrivate;

class DsBus : public ds_base::DsProcess
{
  // Forward declaration of implementation details class
  DS_DECLARE_PRIVATE(DsBus)

public:
  explicit DsBus();
  DsBus(int argc, char* argv[], const std::string& name);
  ~DsBus() override;
  DS_DISABLE_COPY(DsBus)

protected:
  void setupConnections() override;
  void setupPublishers() override;
  void checkProcessStatus(const ros::TimerEvent& event) override;
  void setupParameters() override;

 private:
  std::unique_ptr<DsBusPrivate> d_ptr_;
};

}  // namespace ds_base

#endif  // PROJECT_DS_IOSM_PROCESS_H
