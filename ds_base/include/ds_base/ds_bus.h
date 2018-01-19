//
// Created by ivaughn on 1/15/18.
//

#ifndef PROJECT_DS_IOSM_PROCESS_H
#define PROJECT_DS_IOSM_PROCESS_H

#include <string>
#include <memory>
#include "ds_process.h"

namespace ds_base {

    class DsBus :  public ds_base::DsProcess {
    protected:
        // Forward declaration of implementation details class
        struct Impl;

    public:
        explicit DsBus();
        DsBus(int argc, char* argv[], const std::string &name);

    protected:
        // protected constructors so we can subclass THIS class
        explicit DsBus(std::unique_ptr<Impl> impl);
        DsBus(std::unique_ptr<Impl> impl, int argc, char* argv[], const std::string& name);

      void setupConnections() override;
      void setupPublishers() override;
      void checkProcessStatus(const ros::TimerEvent &event) override;
      void setupParameters() override;

    private:
        // functions to access our implementation structure
        auto d_func() noexcept -> Impl *;
        auto d_func() const noexcept -> Impl const *;
    };

} // namespace ds_base


#endif //PROJECT_DS_IOSM_PROCESS_H

