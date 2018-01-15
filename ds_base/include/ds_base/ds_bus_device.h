//
// Created by ivaughn on 1/15/18.
//

#ifndef PROJECT_DS_BUS_DEVICE_H
#define PROJECT_DS_BUS_DEVICE_H

#include <string>
#include <memory>
#include "ds_process.h"

namespace ds_base {

    class DsBusDevice :  public ds_base::DsProcess {
    protected:
        // Forward declaration of implementation details class
        struct Impl;

    public:
        explicit DsBusDevice();
        DsBusDevice(int argc, char* argv[], const std::string &name);

    protected:
        // protected constructors so we can subclass THIS class
        explicit DsBusDevice(std::unique_ptr<Impl> impl);
        DsBusDevice(std::unique_ptr<Impl> impl, int argc, char* argv[], const std::string& name);

    private:
        // functions to access our implementation structure
        auto d_func() noexcept -> Impl *;
        auto d_func() const noexcept -> Impl const *;
    };

} // namespace ds_base

#endif //PROJECT_DS_BUS_DEVICE_H
