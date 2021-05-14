#pragma once

#include <global_mapping/submap.h>

namespace global_mapping {

class GlobalMap {
    public:

    GlobalMap() = default;

    ~GlobalMap() = default;

    private:
    std::vector<Submap> submaps_;
};    

} // namespace global_mapping