//
// Created by waxz on 5/7/24.
//

#ifndef LIBROSCPP_MEMORY_POOL_HANDLER_HPP
#define LIBROSCPP_MEMORY_POOL_HANDLER_HPP

#include <absl/container/inlined_vector.h>
#include "tinyalloc/tinyalloc.h"

struct MemPoolHandler {

    static constexpr size_t BUFFER_SIZE = 100;
    absl::InlinedVector<void *, BUFFER_SIZE> buffer;
    ta_cfg_t cfg = {nullptr, nullptr,0,0,0};
    size_t count = 0;
    size_t max_size = BUFFER_SIZE;

    MemPoolHandler() =default;

};

#endif //LIBROSCPP_MEMORY_POOL_HANDLER_HPP
