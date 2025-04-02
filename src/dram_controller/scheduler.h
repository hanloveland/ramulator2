#ifndef RAMULATOR_CONTROLLER_SCHEDULER_H
#define RAMULATOR_CONTROLLER_SCHEDULER_H

#include <vector>

#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include "base/base.h"

namespace Ramulator {

class IScheduler {
  RAMULATOR_REGISTER_INTERFACE(IScheduler, "Scheduler", "Memory Controller Request Scheduler");
  public:
    virtual ReqBuffer::iterator compare(ReqBuffer::iterator req1, ReqBuffer::iterator req2, bool req1_ready, bool req2_ready) = 0;

    virtual ReqBuffer::iterator get_best_request(ReqBuffer& buffer) = 0;

    virtual ReqBuffer::iterator get_best_request_refresh_ch(ReqBuffer& buffer) = 0;

    virtual ReqBuffer::iterator get_best_request_with_mask(ReqBuffer& buffer, std::vector<bool>& mask_array) = 0;

    virtual ReqBuffer::iterator get_best_request_prefetch_with_mask(ReqBuffer& buffer, std::vector<bool>& mask_array,bool is_refreshing,bool is_wr_mode) = 0;

    virtual ReqBuffer::iterator get_best_request_refresh_ch_with_mask(ReqBuffer& buffer, std::vector<bool>& mask_array) = 0;

    virtual ReqBuffer::iterator get_best_pre_request(ReqBuffer& buffer) = 0;
};

}       // namespace Ramulator

#endif  // RAMULATOR_CONTROLLER_RAMULATOR_CONTROLLER_SCHEDULER_H_H