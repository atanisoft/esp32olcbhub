/** \copyright
 * Copyright (c) 2020, Mike Dunston
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are  permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \file HealthMonitor.hxx
 *
 * Provides a periodic health report.
 *
 * @author Mike Dunston
 * @date 14 July 2020
 */

#ifndef HEARTBEAT_LED_H_
#define HEARTBEAT_LED_H_

#include <esp_log.h>
#include <executor/Service.hxx>
#include <executor/StateFlow.hxx>
#include <openlcb/SimpleStack.hxx>
#include <utils/logging.h>

#include "hardware.hxx"

namespace esp32olcbhub
{

/// Utility class providing periodic reporting of general health of the
/// Esp32OlcbHub.
class HealthMonitor : public StateFlowBase
{
public:
    /// Constructor.
    ///
    /// @param service is the @ref Service to attach this flow to.
    HealthMonitor(openlcb::SimpleCanStackBase *stack)
                : StateFlowBase(stack->service()), stack_(stack)
    {
        start_flow(STATE(update));
    }

    /// Stops the flow and cancels the timer (if needed).
    void stop()
    {
        shutdown_ = true;
        set_terminated();
        timer_.ensure_triggered();
    }
private:
    /// @ref SimpleCanStackBase that is currently in use.
    openlcb::SimpleCanStackBase *stack_;

    /// @ref StateFlowTimer used for periodic wakeup.
    StateFlowTimer timer_{this};

    /// Interval at which to wake up.
    const uint64_t reportInterval_{SEC_TO_NSEC(30)};

    /// Internal flag to track if a shutdown request has been requested.
    bool shutdown_{false};

    /// Wakes up and blinks the heartbeat LED and prints general health when
    /// the required count of wakeups has expired.
    Action update()
    {
        if (shutdown_)
        {
            return exit();
        }

        UBaseType_t taskCount = uxTaskGetNumberOfTasks();
        unsigned hub_clients = 0;
        if (stack_->get_tcp_hub_server())
        {
            hub_clients = stack_->get_tcp_hub_server()->get_num_clients();
        }
        LOG(INFO, "%s: Free heap: %.2fkB (max block size: %.2fkB), "
#if CONFIG_SPIRAM_SUPPORT
                  "Free PSRAM: %.2fkB (max block size: %.2fkB), "
#endif // CONFIG_SPIRAM_SUPPORT
                  "mainBufferPool: %.2fkB, Hub clients: %u, FreeRTOS tasks: %d"
          , esp_log_system_timestamp()
          , heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024.0f
          , heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) / 1024.0f
#if CONFIG_SPIRAM_SUPPORT
          , heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024.0f
          , heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM) / 1024.0f
#endif // CONFIG_SPIRAM_SUPPORT
          , mainBufferPool->total_size() / 1024.0f, hub_clients, taskCount);
        return sleep_and_call(&timer_, reportInterval_, STATE(update));
    }
};

} // namespace esp32s2io

#endif // HEARTBEAT_LED_H_