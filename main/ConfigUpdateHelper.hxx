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
 * \file DelayReboot.hxx
 *
 * Helper which triggers configuration update.
 *
 * @author Mike Dunston
 * @date 14 July 2020
 */

#ifndef CONFIG_UPDATER_HXX_
#define CONFIG_UPDATER_HXX_

#include <executor/Service.hxx>
#include <executor/StateFlow.hxx>
#include <utils/logging.h>
#include <openlcb/MemoryConfig.hxx>

namespace esp32olcbhub
{

/// Utility class that will fire off a configuration update flow.
class ConfigUpdateHelper : public Singleton<ConfigUpdateHelper>
{
public:
    /// Constructor.
    ///
    /// @param executor is the @ref ExecutorBase to use for sending the update
    /// request.
    /// @param service is the @ref ConfigUpdateService to be invoked.
    ConfigUpdateHelper(ExecutorBase *executor, ConfigUpdateService *service)
        : executor_(executor), updateService_(service)
    {
    }

    /// Queues an update request to be sent to the stack which will run as an
    /// async operation.
    void trigger_update()
    {
        executor_->add(new CallbackExecutable(
            [&]()
            {
                updateService_->trigger_update();
            }));
    }

private:
    /// @ref ExecutorBase to use for triggering the configuration update.
    ExecutorBase *executor_;

    /// @ref ConfigUpdateService to use for the update.
    ConfigUpdateService *updateService_;
};

} // namespace esp32s2io

#endif // CONFIG_UPDATER_HXX_