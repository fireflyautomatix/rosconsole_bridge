/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */

#include <map>

#include <rcutils/logging.h>
#include "rosconsole_bridge/bridge.h"

namespace rosconsole_bridge
{

OutputHandlerROS::OutputHandlerROS(void)
: OutputHandler()
{
}

void OutputHandlerROS::log(
  const std::string & text, console_bridge::LogLevel level,
  const char * filename, int line)
{
  static const std::map<console_bridge::LogLevel, RCUTILS_LOG_SEVERITY> level_mapping{
    {console_bridge::CONSOLE_BRIDGE_LOG_DEBUG, RCUTILS_LOG_SEVERITY_DEBUG},
    {console_bridge::CONSOLE_BRIDGE_LOG_INFO, RCUTILS_LOG_SEVERITY_INFO},
    {console_bridge::CONSOLE_BRIDGE_LOG_WARN, RCUTILS_LOG_SEVERITY_WARN},
    {console_bridge::CONSOLE_BRIDGE_LOG_ERROR, RCUTILS_LOG_SEVERITY_ERROR},
    {console_bridge::CONSOLE_BRIDGE_LOG_NONE, RCUTILS_LOG_SEVERITY_UNSET}
  };

  rcutils_log_location_t location = {"", filename, static_cast<size_t>(line)};

  rcutils_log(&location, level_mapping.at(level), nullptr, "%s", text.c_str());
}

RegisterOutputHandlerProxy::RegisterOutputHandlerProxy(void)
{
  static OutputHandlerROS oh_ros;
  console_bridge::useOutputHandler(&oh_ros);

  // we want the output level to be decided by rosconsole, so we bring all messages to rosconsole
  console_bridge::setLogLevel(console_bridge::CONSOLE_BRIDGE_LOG_DEBUG);
}

RegisterOutputHandlerProxy::~RegisterOutputHandlerProxy()
{
  console_bridge::restorePreviousOutputHandler();
}

}
