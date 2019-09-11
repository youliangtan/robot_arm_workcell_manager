/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gazebo/msgs/world_control.pb.h>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/TransportIface.hh>

#include <string>
#include <thread>

namespace {
struct Args
{
  // Delay before starting the warmup
  double delay = 1.0; // in seconds

  // Time steps to take during warm-up phase
  std::size_t steps = 10;

  // Time to wait between warm-up steps
  double wait = 0.2; // in seconds
};

template<typename T>
void attempt_parse(
    T& result,
    const std::string& key, const std::string& type,
    const int i, const int argc, char* argv[],
    const std::function<T(const std::string&)>& parser)
{
  if(i+1 < argc)
  {
    const std::string arg = std::string(argv[i+1]);
    try
    {
      result = parser(arg);
    }
    catch(const std::invalid_argument& e)
    {
      std::cerr << "[warm_start] Value for [" << key << "] could not be "
                << "parsed into a " << type << " [" << e.what() << "]: " << arg
                << "\n -- The default [" << result
                << "] will be used instead" << std::endl;
    }
  }
  else
  {
    std::cerr << "[warm_start] No value given after argument [" << key << "]"
              << std::endl;
  }
}

Args parse_args(int argc, char* argv[])
{
  Args args;

  const auto uint_parse =
      [](const std::string& s) -> std::size_t { return std::stoul(s); };

  const auto double_parse =
      [](const std::string& s) -> double { return std::stod(s); };

  for(int i=0; i < argc; ++i)
  {
    const std::string arg = argv[i];
    if(arg == "-d")
    {
      attempt_parse<double>(
            args.delay, "-d", "floating point", i, argc, argv, double_parse);
    }
    else if(arg == "-i")
    {
      attempt_parse<std::size_t>(
            args.steps, "-i", "positive integer", i, argc, argv, uint_parse);
    }
    else if(arg == "-w")
    {
      attempt_parse<double>(
            args.wait, "-w", "floating point", i, argc, argv, double_parse);
    }
  }

  return args;
}

} // anonymous namespace

int main(int argc, char* argv[])
{
  const Args args = parse_args(argc, argv);

  std::cout << "[warm_start] Waiting " << args.delay << "s before talking to "
            << "gazebo" << std::endl;
  std::this_thread::sleep_for(std::chrono::duration<double>(args.delay));

  if(!gazebo::transport::init("", 0, 10))
  {
    std::cerr << "[warm_start] Failed to connect to the Gazebo Master. "
              << "Is gazebo running?" << std::endl;
    return 1;
  }

  gazebo::transport::run();

  const double timeout = 10.0;
  gazebo::transport::NodePtr node(new gazebo::transport::Node);
  if(!node->TryInit(gazebo::common::Time(timeout)))
  {
    std::cerr << "[warm_start] Failed to initialize gazebo transport after "
              << timeout << " seconds. Is gazebo hanging during load?"
              << std::endl;
    return 1;
  }

  const std::string topic_name = "~/world_control";
  gazebo::transport::PublisherPtr publisher =
      node->Advertise<gazebo::msgs::WorldControl>(topic_name);

  std::cout << "waiting for connection after advertising" << std::endl;
  if(!publisher->WaitForConnection(gazebo::common::Time(timeout)))
  {
    std::cerr << "[warm_start] Failed to connect to the [" << topic_name
              <<  "] topic after " << timeout << " seconds. Is gazebo okay??"
              << std::endl;
    return 1;
  }

  gazebo::msgs::WorldControl msg;
  msg.set_pause(true);
  msg.set_step(true);

  std::cout << "[warm_start] Warming up gazebo (" << args.steps << " steps "
            << "with " << args.wait << "s delays)..." << std::endl;
  for(std::size_t i=0; i < args.steps; ++i)
  {
    publisher->Publish(msg, true);

    using namespace std::chrono_literals;
    std::this_thread::sleep_for(std::chrono::duration<double>(args.wait));
  }

  // Hopefully by now the controllers from controller_manager should be active,
  // and it should be safe to run the live demo.
  msg.set_pause(false);
  msg.set_step(false);
  publisher->Publish(msg);

  std::cout << "[warm_start] Finished warming" << std::endl;
}
