// Copyright (c) 2026 WeGo Robotics. All rights reserved.
// SPDX-License-Identifier: LicenseRef-Wego-Proprietary

// Synthetic video RPC server. Its reserved DDS domains and loopback NIC are fixed here;
// it creates no robot state, sport, posture or motion channels.
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/internal/internal_idl_decl/Request_.hpp>
#include <unitree/robot/internal/internal_idl_decl/Response_.hpp>

namespace
{
volatile std::sig_atomic_t running = 1;
void stop(int) {running = 0;}
using Request = unitree_api::msg::dds_::Request_;
using Response = unitree_api::msg::dds_::Response_;

std::vector<uint8_t> read_bytes(const std::string & path)
{
  std::ifstream file(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

class VideoService
{
public:
  VideoService(const std::string & side, const std::string & directory)
  : control_(directory + "/" + side + ".control"),
    live_(directory + "/" + side + ".jpg"),
    good_(read_bytes(directory + "/" + side + ".jpg")),
    wrong_size_(read_bytes(directory + "/wrong.jpg")),
    responses_("rt/api/" + side + "_videohub/response"),
    requests_("rt/api/" + side + "_videohub/request")
  {
    if (good_.empty() || wrong_size_.empty()) {throw std::runtime_error("missing test JPEG");}
    responses_.InitChannel();
    requests_.InitChannel([this](const void * raw) {
        receive(*static_cast<const Request *>(raw));
        }, 8);
  }

  ~VideoService() {requests_.CloseChannel();}
  std::atomic<unsigned> samples{0}, unexpected{0};

private:
  void receive(const Request & request)
  {
    Response response;
    response.header().identity() = request.header().identity();
    response.header().status().code() = 0;
    const auto api = request.header().identity().api_id();
    if (api == 1) {
      response.data() = "1.0.0.0";
    } else if (api == 2) {
      // SDK internal NOOP handshake, not a video sample or control request.
    } else if (api == 1001) {
      ++samples;
      std::string mode;
      std::ifstream(control_) >> mode;
      if (mode == "silence") {return;}
      if (mode == "good") {
        response.binary() = good_;
      } else if (mode == "live") {
        response.binary() = read_bytes(live_);
      } else if (mode == "mismatch") {
        response.binary() = wrong_size_;
      } else if (mode == "corrupt") {
        response.binary() = {0x01, 0x02, 0x03, 0x04};
      } else if (mode == "error") {
        response.header().status().code() = 1234;
      } else if (mode != "empty") {
        ++unexpected;
        response.header().status().code() = 1235;
      }
    } else {
      ++unexpected;
      std::cout << "UNEXPECTED_VIDEO_API=" << api << std::endl;
      response.header().status().code() = 1236;
    }
    responses_.Write(response);
  }

  std::string control_, live_;
  std::vector<uint8_t> good_, wrong_size_;
  unitree::robot::ChannelPublisher<Response> responses_;
  unitree::robot::ChannelSubscriber<Request> requests_;
};
}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 2 && argc != 3) {return 2;}
  int domain = 112;
  if (argc == 3) {
    const std::string requested(argv[2]);
    if (requested != "112" && requested != "113" && requested != "114") {return 2;}
    domain = requested == "114" ? 114 : requested == "113" ? 113 : 112;
  }
  std::signal(SIGINT, stop);
  std::signal(SIGTERM, stop);
  unitree::robot::ChannelFactory::Instance()->Init(domain, "lo");
  VideoService front("front", argv[1]), back("back", argv[1]);
  std::cout << "LOOPBACK_VIDEO_READY domain=" << domain << std::endl;
  while (running) {std::this_thread::sleep_for(std::chrono::milliseconds(20));}
  const auto unexpected = front.unexpected.load() + back.unexpected.load();
  std::cout << "VIDEO_REQUESTS front=" << front.samples.load() << " back=" << back.samples.load()
            << " unexpected=" << unexpected << std::endl;
  return unexpected == 0 ? 0 : 3;
}
