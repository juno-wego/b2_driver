// Copyright (c) 2026 WeGo Robotics. All rights reserved.
// SPDX-License-Identifier: LicenseRef-Wego-Proprietary
#include <chrono>
#include <cmath>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <b2_interface/srv/set_motion_mode.hpp>

namespace {
using Trigger=std_srvs::srv::Trigger;
using Mode=b2_interface::srv::SetMotionMode;
using Service=rclcpp::Service<Trigger>;
using Clock=std::chrono::steady_clock;
class PostureAdapter final:public rclcpp::Node {
public:
  PostureAdapter():Node("b2_posture_adapter") {
    rcl_interfaces::msg::ParameterDescriptor descriptor; descriptor.read_only=true;
    timeout_=declare_parameter("response_timeout",5.,descriptor);
    if (!std::isfinite(timeout_) || timeout_<=0) {throw std::invalid_argument("response_timeout must be finite and positive");}
    client_=create_client<Mode>("/b2/motion/set_mode");
    const std::vector<std::pair<std::string,std::uint8_t>> modes{
      {"stand_up",Mode::Request::STAND_UP},{"stand_down",Mode::Request::STAND_DOWN},
      {"balance_stand",Mode::Request::BALANCE_STAND},{"recovery_stand",Mode::Request::RECOVERY_STAND},
      {"damp",Mode::Request::DAMP},{"/b2/stairs/enter",Mode::Request::VISION_WALK_ON},
      {"/b2/stairs/exit",Mode::Request::CLASSIC_WALK_ON}};
    for (const auto & [name,mode]:modes) {
      services_.push_back(create_service<Trigger>(name,[this,mode](Service::SharedPtr service,
        std::shared_ptr<rmw_request_id_t> header,std::shared_ptr<Trigger::Request>) {request(service,header,mode);}));
    }
    timer_=create_wall_timer(std::chrono::milliseconds(50),[this] {
      if (pending_ && !pending_->replied && std::chrono::duration<double>(Clock::now()-pending_->started).count()>=timeout_) {
        reply(pending_->service,pending_->header,false,"B2 SDK response timed out; command outcome is unknown");
        pending_->replied=true;  // Retain the latch until the actual SDK response.
      }
    });
  }
private:
  struct Pending {Service::SharedPtr service; std::shared_ptr<rmw_request_id_t> header; Clock::time_point started; bool replied{false};};
  static void reply(const Service::SharedPtr & service,const std::shared_ptr<rmw_request_id_t> & header,bool okay,const std::string & text) {
    Trigger::Response result; result.success=okay; result.message=text; service->send_response(*header,result);
  }
  void request(const Service::SharedPtr & service,const std::shared_ptr<rmw_request_id_t> & header,std::uint8_t mode) {
    if (pending_) {reply(service,header,false,"A posture request is still pending; no second command was sent"); return;}
    if (!client_->service_is_ready()) {reply(service,header,false,"B2 SDK motion service is unavailable"); return;}
    pending_=Pending{service,header,Clock::now(),false}; auto request=std::make_shared<Mode::Request>(); request->mode=mode;
    try {
      client_->async_send_request(request,[this](rclcpp::Client<Mode>::SharedFuture future) {
        bool okay=false; std::string message;
        try {const auto result=future.get(); okay=result->success && result->result_code==0;
          message=result->message+" (SDK result "+std::to_string(result->result_code)+")";}
        catch (const std::exception & e) {message=std::string("B2 SDK request failed: ")+e.what();}
        if (pending_ && !pending_->replied) {reply(pending_->service,pending_->header,okay,message);}
        pending_.reset();
      });
    } catch (const std::exception & e) {pending_.reset(); reply(service,header,false,e.what());}
  }
  double timeout_{5}; std::optional<Pending> pending_; rclcpp::Client<Mode>::SharedPtr client_;
  std::vector<Service::SharedPtr> services_; rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<PostureAdapter>());}
  catch (const std::exception & e) {RCLCPP_FATAL(rclcpp::get_logger("b2_posture_adapter"),"%s",e.what()); rclcpp::shutdown(); return 1;}
  rclcpp::shutdown(); return 0;
}
