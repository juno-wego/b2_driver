// Copyright (c) 2026 WeGo Robotics. All rights reserved.
// SPDX-License-Identifier: LicenseRef-Wego-Proprietary
#pragma once

#include <atomic>
#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>

namespace b2_base
{
// SDK calls can wait for a remote reply. Keep those waits off the executor
// responsible for state publication, cmd_vel receipt and the fast watchdog.
// Callback groups must be created with automatically_add_to_executor=false.
class SdkCallbackExecutors
{
public:
  SdkCallbackExecutors(
    const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr &node,
    const rclcpp::CallbackGroup::SharedPtr &commands,
    const rclcpp::CallbackGroup::SharedPtr &modes,
    const rclcpp::CallbackGroup::SharedPtr &stops)
  : context_(node->get_context()), command_(options(context_)), operations_(options(context_), 2)
  {
    command_.add_callback_group(commands, node);
    operations_.add_callback_group(modes, node);
    operations_.add_callback_group(stops, node);
    try {
      command_thread_ = std::thread([this] {spin(command_, command_finished_);});
      operation_thread_ = std::thread([this] {spin(operations_, operations_finished_);});
    } catch (...) {stop(); throw;}
  }

  ~SdkCallbackExecutors() {stop();}
  SdkCallbackExecutors(const SdkCallbackExecutors &) = delete;
  SdkCallbackExecutors &operator=(const SdkCallbackExecutors &) = delete;

private:
  static rclcpp::ExecutorOptions options(const rclcpp::Context::SharedPtr &context)
  {
    rclcpp::ExecutorOptions result;
    result.context = context;
    return result;
  }

  void spin(rclcpp::Executor &executor, std::atomic<bool> &finished)
  {
    try {executor.spin();}
    catch (const std::exception &error) {
      RCLCPP_FATAL(rclcpp::get_logger("b2_sdk_io"), "SDK executor failed: %s", error.what());
      rclcpp::shutdown(context_, "B2 SDK callback executor failed");
    }
    finished.store(true, std::memory_order_release);
  }

  static void cancel(rclcpp::Executor &executor, const std::thread &worker,
    const std::atomic<bool> &finished) noexcept
  {
    if (!worker.joinable()) return;
    // cancel() before spin() starts is not a persistent stop request. An
    // immediately destroyed node must not leave a late-starting thread spinning.
    while (!finished.load(std::memory_order_acquire) && !executor.is_spinning()) {
      std::this_thread::yield();
    }
    try {executor.cancel();} catch (...) {}
  }

  void stop() noexcept
  {
    // Cancel both before joining either. Running SDK calls retain their normal
    // bounded timeout and finish before their clients/node members are destroyed.
    cancel(command_, command_thread_, command_finished_);
    cancel(operations_, operation_thread_, operations_finished_);
    if (command_thread_.joinable()) command_thread_.join();
    if (operation_thread_.joinable()) operation_thread_.join();
  }

  rclcpp::Context::SharedPtr context_;
  rclcpp::executors::SingleThreadedExecutor command_;
  rclcpp::executors::MultiThreadedExecutor operations_;
  std::atomic<bool> command_finished_{false}, operations_finished_{false};
  std::thread command_thread_, operation_thread_;
};
}  // namespace b2_base
