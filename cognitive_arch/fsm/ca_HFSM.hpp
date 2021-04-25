// Copyright 2021 The Rebooters
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef COGNITIVE_ARCH__FSM__CA_HFSM_HPP_
#define COGNITIVE_ARCH__FSM__CA_HFSM_HPP_

#include <string>

#include "std_msgs/msg/string.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

namespace cascade_hfsm
{
class ca_HFSM : public rclcpp_cascade_lifecycle::CascadeLifecycleNode
{
public:
  ca_HFSM();
  virtual ~ca_HFSM();

  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state);

  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state);

  virtual void Bedroom2_code_iterative() {}
  virtual void Bedroom2_code_once() {}
  virtual void Tv_code_iterative() {}
  virtual void Tv_code_once() {}
  virtual void Bathroom2_code_iterative() {}
  virtual void Bathroom2_code_once() {}
  virtual void Bedroom1_code_iterative() {}
  virtual void Bedroom1_code_once() {}
  virtual void Bathroom1_code_iterative() {}
  virtual void Bathroom1_code_once() {}
  virtual void Initial_code_iterative() {}
  virtual void Initial_code_once() {}
  virtual void Table_code_iterative() {}
  virtual void Table_code_once() {}
  virtual void Finished_code_iterative() {}
  virtual void Finished_code_once() {}
  virtual void Kitchen_code_iterative() {}
  virtual void Kitchen_code_once() {}

  virtual bool Bedroom1_2_Bathroom1() {return false;}
  virtual bool Tv_2_Kitchen() {return false;}
  virtual bool Bedroom2_2_Bathroom2() {return false;}
  virtual bool Initial_2_Table() {return false;}
  virtual bool Bathroom2_2_Bedroom1() {return false;}
  virtual bool Table_2_Tv() {return false;}
  virtual bool Bathroom1_2_Finished() {return false;}
  virtual bool Kitchen_2_Bedroom2() {return false;}


  void tick();

protected:
  rclcpp::Time state_ts_;

private:
  void deactivateAllDeps();
  void Bedroom2_activateDeps();
  void Tv_activateDeps();
  void Bathroom2_activateDeps();
  void Bedroom1_activateDeps();
  void Bathroom1_activateDeps();
  void Initial_activateDeps();
  void Table_activateDeps();
  void Finished_activateDeps();
  void Kitchen_activateDeps();


  static const int BEDROOM2 = 0;
  static const int TV = 1;
  static const int BATHROOM2 = 2;
  static const int BEDROOM1 = 3;
  static const int BATHROOM1 = 4;
  static const int INITIAL = 5;
  static const int TABLE = 6;
  static const int FINISHED = 7;
  static const int KITCHEN = 8;


  int state_;

  std::string myBaseId_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr loop_timer_;
};

}  // namespace cascade_hfsm

#endif  // COGNITIVE_ARCH__FSM__CA_HFSM_HPP_"
