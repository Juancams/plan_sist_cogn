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

#include <memory>

#include "plansys2_msgs/msg/action_execution_info.hpp"

#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "ca_HFSM.hpp"

using std::placeholders::_1;

class ca_HFSMImpl : public cascade_hfsm::ca_HFSM
{
public:
  ca_HFSMImpl()
  : ca_HFSM()
  {
  }
  
  void init(){
    node_aux = rclcpp::Node::make_shared("node_aux");
    executor_client_ = std::make_shared<plansys2::ExecutorClient>(node_aux);
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>(node_aux);

    init_knowledge();

  }
  void init_knowledge()
  {
    problem_expert_->addInstance(plansys2::Instance{"r2d2", "robot"});
   
    problem_expert_->addInstance(plansys2::Instance{"bedroom2", "room"});
    problem_expert_->addInstance(plansys2::Instance{"bedroom1", "room"});
    problem_expert_->addInstance(plansys2::Instance{"bathroom1", "room"});
    problem_expert_->addInstance(plansys2::Instance{"bathroom2", "room"});
    problem_expert_->addInstance(plansys2::Instance{"kitchen", "room"});
    problem_expert_->addInstance(plansys2::Instance{"dinning_room", "room"});
    problem_expert_->addInstance(plansys2::Instance{"init", "room"});
    problem_expert_->addInstance(plansys2::Instance{"zone1", "zone"});

    problem_expert_->addInstance(plansys2::Instance{"corridor1", "corridor"});
    
    problem_expert_->addPredicate(plansys2::Predicate("(robot_at r2d2 init)"));
    problem_expert_->addPredicate(plansys2::Predicate("(robot_out_zone r2d2)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected init dinning_room)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected dinning_room init)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected corridor1 bathroom1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected bathroom1 corridor1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected corridor1 bathroom2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected bathroom2 corridor1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected bedroom1 corridor1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected corridor1 bedroom1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected bedroom2 corridor1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected corridor1 bedroom2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected dinning_room corridor1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected corridor1 dinning_room)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected dinning_room kitchen)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected kitchen dinning_room)"));
  }
  
  bool Bedroom1_2_Bathroom1() 
  {
    
    return (!executor_client_->execute_and_check_plan());

  }

  bool Dinning_room_2_Kitchen() 
  {
   
    return (!executor_client_->execute_and_check_plan());

  }

  bool Corridor_2_Bedroom2() 
  {
  
    return (!executor_client_->execute_and_check_plan());

  }

  bool Bathroom1_2_Finished() 
  {
   
    return (!executor_client_->execute_and_check_plan());

  }

  bool Bedroom2_2_Bathroom2() 
  {
    
    return (!executor_client_->execute_and_check_plan());

  }

  bool Initial_2_Dinning_room() 
  {
    return true;

  }

  bool Bathroom2_2_Bedroom1()
  {
   
    return (!executor_client_->execute_and_check_plan());

  }

  bool Kitchen_2_Corridor() 
  {
   
    return (!executor_client_->execute_and_check_plan());

  }

  void Initial_code_iterative() {}
  void Initial_code_once() 
  {

  
  }
  void Dinning_room_code_iterative() {}
  void Dinning_room_code_once() 
  {
    problem_expert_->setGoal(plansys2::Goal("(and(explored dinning_room))"));
    executor_client_->start_plan_execution();
  }
  void Kitchen_code_iterative() {}
  void Kitchen_code_once() 
  {
    RCLCPP_INFO(get_logger(), "Dinning room explored");
    problem_expert_->setGoal(plansys2::Goal("(and(explored kitchen))"));
    executor_client_->start_plan_execution();
  }
  void Corridor_code_iterative() {}
  void Corridor_code_once() 
  {
    RCLCPP_INFO(get_logger(), "Kitchen explored");
    problem_expert_->setGoal(plansys2::Goal("(and(explored corridor1))"));
    executor_client_->start_plan_execution();
  }
  void Bedroom2_code_iterative() {}
  void Bedroom2_code_once() 
  {
    RCLCPP_INFO(get_logger(), "Corridor explored");
    problem_expert_->setGoal(plansys2::Goal("(and(explored bedroom2))"));
    executor_client_->start_plan_execution();
  }
  void Bathroom2_code_iterative() {}
  void Bathroom2_code_once() 
  {
    RCLCPP_INFO(get_logger(), "Bedroom2 explored");
    problem_expert_->setGoal(plansys2::Goal("(and(explored bathroom2))"));
    executor_client_->start_plan_execution();
  }

  void Bedroom1_code_iterative() {}
  void Bedroom1_code_once() 
  {
    RCLCPP_INFO(get_logger(), "Bathroom2 explored");
    problem_expert_->setGoal(plansys2::Goal("(and(explored bedroom1))"));
    executor_client_->start_plan_execution();
  }
  void Bathroom1_code_iterative() {}
  void Bathroom1_code_once() 
  {
    RCLCPP_INFO(get_logger(), "Bedroom1 explored");
    problem_expert_->setGoal(plansys2::Goal("(and(explored bathroom1))"));
    executor_client_->start_plan_execution();
  }
  void Finished_code_iterative() {}
  void Finished_code_once() 
  {
    RCLCPP_INFO(get_logger(), "Bathroom1 explored");
  }

private:
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
  rclcpp::Node::SharedPtr node_aux;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto my_hfsm = std::make_shared<ca_HFSMImpl>();

  my_hfsm->init();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(my_hfsm->get_node_base_interface());

  my_hfsm->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  executor.spin_some();

  my_hfsm->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}