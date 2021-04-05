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

#include "house_nav/Controller.hpp"

namespace house_nav 
{

  Controller::Controller(const std::string & name)
  : rclcpp::Node("house_nav_controller")
  {
  }

  void Controller::init()
  {
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>(shared_from_this());
    executor_client_ = std::make_shared<plansys2::ExecutorClient>(shared_from_this());

    init_knowledge();

    if (!executor_client_->start_plan_execution()) {
      RCLCPP_ERROR(get_logger(), "Error starting a new plan (first)");
    }
  }

  void Controller::init_knowledge()
  {
    problem_expert_->addInstance(plansys2::Instance{"r2d2", "robot"});
  problem_expert_->addInstance(plansys2::Instance{"ball", "object"});

    problem_expert_->addInstance(plansys2::Instance{"init", "room"});
    problem_expert_->addInstance(plansys2::Instance{"bedroom2", "room"});
    problem_expert_->addInstance(plansys2::Instance{"bedroom1", "room"});
    problem_expert_->addInstance(plansys2::Instance{"bathroom1", "room"});
    problem_expert_->addInstance(plansys2::Instance{"bathroom2", "room"});
    problem_expert_->addInstance(plansys2::Instance{"kitchen", "room"});
    problem_expert_->addInstance(plansys2::Instance{"dinning_room", "room"});

    problem_expert_->addInstance(plansys2::Instance{"corridor1", "corridor"});

    problem_expert_->addInstance(plansys2::Instance{"entrance", "zone"});
    problem_expert_->addInstance(plansys2::Instance{"next_to_bed", "zone"});
    problem_expert_->addInstance(plansys2::Instance{"desktop", "zone"});
    problem_expert_->addInstance(plansys2::Instance{"fridge", "zone"});
    problem_expert_->addInstance(plansys2::Instance{"sink", "zone"});
    problem_expert_->addInstance(plansys2::Instance{"tv", "zone"});
    problem_expert_->addInstance(plansys2::Instance{"table", "zone"});
    problem_expert_->addInstance(plansys2::Instance{"window", "zone"});
    problem_expert_->addInstance(plansys2::Instance{"bath", "zone"});

    problem_expert_->addPredicate(plansys2::Predicate("(robot_at r2d2 init)"));
    problem_expert_->addPredicate(plansys2::Predicate("(object_at ball dinning_room)"));
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

    problem_expert_->addPredicate(plansys2::Predicate("(zone_in_room next_to_bed bedroom1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(zone_in_room desktop bedroom1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(zone_in_room fridge kitchen)"));
    problem_expert_->addPredicate(plansys2::Predicate("(zone_in_room sink kitchen)"));
    problem_expert_->addPredicate(plansys2::Predicate("(zone_in_room tv dinning_room)"));
    problem_expert_->addPredicate(plansys2::Predicate("(zone_in_room table dinning_room)"));
    problem_expert_->addPredicate(plansys2::Predicate("(zone_in_room window dinning_room)"));
    problem_expert_->addPredicate(plansys2::Predicate("(zone_in_room bath bathroom2)"));

    problem_expert_->setGoal(
      plansys2::Goal(
        "(and(object_at ball kitchen))"));
  }

  void Controller::step()
  {
    if (!executor_client_->execute_and_check_plan()) {  // Plan finished
      auto result = executor_client_->getResult();
      auto feedback = executor_client_->getFeedBack();
      for (const auto & action_feedback : feedback.action_execution_status) {
        std::cout << "[" << action_feedback.action << " " <<
        action_feedback.completion * 100.0 << "%]";
      }
      std::cout << std::endl;
      if (result.value().success) {
        RCLCPP_INFO(get_logger(), "Plan succesfully finished");
      } else {
        RCLCPP_ERROR(get_logger(), "Plan finished with error");
      }
    }
  }

}