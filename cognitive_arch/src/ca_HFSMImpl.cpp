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
    executor_client_ = std::make_shared<plansys2::ExecutorClient>(shared_from_this());
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>(shared_from_this());
  }
  
  bool Bedroom1_2_Bathroom1() 
  {
    return ((now() - state_ts_).seconds() > 5.0);
  }

  bool Dinning_room_2_Kitchen() 
  {
    return ((now() - state_ts_).seconds() > 5.0);
  }

  bool Corridor_2_Bedroom2() 
  {
    return ((now() - state_ts_).seconds() > 5.0);
  }

  bool Bathroom1_2_Finished() 
  {
    return ((now() - state_ts_).seconds() > 5.0);
  }

  bool Bedroom2_2_Bathroom2() 
  {
    return ((now() - state_ts_).seconds() > 5.0);
  }

  bool Initial_2_Dinning_room() 
  {
    return ((now() - state_ts_).seconds() > 5.0);
  }

  bool Bathroom2_2_Bedroom1()
  {
    return ((now() - state_ts_).seconds() > 5.0);
  }

  bool Kitchen_2_Corridor() 
  {
    return ((now() - state_ts_).seconds() > 5.0);
  }

  void Initial_code_iterative() {}
  void Initial_code_once() 
  {
    problem_expert_->addInstance(plansys2::Instance{"r2d2", "robot"});
   
    problem_expert_->addInstance(plansys2::Instance{"bedroom2", "room"});
    problem_expert_->addInstance(plansys2::Instance{"bedroom1", "room"});
    problem_expert_->addInstance(plansys2::Instance{"bathroom1", "room"});
    problem_expert_->addInstance(plansys2::Instance{"bathroom2", "room"});
    problem_expert_->addInstance(plansys2::Instance{"kitchen", "room"});
    problem_expert_->addInstance(plansys2::Instance{"dinning_room", "room"});

    problem_expert_->addInstance(plansys2::Instance{"corridor1", "corridor"});
    
    problem_expert_->addPredicate(plansys2::Predicate("(robot_at r2d2 init)"));
    problem_expert_->addPredicate(plansys2::Predicate("(robot_at r2d2 dinning_room)"));

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
  void Dinning_room_code_iterative() {}
  void Dinning_room_code_once() 
  {
    problem_expert_->setGoal(plansys2::Goal("(and(explored dinning_room))"));
    executor_client_->start_plan_execution();
  }
  void Kitchen_code_iterative() {}
  void Kitchen_code_once() 
  {
    problem_expert_->setGoal(plansys2::Goal("(and(explored kitchen))"));
    executor_client_->start_plan_execution();
  }
  void Corridor_code_iterative() {}
  void Corridor_code_once() 
  {
    problem_expert_->setGoal(plansys2::Goal("(and(explored corridor))"));
    executor_client_->start_plan_execution();
  }
  void Bedroom2_code_iterative() {}
  void Bedroom2_code_once() 
  {
    problem_expert_->setGoal(plansys2::Goal("(and(explored bedroom2))"));
    executor_client_->start_plan_execution();
  }
  void Bathroom2_code_iterative() {}
  void Bathroom2_code_once() 
  {
    problem_expert_->setGoal(plansys2::Goal("(and(explored bathroom2))"));
    executor_client_->start_plan_execution();
  }

  void Bedroom1_code_iterative() {}
  void Bedroom1_code_once() 
  {
    problem_expert_->setGoal(plansys2::Goal("(and(explored bedroom1))"));
    executor_client_->start_plan_execution();
  }
  void Bathroom1_code_iterative() {}
  void Bathroom1_code_once() 
  {
    problem_expert_->setGoal(plansys2::Goal("(and(explored bathroom1))"));
    executor_client_->start_plan_execution();
  }
  void Finished_code_iterative() {}
  void Finished_code_once() 
  {

  }

private:
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto my_hfsm = std::make_shared<ca_HFSMImpl>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(my_hfsm->get_node_base_interface());

  my_hfsm->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  executor.spin_some();

  my_hfsm->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}