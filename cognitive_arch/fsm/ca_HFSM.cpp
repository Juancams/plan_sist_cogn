// NOLINT (legal/copyright)

#include "ca_HFSM.hpp"

namespace cascade_hfsm
{
ca_HFSM::ca_HFSM()
: CascadeLifecycleNode("ca_HFSM"), state_(INITIAL), myBaseId_("ca_HFSM")
{
  declare_parameter("frequency");

  state_ts_ = now();
  state_pub_ = create_publisher<std_msgs::msg::String>("/" + myBaseId_ + "/state", 1);
}

ca_HFSM::~ca_HFSM()
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ca_HFSM::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  deactivateAllDeps();

  state_ = INITIAL;
  state_ts_ = now();

  Initial_activateDeps();
  Initial_code_once();


  double frequency = 5.0;
  get_parameter_or<double>("frequency", frequency, 5.0);

  loop_timer_ = create_wall_timer(
    std::chrono::duration<double, std::ratio<1>>(1.0 / frequency),
    std::bind(&ca_HFSM::tick, this));

  state_pub_->on_activate();
  return CascadeLifecycleNode::on_activate(previous_state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ca_HFSM::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  loop_timer_ = nullptr;

  return CascadeLifecycleNode::on_deactivate(previous_state);
}

void ca_HFSM::tick()
{
  std_msgs::msg::String msg;

  switch (state_) {
    case BEDROOM2:
      Bedroom2_code_iterative();

      msg.data = "Bedroom2";
      state_pub_->publish(msg);

      if (Bedroom2_2_Bathroom2()) {
        deactivateAllDeps();

        state_ = BATHROOM2;
        state_ts_ = now();

        Bathroom2_activateDeps();
        Bathroom2_code_once();
      }
      break;
    case CORRIDOR:
      Corridor_code_iterative();

      msg.data = "Corridor";
      state_pub_->publish(msg);

      if (Corridor_2_Bedroom2()) {
        deactivateAllDeps();

        state_ = BEDROOM2;
        state_ts_ = now();

        Bedroom2_activateDeps();
        Bedroom2_code_once();
      }
      break;
    case BATHROOM2:
      Bathroom2_code_iterative();

      msg.data = "Bathroom2";
      state_pub_->publish(msg);

      if (Bathroom2_2_Bedroom1()) {
        deactivateAllDeps();

        state_ = BEDROOM1;
        state_ts_ = now();

        Bedroom1_activateDeps();
        Bedroom1_code_once();
      }
      break;
    case FINISHED:
      Finished_code_iterative();

      msg.data = "Finished";
      state_pub_->publish(msg);

      break;
    case BEDROOM1:
      Bedroom1_code_iterative();

      msg.data = "Bedroom1";
      state_pub_->publish(msg);

      if (Bedroom1_2_Bathroom1()) {
        deactivateAllDeps();

        state_ = BATHROOM1;
        state_ts_ = now();

        Bathroom1_activateDeps();
        Bathroom1_code_once();
      }
      break;
    case KITCHEN:
      Kitchen_code_iterative();

      msg.data = "Kitchen";
      state_pub_->publish(msg);

      if (Kitchen_2_Corridor()) {
        deactivateAllDeps();

        state_ = CORRIDOR;
        state_ts_ = now();

        Corridor_activateDeps();
        Corridor_code_once();
      }
      break;
    case DINNING_ROOM:
      Dinning_room_code_iterative();

      msg.data = "Dinning_room";
      state_pub_->publish(msg);

      if (Dinning_room_2_Kitchen()) {
        deactivateAllDeps();

        state_ = KITCHEN;
        state_ts_ = now();

        Kitchen_activateDeps();
        Kitchen_code_once();
      }
      break;
    case INITIAL:
      Initial_code_iterative();

      msg.data = "Initial";
      state_pub_->publish(msg);

      if (Initial_2_Dinning_room()) {
        deactivateAllDeps();

        state_ = DINNING_ROOM;
        state_ts_ = now();

        Dinning_room_activateDeps();
        Dinning_room_code_once();
      }
      break;
    case BATHROOM1:
      Bathroom1_code_iterative();

      msg.data = "Bathroom1";
      state_pub_->publish(msg);

      if (Bathroom1_2_Finished()) {
        deactivateAllDeps();

        state_ = FINISHED;
        state_ts_ = now();

        Finished_activateDeps();
        Finished_code_once();
      }
      break;
  }
}

void
ca_HFSM::deactivateAllDeps()
{
}

void
ca_HFSM::Bedroom2_activateDeps()
{
}
void
ca_HFSM::Corridor_activateDeps()
{
}
void
ca_HFSM::Bathroom2_activateDeps()
{
}
void
ca_HFSM::Finished_activateDeps()
{
}
void
ca_HFSM::Bedroom1_activateDeps()
{
}
void
ca_HFSM::Kitchen_activateDeps()
{
}
void
ca_HFSM::Dinning_room_activateDeps()
{
}
void
ca_HFSM::Initial_activateDeps()
{
}
void
ca_HFSM::Bathroom1_activateDeps()
{
}


}  // namespace cascade_hfsm
