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

#ifndef HOUSE_NAV__PLACE_HPP_
#define HOUSE_NAV__PLACE_HPP_

#include <memory>
#include <algorithm>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace house_nav
{


class Place : public plansys2::ActionExecutorClient
{
public:
  Place(const std::string & name, const std::chrono::nanoseconds & rate);

protected:
  virtual void do_work();
  float progress_;
};

}  // namespace house_nav

#endif  // HOUSE_NAV__PLACE_HPP_