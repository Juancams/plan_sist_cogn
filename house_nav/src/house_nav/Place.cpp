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

#include "house_nav/Place.hpp"

namespace house_nav
{

  Place::Place(const std::string & name, const std::chrono::nanoseconds & rate)
  : plansys2::ActionExecutorClient(name, rate)
  {
    progress_ = 0.0;
  }

  void Place::do_work()
  {
    if (progress_ < 1.0) {
      progress_ += 0.2;
      send_feedback(progress_, "Place running");
    } else {
      finish(true, 1.0, "Place completed");

      progress_ = 0.0;
      std::cout << std::endl;
    }

    std::cout << "\r\e[K" << std::flush;
    std::cout << "Placing object ... [" << std::min(100.0, progress_ * 100.0) << "%]  " <<
      std::flush;
  }
};
