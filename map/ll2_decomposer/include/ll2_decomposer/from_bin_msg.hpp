// Copyright 2023 TIER IV, Inc.
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

#pragma once
#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>

#include <lanelet2_core/LaneletMap.h>

namespace yabloc ::ll2_decomposer
{
lanelet::LaneletMapPtr from_bin_msg(const autoware_auto_mapping_msgs::msg::HADMapBin & msg);
}