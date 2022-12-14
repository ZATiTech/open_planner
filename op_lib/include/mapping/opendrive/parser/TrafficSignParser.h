/*
 * ----------------- BEGIN LICENSE BLOCK ---------------------------------
 *
 * Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
 * de Barcelona (UAB).
 * Copyright (C) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: MIT
 *
 * ----------------- END LICENSE BLOCK -----------------------------------
 */

#pragma once

#include "mapping/opendrive/types.hpp"
#include "mapping/opendrive/pugixml.hpp"

namespace opendrive {
namespace parser {

class TrafficSignParser
{
private:
  void ParseBoxAreas(const pugi::xml_node &xmlNode, std::vector<opendrive::BoxComponent> &out_boxareas);

public:
  static void Parse(const pugi::xml_node &xmlNode, std::vector<opendrive::TrafficSign> &out_trafficsigns);
};
}
}
