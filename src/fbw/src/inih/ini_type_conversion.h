/*
 * A32NX
 * Copyright (C) 2020 FlyByWire Simulations and its contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <ini.h>
#include <string>

namespace mINI {
class INITypeConversion {
 public:
  INITypeConversion() = delete;

  static bool getBoolean(mINI::INIStructure structure, const std::string& section, const std::string& key, bool defaultValue = false) {
    if (!structure.has(section) || !structure.get(section).has(key)) {
      return defaultValue;
    }
    return getBooleanFromString(structure.get(section).get(key));
  }

  static double getDouble(mINI::INIStructure structure, const std::string& section, const std::string& key, double defaultValue = 0.0) {
    if (!structure.has(section) || !structure.get(section).has(key)) {
      return defaultValue;
    }

    // exceptions are not supported -> need to convert it without
    double value;
    std::stringstream stream(structure.get(section).get(key));
    stream >> value;

    // check if conversion worked
    if (stream.fail()) {
      return defaultValue;
    }
    return value;
  }

  static int getInteger(mINI::INIStructure structure, const std::string& section, const std::string& key, int defaultValue = 0) {
    if (!structure.has(section) || !structure.get(section).has(key)) {
      return defaultValue;
    }

    // exceptions are not supported -> need to convert it without
    int value;
    std::stringstream stream(structure.get(section).get(key));
    stream >> value;

    // check if conversion worked
    if (stream.fail()) {
      return defaultValue;
    }
    return value;
  }

 private:
  static bool getBooleanFromString(const std::string& value) {
    // transform to lower case string
    std::string local = value;
    transform(local.begin(), local.end(), local.begin(), ::tolower);

    if (local == "1" || local == "true" || local == "yes") {
      return true;
    } else {
      return false;
    }
  }
};
};  // namespace mINI
