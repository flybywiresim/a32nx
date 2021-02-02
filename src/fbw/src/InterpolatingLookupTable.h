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

#include <utility>
#include <vector>

class InterpolatingLookupTable {
 public:
  void initialize(std::vector<std::pair<double, double>> mapping, double minimum, double maximum);

  double get(double value);

 private:
  std::vector<std::pair<double, double>> mappingTable;
  double mappingMinimum = 0;
  double mappingMaximum = 0;
};
