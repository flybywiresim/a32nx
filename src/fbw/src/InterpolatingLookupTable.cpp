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

#include "InterpolatingLookupTable.h"

using namespace std;

void InterpolatingLookupTable::initialize(vector<pair<double, double>> mapping, double minimum, double maximum) {
  mappingTable = std::move(mapping);
  mappingMinimum = minimum;
  mappingMaximum = maximum;
}

double InterpolatingLookupTable::get(double value) {
  if (mappingTable.empty()) {
    // not initialized yet
    return 0;
  }

  // iterate over values and do interpolation
  for (std::size_t i = 0; i < mappingTable.size() - 1; ++i) {
    if (mappingTable[i].first <= value && mappingTable[i + 1].first >= value) {
      // calculate differences
      double diff_x = value - mappingTable[i].first;
      double diff_n = mappingTable[i + 1].first - mappingTable[i].first;

      // interpolation
      double result = mappingTable[i].second + (mappingTable[i + 1].second - mappingTable[i].second) * diff_x / diff_n;

      // clip the result to minimum and maximum
      if (result < mappingMinimum) {
        return mappingMinimum;
      } else if (result > mappingMaximum) {
        return mappingMaximum;
      }

      // no clipping needed -> return result
      return result;
    }
  }

  // not in range
  return 0;
}
