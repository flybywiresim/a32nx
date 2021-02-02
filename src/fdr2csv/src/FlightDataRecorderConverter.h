#pragma once

#include <fstream>

#include "FlyByWire_types.h"

class FlightDataRecorderConverter {
 public:
  FlightDataRecorderConverter() = delete;
  ~FlightDataRecorderConverter() = delete;

  static void writeHeader(std::ofstream& out, const std::string& delimiter);
  static void writeStruct(std::ofstream& out, const std::string& delimiter, const fbw_output& data);
};
