#include <filesystem>
#include <iostream>

#include "CommandLine.hpp"
#include "FlightDataRecorderConverter.h"
#include "AutopilotLaws_types.h"
#include "AutopilotStateMachine_types.h"
#include "FlyByWire_types.h"
#include "zfstream.h"

using namespace std;

int main(int argc, char *argv[]) {
  // variables for command line parameters
  string inFilePath;
  string outFilePath;
  string delimiter = ",";
  bool noCompression = false;
  bool printStructSize = false;
  bool oPrintHelp = false;

  // configuration of command line parameters
  CommandLine args("Converts a32nx fdr files to csv");
  args.addArgument({"-i", "--in"}, &inFilePath, "Input File");
  args.addArgument({"-o", "--out"}, &outFilePath, "Output File");
  args.addArgument({"-d", "--delimiter"}, &delimiter, "Delimiter");
  args.addArgument({"-n", "--no-compression"}, &noCompression, "Input file is not compressed");
  args.addArgument({"-p", "--print-struct-size"}, &printStructSize, "Print struct size");
  args.addArgument({"-h", "--help"}, &oPrintHelp, "Print help message");

  // parse command line
  try {
    args.parse(argc, argv);
  } catch (runtime_error const &e) {
    cout << e.what() << endl;
    return -1;
  }

  // print help
  if (oPrintHelp) {
    args.printHelp();
    cout << endl;
    return 0;
  }

  // print size of struct
  if (printStructSize) {
    cout << "Size of struct for reading is '" << sizeof(fbw_output) << "' bytes" << endl;
  }

  // check parameters
  if (inFilePath.empty()) {
    cout << "Input file parameter missing!" << endl;
    return 1;
  }
  if (!filesystem::exists(inFilePath)) {
    cout << "Input file does not exist!" << endl;
    return 1;
  }
  if (outFilePath.empty()) {
    cout << "Output file parameter missing!" << endl;
    return 1;
  }

  // print information on convert
  cout << "Convert from '" << inFilePath;
  cout << "' to '" << outFilePath;
  cout << "' using delimiter '" << delimiter << "'" << endl;

  // output stream
  ofstream out;
  // open the output file
  out.open(outFilePath, ios::out | ios::trunc);
  // check if file is open
  if (!out.is_open()) {
    cout << "Failed to create output file!" << endl;
    return 1;
  }

  // write header
  FlightDataRecorderConverter::writeHeader(out, delimiter);

  // create input stream
  istream *in;
  if (!noCompression) {
    in = new gzifstream(inFilePath.c_str());
  } else {
    in = new ifstream(inFilePath.c_str(), ios::in | ios::binary);
  }

  // check if stream is ok
  if (!in->good()) {
    cout << "Failed to open input file!" << endl;
    return 1;
  }

  // calculate number of entries
  auto counter = 0;
  auto numberOfEntries = filesystem::file_size(inFilePath) / sizeof(fbw_output);

  // struct for reading
  ap_sm_output data_ap_sm = {};
  ap_raw_output data_ap_laws = {};
  fbw_output data_fbw = {};

  // read one struct from the file
  while (!in->eof()) {
    // read data into structs
    in->read(reinterpret_cast<char *>(&data_ap_sm), sizeof(ap_sm_output));
    in->read(reinterpret_cast<char *>(&data_ap_laws), sizeof(ap_raw_output));
    in->read(reinterpret_cast<char *>(&data_fbw), sizeof(fbw_output));
    // write struct to csv file
    FlightDataRecorderConverter::writeStruct(out, delimiter, data_ap_sm, data_ap_laws, data_fbw);
    // print progress
    if (++counter % 500 == 0) {
      cout << "Processed " << counter << " entries...";
      // return to line start
      cout << "\r";
    }
  }

  // print final value
  cout << "Processed " << counter << " entries." << endl;

  // success
  return 0;
}
