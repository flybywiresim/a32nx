#include <filesystem>
#include <iostream>

#include "CommandLine.hpp"
#include "FlightDataRecorderConverter.h"
#include "FlyByWire_types.h"
#include "zfstream.h"

using namespace std;

int main(int argc, char* argv[]) {
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
  } catch (runtime_error const& e) {
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
  istream* in;
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
  fbw_output data = {};

  // read one struct from the file
  while (!in->eof()) {
    // read data into struct
    in->read(reinterpret_cast<char*>(&data), sizeof(fbw_output));
    // write struct to csv file
    FlightDataRecorderConverter::writeStruct(out, delimiter, data);
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
