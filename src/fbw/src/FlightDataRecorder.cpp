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

#include <INIReader.h>
#include <dirent.h>
#include <stdio.h>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

#include "FlightDataRecorder.h"

using namespace std;

void FlightDataRecorder::initialize() {
  // read configuration
  INIReader configuration(CONFIGURATION_FILEPATH);
  if (configuration.ParseError() < 0) {
    // file does not exist yet -> store the default configuration in a file
    ofstream configFile;
    configFile.open(CONFIGURATION_FILEPATH);
    configFile << "[FlightDataRecorder]" << endl;
    configFile << "Enabled = true" << endl;
    configFile << "MaximumNumberOfFiles = 15" << endl;
    configFile << "MaximumNumberOfEntriesPerFile = 864000" << endl;
    configFile.close();
  }

  // read basic configuration
  isEnabled = configuration.GetBoolean("FlightDataRecorder", "Enabled", true);
  maximumFileCount = configuration.GetInteger("FlightDataRecorder", "MaximumNumberOfFiles", 15);
  maximumSampleCounter = configuration.GetInteger("FlightDataRecorder", "MaximumNumberOfEntriesPerFile", 864000);

  // print configuration
  cout << "WASM: Flight Data Recorder Configuration : Enabled                  "
          "      = "
       << isEnabled << endl;
  cout << "WASM: Flight Data Recorder Configuration : MaximumNumberOfFiles     "
          "      = "
       << maximumFileCount << endl;
  cout << "WASM: Flight Data Recorder Configuration : "
          "MaximumNumberOfEntriesPerFile  = "
       << maximumSampleCounter << endl;
}

void FlightDataRecorder::update(FlyByWireModelClass* model) {
  // check if enabled
  if (!isEnabled) {
    return;
  }

  // do file management
  manageFlightDataRecorderFiles();

  // write data to file
  fileStream->write(reinterpret_cast<char*>(&model->FlyByWire_Y.out), sizeof(model->FlyByWire_Y.out));
}

void FlightDataRecorder::terminate() {
  if (fileStream != nullptr) {
    fileStream->close();
    fileStream = nullptr;
    delete fileStream;
  }
}

void FlightDataRecorder::manageFlightDataRecorderFiles() {
  // increase sample counter
  sampleCounter++;

  // check if file is considered full
  if (sampleCounter >= maximumSampleCounter) {
    // close file and delete
    fileStream->close();
    delete fileStream;
    fileStream = nullptr;
    // reset counter
    sampleCounter = 0;
  }

  if (fileStream == nullptr) {
    // get filename
    string filename = getFlightDataRecorderFilename();
    // create new file
    fileStream = new gzofstream(filename.c_str());
    // clean up directory
    cleanUpFlightDataRecorderFiles();
  }
}

string FlightDataRecorder::getFlightDataRecorderFilename() {
  // get time
  auto in_time_t = chrono::system_clock::to_time_t(chrono::system_clock::now());

  // get filepath based on time
  stringstream result;
  result << put_time(gmtime(&in_time_t), "\\work\\%Y-%m-%d-%H-%M-%S.fdr");

  // return result
  return result.str();
}

void FlightDataRecorder::cleanUpFlightDataRecorderFiles() {
  // vector for directory entries
  vector<string> files;

  // extension
  string extension = "fdr";

  // structure representing an directory entry
  struct dirent* directoryEntry;

  // open directory
  DIR* directory = opendir("\\work");

  // read directory until end
  while ((directoryEntry = readdir(directory)) != NULL) {
    // get filename as string
    string filename = directoryEntry->d_name;

    // check if file has right extension
    if (filename.find(extension, (filename.length() - extension.length())) != string::npos) {
      files.push_back(filename);
    }
  }

  // close directory
  closedir(directory);

  // sort vector
  sort(files.begin(), files.end(), std::greater<>());

  // remove older files
  while (files.size() > maximumFileCount) {
    bool result = remove(("\\work\\" + files.back()).c_str());
    files.pop_back();
  }
}
