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

#include <MSFS/Legacy/gauges.h>
#include <SimConnect.h>

#include "AutopilotLaws.h"
#include "AutopilotStateMachine.h"
#include "Autothrust.h"
#include "FlightDataRecorder.h"
#include "FlyByWire.h"
#include "InterpolatingLookupTable.h"
#include "RateLimiter.h"
#include "SimConnectInterface.h"
#include "ThrottleAxisMapping.h"

class FlyByWireInterface {
 public:
  bool connect();

  void disconnect();

  bool update(double sampleTime);

 private:
  const std::string CONFIGURATION_FILEPATH = "\\work\\ModelConfiguration.ini";

  double previousSimulationTime = 0;

  int currentApproachCapability = 0;
  double previousApproachCapabilityUpdateTime = 0;

  bool flightDirectorSmoothingEnabled = false;
  double flightDirectorSmoothingFactor = 0;
  double flightDirectorSmoothingLimit = 0;
  bool customFlightGuidanceEnabled = false;
  bool autopilotStateMachineEnabled = false;
  bool autopilotLawsEnabled = false;
  bool flyByWireEnabled = false;
  bool autoThrustEnabled = false;
  bool tailstrikeProtectionEnabled = true;

  bool pauseDetected = false;
  bool wasInSlew = false;

  FlightDataRecorder flightDataRecorder;

  SimConnectInterface simConnectInterface;

  FlyByWireModelClass flyByWire;
  FlyByWireModelClass::ExternalInputs_FlyByWire_T flyByWireInput = {};

  AutopilotStateMachineModelClass autopilotStateMachine;
  AutopilotStateMachineModelClass::ExternalInputs_AutopilotStateMachine_T autopilotStateMachineInput = {};
  ap_raw_laws_input autopilotStateMachineOutput;

  AutopilotLawsModelClass autopilotLaws;
  AutopilotLawsModelClass::ExternalInputs_AutopilotLaws_T autopilotLawsInput = {};
  ap_raw_output autopilotLawsOutput;

  AutothrustModelClass autoThrust;
  AutothrustModelClass::ExternalInputs_Autothrust_T autoThrustInput = {};
  athr_output autoThrustOutput;

  InterpolatingLookupTable throttleLookupTable;

  ID idSideStickPositionX;
  ID idSideStickPositionY;

  ID idFmaLateralMode;
  ID idFmaLateralArmed;
  ID idFmaVerticalMode;
  ID idFmaVerticalArmed;
  ID idFmaSoftAltModeActive;
  ID idFmaApproachCapability;

  ID idFlightDirectorBank;
  ID idFlightDirectorPitch;
  ID idFlightDirectorYaw;

  ID idAutopilotActiveAny;
  ID idAutopilotActive_1;
  ID idAutopilotActive_2;

  ID idAutopilotAutothrustMode;

  ID idFcuTrkFpaModeActive;
  ID idFcuSelectedFpa;
  ID idFcuSelectedVs;
  ID idFcuSelectedHeading;

  ID idFcuLocModeActive;
  ID idFcuApprModeActive;
  ID idFcuModeReversionActive;
  ID idFcuModeReversionTrkFpaActive;

  ID idFlightGuidanceAvailable;
  ID idFlightGuidanceCrossTrackError;
  ID idFlightGuidanceTrackAngleError;
  ID idFlightGuidancePhiCommand;

  ID idFwcFlightPhase;
  ID idFmgcFlightPhase;
  ID idFmgcV2;
  ID idFmgcV_APP;
  ID idFmgcV_LS;
  ID idFmgcV_MAX;
  ID idFmgcAltitudeConstraint;
  ID idFmgcThrustReductionAltitude;
  ID idFmgcThrustReductionAltitudeGoAround;
  ID idFmgcAccelerationAltitude;
  ID idFmgcAccelerationAltitudeEngineOut;
  ID idFmgcAccelerationAltitudeGoAround;
  ID idFmgcAccelerationAltitudeGoAroundEngineOut;
  ID idFmgcCruiseAltitude;
  ID idFmgcFlexTemperature;

  ID idAutothrustN1_TLA_1;
  ID idAutothrustN1_TLA_2;
  ID idAutothrustReverse_1;
  ID idAutothrustReverse_2;
  ID idAutothrustThrustLimitType;
  ID idAutothrustThrustLimit;
  ID idAutothrustN1_c_1;
  ID idAutothrustN1_c_2;
  ID idAutothrustStatus;
  ID idAutothrustMode;
  ID idAutothrustModeMessage;
  ID idThrottlePosition3d_1;
  ID idThrottlePosition3d_2;
  InterpolatingLookupTable idThrottlePositionLookupTable3d;

  std::vector<std::shared_ptr<ThrottleAxisMapping>> throttleAxis;

  void loadConfiguration();
  void setupLocalVariables();

  bool readDataAndLocalVariables(double sampleTime);

  bool updateAutopilotStateMachine(double sampleTime);
  bool updateAutopilotLaws(double sampleTime);
  bool updateFlyByWire(double sampleTime);
  bool updateAutothrust(double sampleTime);

  double smoothFlightDirector(double sampleTime, double factor, double limit, double currentValue, double targetValue);
};
