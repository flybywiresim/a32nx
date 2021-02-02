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

#include <ini.h>
#include <ini_type_conversion.h>
#include <iomanip>
#include <iostream>

#include "FlyByWireInterface.h"
#include "SimConnectData.h"

using namespace std;
using namespace mINI;

bool FlyByWireInterface::connect() {
  // load configuration
  loadConfiguration();

  // setup local variables
  setupLocalVariables();

  // initialize model
  autopilotStateMachine.initialize();
  autopilotLaws.initialize();
  autoThrust.initialize();
  flyByWire.initialize();

  // initialize flight data recorder
  flightDataRecorder.initialize();

  // connect to sim connect
  return simConnectInterface.connect(autopilotStateMachineEnabled, autopilotLawsEnabled, flyByWireEnabled, throttleAxis);
}

void FlyByWireInterface::disconnect() {
  // disconnect from sim connect
  simConnectInterface.disconnect();

  // terminate model
  autopilotStateMachine.terminate();
  autopilotLaws.terminate();
  flyByWire.terminate();

  // terminate flight data recorder
  flightDataRecorder.terminate();

  // delete throttle axis mapping -> due to usage of shared_ptr no delete call is needed
  throttleAxis.clear();
}

bool FlyByWireInterface::update(double sampleTime) {
  bool result = true;

  // get data & inputs
  result &= readDataAndLocalVariables(sampleTime);

  // do not process laws in pause or slew
  if (simConnectInterface.getSimData().slew_on) {
    wasInSlew = true;
    return result;
  } else if (pauseDetected) {
    return result;
  }

  // update autopilot state machine
  result &= updateAutopilotStateMachine(sampleTime);

  // update autopilot laws
  result &= updateAutopilotLaws(sampleTime);

  // update fly-by-wire
  result &= updateFlyByWire(sampleTime);

  // get throttle data and process it
  result &= updateAutothrust(sampleTime);

  // update flight data recorder
  flightDataRecorder.update(&autopilotStateMachine, &autopilotLaws, &autoThrust, &flyByWire);

  // if default AP is on -> disconnect it
  if (simConnectInterface.getSimData().autopilot_master_on) {
    simConnectInterface.sendEvent(SimConnectInterface::Events::AUTOPILOT_OFF);
  }

  // reset was in slew flag
  wasInSlew = false;

  // return result
  return result;
}

void FlyByWireInterface::loadConfiguration() {
  // parse from ini file
  INIStructure iniStructure;
  INIFile iniFile(CONFIGURATION_FILEPATH);
  iniFile.read(iniStructure);

  // load values
  autopilotStateMachineEnabled = INITypeConversion::getBoolean(iniStructure, "MODEL", "AUTOPILOT_STATE_MACHINE_ENABLED", true);
  autopilotLawsEnabled = INITypeConversion::getBoolean(iniStructure, "MODEL", "AUTOPILOT_LAWS_ENABLED", true);
  autoThrustEnabled = INITypeConversion::getBoolean(iniStructure, "MODEL", "AUTOTHRUST_ENABLED", true);
  flyByWireEnabled = INITypeConversion::getBoolean(iniStructure, "MODEL", "FLY_BY_WIRE_ENABLED", true);
  tailstrikeProtectionEnabled = INITypeConversion::getBoolean(iniStructure, "MODEL", "TAILSTRIKE_PROTECTION_ENABLED", false);
  customFlightGuidanceEnabled = INITypeConversion::getBoolean(iniStructure, "AUTOPILOT", "CUSTOM_FLIGHT_GUIDANCE_ENABLED", false);
  flightDirectorSmoothingEnabled = INITypeConversion::getBoolean(iniStructure, "AUTOPILOT", "FLIGHT_DIRECTOR_SMOOTHING_ENABLED", true);
  flightDirectorSmoothingFactor = INITypeConversion::getDouble(iniStructure, "AUTOPILOT", "FLIGHT_DIRECTOR_SMOOTHING_FACTOR", 2.5);
  flightDirectorSmoothingLimit = INITypeConversion::getDouble(iniStructure, "AUTOPILOT", "FLIGHT_DIRECTOR_SMOOTHING_LIMIT", 20);

  // print configuration into console
  std::cout << "WASM: MODEL     : AUTOPILOT_STATE_MACHINE_ENABLED   = " << autopilotStateMachineEnabled << endl;
  std::cout << "WASM: MODEL     : AUTOPILOT_LAWS_ENABLED            = " << autopilotLawsEnabled << endl;
  std::cout << "WASM: MODEL     : AUTOTHRUST_ENABLED                = " << autoThrustEnabled << endl;
  std::cout << "WASM: MODEL     : FLY_BY_WIRE_ENABLED               = " << flyByWireEnabled << endl;
  std::cout << "WASM: MODEL     : TAILSTRIKE_PROTECTION_ENABLED     = " << tailstrikeProtectionEnabled << endl;
  std::cout << "WASM: AUTOPILOT : CUSTOM_FLIGHT_GUIDANCE_ENABLED    = " << customFlightGuidanceEnabled << endl;
  std::cout << "WASM: AUTOPILOT : FLIGHT_DIRECTOR_SMOOTHING_ENABLED = " << flightDirectorSmoothingEnabled << endl;
  std::cout << "WASM: AUTOPILOT : FLIGHT_DIRECTOR_SMOOTHING_FACTOR  = " << flightDirectorSmoothingFactor << endl;
  std::cout << "WASM: AUTOPILOT : FLIGHT_DIRECTOR_SMOOTHING_LIMIT   = " << flightDirectorSmoothingLimit << endl;

  // create axis and load configuration
  for (size_t i = 1; i <= 2; i++) {
    // create new mapping
    auto axis = make_shared<ThrottleAxisMapping>(i);
    // load configuration from file
    axis->loadFromFile();
    // store axis
    throttleAxis.emplace_back(axis);
  }

  // create mapping for 3D animation position
  vector<pair<double, double>> mappingTable3d;
  mappingTable3d.emplace_back(-20.0, 0.0);
  mappingTable3d.emplace_back(0.0, 25.0);
  mappingTable3d.emplace_back(25.0, 50.0);
  mappingTable3d.emplace_back(35.0, 75.0);
  mappingTable3d.emplace_back(45.0, 100.0);
  idThrottlePositionLookupTable3d.initialize(mappingTable3d, 0, 100);
}

void FlyByWireInterface::setupLocalVariables() {
  // register L variables for the sidestick
  idSideStickPositionX = register_named_variable("A32NX_SIDESTICK_POSITION_X");
  idSideStickPositionY = register_named_variable("A32NX_SIDESTICK_POSITION_Y");

  // register L variable for custom fly-by-wire interface
  idFmaLateralMode = register_named_variable("A32NX_FMA_LATERAL_MODE");
  idFmaLateralArmed = register_named_variable("A32NX_FMA_LATERAL_ARMED");
  idFmaVerticalMode = register_named_variable("A32NX_FMA_VERTICAL_MODE");
  idFmaVerticalArmed = register_named_variable("A32NX_FMA_VERTICAL_ARMED");
  idFmaSoftAltModeActive = register_named_variable("A32NX_FMA_SOFT_ALT_MODE");
  idFmaApproachCapability = register_named_variable("A32NX_ApproachCapability");

  // register L variable for flight director
  idFlightDirectorBank = register_named_variable("A32NX_FLIGHT_DIRECTOR_BANK");
  idFlightDirectorPitch = register_named_variable("A32NX_FLIGHT_DIRECTOR_PITCH");
  idFlightDirectorYaw = register_named_variable("A32NX_FLIGHT_DIRECTOR_YAW");

  // register L variables for autopilot
  idAutopilotActiveAny = register_named_variable("A32NX_AUTOPILOT_ACTIVE");
  idAutopilotActive_1 = register_named_variable("A32NX_AUTOPILOT_1_ACTIVE");
  idAutopilotActive_2 = register_named_variable("A32NX_AUTOPILOT_2_ACTIVE");

  idAutopilotAutothrustMode = register_named_variable("A32NX_AUTOPILOT_AUTOTHRUST_MODE");

  // register L variables for flight guidance
  idFwcFlightPhase = register_named_variable("A32NX_FWC_FLIGHT_PHASE");
  idFmgcFlightPhase = register_named_variable("A32NX_FMGC_FLIGHT_PHASE");
  idFmgcV2 = register_named_variable("AIRLINER_V2_SPEED");
  idFmgcV_APP = register_named_variable("AIRLINER_VAPP_SPEED");
  idFmgcV_LS = register_named_variable("A32NX_SPEEDS_VLS");
  idFmgcV_MAX = register_named_variable("A32NX_SPEEDS_VMAX");

  idFmgcAltitudeConstraint = register_named_variable("A32NX_AP_CSTN_ALT");
  idFmgcThrustReductionAltitude = register_named_variable("AIRLINER_THR_RED_ALT");
  idFmgcThrustReductionAltitudeGoAround = register_named_variable("AIRLINER_THR_RED_ALT_GOAROUND");
  idFmgcAccelerationAltitude = register_named_variable("AIRLINER_ACC_ALT");
  idFmgcAccelerationAltitudeEngineOut = register_named_variable("A32NX_ENG_OUT_ACC_ALT");
  idFmgcAccelerationAltitudeGoAround = register_named_variable("AIRLINER_ACC_ALT_GOAROUND");
  idFmgcAccelerationAltitudeGoAroundEngineOut = register_named_variable("AIRLINER_ENG_OUT_ACC_ALT_GOAROUND");
  idFmgcCruiseAltitude = register_named_variable("AIRLINER_CRUISE_ALTITUDE");
  idFmgcFlexTemperature = register_named_variable("AIRLINER_TO_FLEX_TEMP");

  idFlightGuidanceAvailable = register_named_variable("A32NX_FG_AVAIL");
  idFlightGuidanceCrossTrackError = register_named_variable("A32NX_FG_CROSS_TRACK_ERROR");
  idFlightGuidanceTrackAngleError = register_named_variable("A32NX_FG_TRACK_ANGLE_ERROR");
  idFlightGuidancePhiCommand = register_named_variable("A32NX_FG_PHI_COMMAND");

  idFcuTrkFpaModeActive = register_named_variable("A32NX_TRK_FPA_MODE_ACTIVE");
  idFcuSelectedFpa = register_named_variable("A32NX_AUTOPILOT_FPA_SELECTED");
  idFcuSelectedVs = register_named_variable("A32NX_AUTOPILOT_VS_SELECTED");
  idFcuSelectedHeading = register_named_variable("A32NX_AUTOPILOT_HEADING_SELECTED");

  idFcuLocModeActive = register_named_variable("A32NX_FCU_LOC_MODE_ACTIVE");
  idFcuApprModeActive = register_named_variable("A32NX_FCU_APPR_MODE_ACTIVE");
  idFcuModeReversionActive = register_named_variable("A32NX_FCU_MODE_REVERSION_ACTIVE");
  idFcuModeReversionTrkFpaActive = register_named_variable("A32NX_FCU_MODE_REVERSION_TRK_FPA_ACTIVE");

  idThrottlePosition3d_1 = register_named_variable("A32NX_3D_THROTTLE_LEVER_POSITION_1");
  idThrottlePosition3d_2 = register_named_variable("A32NX_3D_THROTTLE_LEVER_POSITION_2");

  idAutothrustStatus = register_named_variable("A32NX_AUTOTHRUST_STATUS");
  idAutothrustMode = register_named_variable("A32NX_AUTOTHRUST_MODE");
  idAutothrustModeMessage = register_named_variable("A32NX_AUTOTHRUST_MODE_MESSAGE");

  idAutothrustThrustLimitType = register_named_variable("A32NX_AUTOTHRUST_THRUST_LIMIT_TYPE");
  idAutothrustThrustLimit = register_named_variable("A32NX_AUTOTHRUST_THRUST_LIMIT");
  idAutothrustN1_TLA_1 = register_named_variable("A32NX_AUTOTHRUST_TLA_N1:1");
  idAutothrustN1_TLA_2 = register_named_variable("A32NX_AUTOTHRUST_TLA_N1:2");
  idAutothrustReverse_1 = register_named_variable("A32NX_AUTOTHRUST_REVERSE:1");
  idAutothrustReverse_2 = register_named_variable("A32NX_AUTOTHRUST_REVERSE:2");
  idAutothrustN1_c_1 = register_named_variable("A32NX_AUTOTHRUST_N1_COMMANDED:1");
  idAutothrustN1_c_2 = register_named_variable("A32NX_AUTOTHRUST_N1_COMMANDED:2");
}

bool FlyByWireInterface::readDataAndLocalVariables(double sampleTime) {
  // reset input
  simConnectInterface.resetSimInputAutopilot();

  // request data
  if (!simConnectInterface.requestData()) {
    std::cout << "WASM: Request data failed!" << endl;
    return false;
  }

  // read data
  if (!simConnectInterface.readData()) {
    std::cout << "WASM: Read data failed!" << endl;
    return false;
  }

  // get sim data
  SimData simData = simConnectInterface.getSimData();

  // read local variables and update client data
  // update client data for flight guidance
  if (!autopilotStateMachineEnabled || !autopilotLawsEnabled) {
    ClientDataLocalVariables clientDataLocalVariables = {
        get_named_variable_value(idFmgcFlightPhase),
        get_named_variable_value(idFmgcV2),
        get_named_variable_value(idFmgcV_APP),
        get_named_variable_value(idFmgcV_LS),
        customFlightGuidanceEnabled ? 1.0 : simData.gpsIsFlightPlanActive,
        get_named_variable_value(idFmgcAltitudeConstraint),
        get_named_variable_value(idFmgcThrustReductionAltitude),
        get_named_variable_value(idFmgcThrustReductionAltitudeGoAround),
        get_named_variable_value(idFmgcAccelerationAltitude),
        get_named_variable_value(idFmgcAccelerationAltitudeEngineOut),
        get_named_variable_value(idFmgcAccelerationAltitudeGoAround),
        get_named_variable_value(idFmgcAccelerationAltitudeGoAroundEngineOut),
        get_named_variable_value(idFmgcCruiseAltitude),
        0,  // DIR-TO trigger
        get_named_variable_value(idFcuTrkFpaModeActive),
        get_named_variable_value(idFcuSelectedVs),
        get_named_variable_value(idFcuSelectedFpa),
        get_named_variable_value(idFcuSelectedHeading),
        customFlightGuidanceEnabled ? get_named_variable_value(idFlightGuidanceCrossTrackError) : simData.gpsWpCrossTrack,
        customFlightGuidanceEnabled ? get_named_variable_value(idFlightGuidanceTrackAngleError) : simData.gpsWpTrackAngleError,
        customFlightGuidanceEnabled ? get_named_variable_value(idFlightGuidancePhiCommand) : 0};
    simConnectInterface.setClientDataLocalVariables(clientDataLocalVariables);
  }

  // detect pause
  if ((simData.simulationTime == previousSimulationTime) || (simData.simulationTime < 0.2)) {
    pauseDetected = true;
  } else {
    pauseDetected = false;
  }
  previousSimulationTime = simData.simulationTime;

  // success
  return true;
}

bool FlyByWireInterface::updateAutopilotStateMachine(double sampleTime) {
  // get data from interface ------------------------------------------------------------------------------------------
  SimData simData = simConnectInterface.getSimData();
  SimInputAutopilot simInputAutopilot = simConnectInterface.getSimInputAutopilot();
  // get also input for AP --------------------------------------------------------------------------------------------

  // update state machine ---------------------------------------------------------------------------------------------
  if (autopilotStateMachineEnabled) {
    // time -----------------------------------------------------------------------------------------------------------
    autopilotStateMachineInput.in.time.dt = sampleTime;
    autopilotStateMachineInput.in.time.simulation_time = simData.simulationTime;

    // data -----------------------------------------------------------------------------------------------------------
    autopilotStateMachineInput.in.data.Theta_deg = simData.Theta_deg;
    autopilotStateMachineInput.in.data.Phi_deg = simData.Phi_deg;
    autopilotStateMachineInput.in.data.q_rad_s = simData.bodyRotationVelocity.x;
    autopilotStateMachineInput.in.data.r_rad_s = simData.bodyRotationVelocity.y;
    autopilotStateMachineInput.in.data.p_rad_s = simData.bodyRotationVelocity.z;
    autopilotStateMachineInput.in.data.V_ias_kn = simData.V_ias_kn;
    autopilotStateMachineInput.in.data.V_tas_kn = simData.V_tas_kn;
    autopilotStateMachineInput.in.data.V_mach = simData.V_mach;
    autopilotStateMachineInput.in.data.V_gnd_kn = simData.V_gnd_kn;
    autopilotStateMachineInput.in.data.alpha_deg = simData.alpha_deg;
    autopilotStateMachineInput.in.data.H_ft = simData.H_ft;
    autopilotStateMachineInput.in.data.H_ind_ft = simData.H_ind_ft;
    autopilotStateMachineInput.in.data.H_radio_ft = simData.H_radio_ft;
    autopilotStateMachineInput.in.data.H_dot_ft_min = simData.H_dot_fpm;
    autopilotStateMachineInput.in.data.Psi_magnetic_deg = simData.Psi_magnetic_deg;
    autopilotStateMachineInput.in.data.Psi_magnetic_track_deg = simData.Psi_magnetic_track_deg;
    autopilotStateMachineInput.in.data.Psi_true_deg = simData.Psi_true_deg;
    autopilotStateMachineInput.in.data.bx_m_s2 = simData.bx_m_s2;
    autopilotStateMachineInput.in.data.by_m_s2 = simData.by_m_s2;
    autopilotStateMachineInput.in.data.bz_m_s2 = simData.bz_m_s2;
    autopilotStateMachineInput.in.data.nav_valid = (simData.nav_valid != 0);
    autopilotStateMachineInput.in.data.nav_loc_deg = simData.nav_loc_deg;
    autopilotStateMachineInput.in.data.nav_dme_valid = (simData.nav_dme_valid != 0);
    autopilotStateMachineInput.in.data.nav_dme_nmi = simData.nav_dme_nmi;
    autopilotStateMachineInput.in.data.nav_loc_valid = (simData.nav_loc_valid != 0);
    autopilotStateMachineInput.in.data.nav_loc_error_deg = simData.nav_loc_error_deg;
    autopilotStateMachineInput.in.data.nav_gs_valid = (simData.nav_gs_valid != 0);
    autopilotStateMachineInput.in.data.nav_gs_error_deg = simData.nav_gs_error_deg;
    autopilotStateMachineInput.in.data.flight_guidance_xtk_nmi =
        customFlightGuidanceEnabled ? get_named_variable_value(idFlightGuidanceCrossTrackError) : simData.gpsWpCrossTrack;
    autopilotStateMachineInput.in.data.flight_guidance_tae_deg =
        customFlightGuidanceEnabled ? get_named_variable_value(idFlightGuidanceTrackAngleError) : simData.gpsWpTrackAngleError;
    autopilotStateMachineInput.in.data.flight_guidance_phi_deg =
        customFlightGuidanceEnabled ? get_named_variable_value(idFlightGuidancePhiCommand) : 0;
    autopilotStateMachineInput.in.data.flight_phase = get_named_variable_value(idFmgcFlightPhase);
    autopilotStateMachineInput.in.data.V2_kn = get_named_variable_value(idFmgcV2);
    autopilotStateMachineInput.in.data.VAPP_kn = get_named_variable_value(idFmgcV_APP);
    autopilotStateMachineInput.in.data.VLS_kn = get_named_variable_value(idFmgcV_LS);
    autopilotStateMachineInput.in.data.is_flight_plan_available =
        customFlightGuidanceEnabled ? get_named_variable_value(idFlightGuidanceAvailable) : simData.gpsIsFlightPlanActive;
    autopilotStateMachineInput.in.data.altitude_constraint_ft = get_named_variable_value(idFmgcAltitudeConstraint);
    autopilotStateMachineInput.in.data.thrust_reduction_altitude = get_named_variable_value(idFmgcThrustReductionAltitude);
    autopilotStateMachineInput.in.data.thrust_reduction_altitude_go_around =
        get_named_variable_value(idFmgcThrustReductionAltitudeGoAround);
    autopilotStateMachineInput.in.data.acceleration_altitude = get_named_variable_value(idFmgcAccelerationAltitude);
    autopilotStateMachineInput.in.data.acceleration_altitude_engine_out = get_named_variable_value(idFmgcAccelerationAltitudeEngineOut);
    autopilotStateMachineInput.in.data.acceleration_altitude_go_around = get_named_variable_value(idFmgcAccelerationAltitudeGoAround);
    autopilotStateMachineInput.in.data.acceleration_altitude_go_around_engine_out =
        get_named_variable_value(idFmgcAccelerationAltitudeGoAroundEngineOut);
    autopilotStateMachineInput.in.data.cruise_altitude = get_named_variable_value(idFmgcCruiseAltitude);
    autopilotStateMachineInput.in.data.throttle_lever_1_pos = throttleAxis[0]->getTLA();
    autopilotStateMachineInput.in.data.throttle_lever_2_pos = throttleAxis[1]->getTLA();
    autopilotStateMachineInput.in.data.gear_strut_compression_1 = simData.gear_animation_pos_1;
    autopilotStateMachineInput.in.data.gear_strut_compression_2 = simData.gear_animation_pos_2;
    autopilotStateMachineInput.in.data.zeta_pos = simData.zeta_pos;
    autopilotStateMachineInput.in.data.flaps_handle_index = simData.flaps_handle_index;
    autopilotStateMachineInput.in.data.is_engine_operative_1 = simData.engine_combustion_1;
    autopilotStateMachineInput.in.data.is_engine_operative_2 = simData.engine_combustion_2;

    // input ----------------------------------------------------------------------------------------------------------
    autopilotStateMachineInput.in.input.FD_active = simData.ap_fd_1_active | simData.ap_fd_2_active;
    autopilotStateMachineInput.in.input.AP_1_push = simInputAutopilot.AP_1_push;
    autopilotStateMachineInput.in.input.AP_2_push = simInputAutopilot.AP_2_push;
    autopilotStateMachineInput.in.input.AP_DISCONNECT_push = simInputAutopilot.AP_disconnect || wasInSlew;
    autopilotStateMachineInput.in.input.HDG_push = simInputAutopilot.HDG_push;
    autopilotStateMachineInput.in.input.HDG_pull = simInputAutopilot.HDG_pull;
    autopilotStateMachineInput.in.input.ALT_push = simInputAutopilot.ALT_push;
    autopilotStateMachineInput.in.input.ALT_pull = simInputAutopilot.ALT_pull;
    autopilotStateMachineInput.in.input.VS_push = simInputAutopilot.VS_push;
    autopilotStateMachineInput.in.input.VS_pull = simInputAutopilot.VS_pull;
    autopilotStateMachineInput.in.input.LOC_push = simInputAutopilot.LOC_push;
    autopilotStateMachineInput.in.input.APPR_push = simInputAutopilot.APPR_push;
    autopilotStateMachineInput.in.input.V_fcu_kn = simData.ap_V_c_kn;
    autopilotStateMachineInput.in.input.H_fcu_ft = simData.ap_H_c_ft;
    autopilotStateMachineInput.in.input.H_constraint_ft = get_named_variable_value(idFmgcAltitudeConstraint);
    autopilotStateMachineInput.in.input.H_dot_fcu_fpm = get_named_variable_value(idFcuSelectedVs);
    autopilotStateMachineInput.in.input.FPA_fcu_deg = get_named_variable_value(idFcuSelectedFpa);
    autopilotStateMachineInput.in.input.Psi_fcu_deg = get_named_variable_value(idFcuSelectedHeading);
    autopilotStateMachineInput.in.input.TRK_FPA_mode = get_named_variable_value(idFcuTrkFpaModeActive);
    autopilotStateMachineInput.in.input.DIR_TO_trigger = 0;
    autopilotStateMachineInput.in.input.is_FLX_active = autoThrust.getExternalOutputs().out.data_computed.is_FLX_active;
    autopilotStateMachineInput.in.input.Slew_trigger = wasInSlew;

    // step the model -------------------------------------------------------------------------------------------------
    autopilotStateMachine.setExternalInputs(&autopilotStateMachineInput);
    autopilotStateMachine.step();

    // result
    autopilotStateMachineOutput = autopilotStateMachine.getExternalOutputs().out.output;
  } else {
    // read client data written by simulink
    ClientDataAutopilotStateMachine clientData = simConnectInterface.getClientDataAutopilotStateMachine();
    autopilotStateMachineOutput.enabled_AP1 = clientData.enabled_AP1;
    autopilotStateMachineOutput.enabled_AP2 = clientData.enabled_AP2;
    autopilotStateMachineOutput.lateral_law = clientData.lateral_law;
    autopilotStateMachineOutput.lateral_mode = clientData.lateral_mode;
    autopilotStateMachineOutput.lateral_mode_armed = clientData.lateral_mode_armed;
    autopilotStateMachineOutput.vertical_law = clientData.vertical_law;
    autopilotStateMachineOutput.vertical_mode = clientData.vertical_mode;
    autopilotStateMachineOutput.vertical_mode_armed = clientData.vertical_mode_armed;
    autopilotStateMachineOutput.mode_reversion = clientData.mode_reversion;
    autopilotStateMachineOutput.mode_reversion_TRK_FPA = clientData.mode_reversion_TRK_FPA;
    autopilotStateMachineOutput.autothrust_mode = clientData.autothrust_mode;
    autopilotStateMachineOutput.Psi_c_deg = clientData.Psi_c_deg;
    autopilotStateMachineOutput.H_c_ft = clientData.H_c_ft;
    autopilotStateMachineOutput.H_dot_c_fpm = clientData.H_dot_c_fpm;
    autopilotStateMachineOutput.FPA_c_deg = clientData.FPA_c_deg;
    autopilotStateMachineOutput.V_c_kn = clientData.V_c_kn;
    autopilotStateMachineOutput.ALT_soft_mode_active = clientData.ALT_soft_mode_active;
  }

  // update autopilot state -------------------------------------------------------------------------------------------
  set_named_variable_value(idAutopilotActiveAny, autopilotStateMachineOutput.enabled_AP1 || autopilotStateMachineOutput.enabled_AP2);
  set_named_variable_value(idAutopilotActive_1, autopilotStateMachineOutput.enabled_AP1);
  set_named_variable_value(idAutopilotActive_2, autopilotStateMachineOutput.enabled_AP2);

  bool isLocArmed = static_cast<unsigned long long>(autopilotStateMachineOutput.lateral_mode_armed) >> 1 & 0x01;
  bool isLocEngaged = autopilotStateMachineOutput.lateral_mode >= 30 && autopilotStateMachineOutput.lateral_mode <= 34;
  bool isGsArmed = static_cast<unsigned long long>(autopilotStateMachineOutput.vertical_mode_armed) >> 4 & 0x01;
  bool isGsEngaged = autopilotStateMachineOutput.vertical_mode >= 30 && autopilotStateMachineOutput.vertical_mode <= 34;
  set_named_variable_value(idFcuLocModeActive, (isLocArmed || isLocEngaged) && !(isGsArmed || isGsEngaged));
  set_named_variable_value(idFcuApprModeActive, (isLocArmed || isLocEngaged) && (isGsArmed || isGsEngaged));
  set_named_variable_value(idFcuModeReversionActive, autopilotStateMachineOutput.mode_reversion);
  set_named_variable_value(idFcuModeReversionTrkFpaActive, autopilotStateMachineOutput.mode_reversion_TRK_FPA);

  // update autothrust mode -------------------------------------------------------------------------------------------
  set_named_variable_value(idAutopilotAutothrustMode, autopilotStateMachineOutput.autothrust_mode);

  // update FMA variables ---------------------------------------------------------------------------------------------
  set_named_variable_value(idFmaLateralMode, autopilotStateMachineOutput.lateral_mode);
  set_named_variable_value(idFmaLateralArmed, autopilotStateMachineOutput.lateral_mode_armed);
  set_named_variable_value(idFmaVerticalMode, autopilotStateMachineOutput.vertical_mode);
  set_named_variable_value(idFmaVerticalArmed, autopilotStateMachineOutput.vertical_mode_armed);
  set_named_variable_value(idFmaSoftAltModeActive, autopilotStateMachineOutput.ALT_soft_mode_active);

  // calculate and set approach capability
  // when no RA is available at all -> CAT1, at least one RA is needed to get into CAT2 or higher
  // CAT3 requires two valid RA which are not simulated yet
  bool landModeArmedOrActive = (isLocArmed || isLocEngaged) && (isGsArmed || isGsEngaged);
  int numberOfAutopilotsEngaged = autopilotStateMachineOutput.enabled_AP1 + autopilotStateMachineOutput.enabled_AP2;
  bool autoThrustEngaged = autoThrustOutput.status == 2;
  bool radioAltimeterAvailable = (simData.H_radio_ft <= 5000);
  bool isCat1 = landModeArmedOrActive;
  bool isCat2 = landModeArmedOrActive && radioAltimeterAvailable && !autoThrustEngaged && numberOfAutopilotsEngaged >= 1;
  bool isCat3S = landModeArmedOrActive && radioAltimeterAvailable && autoThrustEngaged && numberOfAutopilotsEngaged >= 1;
  bool isCat3D = landModeArmedOrActive && radioAltimeterAvailable && autoThrustEngaged && numberOfAutopilotsEngaged == 2;
  int newApproachCapability = currentApproachCapability;

  if (currentApproachCapability == 0) {
    if (isCat1) {
      newApproachCapability = 1;
    }
  } else if (currentApproachCapability == 1) {
    if (!isCat1) {
      newApproachCapability = 0;
    }
    if (isCat3S) {
      newApproachCapability = 3;
    } else if (isCat2) {
      newApproachCapability = 2;
    }
  } else if (currentApproachCapability == 2) {
    if (isCat3D) {
      newApproachCapability = 4;
    } else if (isCat3S) {
      newApproachCapability = 3;
    } else if (!isCat2) {
      newApproachCapability = 1;
    }
  } else if (currentApproachCapability == 3) {
    if ((simData.H_radio_ft > 100) || (simData.H_radio_ft < 100 && numberOfAutopilotsEngaged == 0)) {
      if (isCat3D) {
        newApproachCapability = 4;
      } else if (!isCat3S && !isCat2) {
        newApproachCapability = 1;
      } else if (!isCat3S && isCat2) {
        newApproachCapability = 2;
      }
    }
  } else if (currentApproachCapability == 4) {
    if ((simData.H_radio_ft > 100) || (simData.H_radio_ft < 100 && numberOfAutopilotsEngaged == 0)) {
      if (!autoThrustEngaged) {
        newApproachCapability = 2;
      } else if (!isCat3D) {
        newApproachCapability = 3;
      }
    }
  }

  bool doUpdate = false;
  bool canDowngrade = (simData.simulationTime - previousApproachCapabilityUpdateTime) > 3.0;
  bool canUpgrade = (simData.simulationTime - previousApproachCapabilityUpdateTime) > 1.5;
  if (newApproachCapability != currentApproachCapability) {
    doUpdate = (newApproachCapability == 0 && currentApproachCapability == 1) ||
               (newApproachCapability == 1 && currentApproachCapability == 0) ||
               (newApproachCapability > currentApproachCapability && canUpgrade) ||
               (newApproachCapability < currentApproachCapability && canDowngrade);
  } else {
    previousApproachCapabilityUpdateTime = simData.simulationTime;
  }

  if (doUpdate) {
    currentApproachCapability = newApproachCapability;
    set_named_variable_value(idFmaApproachCapability, currentApproachCapability);
    previousApproachCapabilityUpdateTime = simData.simulationTime;
  }

  // return result ----------------------------------------------------------------------------------------------------
  return true;
}

bool FlyByWireInterface::updateAutopilotLaws(double sampleTime) {
  // get data from interface ------------------------------------------------------------------------------------------
  SimData simData = simConnectInterface.getSimData();

  // update laws ------------------------------------------------------------------------------------------------------
  if (autopilotLawsEnabled) {
    // time -----------------------------------------------------------------------------------------------------------
    autopilotLawsInput.in.time.dt = sampleTime;
    autopilotLawsInput.in.time.simulation_time = simData.simulationTime;

    // data -----------------------------------------------------------------------------------------------------------
    autopilotLawsInput.in.data.Theta_deg = simData.Theta_deg;
    autopilotLawsInput.in.data.Phi_deg = simData.Phi_deg;
    autopilotLawsInput.in.data.q_rad_s = simData.bodyRotationVelocity.x;
    autopilotLawsInput.in.data.r_rad_s = simData.bodyRotationVelocity.y;
    autopilotLawsInput.in.data.p_rad_s = simData.bodyRotationVelocity.z;
    autopilotLawsInput.in.data.V_ias_kn = simData.V_ias_kn;
    autopilotLawsInput.in.data.V_tas_kn = simData.V_tas_kn;
    autopilotLawsInput.in.data.V_mach = simData.V_mach;
    autopilotLawsInput.in.data.V_gnd_kn = simData.V_gnd_kn;
    autopilotLawsInput.in.data.alpha_deg = simData.alpha_deg;
    autopilotLawsInput.in.data.H_ft = simData.H_ft;
    autopilotLawsInput.in.data.H_ind_ft = simData.H_ind_ft;
    autopilotLawsInput.in.data.H_radio_ft = simData.H_radio_ft;
    autopilotLawsInput.in.data.H_dot_ft_min = simData.H_dot_fpm;
    autopilotLawsInput.in.data.Psi_magnetic_deg = simData.Psi_magnetic_deg;
    autopilotLawsInput.in.data.Psi_magnetic_track_deg = simData.Psi_magnetic_track_deg;
    autopilotLawsInput.in.data.Psi_true_deg = simData.Psi_true_deg;
    autopilotLawsInput.in.data.bx_m_s2 = simData.bx_m_s2;
    autopilotLawsInput.in.data.by_m_s2 = simData.by_m_s2;
    autopilotLawsInput.in.data.bz_m_s2 = simData.bz_m_s2;
    autopilotLawsInput.in.data.nav_valid = (simData.nav_valid != 0);
    autopilotLawsInput.in.data.nav_loc_deg = simData.nav_loc_deg;
    autopilotLawsInput.in.data.nav_dme_valid = (simData.nav_dme_valid != 0);
    autopilotLawsInput.in.data.nav_dme_nmi = simData.nav_dme_nmi;
    autopilotLawsInput.in.data.nav_loc_valid = (simData.nav_loc_valid != 0);
    autopilotLawsInput.in.data.nav_loc_error_deg = simData.nav_loc_error_deg;
    autopilotLawsInput.in.data.nav_gs_valid = (simData.nav_gs_valid != 0);
    autopilotLawsInput.in.data.nav_gs_error_deg = simData.nav_gs_error_deg;
    autopilotLawsInput.in.data.flight_guidance_xtk_nmi =
        customFlightGuidanceEnabled ? get_named_variable_value(idFlightGuidanceCrossTrackError) : simData.gpsWpCrossTrack;
    autopilotLawsInput.in.data.flight_guidance_tae_deg =
        customFlightGuidanceEnabled ? get_named_variable_value(idFlightGuidanceTrackAngleError) : simData.gpsWpTrackAngleError;
    autopilotLawsInput.in.data.flight_guidance_phi_deg =
        customFlightGuidanceEnabled ? get_named_variable_value(idFlightGuidancePhiCommand) : 0;
    autopilotLawsInput.in.data.flight_phase = get_named_variable_value(idFmgcFlightPhase);
    autopilotLawsInput.in.data.V2_kn = get_named_variable_value(idFmgcV2);
    autopilotLawsInput.in.data.VAPP_kn = get_named_variable_value(idFmgcV_APP);
    autopilotLawsInput.in.data.VLS_kn = get_named_variable_value(idFmgcV_LS);
    autopilotLawsInput.in.data.is_flight_plan_available =
        customFlightGuidanceEnabled ? get_named_variable_value(idFlightGuidanceAvailable) : simData.gpsIsFlightPlanActive;
    autopilotLawsInput.in.data.altitude_constraint_ft = get_named_variable_value(idFmgcAltitudeConstraint);
    autopilotLawsInput.in.data.thrust_reduction_altitude = get_named_variable_value(idFmgcThrustReductionAltitude);
    autopilotLawsInput.in.data.thrust_reduction_altitude_go_around = get_named_variable_value(idFmgcThrustReductionAltitudeGoAround);
    autopilotLawsInput.in.data.acceleration_altitude = get_named_variable_value(idFmgcAccelerationAltitude);
    autopilotLawsInput.in.data.acceleration_altitude_engine_out = get_named_variable_value(idFmgcAccelerationAltitudeEngineOut);
    autopilotLawsInput.in.data.acceleration_altitude_go_around = get_named_variable_value(idFmgcAccelerationAltitudeGoAround);
    autopilotLawsInput.in.data.acceleration_altitude_go_around_engine_out =
        get_named_variable_value(idFmgcAccelerationAltitudeGoAroundEngineOut);
    autopilotLawsInput.in.data.throttle_lever_1_pos = throttleAxis[0]->getTLA();
    autopilotLawsInput.in.data.throttle_lever_2_pos = throttleAxis[1]->getTLA();
    autopilotLawsInput.in.data.gear_strut_compression_1 = simData.gear_animation_pos_1;
    autopilotLawsInput.in.data.gear_strut_compression_2 = simData.gear_animation_pos_2;
    autopilotLawsInput.in.data.zeta_pos = simData.zeta_pos;
    autopilotLawsInput.in.data.flaps_handle_index = simData.flaps_handle_index;
    autopilotLawsInput.in.data.is_engine_operative_1 = simData.engine_combustion_1;
    autopilotLawsInput.in.data.is_engine_operative_2 = simData.engine_combustion_2;

    // input ----------------------------------------------------------------------------------------------------------
    autopilotLawsInput.in.input = autopilotStateMachineOutput;

    // step the model -------------------------------------------------------------------------------------------------
    autopilotLaws.setExternalInputs(&autopilotLawsInput);
    autopilotLaws.step();

    // result ---------------------------------------------------------------------------------------------------------
    autopilotLawsOutput = autopilotLaws.getExternalOutputs().out.output;
  } else {
    if (autopilotStateMachineEnabled) {
      // send data to client data to be read by simulink
      ClientDataAutopilotStateMachine clientDataStateMachine = {autopilotStateMachineOutput.enabled_AP1,
                                                                autopilotStateMachineOutput.enabled_AP2,
                                                                autopilotStateMachineOutput.lateral_law,
                                                                autopilotStateMachineOutput.lateral_mode,
                                                                autopilotStateMachineOutput.lateral_mode_armed,
                                                                autopilotStateMachineOutput.vertical_law,
                                                                autopilotStateMachineOutput.vertical_mode,
                                                                autopilotStateMachineOutput.vertical_mode_armed,
                                                                autopilotStateMachineOutput.mode_reversion,
                                                                autopilotStateMachineOutput.mode_reversion_TRK_FPA,
                                                                autopilotStateMachineOutput.autothrust_mode,
                                                                autopilotStateMachineOutput.Psi_c_deg,
                                                                autopilotStateMachineOutput.H_c_ft,
                                                                autopilotStateMachineOutput.H_dot_c_fpm,
                                                                autopilotStateMachineOutput.FPA_c_deg,
                                                                autopilotStateMachineOutput.V_c_kn,
                                                                autopilotStateMachineOutput.ALT_soft_mode_active};
      simConnectInterface.setClientDataAutopilotStateMachine(clientDataStateMachine);
    }
    // read client data written by simulink
    ClientDataAutopilotLaws clientDataLaws = simConnectInterface.getClientDataAutopilotLaws();
    autopilotLawsOutput.ap_on = clientDataLaws.enableAutopilot;
    autopilotLawsOutput.flight_director.Theta_c_deg = clientDataLaws.flightDirectorTheta;
    autopilotLawsOutput.autopilot.Theta_c_deg = clientDataLaws.autopilotTheta;
    autopilotLawsOutput.flight_director.Phi_c_deg = clientDataLaws.flightDirectorPhi;
    autopilotLawsOutput.autopilot.Phi_c_deg = clientDataLaws.autopilotPhi;
    autopilotLawsOutput.flight_director.Beta_c_deg = clientDataLaws.autopilotBeta;
    autopilotLawsOutput.autopilot.Beta_c_deg = clientDataLaws.autopilotBeta;
  }

  // update flight director -------------------------------------------------------------------------------------------
  double fdPitch = -1.0 * autopilotLawsOutput.flight_director.Theta_c_deg;
  double fdBank = -1.0 * autopilotLawsOutput.flight_director.Phi_c_deg;
  double fdYaw = autopilotLawsOutput.flight_director.Beta_c_deg;
  if (flightDirectorSmoothingEnabled) {
    fdPitch = smoothFlightDirector(sampleTime, flightDirectorSmoothingFactor, flightDirectorSmoothingLimit,
                                   get_named_variable_value(idFlightDirectorPitch), fdPitch);
    fdBank = smoothFlightDirector(sampleTime, flightDirectorSmoothingFactor, flightDirectorSmoothingLimit,
                                  get_named_variable_value(idFlightDirectorBank), fdBank);
    fdYaw = smoothFlightDirector(sampleTime, flightDirectorSmoothingFactor, flightDirectorSmoothingLimit,
                                 get_named_variable_value(idFlightDirectorYaw), fdYaw);
  }
  set_named_variable_value(idFlightDirectorPitch, fdPitch);
  set_named_variable_value(idFlightDirectorBank, fdBank);
  set_named_variable_value(idFlightDirectorYaw, fdYaw);

  // return result ----------------------------------------------------------------------------------------------------
  return true;
}

bool FlyByWireInterface::updateFlyByWire(double sampleTime) {
  // get data from interface ------------------------------------------------------------------------------------------
  SimData simData = simConnectInterface.getSimData();
  SimInput simInput = simConnectInterface.getSimInput();

  // update fly-by-wire -----------------------------------------------------------------------------------------------
  if (flyByWireEnabled) {
    // fill time into model -------------------------------------------------------------------------------------------
    flyByWireInput.in.time.dt = sampleTime;
    flyByWireInput.in.time.simulation_time = simData.simulationTime;

    // fill data into model -------------------------------------------------------------------------------------------
    flyByWireInput.in.data.nz_g = simData.nz_g;
    flyByWireInput.in.data.Theta_deg = simData.Theta_deg;
    flyByWireInput.in.data.Phi_deg = simData.Phi_deg;
    flyByWireInput.in.data.q_rad_s = simData.bodyRotationVelocity.x;
    flyByWireInput.in.data.r_rad_s = simData.bodyRotationVelocity.y;
    flyByWireInput.in.data.p_rad_s = simData.bodyRotationVelocity.z;
    flyByWireInput.in.data.q_dot_rad_s2 = simData.bodyRotationAcceleration.x;
    flyByWireInput.in.data.r_dot_rad_s2 = simData.bodyRotationAcceleration.y;
    flyByWireInput.in.data.p_dot_rad_s2 = simData.bodyRotationAcceleration.z;
    flyByWireInput.in.data.psi_magnetic_deg = simData.Psi_magnetic_deg;
    flyByWireInput.in.data.psi_true_deg = simData.Psi_true_deg;
    flyByWireInput.in.data.eta_pos = simData.eta_pos;
    flyByWireInput.in.data.eta_trim_deg = simData.eta_trim_deg;
    flyByWireInput.in.data.xi_pos = simData.xi_pos;
    flyByWireInput.in.data.zeta_pos = simData.zeta_pos;
    flyByWireInput.in.data.zeta_trim_pos = simData.zeta_trim_pos;
    flyByWireInput.in.data.alpha_deg = simData.alpha_deg;
    flyByWireInput.in.data.beta_deg = simData.beta_deg;
    flyByWireInput.in.data.beta_dot_deg_s = simData.beta_dot_deg_s;
    flyByWireInput.in.data.V_ias_kn = simData.V_ias_kn;
    flyByWireInput.in.data.V_tas_kn = simData.V_tas_kn;
    flyByWireInput.in.data.V_mach = simData.V_mach;
    flyByWireInput.in.data.H_ft = simData.H_ft;
    flyByWireInput.in.data.H_ind_ft = simData.H_ind_ft;
    flyByWireInput.in.data.H_radio_ft = simData.H_radio_ft;
    flyByWireInput.in.data.CG_percent_MAC = simData.CG_percent_MAC;
    flyByWireInput.in.data.total_weight_kg = simData.total_weight_kg;
    flyByWireInput.in.data.gear_animation_pos_0 = simData.gear_animation_pos_0;
    flyByWireInput.in.data.gear_animation_pos_1 = simData.gear_animation_pos_1;
    flyByWireInput.in.data.gear_animation_pos_2 = simData.gear_animation_pos_2;
    flyByWireInput.in.data.flaps_handle_index = simData.flaps_handle_index;
    flyByWireInput.in.data.spoilers_left_pos = simData.spoilers_left_pos;
    flyByWireInput.in.data.spoilers_right_pos = simData.spoilers_right_pos;
    flyByWireInput.in.data.autopilot_master_on = simData.autopilot_master_on;
    flyByWireInput.in.data.slew_on = simData.slew_on;
    flyByWireInput.in.data.pause_on = pauseDetected;
    flyByWireInput.in.data.autopilot_custom_on = autopilotLawsOutput.ap_on;
    flyByWireInput.in.data.autopilot_custom_Theta_c_deg = autopilotLawsOutput.autopilot.Theta_c_deg;
    flyByWireInput.in.data.autopilot_custom_Phi_c_deg = autopilotLawsOutput.autopilot.Phi_c_deg;
    flyByWireInput.in.data.autopilot_custom_Beta_c_deg = autopilotLawsOutput.autopilot.Beta_c_deg;
    flyByWireInput.in.data.tracking_mode_on_override = 0;
    flyByWireInput.in.data.simulation_rate = simData.simulation_rate;
    flyByWireInput.in.data.ice_structure_percent = simData.ice_structure_percent;
    flyByWireInput.in.data.linear_cl_alpha_per_deg = simData.linear_cl_alpha_per_deg;
    flyByWireInput.in.data.alpha_stall_deg = simData.alpha_stall_deg;
    flyByWireInput.in.data.alpha_zero_lift_deg = simData.alpha_zero_lift_deg;
    flyByWireInput.in.data.ambient_density_kg_per_m3 = simData.ambient_density_kg_per_m3;
    flyByWireInput.in.data.ambient_pressure_mbar = simData.ambient_pressure_mbar;
    flyByWireInput.in.data.ambient_temperature_celsius = simData.ambient_temperature_celsius;
    flyByWireInput.in.data.ambient_wind_x_kn = simData.ambient_wind_x_kn;
    flyByWireInput.in.data.ambient_wind_y_kn = simData.ambient_wind_y_kn;
    flyByWireInput.in.data.ambient_wind_z_kn = simData.ambient_wind_z_kn;
    flyByWireInput.in.data.ambient_wind_velocity_kn = simData.ambient_wind_velocity_kn;
    flyByWireInput.in.data.ambient_wind_direction_deg = simData.ambient_wind_direction_deg;
    flyByWireInput.in.data.total_air_temperature_celsius = simData.total_air_temperature_celsius;
    flyByWireInput.in.data.latitude_deg = simData.latitude_deg;
    flyByWireInput.in.data.longitude_deg = simData.longitude_deg;
    flyByWireInput.in.data.engine_1_thrust_lbf = simData.engine_1_thrust_lbf;
    flyByWireInput.in.data.engine_2_thrust_lbf = simData.engine_2_thrust_lbf;
    flyByWireInput.in.data.thrust_lever_1_pos = throttleAxis[0]->getTLA();
    flyByWireInput.in.data.thrust_lever_2_pos = throttleAxis[1]->getTLA();
    flyByWireInput.in.data.tailstrike_protection_on = tailstrikeProtectionEnabled;

    // process the sidestick handling ---------------------------------------------------------------------------------
    // write sidestick position
    set_named_variable_value(idSideStickPositionX, -1.0 * simInput.inputs[1]);
    set_named_variable_value(idSideStickPositionY, -1.0 * simInput.inputs[0]);
    // fill inputs into model
    flyByWireInput.in.input.delta_eta_pos = simInput.inputs[0];
    flyByWireInput.in.input.delta_xi_pos = simInput.inputs[1];
    flyByWireInput.in.input.delta_zeta_pos = simInput.inputs[2];

    // step the model -------------------------------------------------------------------------------------------------
    flyByWire.setExternalInputs(&flyByWireInput);
    flyByWire.step();

    // when tracking mode is on do not write anything -----------------------------------------------------------------
    FlyByWireModelClass::ExternalOutputs_FlyByWire_T flyByWireOutput = flyByWire.getExternalOutputs();

    if (flyByWireOutput.out.sim.data_computed.tracking_mode_on) {
      return true;
    }

    // object to write with trim
    SimOutput output = {flyByWireOutput.out.output.eta_pos, flyByWireOutput.out.output.xi_pos, flyByWireOutput.out.output.zeta_pos};

    // send data via sim connect
    if (!simConnectInterface.sendData(output)) {
      std::cout << "WASM: Write data failed!" << endl;
      return false;
    }

    if (flyByWireOutput.out.output.eta_trim_deg_should_write) {
      // object to write without trim
      SimOutputEtaTrim output = {flyByWireOutput.out.output.eta_trim_deg};

      // send data via sim connect
      if (!simConnectInterface.sendData(output)) {
        std::cout << "WASM: Write data failed!" << endl;
        return false;
      }
    }

    if (flyByWireOutput.out.output.zeta_trim_pos_should_write) {
      // object to write without trim
      SimOutputZetaTrim output = {flyByWireOutput.out.output.zeta_trim_pos};

      // send data via sim connect
      if (!simConnectInterface.sendData(output)) {
        std::cout << "WASM: Write data failed!" << endl;
        return false;
      }
    }
  } else {
    // send data to client data to be read by simulink
    ClientDataAutopilotLaws clientDataLaws = {autopilotLawsOutput.ap_on,
                                              autopilotLawsOutput.flight_director.Theta_c_deg,
                                              autopilotLawsOutput.autopilot.Theta_c_deg,
                                              autopilotLawsOutput.flight_director.Phi_c_deg,
                                              autopilotLawsOutput.autopilot.Phi_c_deg,
                                              autopilotLawsOutput.autopilot.Beta_c_deg};
    simConnectInterface.setClientDataAutopilotLaws(clientDataLaws);
  }

  // success ----------------------------------------------------------------------------------------------------------
  return true;
}

bool FlyByWireInterface::updateAutothrust(double sampleTime) {
  // get sim data
  SimData simData = simConnectInterface.getSimData();

  // set position for 3D animation
  set_named_variable_value(idThrottlePosition3d_1, idThrottlePositionLookupTable3d.get(throttleAxis[0]->getTLA()));
  set_named_variable_value(idThrottlePosition3d_2, idThrottlePositionLookupTable3d.get(throttleAxis[1]->getTLA()));

  // set client data if needed
  if (!autoThrustEnabled || !autopilotStateMachineEnabled || !flyByWireEnabled) {
    ClientDataLocalVariablesAutothrust ClientDataLocalVariablesAutothrust = {
        simConnectInterface.getSimInputThrottles().ATHR_push,
        throttleAxis[0]->getTLA(),
        throttleAxis[1]->getTLA(),
        simData.ap_V_c_kn,
        get_named_variable_value(idFmgcV_LS),
        get_named_variable_value(idFmgcV_MAX),
        -45,   // REV
        19.5,  // IDLE
        80,    // CLB
        81,    // FLX
        81,    // MCT
        85,    // TOGA
        get_named_variable_value(idFmgcFlexTemperature),
        autopilotStateMachineOutput.autothrust_mode,
        simData.is_mach_mode_active,  // mach mode
        0,                            // alpha floor
        autopilotStateMachineOutput.vertical_mode >= 30 && autopilotStateMachineOutput.vertical_mode <= 34,
        autopilotStateMachineOutput.vertical_mode == 40,
        autopilotStateMachineOutput.vertical_mode == 41,
        get_named_variable_value(idFmgcThrustReductionAltitude),
        get_named_variable_value(idFmgcThrustReductionAltitudeGoAround),
    };
    simConnectInterface.setClientDataLocalVariablesAutothrust(ClientDataLocalVariablesAutothrust);
  }

  if (autoThrustEnabled) {
    autoThrustInput.in.time.dt = sampleTime;
    autoThrustInput.in.time.simulation_time = simData.simulationTime;

    autoThrustInput.in.data.nz_g = simData.nz_g;
    autoThrustInput.in.data.Theta_deg = simData.Theta_deg;
    autoThrustInput.in.data.Phi_deg = simData.Phi_deg;
    autoThrustInput.in.data.V_ias_kn = simData.V_ias_kn;
    autoThrustInput.in.data.V_tas_kn = simData.V_tas_kn;
    autoThrustInput.in.data.V_mach = simData.V_mach;
    autoThrustInput.in.data.V_gnd_kn = simData.V_gnd_kn;
    autoThrustInput.in.data.alpha_deg = simData.alpha_deg;
    autoThrustInput.in.data.H_ft = simData.H_ft;
    autoThrustInput.in.data.H_ind_ft = simData.H_ind_ft;
    autoThrustInput.in.data.H_radio_ft = simData.H_radio_ft;
    autoThrustInput.in.data.H_dot_fpm = simData.H_dot_fpm;
    autoThrustInput.in.data.bx_m_s2 = simData.bx_m_s2;
    autoThrustInput.in.data.by_m_s2 = simData.by_m_s2;
    autoThrustInput.in.data.bz_m_s2 = simData.bz_m_s2;
    autoThrustInput.in.data.gear_strut_compression_1 = simData.gear_animation_pos_1;
    autoThrustInput.in.data.gear_strut_compression_2 = simData.gear_animation_pos_2;
    autoThrustInput.in.data.flap_handle_index = simData.flaps_handle_index;
    autoThrustInput.in.data.is_engine_operative_1 = simData.engine_combustion_1;
    autoThrustInput.in.data.is_engine_operative_2 = simData.engine_combustion_2;
    autoThrustInput.in.data.commanded_engine_N1_1_percent = simData.commanded_engine_N1_1_percent;
    autoThrustInput.in.data.commanded_engine_N1_2_percent = simData.commanded_engine_N1_2_percent;
    autoThrustInput.in.data.engine_N1_1_percent = simData.engine_N1_1_percent;
    autoThrustInput.in.data.engine_N1_2_percent = simData.engine_N1_2_percent;
    autoThrustInput.in.data.corrected_engine_N1_1_percent = simData.corrected_engine_N1_1_percent;
    autoThrustInput.in.data.corrected_engine_N1_2_percent = simData.corrected_engine_N1_2_percent;
    autoThrustInput.in.data.SAT_degC = simData.ambient_temperature_celsius;

    autoThrustInput.in.input.ATHR_push = simConnectInterface.getSimInputThrottles().ATHR_push;
    autoThrustInput.in.input.TLA_1_deg = throttleAxis[0]->getTLA();
    autoThrustInput.in.input.TLA_2_deg = throttleAxis[1]->getTLA();
    autoThrustInput.in.input.V_c_kn = simData.ap_V_c_kn;
    autoThrustInput.in.input.V_LS_kn = get_named_variable_value(idFmgcV_LS);
    autoThrustInput.in.input.V_MAX_kn = get_named_variable_value(idFmgcV_MAX);
    autoThrustInput.in.input.thrust_limit_REV_percent = -45;
    autoThrustInput.in.input.thrust_limit_IDLE_percent = 19.5;
    autoThrustInput.in.input.thrust_limit_CLB_percent = 80.0;
    autoThrustInput.in.input.thrust_limit_MCT_percent = 81.0;
    autoThrustInput.in.input.thrust_limit_FLEX_percent = 81.0;
    autoThrustInput.in.input.thrust_limit_TOGA_percent = 85.0;
    autoThrustInput.in.input.flex_temperature_degC = get_named_variable_value(idFmgcFlexTemperature);
    autoThrustInput.in.input.mode_requested = autopilotStateMachineOutput.autothrust_mode;
    autoThrustInput.in.input.is_mach_mode_active = simData.is_mach_mode_active;
    autoThrustInput.in.input.alpha_floor_condition = 0;
    autoThrustInput.in.input.is_approach_mode_active =
        autopilotStateMachineOutput.vertical_mode >= 30 && autopilotStateMachineOutput.vertical_mode <= 34;
    autoThrustInput.in.input.is_SRS_TO_mode_active = autopilotStateMachineOutput.vertical_mode == 40;
    autoThrustInput.in.input.is_SRS_GA_mode_active = autopilotStateMachineOutput.vertical_mode == 41;
    autoThrustInput.in.input.thrust_reduction_altitude = get_named_variable_value(idFmgcThrustReductionAltitude);
    autoThrustInput.in.input.thrust_reduction_altitude_go_around = get_named_variable_value(idFmgcThrustReductionAltitudeGoAround);

    // step the model -------------------------------------------------------------------------------------------------
    autoThrust.setExternalInputs(&autoThrustInput);
    autoThrust.step();

    // get output from model ------------------------------------------------------------------------------------------
    autoThrustOutput = autoThrust.getExternalOutputs().out.output;

    // write output to sim --------------------------------------------------------------------------------------------
    SimOutputThrottles simOutputThrottles = {autoThrustOutput.sim_throttle_lever_1_pos, autoThrustOutput.sim_throttle_lever_2_pos,
                                             autoThrustOutput.sim_thrust_mode_1, autoThrustOutput.sim_thrust_mode_2};
    if (!simConnectInterface.sendData(simOutputThrottles)) {
      std::cout << "WASM: Write data failed!" << endl;
      return false;
    }
  } else {
    // read data from client data
    ClientDataAutothrust clientData = simConnectInterface.getClientDataAutothrust();
    autoThrustOutput.N1_TLA_1_percent = clientData.N1_TLA_1_percent;
    autoThrustOutput.N1_TLA_2_percent = clientData.N1_TLA_2_percent;
    autoThrustOutput.is_in_reverse_1 = clientData.is_in_reverse_1;
    autoThrustOutput.is_in_reverse_2 = clientData.is_in_reverse_2;
    autoThrustOutput.thrust_limit_type = static_cast<athr_thrust_limit_type>(clientData.thrust_limit_type);
    autoThrustOutput.thrust_limit_percent = clientData.thrust_limit_percent;
    autoThrustOutput.N1_c_1_percent = clientData.N1_c_1_percent;
    autoThrustOutput.N1_c_2_percent = clientData.N1_c_2_percent;
    autoThrustOutput.status = static_cast<athr_status>(clientData.status);
    autoThrustOutput.mode = static_cast<athr_mode>(clientData.mode);
    autoThrustOutput.mode_message = static_cast<athr_mode_message>(clientData.mode_message);
  }

  // update local variables
  set_named_variable_value(idAutothrustN1_TLA_1, autoThrustOutput.N1_TLA_1_percent);
  set_named_variable_value(idAutothrustN1_TLA_2, autoThrustOutput.N1_TLA_2_percent);
  set_named_variable_value(idAutothrustReverse_1, autoThrustOutput.is_in_reverse_1);
  set_named_variable_value(idAutothrustReverse_2, autoThrustOutput.is_in_reverse_2);
  set_named_variable_value(idAutothrustThrustLimitType, autoThrustOutput.thrust_limit_type);
  set_named_variable_value(idAutothrustThrustLimit, autoThrustOutput.thrust_limit_percent);
  set_named_variable_value(idAutothrustN1_c_1, autoThrustOutput.N1_c_1_percent);
  set_named_variable_value(idAutothrustN1_c_2, autoThrustOutput.N1_c_2_percent);
  set_named_variable_value(idAutothrustStatus, autoThrustOutput.status);
  set_named_variable_value(idAutothrustMode, autoThrustOutput.mode);
  set_named_variable_value(idAutothrustModeMessage, autoThrustOutput.mode_message);

  // reset button state
  simConnectInterface.resetSimInputThrottles();

  // success
  return true;
}

double FlyByWireInterface::smoothFlightDirector(double sampleTime, double factor, double limit, double currentValue, double targetValue) {
  double difference = (targetValue - currentValue);
  if (difference >= 0) {
    difference = min(+1.0 * limit, difference);
  } else {
    difference = max(-1.0 * limit, difference);
  }
  return currentValue + (difference * min(1.0, sampleTime * factor));
}
