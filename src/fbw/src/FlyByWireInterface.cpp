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
#include <iomanip>
#include <iostream>

#include "FlyByWireInterface.h"
#include "SimConnectData.h"

using namespace std;

bool FlyByWireInterface::connect() {
  // setup local variables
  setupLocalVariables();

  // set rate for throttle override
  rateLimiterEngine_1.setRate(3);
  rateLimiterEngine_2.setRate(3);

  // initialize throttle system
  initializeThrottles();

  // initialize model
  autopilotStateMachine.initialize();
  autopilotLaws.initialize();
  flyByWire.initialize();

  // initialize flight data recorder
  flightDataRecorder.initialize();

  // read config for models
  loadConfiguration();

  // connect to sim connect
  return simConnectInterface.connect(isThrottleHandlingEnabled, idleThrottleInput, useReverseOnAxis,
                                     autopilotStateMachineEnabled, autopilotLawsEnabled, flyByWireEnabled);
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
}

bool FlyByWireInterface::update(double sampleTime) {
  bool result = true;

  // get data & inputs
  result &= readDataAndLocalVariables(sampleTime);

  // update autopilot state machine
  result &= updateAutopilotStateMachine(sampleTime);

  // update autopilot laws
  result &= updateAutopilotLaws(sampleTime);

  // update fly-by-wire
  result &= updateFlyByWire(sampleTime);

  // get throttle data and process it
  if (isThrottleHandlingEnabled) {
    result &= processThrottles();
  }

  // update flight data recorder
  flightDataRecorder.update(&autopilotStateMachine, &autopilotLaws, &flyByWire);

  // return result
  return result;
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
        get_named_variable_value(idFlightPhase),
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
        get_named_variable_value(idFmgcCruiseAltitude),
        0,
        get_named_variable_value(idFcuTrkFpaModeActive),
        get_named_variable_value(idFcuSelectedVs),
        get_named_variable_value(idFcuSelectedFpa),
        get_named_variable_value(idFcuSelectedHeading),
        customFlightGuidanceEnabled ? get_named_variable_value(idFlightGuidanceCrossTrackError)
                                    : simData.gpsWpCrossTrack,
        customFlightGuidanceEnabled ? get_named_variable_value(idFlightGuidanceTrackAngleError)
                                    : simData.gpsWpTrackAngleError,
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
    autopilotStateMachine.AutopilotStateMachine_U.in.time.dt = sampleTime;
    autopilotStateMachine.AutopilotStateMachine_U.in.time.simulation_time = simData.simulationTime;

    // data -----------------------------------------------------------------------------------------------------------
    autopilotStateMachine.AutopilotStateMachine_U.in.data.Theta_deg = simData.Theta_deg;
    autopilotStateMachine.AutopilotStateMachine_U.in.data.Phi_deg = simData.Phi_deg;
    autopilotStateMachine.AutopilotStateMachine_U.in.data.q_rad_s = simData.bodyRotationVelocity.x;
    autopilotStateMachine.AutopilotStateMachine_U.in.data.r_rad_s = simData.bodyRotationVelocity.y;
    autopilotStateMachine.AutopilotStateMachine_U.in.data.p_rad_s = simData.bodyRotationVelocity.z;
    autopilotStateMachine.AutopilotStateMachine_U.in.data.V_ias_kn = simData.V_ias_kn;
    autopilotStateMachine.AutopilotStateMachine_U.in.data.V_tas_kn = simData.V_tas_kn;
    autopilotStateMachine.AutopilotStateMachine_U.in.data.V_mach = simData.V_mach;
    autopilotStateMachine.AutopilotStateMachine_U.in.data.V_gnd_kn = simData.V_gnd_kn;
    autopilotStateMachine.AutopilotStateMachine_U.in.data.alpha_deg = simData.alpha_deg;
    autopilotStateMachine.AutopilotStateMachine_U.in.data.H_ft = simData.H_ft;
    autopilotStateMachine.AutopilotStateMachine_U.in.data.H_ind_ft = simData.H_ind_ft;
    autopilotStateMachine.AutopilotStateMachine_U.in.data.H_radio_ft = simData.H_radio_ft;
    autopilotStateMachine.AutopilotStateMachine_U.in.data.H_dot_ft_min = simData.H_dot_fpm;
    autopilotStateMachine.AutopilotStateMachine_U.in.data.Psi_magnetic_deg = simData.Psi_magnetic_deg;
    autopilotStateMachine.AutopilotStateMachine_U.in.data.Psi_magnetic_track_deg = simData.Psi_magnetic_track_deg;
    autopilotStateMachine.AutopilotStateMachine_U.in.data.Psi_true_deg = simData.Psi_true_deg;
    autopilotStateMachine.AutopilotStateMachine_U.in.data.bx_m_s2 = simData.bx_m_s2;
    autopilotStateMachine.AutopilotStateMachine_U.in.data.by_m_s2 = simData.by_m_s2;
    autopilotStateMachine.AutopilotStateMachine_U.in.data.bz_m_s2 = simData.bz_m_s2;
    autopilotStateMachine.AutopilotStateMachine_U.in.data.nav_valid = (simData.nav_valid != 0);
    autopilotStateMachine.AutopilotStateMachine_U.in.data.nav_loc_deg = simData.nav_loc_deg;
    autopilotStateMachine.AutopilotStateMachine_U.in.data.nav_dme_valid = (simData.nav_dme_valid != 0);
    autopilotStateMachine.AutopilotStateMachine_U.in.data.nav_dme_nmi = simData.nav_dme_nmi;
    autopilotStateMachine.AutopilotStateMachine_U.in.data.nav_loc_valid = (simData.nav_loc_valid != 0);
    autopilotStateMachine.AutopilotStateMachine_U.in.data.nav_loc_error_deg = simData.nav_loc_error_deg;
    autopilotStateMachine.AutopilotStateMachine_U.in.data.nav_gs_valid = (simData.nav_gs_valid != 0);
    autopilotStateMachine.AutopilotStateMachine_U.in.data.nav_gs_error_deg = simData.nav_gs_error_deg;
    autopilotStateMachine.AutopilotStateMachine_U.in.data.flight_guidance_xtk_nmi =
        customFlightGuidanceEnabled ? get_named_variable_value(idFlightGuidanceCrossTrackError)
                                    : simData.gpsWpCrossTrack;
    autopilotStateMachine.AutopilotStateMachine_U.in.data.flight_guidance_tae_deg =
        customFlightGuidanceEnabled ? get_named_variable_value(idFlightGuidanceTrackAngleError)
                                    : simData.gpsWpTrackAngleError;
    autopilotStateMachine.AutopilotStateMachine_U.in.data.flight_guidance_phi_deg =
        customFlightGuidanceEnabled ? get_named_variable_value(idFlightGuidancePhiCommand) : 0;
    autopilotStateMachine.AutopilotStateMachine_U.in.data.flight_phase = get_named_variable_value(idFlightPhase);
    autopilotStateMachine.AutopilotStateMachine_U.in.data.V2_kn = get_named_variable_value(idFmgcV2);
    autopilotStateMachine.AutopilotStateMachine_U.in.data.VAPP_kn = get_named_variable_value(idFmgcV_APP);
    autopilotStateMachine.AutopilotStateMachine_U.in.data.VLS_kn = get_named_variable_value(idFmgcV_LS);
    autopilotStateMachine.AutopilotStateMachine_U.in.data.is_flight_plan_available =
        customFlightGuidanceEnabled ? get_named_variable_value(idFlightGuidanceAvailable)
                                    : simData.gpsIsFlightPlanActive;
    autopilotStateMachine.AutopilotStateMachine_U.in.data.altitude_constraint_ft =
        get_named_variable_value(idFmgcAltitudeConstraint);
    autopilotStateMachine.AutopilotStateMachine_U.in.data.thrust_reduction_altitude =
        get_named_variable_value(idFmgcThrustReductionAltitude);
    autopilotStateMachine.AutopilotStateMachine_U.in.data.thrust_reduction_altitude_go_around =
        get_named_variable_value(idFmgcThrustReductionAltitudeGoAround);
    autopilotStateMachine.AutopilotStateMachine_U.in.data.acceleration_altitude =
        get_named_variable_value(idFmgcAccelerationAltitude);
    autopilotStateMachine.AutopilotStateMachine_U.in.data.acceleration_altitude_engine_out =
        get_named_variable_value(idFmgcAccelerationAltitudeEngineOut);
    autopilotStateMachine.AutopilotStateMachine_U.in.data.acceleration_altitude_go_around =
        get_named_variable_value(idFmgcAccelerationAltitudeGoAround);
    autopilotStateMachine.AutopilotStateMachine_U.in.data.cruise_altitude =
        get_named_variable_value(idFmgcCruiseAltitude);
    autopilotStateMachine.AutopilotStateMachine_U.in.data.throttle_lever_1_pos = simData.throttle_lever_1_pos;
    autopilotStateMachine.AutopilotStateMachine_U.in.data.throttle_lever_2_pos = simData.throttle_lever_2_pos;
    autopilotStateMachine.AutopilotStateMachine_U.in.data.gear_strut_compression_1 = simData.gear_animation_pos_1;
    autopilotStateMachine.AutopilotStateMachine_U.in.data.gear_strut_compression_2 = simData.gear_animation_pos_2;
    autopilotStateMachine.AutopilotStateMachine_U.in.data.zeta_pos = simData.zeta_pos;
    autopilotStateMachine.AutopilotStateMachine_U.in.data.flaps_handle_index = simData.flaps_handle_index;

    // input ----------------------------------------------------------------------------------------------------------
    autopilotStateMachine.AutopilotStateMachine_U.in.input.FD_active = simData.ap_fd_1_active | simData.ap_fd_2_active;
    autopilotStateMachine.AutopilotStateMachine_U.in.input.AP_1_push = simInputAutopilot.AP_1_push;
    autopilotStateMachine.AutopilotStateMachine_U.in.input.AP_2_push = simInputAutopilot.AP_2_push;
    autopilotStateMachine.AutopilotStateMachine_U.in.input.AP_DISCONNECT_push = simInputAutopilot.AP_disconnect;
    autopilotStateMachine.AutopilotStateMachine_U.in.input.HDG_push = simInputAutopilot.HDG_push;
    autopilotStateMachine.AutopilotStateMachine_U.in.input.HDG_pull = simInputAutopilot.HDG_pull;
    autopilotStateMachine.AutopilotStateMachine_U.in.input.ALT_push = simInputAutopilot.ALT_push;
    autopilotStateMachine.AutopilotStateMachine_U.in.input.ALT_pull = simInputAutopilot.ALT_pull;
    autopilotStateMachine.AutopilotStateMachine_U.in.input.VS_push = simInputAutopilot.VS_push;
    autopilotStateMachine.AutopilotStateMachine_U.in.input.VS_pull = simInputAutopilot.VS_pull;
    autopilotStateMachine.AutopilotStateMachine_U.in.input.LOC_push = simInputAutopilot.LOC_push;
    autopilotStateMachine.AutopilotStateMachine_U.in.input.APPR_push = simInputAutopilot.APPR_push;
    autopilotStateMachine.AutopilotStateMachine_U.in.input.V_fcu_kn = simData.ap_V_c_kn;
    autopilotStateMachine.AutopilotStateMachine_U.in.input.H_fcu_ft = simData.ap_H_c_ft;
    autopilotStateMachine.AutopilotStateMachine_U.in.input.H_constraint_ft =
        get_named_variable_value(idFmgcAltitudeConstraint);
    autopilotStateMachine.AutopilotStateMachine_U.in.input.H_dot_fcu_fpm = get_named_variable_value(idFcuSelectedVs);
    autopilotStateMachine.AutopilotStateMachine_U.in.input.FPA_fcu_deg = get_named_variable_value(idFcuSelectedFpa);
    autopilotStateMachine.AutopilotStateMachine_U.in.input.Psi_fcu_deg = get_named_variable_value(idFcuSelectedHeading);
    autopilotStateMachine.AutopilotStateMachine_U.in.input.TRK_FPA_mode =
        get_named_variable_value(idFcuTrkFpaModeActive);
    autopilotStateMachine.AutopilotStateMachine_U.in.input.DIR_TO_trigger = 0;

    // step the model -------------------------------------------------------------------------------------------------
    autopilotStateMachine.step();

    // result
    autopilotStateMachineOutput = autopilotStateMachine.AutopilotStateMachine_Y.out.output;
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
  set_named_variable_value(idAutopilotActiveAny,
                           autopilotStateMachineOutput.enabled_AP1 || autopilotStateMachineOutput.enabled_AP2);
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
  set_named_variable_value(idAutothrustMode, autopilotStateMachineOutput.autothrust_mode);

  if (autoThrustWorkaroundEnabled && simData.isAutoThrottleActive) {
    if (autopilotStateMachineOutput.autothrust_mode == 2) {
      // IDLE
      rateLimiterEngine_1.update(25, sampleTime);
      rateLimiterEngine_2.update(25, sampleTime);
      SimOutputEngineOverride override = {rateLimiterEngine_1.getValue(), rateLimiterEngine_2.getValue()};
      simConnectInterface.sendData(override);
    } else if (autopilotStateMachineOutput.autothrust_mode == 3) {
      // CLB
      double target = min(95, 80 + ((15.0 / 30000.0) * simData.H_ft));
      rateLimiterEngine_1.update(target, sampleTime);
      rateLimiterEngine_2.update(target, sampleTime);
      SimOutputEngineOverride override = {rateLimiterEngine_1.getValue(), rateLimiterEngine_2.getValue()};
      simConnectInterface.sendData(override);
    } else {
      // NONE or SPEED (in our case -> tracking mode)
      rateLimiterEngine_1.reset(simData.engine_n1_1);
      rateLimiterEngine_2.reset(simData.engine_n1_2);
    }
  }

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
  int autoThrustEngaged = simData.isAutoThrottleActive;
  bool radioAltimeterAvailable = (simData.H_radio_ft <= 5000);
  bool isCat1 = landModeArmedOrActive;
  bool isCat2 =
      landModeArmedOrActive && radioAltimeterAvailable && !autoThrustEngaged && numberOfAutopilotsEngaged >= 1;
  bool isCat3S =
      landModeArmedOrActive && radioAltimeterAvailable && autoThrustEngaged && numberOfAutopilotsEngaged >= 1;
  bool isCat3D =
      landModeArmedOrActive && radioAltimeterAvailable && autoThrustEngaged && numberOfAutopilotsEngaged == 2;
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
    autopilotLaws.AutopilotLaws_U.in.time.dt = sampleTime;
    autopilotLaws.AutopilotLaws_U.in.time.simulation_time = simData.simulationTime;

    // data -----------------------------------------------------------------------------------------------------------
    autopilotLaws.AutopilotLaws_U.in.data.Theta_deg = simData.Theta_deg;
    autopilotLaws.AutopilotLaws_U.in.data.Phi_deg = simData.Phi_deg;
    autopilotLaws.AutopilotLaws_U.in.data.q_rad_s = simData.bodyRotationVelocity.x;
    autopilotLaws.AutopilotLaws_U.in.data.r_rad_s = simData.bodyRotationVelocity.y;
    autopilotLaws.AutopilotLaws_U.in.data.p_rad_s = simData.bodyRotationVelocity.z;
    autopilotLaws.AutopilotLaws_U.in.data.V_ias_kn = simData.V_ias_kn;
    autopilotLaws.AutopilotLaws_U.in.data.V_tas_kn = simData.V_tas_kn;
    autopilotLaws.AutopilotLaws_U.in.data.V_mach = simData.V_mach;
    autopilotLaws.AutopilotLaws_U.in.data.V_gnd_kn = simData.V_gnd_kn;
    autopilotLaws.AutopilotLaws_U.in.data.alpha_deg = simData.alpha_deg;
    autopilotLaws.AutopilotLaws_U.in.data.H_ft = simData.H_ft;
    autopilotLaws.AutopilotLaws_U.in.data.H_ind_ft = simData.H_ind_ft;
    autopilotLaws.AutopilotLaws_U.in.data.H_radio_ft = simData.H_radio_ft;
    autopilotLaws.AutopilotLaws_U.in.data.H_dot_ft_min = simData.H_dot_fpm;
    autopilotLaws.AutopilotLaws_U.in.data.Psi_magnetic_deg = simData.Psi_magnetic_deg;
    autopilotLaws.AutopilotLaws_U.in.data.Psi_magnetic_track_deg = simData.Psi_magnetic_track_deg;
    autopilotLaws.AutopilotLaws_U.in.data.Psi_true_deg = simData.Psi_true_deg;
    autopilotLaws.AutopilotLaws_U.in.data.bx_m_s2 = simData.bx_m_s2;
    autopilotLaws.AutopilotLaws_U.in.data.by_m_s2 = simData.by_m_s2;
    autopilotLaws.AutopilotLaws_U.in.data.bz_m_s2 = simData.bz_m_s2;
    autopilotLaws.AutopilotLaws_U.in.data.nav_valid = (simData.nav_valid != 0);
    autopilotLaws.AutopilotLaws_U.in.data.nav_loc_deg = simData.nav_loc_deg;
    autopilotLaws.AutopilotLaws_U.in.data.nav_dme_valid = (simData.nav_dme_valid != 0);
    autopilotLaws.AutopilotLaws_U.in.data.nav_dme_nmi = simData.nav_dme_nmi;
    autopilotLaws.AutopilotLaws_U.in.data.nav_loc_valid = (simData.nav_loc_valid != 0);
    autopilotLaws.AutopilotLaws_U.in.data.nav_loc_error_deg = simData.nav_loc_error_deg;
    autopilotLaws.AutopilotLaws_U.in.data.nav_gs_valid = (simData.nav_gs_valid != 0);
    autopilotLaws.AutopilotLaws_U.in.data.nav_gs_error_deg = simData.nav_gs_error_deg;
    autopilotLaws.AutopilotLaws_U.in.data.flight_guidance_xtk_nmi =
        customFlightGuidanceEnabled ? get_named_variable_value(idFlightGuidanceCrossTrackError)
                                    : simData.gpsWpCrossTrack;
    autopilotLaws.AutopilotLaws_U.in.data.flight_guidance_tae_deg =
        customFlightGuidanceEnabled ? get_named_variable_value(idFlightGuidanceTrackAngleError)
                                    : simData.gpsWpTrackAngleError;
    autopilotLaws.AutopilotLaws_U.in.data.flight_guidance_phi_deg =
        customFlightGuidanceEnabled ? get_named_variable_value(idFlightGuidancePhiCommand) : 0;
    autopilotLaws.AutopilotLaws_U.in.data.flight_phase = get_named_variable_value(idFlightPhase);
    autopilotLaws.AutopilotLaws_U.in.data.V2_kn = get_named_variable_value(idFmgcV2);
    autopilotLaws.AutopilotLaws_U.in.data.VAPP_kn = get_named_variable_value(idFmgcV_APP);
    autopilotLaws.AutopilotLaws_U.in.data.VLS_kn = get_named_variable_value(idFmgcV_LS);
    autopilotLaws.AutopilotLaws_U.in.data.is_flight_plan_available =
        customFlightGuidanceEnabled ? get_named_variable_value(idFlightGuidanceAvailable)
                                    : simData.gpsIsFlightPlanActive;
    autopilotLaws.AutopilotLaws_U.in.data.altitude_constraint_ft = get_named_variable_value(idFmgcAltitudeConstraint);
    autopilotLaws.AutopilotLaws_U.in.data.thrust_reduction_altitude =
        get_named_variable_value(idFmgcThrustReductionAltitude);
    autopilotLaws.AutopilotLaws_U.in.data.thrust_reduction_altitude_go_around =
        get_named_variable_value(idFmgcThrustReductionAltitudeGoAround);
    autopilotLaws.AutopilotLaws_U.in.data.acceleration_altitude = get_named_variable_value(idFmgcAccelerationAltitude);
    autopilotLaws.AutopilotLaws_U.in.data.acceleration_altitude_engine_out =
        get_named_variable_value(idFmgcAccelerationAltitudeEngineOut);
    autopilotLaws.AutopilotLaws_U.in.data.acceleration_altitude_go_around =
        get_named_variable_value(idFmgcAccelerationAltitudeGoAround);
    autopilotLaws.AutopilotLaws_U.in.data.throttle_lever_1_pos = simData.throttle_lever_1_pos;
    autopilotLaws.AutopilotLaws_U.in.data.throttle_lever_2_pos = simData.throttle_lever_2_pos;
    autopilotLaws.AutopilotLaws_U.in.data.gear_strut_compression_1 = simData.gear_animation_pos_1;
    autopilotLaws.AutopilotLaws_U.in.data.gear_strut_compression_2 = simData.gear_animation_pos_2;
    autopilotLaws.AutopilotLaws_U.in.data.zeta_pos = simData.zeta_pos;
    autopilotLaws.AutopilotLaws_U.in.data.flaps_handle_index = simData.flaps_handle_index;

    // input ----------------------------------------------------------------------------------------------------------
    autopilotLaws.AutopilotLaws_U.in.input = autopilotStateMachineOutput;

    // step the model -------------------------------------------------------------------------------------------------
    autopilotLaws.step();

    // result ---------------------------------------------------------------------------------------------------------
    autopilotLawsOutput = autopilotLaws.AutopilotLaws_Y.out.output;
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
    flyByWire.FlyByWire_U.in.time.dt = sampleTime;
    flyByWire.FlyByWire_U.in.time.simulation_time = simData.simulationTime;

    // fill data into model -------------------------------------------------------------------------------------------
    flyByWire.FlyByWire_U.in.data.nz_g = simData.nz_g;
    flyByWire.FlyByWire_U.in.data.Theta_deg = simData.Theta_deg;
    flyByWire.FlyByWire_U.in.data.Phi_deg = simData.Phi_deg;
    flyByWire.FlyByWire_U.in.data.q_rad_s = simData.bodyRotationVelocity.x;
    flyByWire.FlyByWire_U.in.data.r_rad_s = simData.bodyRotationVelocity.y;
    flyByWire.FlyByWire_U.in.data.p_rad_s = simData.bodyRotationVelocity.z;
    flyByWire.FlyByWire_U.in.data.q_dot_rad_s2 = simData.bodyRotationAcceleration.x;
    flyByWire.FlyByWire_U.in.data.r_dot_rad_s2 = simData.bodyRotationAcceleration.y;
    flyByWire.FlyByWire_U.in.data.p_dot_rad_s2 = simData.bodyRotationAcceleration.z;
    flyByWire.FlyByWire_U.in.data.psi_magnetic_deg = simData.Psi_magnetic_deg;
    flyByWire.FlyByWire_U.in.data.psi_true_deg = simData.Psi_true_deg;
    flyByWire.FlyByWire_U.in.data.eta_pos = simData.eta_pos;
    flyByWire.FlyByWire_U.in.data.eta_trim_deg = simData.eta_trim_deg;
    flyByWire.FlyByWire_U.in.data.xi_pos = simData.xi_pos;
    flyByWire.FlyByWire_U.in.data.zeta_pos = simData.zeta_pos;
    flyByWire.FlyByWire_U.in.data.zeta_trim_pos = simData.zeta_trim_pos;
    flyByWire.FlyByWire_U.in.data.alpha_deg = simData.alpha_deg;
    flyByWire.FlyByWire_U.in.data.beta_deg = simData.beta_deg;
    flyByWire.FlyByWire_U.in.data.beta_dot_deg_s = simData.beta_dot_deg_s;
    flyByWire.FlyByWire_U.in.data.V_ias_kn = simData.V_ias_kn;
    flyByWire.FlyByWire_U.in.data.V_tas_kn = simData.V_tas_kn;
    flyByWire.FlyByWire_U.in.data.V_mach = simData.V_mach;
    flyByWire.FlyByWire_U.in.data.H_ft = simData.H_ft;
    flyByWire.FlyByWire_U.in.data.H_ind_ft = simData.H_ind_ft;
    flyByWire.FlyByWire_U.in.data.H_radio_ft = simData.H_radio_ft;
    flyByWire.FlyByWire_U.in.data.CG_percent_MAC = simData.CG_percent_MAC;
    flyByWire.FlyByWire_U.in.data.total_weight_kg = simData.total_weight_kg;
    flyByWire.FlyByWire_U.in.data.gear_animation_pos_0 = simData.gear_animation_pos_0;
    flyByWire.FlyByWire_U.in.data.gear_animation_pos_1 = simData.gear_animation_pos_1;
    flyByWire.FlyByWire_U.in.data.gear_animation_pos_2 = simData.gear_animation_pos_2;
    flyByWire.FlyByWire_U.in.data.flaps_handle_index = simData.flaps_handle_index;
    flyByWire.FlyByWire_U.in.data.spoilers_left_pos = simData.spoilers_left_pos;
    flyByWire.FlyByWire_U.in.data.spoilers_right_pos = simData.spoilers_right_pos;
    flyByWire.FlyByWire_U.in.data.autopilot_master_on = simData.autopilot_master_on;
    flyByWire.FlyByWire_U.in.data.slew_on = simData.slew_on;
    flyByWire.FlyByWire_U.in.data.pause_on = pauseDetected;
    flyByWire.FlyByWire_U.in.data.autopilot_custom_on = autopilotLawsOutput.ap_on;
    flyByWire.FlyByWire_U.in.data.autopilot_custom_Theta_c_deg = autopilotLawsOutput.autopilot.Theta_c_deg;
    flyByWire.FlyByWire_U.in.data.autopilot_custom_Phi_c_deg = autopilotLawsOutput.autopilot.Phi_c_deg;
    flyByWire.FlyByWire_U.in.data.autopilot_custom_Beta_c_deg = autopilotLawsOutput.autopilot.Beta_c_deg;
    flyByWire.FlyByWire_U.in.data.tracking_mode_on_override = 0;
    flyByWire.FlyByWire_U.in.data.simulation_rate = simData.simulation_rate;
    flyByWire.FlyByWire_U.in.data.ice_structure_percent = simData.ice_structure_percent;
    flyByWire.FlyByWire_U.in.data.linear_cl_alpha_per_deg = simData.linear_cl_alpha_per_deg;
    flyByWire.FlyByWire_U.in.data.alpha_stall_deg = simData.alpha_stall_deg;
    flyByWire.FlyByWire_U.in.data.alpha_zero_lift_deg = simData.alpha_zero_lift_deg;
    flyByWire.FlyByWire_U.in.data.ambient_density_kg_per_m3 = simData.ambient_density_kg_per_m3;
    flyByWire.FlyByWire_U.in.data.ambient_pressure_mbar = simData.ambient_pressure_mbar;
    flyByWire.FlyByWire_U.in.data.ambient_temperature_celsius = simData.ambient_temperature_celsius;
    flyByWire.FlyByWire_U.in.data.ambient_wind_x_kn = simData.ambient_wind_x_kn;
    flyByWire.FlyByWire_U.in.data.ambient_wind_y_kn = simData.ambient_wind_y_kn;
    flyByWire.FlyByWire_U.in.data.ambient_wind_z_kn = simData.ambient_wind_z_kn;
    flyByWire.FlyByWire_U.in.data.ambient_wind_velocity_kn = simData.ambient_wind_velocity_kn;
    flyByWire.FlyByWire_U.in.data.ambient_wind_direction_deg = simData.ambient_wind_direction_deg;
    flyByWire.FlyByWire_U.in.data.total_air_temperature_celsius = simData.total_air_temperature_celsius;
    flyByWire.FlyByWire_U.in.data.latitude_deg = simData.latitude_deg;
    flyByWire.FlyByWire_U.in.data.longitude_deg = simData.longitude_deg;
    flyByWire.FlyByWire_U.in.data.engine_1_thrust_lbf = simData.engine_1_thrust_lbf;
    flyByWire.FlyByWire_U.in.data.engine_2_thrust_lbf = simData.engine_2_thrust_lbf;
    flyByWire.FlyByWire_U.in.data.thrust_lever_1_pos = simData.throttle_lever_1_pos;
    flyByWire.FlyByWire_U.in.data.thrust_lever_2_pos = simData.throttle_lever_2_pos;

    // process the sidestick handling ---------------------------------------------------------------------------------
    // use the values read from input as sidestick left
    double sideStickLeftPositionX = -1.0 * simInput.inputs[1];
    double sideStickLeftPositionY = -1.0 * simInput.inputs[0];
    // read the values from sidestick right
    double sideStickRightPositionX = get_named_variable_value(idSideStickRightPositionX);
    double sideStickRightPositionY = get_named_variable_value(idSideStickRightPositionY);
    // add them together and clamp them
    double sideStickPositionX = sideStickLeftPositionX + sideStickRightPositionX;
    sideStickPositionX = min(1.0, sideStickPositionX);
    sideStickPositionX = max(-1.0, sideStickPositionX);
    double sideStickPositionY = sideStickLeftPositionY + sideStickRightPositionY;
    sideStickPositionY = min(1.0, sideStickPositionY);
    sideStickPositionY = max(-1.0, sideStickPositionY);
    // write them as sidestick position
    set_named_variable_value(idSideStickLeftPositionX, sideStickLeftPositionX);
    set_named_variable_value(idSideStickLeftPositionY, sideStickLeftPositionY);
    set_named_variable_value(idSideStickPositionX, sideStickPositionX);
    set_named_variable_value(idSideStickPositionY, sideStickPositionY);

    // rudder handling
    double rudderPositionOverrideOn = get_named_variable_value(idRudderPositionOverrideOn);
    double rudderPosition = simInput.inputs[2];
    if (rudderPositionOverrideOn == 0) {
      set_named_variable_value(idRudderPosition, rudderPosition);
    } else {
      rudderPosition = get_named_variable_value(idRudderPosition);
    }

    // fill inputs into model
    flyByWire.FlyByWire_U.in.input.delta_eta_pos = -1.0 * sideStickPositionY;
    flyByWire.FlyByWire_U.in.input.delta_xi_pos = -1.0 * sideStickPositionX;
    flyByWire.FlyByWire_U.in.input.delta_zeta_pos = rudderPosition;

    // step the model -------------------------------------------------------------------------------------------------
    flyByWire.step();

    // when tracking mode is on do not write anything -----------------------------------------------------------------
    if (flyByWire.FlyByWire_Y.out.sim.data_computed.tracking_mode_on) {
      return true;
    }

    // object to write with trim
    SimOutput output = {flyByWire.FlyByWire_Y.out.output.eta_pos, flyByWire.FlyByWire_Y.out.output.xi_pos,
                        flyByWire.FlyByWire_Y.out.output.zeta_pos};

    // send data via sim connect
    if (!simConnectInterface.sendData(output)) {
      std::cout << "WASM: Write data failed!" << endl;
      return false;
    }

    if (flyByWire.FlyByWire_Y.out.output.eta_trim_deg_should_write) {
      // object to write without trim
      SimOutputEtaTrim output = {flyByWire.FlyByWire_Y.out.output.eta_trim_deg};

      // send data via sim connect
      if (!simConnectInterface.sendData(output)) {
        std::cout << "WASM: Write data failed!" << endl;
        return false;
      }
    }

    if (flyByWire.FlyByWire_Y.out.output.zeta_trim_pos_should_write) {
      // object to write without trim
      SimOutputZetaTrim output = {flyByWire.FlyByWire_Y.out.output.zeta_trim_pos};

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

void FlyByWireInterface::setupLocalVariables() {
  // register L variables for the sidestick
  idSideStickPositionX = register_named_variable("A32NX_SIDESTICK_POSITION_X");
  idSideStickPositionY = register_named_variable("A32NX_SIDESTICK_POSITION_Y");
  // register L variables for the sidestick on the left side
  idSideStickLeftPositionX = register_named_variable("A32NX_SIDESTICK_LEFT_POSITION_X");
  idSideStickLeftPositionY = register_named_variable("A32NX_SIDESTICK_LEFT_POSITION_Y");
  // register L variables for the sidestick on the right side
  idSideStickRightPositionX = register_named_variable("A32NX_SIDESTICK_RIGHT_POSITION_X");
  idSideStickRightPositionY = register_named_variable("A32NX_SIDESTICK_RIGHT_POSITION_Y");
  // register L variables for the rudder handling
  idRudderPositionOverrideOn = register_named_variable("A32NX_RUDDER_POSITION_OVERRIDE_ON");
  idRudderPosition = register_named_variable("A32NX_RUDDER_POSITION");
  // register L variables for the throttle handling
  idThrottlePositionOverrideOn = register_named_variable("A32NX_THROTTLE_POSITION_OVERRIDE_ON");
  idThrottlePosition_1 = register_named_variable("A32NX_THROTTLE_POSITION_1");
  idThrottlePosition_2 = register_named_variable("A32NX_THROTTLE_POSITION_2");

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

  idAutothrustMode = register_named_variable("A32NX_AUTOPILOT_AUTOTHRUST_MODE");

  // register L variables for flight guidance
  idFlightPhase = register_named_variable("A32NX_FWC_FLIGHT_PHASE");
  idFmgcV2 = register_named_variable("AIRLINER_V2_SPEED");
  idFmgcV_APP = register_named_variable("AIRLINER_VAPP_SPEED");
  idFmgcV_LS = register_named_variable("AIRLINER_VLS_SPEED");

  // idFmgcFlightPlanAvailable = register_named_variable("X");
  idFmgcAltitudeConstraint = register_named_variable("A32NX_AP_CSTN_ALT");
  idFmgcThrustReductionAltitude = register_named_variable("AIRLINER_THR_RED_ALT");
  idFmgcThrustReductionAltitudeGoAround = register_named_variable("AIRLINER_THR_RED_ALT_GOAROUND");
  idFmgcAccelerationAltitude = register_named_variable("AIRLINER_ACC_ALT");
  idFmgcAccelerationAltitudeEngineOut = register_named_variable("AIRLINER_ACC_ALT_ENGINEOUT");
  idFmgcAccelerationAltitudeGoAround = register_named_variable("AIRLINER_ACC_ALT_GOAROUND");
  idFmgcCruiseAltitude = register_named_variable("AIRLINER_CRUISE_ALTITUDE");

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
}

void FlyByWireInterface::loadConfiguration() {
  INIReader configuration(MODEL_CONFIGURATION_FILEPATH);
  autopilotStateMachineEnabled = configuration.GetBoolean("Model", "AutopilotStateMachineEnabled", true);
  autopilotLawsEnabled = configuration.GetBoolean("Model", "AutopilotLawsEnabled", true);
  flyByWireEnabled = configuration.GetBoolean("Model", "FlyByWireEnabled", true);
  customFlightGuidanceEnabled = configuration.GetBoolean("Autopilot", "CustomFlightGuidanceEnabled", false);
  flightDirectorSmoothingEnabled = configuration.GetBoolean("Autopilot", "FlightDirectorSmoothingEnabled", true);
  flightDirectorSmoothingFactor = configuration.GetReal("Autopilot", "FlightDirectorSmoothingFactor", 2.5);
  flightDirectorSmoothingLimit = configuration.GetReal("Autopilot", "FlightDirectorSmoothingLimit", 20);
  autoThrustWorkaroundEnabled = configuration.GetBoolean("Autopilot", "AutoThrustWorkaroundEnabled", true);
  std::cout << "WASM: Model Configuration : AutopilotStateMachineEnabled = " << autopilotStateMachineEnabled << endl;
  std::cout << "WASM: Model Configuration : AutopilotLawsEnabled         = " << autopilotLawsEnabled << endl;
  std::cout << "WASM: Model Configuration : FlyByWireEnabled             = " << flyByWireEnabled << endl;
  std::cout << "WASM: Autopilot Configuration : CustomFlightGuidanceEnabled = " << customFlightGuidanceEnabled << endl;
  std::cout << "WASM: Autopilot Configuration : FlightDirectorSmoothingEnabled = " << flightDirectorSmoothingEnabled
            << endl;
  std::cout << "WASM: Autopilot Configuration : FlightDirectorSmoothingFactor = " << flightDirectorSmoothingFactor
            << endl;
  std::cout << "WASM: Autopilot Configuration : FlightDirectorSmoothingLimit = " << flightDirectorSmoothingLimit
            << endl;
  std::cout << "WASM: Autopilot Configuration : AutoThrustWorkaroundEnabled = " << autoThrustWorkaroundEnabled << endl;
}

void FlyByWireInterface::initializeThrottles() {
  // read configuration
  INIReader configuration(THROTTLE_CONFIGURATION_FILEPATH);
  if (configuration.ParseError() < 0) {
    // file does not exist yet -> store the default configuration in a file
    ofstream configFile;
    configFile.open(THROTTLE_CONFIGURATION_FILEPATH);
    configFile << "[Throttle]" << endl;
    configFile << "Log = true" << endl;
    configFile << "Enabled = true" << endl;
    configFile << "ReverseOnAxis = false" << endl;
    configFile << "ReverseIdle = true" << endl;
    configFile << "DetentDeadZone = 2.0" << endl;
    configFile << "DetentReverseFull = -1.00" << endl;
    configFile << "DetentReverseIdle = -0.90" << endl;
    configFile << "DetentIdle = -1.00" << endl;
    configFile << "DetentClimb = 0.89" << endl;
    configFile << "DetentFlexMct = 0.95" << endl;
    configFile << "DetentTakeOffGoAround = 1.00" << endl;
    configFile.close();
  }

  // read basic configuration
  isThrottleLoggingEnabled = configuration.GetBoolean("Throttle", "Log", true);
  isThrottleHandlingEnabled = configuration.GetBoolean("Throttle", "Enabled", true);
  useReverseOnAxis = configuration.GetBoolean("Throttle", "ReverseOnAxis", false);
  useReverseIdle = configuration.GetBoolean("Throttle", "ReverseIdle", false);
  throttleDetentDeadZone = configuration.GetReal("Throttle", "DetentDeadZone", 0.0);
  // read mapping configuration
  vector<pair<double, double>> mappingTable;
  if (useReverseOnAxis) {
    mappingTable.emplace_back(configuration.GetReal("Throttle", "DetentReverseFull", -1.00), -20.00);
    if (useReverseIdle) {
      mappingTable.emplace_back(configuration.GetReal("Throttle", "DetentReverseIdle", -0.70), -5.00);
    }
  }
  mappingTable.emplace_back(configuration.GetReal("Throttle", "DetentIdle", useReverseOnAxis ? 0.00 : -1.00), 0.00);
  mappingTable.emplace_back(configuration.GetReal("Throttle", "DetentClimb", 0.89), 89.00);
  mappingTable.emplace_back(configuration.GetReal("Throttle", "DetentFlexMct", 0.95), 95.00);
  mappingTable.emplace_back(configuration.GetReal("Throttle", "DetentTakeOffGoAround", 1.00), 100.00);

  // remember idle throttle setting
  if (useReverseOnAxis) {
    if (useReverseIdle) {
      idleThrottleInput = mappingTable[2].first;
    } else {
      idleThrottleInput = mappingTable[1].first;
    }
  } else {
    idleThrottleInput = mappingTable[0].first;
  }

  // print config
  std::cout << "WASM: Throttle Configuration : Log                   = " << isThrottleLoggingEnabled << endl;
  std::cout << "WASM: Throttle Configuration : Enabled               = " << isThrottleHandlingEnabled << endl;
  std::cout << "WASM: Throttle Configuration : ReverseOnAxis         = " << useReverseOnAxis << endl;
  std::cout << "WASM: Throttle Configuration : ReverseIdle           = " << useReverseIdle << endl;
  int index = 0;
  if (useReverseOnAxis) {
    std::cout << "WASM: Throttle Configuration : DetentReverseFull     = " << mappingTable[index++].first << endl;
    if (useReverseIdle) {
      std::cout << "WASM: Throttle Configuration : DetentReverseIdle     = " << mappingTable[index++].first << endl;
    }
  }
  std::cout << "WASM: Throttle Configuration : DetentIdle            = " << mappingTable[index++].first << endl;
  std::cout << "WASM: Throttle Configuration : DetentClimb           = " << mappingTable[index++].first << endl;
  std::cout << "WASM: Throttle Configuration : DetentFlexMct         = " << mappingTable[index++].first << endl;
  std::cout << "WASM: Throttle Configuration : DetentTakeOffGoAround = " << mappingTable[index++].first << endl;

  // initialize lookup table
  throttleLookupTable.initialize(mappingTable, -20, 100);
}

bool FlyByWireInterface::processThrottles() {
  // get data from simconnect
  auto simInputThrottles = simConnectInterface.getSimInputThrottles();

  // process the data (lut)
  SimOutputThrottles simOutputThrottles = {throttleLookupTable.get(simInputThrottles.throttles[0]),
                                           throttleLookupTable.get(simInputThrottles.throttles[1])};

  // detect reverse situation
  if (!useReverseOnAxis && simConnectInterface.getIsReverseToggleActive(0)) {
    simOutputThrottles.throttleLeverPosition_1 = -10.0 * (simInputThrottles.throttles[0] + 1);
  }
  if (!useReverseOnAxis && simConnectInterface.getIsReverseToggleActive(1)) {
    simOutputThrottles.throttleLeverPosition_2 = -10.0 * (simInputThrottles.throttles[1] + 1);
  }

  // detect if override is active
  if (get_named_variable_value(idThrottlePositionOverrideOn) == 0) {
    set_named_variable_value(idThrottlePosition_1, simOutputThrottles.throttleLeverPosition_1);
    set_named_variable_value(idThrottlePosition_2, simOutputThrottles.throttleLeverPosition_2);
  } else {
    simOutputThrottles.throttleLeverPosition_1 = get_named_variable_value(idThrottlePosition_1);
    simOutputThrottles.throttleLeverPosition_2 = get_named_variable_value(idThrottlePosition_2);
  }

  // clip when aircraft is in flight
  if (!flyByWire.FlyByWire_Y.out.sim.data_computed.on_ground) {
    simOutputThrottles.throttleLeverPosition_1 = max(0, simOutputThrottles.throttleLeverPosition_1);
    simOutputThrottles.throttleLeverPosition_2 = max(0, simOutputThrottles.throttleLeverPosition_2);
  }

  // add deadzone around detents
  simOutputThrottles.throttleLeverPosition_1 =
      calculateDeadzones(throttleDetentDeadZone, simOutputThrottles.throttleLeverPosition_1);
  simOutputThrottles.throttleLeverPosition_2 =
      calculateDeadzones(throttleDetentDeadZone, simOutputThrottles.throttleLeverPosition_2);

  // if enabled, print values
  if (isThrottleLoggingEnabled) {
    if (lastThrottleInput_1 != simInputThrottles.throttles[0] ||
        lastThrottleInput_2 != simInputThrottles.throttles[1]) {
      // print values
      std::cout << fixed << setprecision(2) << "WASM";
      std::cout << " : Throttle 1: " << setw(5) << simInputThrottles.throttles[0];
      std::cout << " -> " << setw(6) << simOutputThrottles.throttleLeverPosition_1;
      std::cout << " ; Throttle 2: " << setw(5) << simInputThrottles.throttles[1];
      std::cout << " -> " << setw(6) << simOutputThrottles.throttleLeverPosition_2;
      std::cout << endl;

      // store values for next iteration
      lastThrottleInput_1 = simInputThrottles.throttles[0];
      lastThrottleInput_2 = simInputThrottles.throttles[1];
    }
  }

  // write output to sim
  if (!simConnectInterface.sendData(simOutputThrottles)) {
    std::cout << "WASM: Write data failed!" << endl;
    return false;
  }

  // determine if autothrust armed event needs to be triggered
  if (simConnectInterface.getIsAutothrottlesArmed() && simOutputThrottles.throttleLeverPosition_1 < 1 &&
      simOutputThrottles.throttleLeverPosition_2 < 1) {
    if (!simConnectInterface.sendAutoThrustArmEvent()) {
      std::cout << "WASM: Write data failed!" << endl;
      return false;
    }
  }

  // success
  return true;
}

double FlyByWireInterface::calculateDeadzones(double deadzone, double input) {
  double result = input;
  if (useReverseOnAxis) {
    result = calculateDeadzone(deadzone, -20.0, result);
    if (useReverseIdle) {
      result = calculateDeadzone(deadzone, -5.0, result);
    }
  }
  result = calculateDeadzone(deadzone, 0.0, result);
  result = calculateDeadzone(deadzone, 89.0, result);
  result = calculateDeadzone(deadzone, 95.0, result);
  return result;
}

double FlyByWireInterface::calculateDeadzone(double deadzone, double target, double input) {
  if (input <= (target + deadzone) && input >= (target - deadzone)) {
    return target;
  }
  return input;
}

double FlyByWireInterface::smoothFlightDirector(double sampleTime,
                                                double factor,
                                                double limit,
                                                double currentValue,
                                                double targetValue) {
  double difference = (targetValue - currentValue);
  if (difference >= 0) {
    difference = min(+1.0 * limit, difference);
  } else {
    difference = max(-1.0 * limit, difference);
  }
  return currentValue + (difference * min(1.0, sampleTime * factor));
}
