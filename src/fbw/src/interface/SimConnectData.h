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

struct SimData {
  double nz_g;
  double Theta_deg;
  double Phi_deg;
  SIMCONNECT_DATA_XYZ bodyRotationVelocity;
  SIMCONNECT_DATA_XYZ bodyRotationAcceleration;
  double psi_magnetic_deg;
  double psi_true_deg;
  double eta_pos;
  double eta_trim_deg;
  double xi_pos;
  double zeta_pos;
  double zeta_trim_pos;
  double alpha_deg;
  double beta_deg;
  double beta_dot_deg_s;
  double V_ias_kn;
  double V_tas_kn;
  double V_mach;
  double H_ft;
  double H_ind_ft;
  double H_radio_ft;
  double CG_percent_MAC;
  double total_weight_kg;
  double gear_animation_pos_0;
  double gear_animation_pos_1;
  double gear_animation_pos_2;
  double flaps_handle_index;
  double spoilers_left_pos;
  double spoilers_right_pos;
  bool autopilot_master_on;
  bool slew_on;
  double simulationTime;
  double simulation_rate;
  double ice_structure_percent;
  double linear_cl_alpha_per_deg;
  double alpha_stall_deg;
  double alpha_zero_lift_deg;
  double ambient_density_kg_per_m3;
  double ambient_pressure_mbar;
  double ambient_temperature_celsius;
  double ambient_wind_x_kn;
  double ambient_wind_y_kn;
  double ambient_wind_z_kn;
  double ambient_wind_velocity_kn;
  double ambient_wind_direction_deg;
  double total_air_temperature_celsius;
  double latitude_deg;
  double longitude_deg;
  double throttle_lever_1_pos;
  double throttle_lever_2_pos;
  double engine_1_thrust_lbf;
  double engine_2_thrust_lbf;
};

struct SimInput {
  double inputs[3];
};

struct SimInputThrottles {
  double throttles[2];
};

struct SimOutput {
  double eta;
  double xi;
  double zeta;
};

struct SimOutputEtaTrim {
  double eta_trim_deg;
};

struct SimOutputZetaTrim {
  double zeta_trim_pos;
};

struct SimOutputThrottles {
  double throttleLeverPosition_1;
  double throttleLeverPosition_2;
};

struct SimInputClientDataAutopilot {
  unsigned long long enableAutopilot;
  double flightDirectorTheta;
  double autopilotTheta;
  double flightDirectorPhi;
  double autopilotPhi;
  double autopilotBeta;
  double fmaLateralMode;
  double fmaLateralArmed;
  double fmaVerticalMode;
  double fmaVerticalArmed;
};
