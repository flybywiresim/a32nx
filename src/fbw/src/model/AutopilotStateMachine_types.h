#ifndef RTW_HEADER_AutopilotStateMachine_types_h_
#define RTW_HEADER_AutopilotStateMachine_types_h_
#include "rtwtypes.h"
#include "multiword_types.h"
#ifndef DEFINED_TYPEDEF_FOR_ap_raw_time_
#define DEFINED_TYPEDEF_FOR_ap_raw_time_

typedef struct {
  real_T dt;
  real_T simulation_time;
} ap_raw_time;

#endif

#ifndef DEFINED_TYPEDEF_FOR_ap_raw_data_
#define DEFINED_TYPEDEF_FOR_ap_raw_data_

typedef struct {
  real_T Theta_deg;
  real_T Phi_deg;
  real_T q_rad_s;
  real_T r_rad_s;
  real_T p_rad_s;
  real_T V_ias_kn;
  real_T V_tas_kn;
  real_T V_mach;
  real_T V_gnd_kn;
  real_T alpha_deg;
  real_T H_ft;
  real_T H_ind_ft;
  real_T H_radio_ft;
  real_T H_dot_ft_min;
  real_T Psi_magnetic_deg;
  real_T Psi_magnetic_track_deg;
  real_T Psi_true_deg;
  real_T bx_m_s2;
  real_T by_m_s2;
  real_T bz_m_s2;
  boolean_T nav_valid;
  real_T nav_loc_deg;
  real_T nav_dme_valid;
  real_T nav_dme_nmi;
  boolean_T nav_loc_valid;
  real_T nav_loc_error_deg;
  boolean_T nav_gs_valid;
  real_T nav_gs_error_deg;
  real_T flight_guidance_xtk_nmi;
  real_T flight_guidance_tae_deg;
  real_T flight_guidance_phi_deg;
  real_T flight_phase;
  real_T V2_kn;
  real_T VAPP_kn;
  real_T VLS_kn;
  boolean_T is_flight_plan_available;
  real_T altitude_constraint_ft;
  real_T thrust_reduction_altitude;
  real_T thrust_reduction_altitude_go_around;
  real_T acceleration_altitude;
  real_T acceleration_altitude_engine_out;
  real_T acceleration_altitude_go_around;
  real_T cruise_altitude;
  real_T gear_strut_compression_1;
  real_T gear_strut_compression_2;
  real_T zeta_pos;
  real_T throttle_lever_1_pos;
  real_T throttle_lever_2_pos;
  real_T flaps_handle_index;
} ap_raw_data;

#endif

#ifndef DEFINED_TYPEDEF_FOR_ap_raw_sm_input_
#define DEFINED_TYPEDEF_FOR_ap_raw_sm_input_

typedef struct {
  boolean_T FD_active;
  boolean_T AP_1_push;
  boolean_T AP_2_push;
  boolean_T AP_DISCONNECT_push;
  boolean_T HDG_push;
  boolean_T HDG_pull;
  boolean_T ALT_push;
  boolean_T ALT_pull;
  boolean_T VS_push;
  boolean_T VS_pull;
  boolean_T LOC_push;
  boolean_T APPR_push;
  real_T V_fcu_kn;
  real_T Psi_fcu_deg;
  real_T H_fcu_ft;
  real_T H_constraint_ft;
  real_T H_dot_fcu_fpm;
  real_T FPA_fcu_deg;
  boolean_T TRK_FPA_mode;
  boolean_T DIR_TO_trigger;
} ap_raw_sm_input;

#endif

#ifndef DEFINED_TYPEDEF_FOR_ap_sm_input_
#define DEFINED_TYPEDEF_FOR_ap_sm_input_

typedef struct {
  ap_raw_time time;
  ap_raw_data data;
  ap_raw_sm_input input;
} ap_sm_input;

#endif

#ifndef DEFINED_TYPEDEF_FOR_ap_lateral_armed_
#define DEFINED_TYPEDEF_FOR_ap_lateral_armed_

typedef struct {
  boolean_T NAV;
  boolean_T LOC;
} ap_lateral_armed;

#endif

#ifndef DEFINED_TYPEDEF_FOR_ap_lateral_condition_
#define DEFINED_TYPEDEF_FOR_ap_lateral_condition_

typedef struct {
  boolean_T NAV;
  boolean_T LOC_CPT;
  boolean_T LOC_TRACK;
  boolean_T LAND;
  boolean_T FLARE;
  boolean_T ROLL_OUT;
  boolean_T GA_TRACK;
} ap_lateral_condition;

#endif

#ifndef DEFINED_TYPEDEF_FOR_ap_data_
#define DEFINED_TYPEDEF_FOR_ap_data_

typedef struct {
  real_T Theta_deg;
  real_T Phi_deg;
  real_T qk_deg_s;
  real_T rk_deg_s;
  real_T pk_deg_s;
  real_T V_ias_kn;
  real_T V_tas_kn;
  real_T V_mach;
  real_T V_gnd_kn;
  real_T alpha_deg;
  real_T H_ft;
  real_T H_ind_ft;
  real_T H_radio_ft;
  real_T H_dot_ft_min;
  real_T Psi_magnetic_deg;
  real_T Psi_magnetic_track_deg;
  real_T Psi_true_deg;
  real_T bx_m_s2;
  real_T by_m_s2;
  real_T bz_m_s2;
  boolean_T nav_valid;
  real_T nav_loc_deg;
  real_T nav_dme_valid;
  real_T nav_dme_nmi;
  boolean_T nav_loc_valid;
  real_T nav_loc_error_deg;
  boolean_T nav_gs_valid;
  real_T nav_gs_error_deg;
  real_T flight_guidance_xtk_nmi;
  real_T flight_guidance_tae_deg;
  real_T flight_guidance_phi_deg;
  real_T flight_phase;
  real_T V2_kn;
  real_T VAPP_kn;
  real_T VLS_kn;
  boolean_T is_flight_plan_available;
  real_T altitude_constraint_ft;
  real_T thrust_reduction_altitude;
  real_T thrust_reduction_altitude_go_around;
  real_T acceleration_altitude;
  real_T acceleration_altitude_engine_out;
  real_T acceleration_altitude_go_around;
  real_T cruise_altitude;
  real_T on_ground;
  real_T zeta_deg;
  real_T throttle_lever_1_pos;
  real_T throttle_lever_2_pos;
  real_T flaps_handle_index;
} ap_data;

#endif

#ifndef DEFINED_TYPEDEF_FOR_ap_sm_data_computed_
#define DEFINED_TYPEDEF_FOR_ap_sm_data_computed_

typedef struct {
  real_T time_since_touchdown;
  real_T time_since_lift_off;
  boolean_T H_fcu_in_selection;
  boolean_T H_constraint_valid;
  boolean_T Psi_fcu_in_selection;
  boolean_T gs_convergent_towards_beam;
  real_T H_dot_radio_fpm;
} ap_sm_data_computed;

#endif

#ifndef DEFINED_TYPEDEF_FOR_lateral_mode_
#define DEFINED_TYPEDEF_FOR_lateral_mode_

typedef enum {
  lateral_mode_NONE = 0,
  lateral_mode_HDG = 10,
  lateral_mode_TRACK = 11,
  lateral_mode_NAV = 20,
  lateral_mode_LOC_CPT = 30,
  lateral_mode_LOC_TRACK = 31,
  lateral_mode_LAND = 32,
  lateral_mode_FLARE = 33,
  lateral_mode_ROLL_OUT = 34,
  lateral_mode_RWY = 40,
  lateral_mode_RWY_TRACK = 41,
  lateral_mode_GA_TRACK = 50
} lateral_mode;

#endif

#ifndef DEFINED_TYPEDEF_FOR_lateral_law_
#define DEFINED_TYPEDEF_FOR_lateral_law_

typedef enum {
  lateral_law_NONE = 0,
  lateral_law_HDG,
  lateral_law_TRACK,
  lateral_law_HPATH,
  lateral_law_LOC_CPT,
  lateral_law_LOC_TRACK,
  lateral_law_ROLL_OUT
} lateral_law;

#endif

#ifndef DEFINED_TYPEDEF_FOR_ap_lateral_output_
#define DEFINED_TYPEDEF_FOR_ap_lateral_output_

typedef struct {
  lateral_mode mode;
  boolean_T mode_reversion;
  boolean_T mode_reversion_TRK_FPA;
  lateral_law law;
  real_T Psi_c_deg;
} ap_lateral_output;

#endif

#ifndef DEFINED_TYPEDEF_FOR_vertical_mode_
#define DEFINED_TYPEDEF_FOR_vertical_mode_

typedef enum {
  vertical_mode_NONE = 0,
  vertical_mode_ALT = 10,
  vertical_mode_ALT_CPT = 11,
  vertical_mode_OP_CLB = 12,
  vertical_mode_OP_DES = 13,
  vertical_mode_VS = 14,
  vertical_mode_FPA = 15,
  vertical_mode_ALT_CST = 20,
  vertical_mode_ALT_CST_CPT = 21,
  vertical_mode_CLB = 22,
  vertical_mode_DES = 23,
  vertical_mode_GS_CPT = 30,
  vertical_mode_GS_TRACK = 31,
  vertical_mode_LAND = 32,
  vertical_mode_FLARE = 33,
  vertical_mode_ROLL_OUT = 34,
  vertical_mode_SRS = 40,
  vertical_mode_SRS_GA = 41
} vertical_mode;

#endif

#ifndef DEFINED_TYPEDEF_FOR_athr_mode_
#define DEFINED_TYPEDEF_FOR_athr_mode_

typedef enum {
  athr_mode_NONE = 0,
  athr_mode_SPEED,
  athr_mode_THRUST_IDLE,
  athr_mode_THRUST_CLB
} athr_mode;

#endif

#ifndef DEFINED_TYPEDEF_FOR_vertical_law_
#define DEFINED_TYPEDEF_FOR_vertical_law_

typedef enum {
  vertical_law_NONE = 0,
  vertical_law_ALT_HOLD,
  vertical_law_ALT_ACQ,
  vertical_law_SPD_MACH,
  vertical_law_VS,
  vertical_law_FPA,
  vertical_law_GS,
  vertical_law_FLARE,
  vertical_law_SRS
} vertical_law;

#endif

#ifndef DEFINED_TYPEDEF_FOR_ap_lateral_
#define DEFINED_TYPEDEF_FOR_ap_lateral_

typedef struct {
  ap_lateral_armed armed;
  ap_lateral_condition condition;
  ap_lateral_output output;
} ap_lateral;

#endif

#ifndef DEFINED_TYPEDEF_FOR_ap_vertical_armed_
#define DEFINED_TYPEDEF_FOR_ap_vertical_armed_

typedef struct {
  boolean_T ALT;
  boolean_T ALT_CST;
  boolean_T CLB;
  boolean_T DES;
  boolean_T GS;
} ap_vertical_armed;

#endif

#ifndef DEFINED_TYPEDEF_FOR_ap_vertical_condition_
#define DEFINED_TYPEDEF_FOR_ap_vertical_condition_

typedef struct {
  boolean_T ALT;
  boolean_T ALT_CPT;
  boolean_T ALT_CST;
  boolean_T ALT_CST_CPT;
  boolean_T CLB;
  boolean_T DES;
  boolean_T GS_CPT;
  boolean_T GS_TRACK;
  boolean_T LAND;
  boolean_T FLARE;
  boolean_T ROLL_OUT;
  boolean_T SRS;
  boolean_T SRS_GA;
  boolean_T THR_RED;
  boolean_T H_fcu_active;
} ap_vertical_condition;

#endif

#ifndef DEFINED_TYPEDEF_FOR_ap_vertical_output_
#define DEFINED_TYPEDEF_FOR_ap_vertical_output_

typedef struct {
  vertical_mode mode;
  athr_mode mode_autothrust;
  boolean_T mode_reversion;
  vertical_law law;
  real_T H_c_ft;
  real_T H_dot_c_fpm;
  real_T FPA_c_deg;
  real_T V_c_kn;
  boolean_T ALT_soft_mode_active;
} ap_vertical_output;

#endif

#ifndef DEFINED_TYPEDEF_FOR_ap_vertical_
#define DEFINED_TYPEDEF_FOR_ap_vertical_

typedef struct {
  ap_vertical_armed armed;
  ap_vertical_condition condition;
  ap_vertical_output output;
} ap_vertical;

#endif

#ifndef DEFINED_TYPEDEF_FOR_ap_raw_laws_input_
#define DEFINED_TYPEDEF_FOR_ap_raw_laws_input_

typedef struct {
  real_T enabled_AP1;
  real_T enabled_AP2;
  real_T lateral_law;
  real_T lateral_mode;
  real_T lateral_mode_armed;
  real_T vertical_law;
  real_T vertical_mode;
  real_T vertical_mode_armed;
  real_T mode_reversion;
  boolean_T mode_reversion_TRK_FPA;
  real_T autothrust_mode;
  real_T Psi_c_deg;
  real_T H_c_ft;
  real_T H_dot_c_fpm;
  real_T FPA_c_deg;
  real_T V_c_kn;
  boolean_T ALT_soft_mode_active;
} ap_raw_laws_input;

#endif

#ifndef DEFINED_TYPEDEF_FOR_ap_sm_output_
#define DEFINED_TYPEDEF_FOR_ap_sm_output_

typedef struct {
  ap_raw_time time;
  ap_data data;
  ap_sm_data_computed data_computed;
  ap_raw_sm_input input;
  ap_lateral lateral;
  ap_lateral lateral_previous;
  ap_vertical vertical;
  ap_vertical vertical_previous;
  ap_raw_laws_input output;
} ap_sm_output;

#endif

typedef struct Parameters_AutopilotStateMachine_T_ Parameters_AutopilotStateMachine_T;

#endif

