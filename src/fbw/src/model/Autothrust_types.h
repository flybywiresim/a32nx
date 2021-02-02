#ifndef RTW_HEADER_Autothrust_types_h_
#define RTW_HEADER_Autothrust_types_h_
#include "rtwtypes.h"

#ifndef DEFINED_TYPEDEF_FOR_athr_status_
#define DEFINED_TYPEDEF_FOR_athr_status_

typedef enum {
  athr_status_DISENGAGED = 0,
  athr_status_ENGAGED_ARMED,
  athr_status_ENGAGED_ACTIVE
} athr_status;

#endif

#ifndef DEFINED_TYPEDEF_FOR_athr_time_
#define DEFINED_TYPEDEF_FOR_athr_time_

typedef struct {
  real_T dt;
  real_T simulation_time;
} athr_time;

#endif

#ifndef DEFINED_TYPEDEF_FOR_athr_raw_data_
#define DEFINED_TYPEDEF_FOR_athr_raw_data_

typedef struct {
  real_T nz_g;
  real_T Theta_deg;
  real_T Phi_deg;
  real_T V_ias_kn;
  real_T V_tas_kn;
  real_T V_mach;
  real_T V_gnd_kn;
  real_T alpha_deg;
  real_T H_ft;
  real_T H_ind_ft;
  real_T H_radio_ft;
  real_T H_dot_fpm;
  real_T bx_m_s2;
  real_T by_m_s2;
  real_T bz_m_s2;
  real_T gear_strut_compression_1;
  real_T gear_strut_compression_2;
  real_T flap_handle_index;
  boolean_T is_engine_operative_1;
  boolean_T is_engine_operative_2;
  real_T commanded_engine_N1_1_percent;
  real_T commanded_engine_N1_2_percent;
  real_T engine_N1_1_percent;
  real_T engine_N1_2_percent;
  real_T corrected_engine_N1_1_percent;
  real_T corrected_engine_N1_2_percent;
  real_T SAT_degC;
} athr_raw_data;

#endif

#ifndef DEFINED_TYPEDEF_FOR_athr_input_
#define DEFINED_TYPEDEF_FOR_athr_input_

typedef struct {
  boolean_T ATHR_push;
  real_T TLA_1_deg;
  real_T TLA_2_deg;
  real_T V_c_kn;
  real_T V_LS_kn;
  real_T V_MAX_kn;
  real_T thrust_limit_REV_percent;
  real_T thrust_limit_IDLE_percent;
  real_T thrust_limit_CLB_percent;
  real_T thrust_limit_MCT_percent;
  real_T thrust_limit_FLEX_percent;
  real_T thrust_limit_TOGA_percent;
  real_T flex_temperature_degC;
  real_T mode_requested;
  boolean_T is_mach_mode_active;
  boolean_T alpha_floor_condition;
  boolean_T is_approach_mode_active;
  boolean_T is_SRS_TO_mode_active;
  boolean_T is_SRS_GA_mode_active;
  real_T thrust_reduction_altitude;
  real_T thrust_reduction_altitude_go_around;
} athr_input;

#endif

#ifndef DEFINED_TYPEDEF_FOR_athr_in_
#define DEFINED_TYPEDEF_FOR_athr_in_

typedef struct {
  athr_time time;
  athr_raw_data data;
  athr_input input;
} athr_in;

#endif

#ifndef DEFINED_TYPEDEF_FOR_athr_thrust_limit_type_
#define DEFINED_TYPEDEF_FOR_athr_thrust_limit_type_

typedef enum {
  athr_thrust_limit_type_NONE = 0,
  athr_thrust_limit_type_CLB,
  athr_thrust_limit_type_MCT,
  athr_thrust_limit_type_FLEX,
  athr_thrust_limit_type_TOGA,
  athr_thrust_limit_type_REVERSE
} athr_thrust_limit_type;

#endif

#ifndef DEFINED_TYPEDEF_FOR_athr_mode_
#define DEFINED_TYPEDEF_FOR_athr_mode_

typedef enum {
  athr_mode_NONE = 0,
  athr_mode_MAN_TOGA,
  athr_mode_MAN_GA_SOFT,
  athr_mode_MAN_FLEX,
  athr_mode_MAN_DTO,
  athr_mode_MAN_MCT,
  athr_mode_MAN_THR,
  athr_mode_SPEED,
  athr_mode_MACH,
  athr_mode_THR_MCT,
  athr_mode_THR_CLB,
  athr_mode_THR_LVR,
  athr_mode_THR_IDLE,
  athr_mode_A_FLOOR,
  athr_mode_TOGA_LK
} athr_mode;

#endif

#ifndef DEFINED_TYPEDEF_FOR_athr_mode_message_
#define DEFINED_TYPEDEF_FOR_athr_mode_message_

typedef enum {
  athr_mode_message_NONE = 0,
  athr_mode_message_THR_LK,
  athr_mode_message_LVR_TOGA,
  athr_mode_message_LVR_CLB,
  athr_mode_message_LVR_MCT,
  athr_mode_message_LVR_ASYM
} athr_mode_message;

#endif

#ifndef DEFINED_TYPEDEF_FOR_athr_data_
#define DEFINED_TYPEDEF_FOR_athr_data_

typedef struct {
  real_T nz_g;
  real_T Theta_deg;
  real_T Phi_deg;
  real_T V_ias_kn;
  real_T V_tas_kn;
  real_T V_mach;
  real_T V_gnd_kn;
  real_T alpha_deg;
  real_T H_ft;
  real_T H_ind_ft;
  real_T H_radio_ft;
  real_T H_dot_fpm;
  real_T ax_m_s2;
  real_T ay_m_s2;
  real_T az_m_s2;
  real_T bx_m_s2;
  real_T by_m_s2;
  real_T bz_m_s2;
  boolean_T on_ground;
  real_T flap_handle_index;
  boolean_T is_engine_operative_1;
  boolean_T is_engine_operative_2;
  real_T commanded_engine_N1_1_percent;
  real_T commanded_engine_N1_2_percent;
  real_T engine_N1_1_percent;
  real_T engine_N1_2_percent;
  real_T SAT_degC;
} athr_data;

#endif

#ifndef DEFINED_TYPEDEF_FOR_athr_data_computed_
#define DEFINED_TYPEDEF_FOR_athr_data_computed_

typedef struct {
  boolean_T TLA_in_active_range;
  boolean_T is_FLX_active;
  boolean_T ATHR_push;
  boolean_T ATHR_disabled;
} athr_data_computed;

#endif

#ifndef DEFINED_TYPEDEF_FOR_athr_output_
#define DEFINED_TYPEDEF_FOR_athr_output_

typedef struct {
  real_T sim_throttle_lever_1_pos;
  real_T sim_throttle_lever_2_pos;
  real_T sim_thrust_mode_1;
  real_T sim_thrust_mode_2;
  real_T N1_TLA_1_percent;
  real_T N1_TLA_2_percent;
  boolean_T is_in_reverse_1;
  boolean_T is_in_reverse_2;
  athr_thrust_limit_type thrust_limit_type;
  real_T thrust_limit_percent;
  real_T N1_c_1_percent;
  real_T N1_c_2_percent;
  athr_status status;
  athr_mode mode;
  athr_mode_message mode_message;
} athr_output;

#endif

#ifndef DEFINED_TYPEDEF_FOR_athr_out_
#define DEFINED_TYPEDEF_FOR_athr_out_

typedef struct {
  athr_time time;
  athr_data data;
  athr_data_computed data_computed;
  athr_input input;
  athr_output output;
} athr_out;

#endif
#endif

