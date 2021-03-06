#include "Autothrust.h"
#include "Autothrust_private.h"
#include "look1_binlxpw.h"
#include "look2_binlxpw.h"

const uint8_T Autothrust_IN_InAir = 1U;
const uint8_T Autothrust_IN_OnGround = 2U;
void AutothrustModelClass::Autothrust_TLAComputation1(const athr_out *rtu_in, real_T rtu_TLA, real_T *rty_N1c, boolean_T
  *rty_inReverse)
{
  real_T N1_begin;
  real_T N1_end;
  real_T u0;
  int32_T TLA_begin;
  int32_T TLA_end;
  u0 = rtu_TLA;
  *rty_inReverse = (rtu_TLA < 0.0);
  if (rtu_TLA >= 0.0) {
    if (rtu_TLA <= 25.0) {
      TLA_begin = 0;
      N1_begin = rtu_in->input.thrust_limit_IDLE_percent;
      TLA_end = 25;
      N1_end = rtu_in->input.thrust_limit_CLB_percent;
    } else if (rtu_TLA <= 35.0) {
      TLA_begin = 25;
      N1_begin = rtu_in->input.thrust_limit_CLB_percent;
      TLA_end = 35;
      if (rtu_in->data_computed.is_FLX_active) {
        N1_end = rtu_in->input.thrust_limit_FLEX_percent;
      } else {
        N1_end = rtu_in->input.thrust_limit_MCT_percent;
      }
    } else {
      TLA_begin = 35;
      if (rtu_in->data_computed.is_FLX_active) {
        N1_begin = rtu_in->input.thrust_limit_FLEX_percent;
      } else {
        N1_begin = rtu_in->input.thrust_limit_MCT_percent;
      }

      TLA_end = 45;
      N1_end = rtu_in->input.thrust_limit_TOGA_percent;
    }
  } else {
    u0 = std::abs(rtu_TLA);
    if (u0 <= 6.0) {
      u0 = 6.0;
    }

    TLA_begin = 6;
    N1_begin = std::abs(rtu_in->input.thrust_limit_IDLE_percent);
    TLA_end = 20;
    N1_end = std::abs(rtu_in->input.thrust_limit_REV_percent);
  }

  *rty_N1c = (N1_end - N1_begin) / static_cast<real_T>(TLA_end - TLA_begin) * (u0 - static_cast<real_T>(TLA_begin)) +
    N1_begin;
}

void AutothrustModelClass::Autothrust_ThrustMode1(real_T rtu_u, real_T *rty_y)
{
  if (rtu_u < 0.0) {
    *rty_y = 1.0;
  } else if (rtu_u == 0.0) {
    *rty_y = 2.0;
  } else if ((rtu_u > 0.0) && (rtu_u < 25.0)) {
    *rty_y = 3.0;
  } else if ((rtu_u >= 25.0) && (rtu_u < 35.0)) {
    *rty_y = 4.0;
  } else if ((rtu_u >= 35.0) && (rtu_u < 45.0)) {
    *rty_y = 5.0;
  } else if (rtu_u == 45.0) {
    *rty_y = 6.0;
  } else {
    *rty_y = 0.0;
  }
}

void AutothrustModelClass::step()
{
  athr_out rtb_BusAssignment;
  real_T max_TLA_tmp;
  real_T rtb_Divide;
  real_T rtb_Gain_f;
  real_T rtb_Saturation;
  real_T rtb_Saturation1;
  real_T rtb_Sum2;
  real_T rtb_Switch_d;
  real_T rtb_Switch_f_idx_1;
  real_T rtb_y_b;
  int32_T rtb_on_ground;
  boolean_T ATHR_ENGAGED_tmp;
  boolean_T ATHR_ENGAGED_tmp_0;
  boolean_T ATHR_ENGAGED_tmp_1;
  boolean_T cUseAutoThrustControl;
  boolean_T condition_AP_FD_ATHR_Specific;
  boolean_T condition_THR_LK_tmp;
  boolean_T rtb_BusConversion_InsertedFor_BusAssignment_at_inport_1_BusCreator1_g_ATHR_push;
  boolean_T rtb_NOT1;
  boolean_T rtb_inReverse;
  boolean_T rtb_out;
  boolean_T rtb_y_h;
  athr_status rtb_status;
  rtb_Saturation = Autothrust_P.Gain_Gain_p * Autothrust_U.in.data.gear_strut_compression_1 -
    Autothrust_P.Constant1_Value_d;
  if (rtb_Saturation > Autothrust_P.Saturation_UpperSat) {
    rtb_Saturation = Autothrust_P.Saturation_UpperSat;
  } else {
    if (rtb_Saturation < Autothrust_P.Saturation_LowerSat) {
      rtb_Saturation = Autothrust_P.Saturation_LowerSat;
    }
  }

  rtb_Saturation1 = Autothrust_P.Gain1_Gain * Autothrust_U.in.data.gear_strut_compression_2 -
    Autothrust_P.Constant1_Value_d;
  if (rtb_Saturation1 > Autothrust_P.Saturation1_UpperSat) {
    rtb_Saturation1 = Autothrust_P.Saturation1_UpperSat;
  } else {
    if (rtb_Saturation1 < Autothrust_P.Saturation1_LowerSat) {
      rtb_Saturation1 = Autothrust_P.Saturation1_LowerSat;
    }
  }

  if (Autothrust_DWork.is_active_c5_Autothrust == 0U) {
    Autothrust_DWork.is_active_c5_Autothrust = 1U;
    Autothrust_DWork.is_c5_Autothrust = Autothrust_IN_OnGround;
    rtb_on_ground = 1;
  } else if (Autothrust_DWork.is_c5_Autothrust == 1) {
    if ((rtb_Saturation > 0.05) || (rtb_Saturation1 > 0.05)) {
      Autothrust_DWork.is_c5_Autothrust = Autothrust_IN_OnGround;
      rtb_on_ground = 1;
    } else {
      rtb_on_ground = 0;
    }
  } else {
    if ((rtb_Saturation == 0.0) && (rtb_Saturation1 == 0.0)) {
      Autothrust_DWork.is_c5_Autothrust = Autothrust_IN_InAir;
      rtb_on_ground = 0;
    } else {
      rtb_on_ground = 1;
    }
  }

  rtb_BusAssignment = Autothrust_P.athr_out_MATLABStruct;
  rtb_BusAssignment.time = Autothrust_U.in.time;
  rtb_BusAssignment.data.nz_g = Autothrust_U.in.data.nz_g;
  rtb_BusAssignment.data.V_ias_kn = Autothrust_U.in.data.V_ias_kn;
  rtb_BusAssignment.data.V_tas_kn = Autothrust_U.in.data.V_tas_kn;
  rtb_BusAssignment.data.V_mach = Autothrust_U.in.data.V_mach;
  rtb_BusAssignment.data.V_gnd_kn = Autothrust_U.in.data.V_gnd_kn;
  rtb_BusAssignment.data.alpha_deg = Autothrust_U.in.data.alpha_deg;
  rtb_BusAssignment.data.H_ft = Autothrust_U.in.data.H_ft;
  rtb_BusAssignment.data.H_ind_ft = Autothrust_U.in.data.H_ind_ft;
  rtb_BusAssignment.data.H_radio_ft = Autothrust_U.in.data.H_radio_ft;
  rtb_BusAssignment.data.H_dot_fpm = Autothrust_U.in.data.H_dot_fpm;
  rtb_BusAssignment.data.bx_m_s2 = Autothrust_U.in.data.bx_m_s2;
  rtb_BusAssignment.data.by_m_s2 = Autothrust_U.in.data.by_m_s2;
  rtb_BusAssignment.data.bz_m_s2 = Autothrust_U.in.data.bz_m_s2;
  rtb_BusAssignment.data.on_ground = (rtb_on_ground != 0);
  rtb_BusAssignment.data.flap_handle_index = Autothrust_U.in.data.flap_handle_index;
  rtb_BusAssignment.data.is_engine_operative_1 = Autothrust_U.in.data.is_engine_operative_1;
  rtb_BusAssignment.data.is_engine_operative_2 = Autothrust_U.in.data.is_engine_operative_2;
  rtb_BusAssignment.data.commanded_engine_N1_1_percent = (Autothrust_U.in.data.engine_N1_1_percent +
    Autothrust_U.in.data.commanded_engine_N1_1_percent) - Autothrust_U.in.data.corrected_engine_N1_1_percent;
  rtb_BusAssignment.data.commanded_engine_N1_2_percent = (Autothrust_U.in.data.engine_N1_2_percent +
    Autothrust_U.in.data.commanded_engine_N1_2_percent) - Autothrust_U.in.data.corrected_engine_N1_2_percent;
  rtb_BusAssignment.data.engine_N1_1_percent = Autothrust_U.in.data.engine_N1_1_percent;
  rtb_BusAssignment.data.engine_N1_2_percent = Autothrust_U.in.data.engine_N1_2_percent;
  rtb_BusAssignment.data.SAT_degC = Autothrust_U.in.data.SAT_degC;
  rtb_BusAssignment.input = Autothrust_U.in.input;
  rtb_y_b = look2_binlxpw(Autothrust_U.in.data.SAT_degC, Autothrust_U.in.data.H_ft,
    Autothrust_P.MAXIMUMCONTINUOUS_bp01Data, Autothrust_P.MAXIMUMCONTINUOUS_bp02Data,
    Autothrust_P.MAXIMUMCONTINUOUS_tableData, Autothrust_P.MAXIMUMCONTINUOUS_maxIndex, 26U);
  rtb_Saturation = look2_binlxpw(Autothrust_U.in.data.SAT_degC, Autothrust_U.in.data.H_ft,
    Autothrust_P.MAXIMUMTAKEOFF_bp01Data, Autothrust_P.MAXIMUMTAKEOFF_bp02Data, Autothrust_P.MAXIMUMTAKEOFF_tableData,
    Autothrust_P.MAXIMUMTAKEOFF_maxIndex, 36U);
  rtb_BusAssignment.input.thrust_limit_CLB_percent = look2_binlxpw(Autothrust_U.in.data.SAT_degC,
    Autothrust_U.in.data.H_ft, Autothrust_P.MAXIMUMCLIMB_bp01Data, Autothrust_P.MAXIMUMCLIMB_bp02Data,
    Autothrust_P.MAXIMUMCLIMB_tableData, Autothrust_P.MAXIMUMCLIMB_maxIndex, 26U);
  rtb_BusAssignment.input.thrust_limit_FLEX_percent = rtb_y_b;
  rtb_BusAssignment.input.thrust_limit_MCT_percent = rtb_y_b;
  rtb_BusAssignment.input.thrust_limit_TOGA_percent = rtb_Saturation;
  if (!Autothrust_DWork.eventTime_not_empty) {
    Autothrust_DWork.eventTime = Autothrust_U.in.time.simulation_time;
    Autothrust_DWork.eventTime_not_empty = true;
  }

  if ((!Autothrust_U.in.input.ATHR_push) || (Autothrust_DWork.eventTime == 0.0)) {
    Autothrust_DWork.eventTime = Autothrust_U.in.time.simulation_time;
  }

  Autothrust_DWork.Memory_PreviousInput = Autothrust_P.Logic_table[(((static_cast<uint32_T>
    (Autothrust_U.in.time.simulation_time - Autothrust_DWork.eventTime >= Autothrust_P.CompareToConstant_const) << 1) +
    false) << 1) + Autothrust_DWork.Memory_PreviousInput];
  if (!Autothrust_DWork.eventTime_not_empty_i) {
    Autothrust_DWork.eventTime_n = Autothrust_U.in.time.simulation_time;
    Autothrust_DWork.eventTime_not_empty_i = true;
  }

  if ((Autothrust_U.in.input.ATHR_push != Autothrust_P.CompareToConstant1_const) || (Autothrust_DWork.eventTime_n == 0.0))
  {
    Autothrust_DWork.eventTime_n = Autothrust_U.in.time.simulation_time;
  }

  Autothrust_DWork.Memory_PreviousInput_f = Autothrust_P.Logic_table_g[(((Autothrust_U.in.time.simulation_time -
    Autothrust_DWork.eventTime_n >= Autothrust_P.CompareToConstant2_const) + (static_cast<uint32_T>
    (Autothrust_DWork.Delay_DSTATE_b) << 1)) << 1) + Autothrust_DWork.Memory_PreviousInput_f];
  if (Autothrust_U.in.data.is_engine_operative_1 && Autothrust_U.in.data.is_engine_operative_2) {
    rtb_out = ((Autothrust_U.in.input.TLA_1_deg >= 0.0) && (Autothrust_U.in.input.TLA_1_deg <= 25.0) &&
               (Autothrust_U.in.input.TLA_2_deg >= 0.0) && (Autothrust_U.in.input.TLA_2_deg <= 25.0));
  } else {
    rtb_out = ((Autothrust_U.in.data.is_engine_operative_1 && (Autothrust_U.in.input.TLA_1_deg >= 0.0) &&
                (Autothrust_U.in.input.TLA_1_deg <= 35.0)) || (Autothrust_U.in.data.is_engine_operative_2 &&
                (Autothrust_U.in.input.TLA_2_deg >= 0.0) && (Autothrust_U.in.input.TLA_2_deg <= 35.0)));
  }

  rtb_inReverse = ((Autothrust_U.in.input.flex_temperature_degC > Autothrust_U.in.data.SAT_degC) &&
                   (Autothrust_U.in.input.flex_temperature_degC != 0.0));
  Autothrust_DWork.latch = ((rtb_inReverse && (rtb_on_ground != 0) && (Autothrust_U.in.input.TLA_1_deg == 35.0) &&
    (Autothrust_U.in.input.TLA_2_deg == 35.0)) || Autothrust_DWork.latch);
  Autothrust_DWork.latch = (((!Autothrust_DWork.latch) || (((Autothrust_U.in.input.TLA_1_deg != 25.0) ||
    (Autothrust_U.in.input.TLA_2_deg != 25.0)) && ((Autothrust_U.in.input.TLA_1_deg != 45.0) ||
    (Autothrust_U.in.input.TLA_2_deg != 45.0)))) && Autothrust_DWork.latch);
  rtb_y_h = ((rtb_inReverse && (rtb_on_ground != 0)) || ((rtb_on_ground == 0) && Autothrust_DWork.latch));
  Autothrust_DWork.Delay_DSTATE_b = (static_cast<int32_T>(Autothrust_U.in.input.ATHR_push) > static_cast<int32_T>
    (Autothrust_P.CompareToConstant_const_o));
  rtb_NOT1 = (Autothrust_DWork.Delay_DSTATE_b && (!Autothrust_DWork.Memory_PreviousInput_f));
  rtb_BusConversion_InsertedFor_BusAssignment_at_inport_1_BusCreator1_g_ATHR_push = rtb_NOT1;
  Autothrust_TLAComputation1(&rtb_BusAssignment, Autothrust_U.in.input.TLA_1_deg, &rtb_Saturation, &rtb_inReverse);
  Autothrust_TLAComputation1(&rtb_BusAssignment, Autothrust_U.in.input.TLA_2_deg, &rtb_Saturation1, &rtb_NOT1);
  if (!Autothrust_DWork.prev_TLA_1_not_empty) {
    Autothrust_DWork.prev_TLA_1 = Autothrust_U.in.input.TLA_1_deg;
    Autothrust_DWork.prev_TLA_1_not_empty = true;
  }

  if (!Autothrust_DWork.prev_TLA_2_not_empty) {
    Autothrust_DWork.prev_TLA_2 = Autothrust_U.in.input.TLA_2_deg;
    Autothrust_DWork.prev_TLA_2_not_empty = true;
  }

  condition_AP_FD_ATHR_Specific = !Autothrust_DWork.Memory_PreviousInput;
  ATHR_ENGAGED_tmp = !Autothrust_DWork.ATHR_ENGAGED;
  ATHR_ENGAGED_tmp_0 = ((Autothrust_DWork.prev_TLA_1 <= 0.0) || (Autothrust_U.in.input.TLA_1_deg != 0.0));
  ATHR_ENGAGED_tmp_1 = ((Autothrust_DWork.prev_TLA_2 <= 0.0) || (Autothrust_U.in.input.TLA_2_deg != 0.0));
  Autothrust_DWork.ATHR_ENGAGED = ((condition_AP_FD_ATHR_Specific && (((rtb_on_ground == 0) && ATHR_ENGAGED_tmp &&
    rtb_BusConversion_InsertedFor_BusAssignment_at_inport_1_BusCreator1_g_ATHR_push) ||
    (((Autothrust_U.in.input.TLA_1_deg == 45.0) && (Autothrust_U.in.input.TLA_2_deg == 45.0)) || (rtb_y_h &&
    (Autothrust_U.in.input.TLA_1_deg >= 35.0) && (Autothrust_U.in.input.TLA_2_deg >= 35.0))) ||
    Autothrust_U.in.input.alpha_floor_condition)) || (condition_AP_FD_ATHR_Specific &&
    ((!rtb_BusConversion_InsertedFor_BusAssignment_at_inport_1_BusCreator1_g_ATHR_push) || ATHR_ENGAGED_tmp) &&
    ((ATHR_ENGAGED_tmp_0 || ATHR_ENGAGED_tmp_1) && (ATHR_ENGAGED_tmp_0 || (Autothrust_U.in.input.TLA_2_deg != 0.0)) &&
     (ATHR_ENGAGED_tmp_1 || (Autothrust_U.in.input.TLA_1_deg != 0.0))) && Autothrust_DWork.ATHR_ENGAGED));
  condition_AP_FD_ATHR_Specific = (Autothrust_DWork.ATHR_ENGAGED && (rtb_out ||
    Autothrust_U.in.input.alpha_floor_condition));
  if (Autothrust_DWork.ATHR_ENGAGED && condition_AP_FD_ATHR_Specific) {
    rtb_status = athr_status_ENGAGED_ACTIVE;
  } else if (Autothrust_DWork.ATHR_ENGAGED && (!condition_AP_FD_ATHR_Specific)) {
    rtb_status = athr_status_ENGAGED_ARMED;
  } else {
    rtb_status = athr_status_DISENGAGED;
  }

  Autothrust_DWork.prev_TLA_1 = Autothrust_U.in.input.TLA_1_deg;
  Autothrust_DWork.prev_TLA_2 = Autothrust_U.in.input.TLA_2_deg;
  if (Autothrust_U.in.input.TLA_1_deg > Autothrust_U.in.input.TLA_2_deg) {
    max_TLA_tmp = Autothrust_U.in.input.TLA_1_deg;
  } else {
    max_TLA_tmp = Autothrust_U.in.input.TLA_2_deg;
  }

  Autothrust_DWork.pConditionAlphaFloor = (Autothrust_U.in.input.alpha_floor_condition || ((!(rtb_status ==
    athr_status_DISENGAGED)) && Autothrust_DWork.pConditionAlphaFloor));
  if (rtb_status == athr_status_DISENGAGED) {
    Autothrust_DWork.pMode = athr_mode_NONE;
  } else if ((rtb_status == athr_status_ENGAGED_ARMED) && ((Autothrust_U.in.input.TLA_1_deg == 45.0) ||
              (Autothrust_U.in.input.TLA_2_deg == 45.0))) {
    Autothrust_DWork.pMode = athr_mode_MAN_TOGA;
  } else if ((rtb_status == athr_status_ENGAGED_ARMED) && rtb_y_h && (max_TLA_tmp == 35.0)) {
    Autothrust_DWork.pMode = athr_mode_MAN_FLEX;
  } else if ((rtb_status == athr_status_ENGAGED_ARMED) && ((Autothrust_U.in.input.TLA_1_deg == 35.0) ||
              (Autothrust_U.in.input.TLA_2_deg == 35.0))) {
    Autothrust_DWork.pMode = athr_mode_MAN_MCT;
  } else {
    condition_AP_FD_ATHR_Specific = (Autothrust_U.in.data.is_engine_operative_1 &&
      (!Autothrust_U.in.data.is_engine_operative_2));
    ATHR_ENGAGED_tmp = (Autothrust_U.in.data.is_engine_operative_2 && (!Autothrust_U.in.data.is_engine_operative_1));
    if ((rtb_status == athr_status_ENGAGED_ACTIVE) && (Autothrust_U.in.input.mode_requested == 3.0) &&
        ((condition_AP_FD_ATHR_Specific && (Autothrust_U.in.input.TLA_1_deg == 35.0) && (Autothrust_U.in.input.TLA_2_deg
           <= 35.0)) || (ATHR_ENGAGED_tmp && (Autothrust_U.in.input.TLA_2_deg == 35.0) &&
                         (Autothrust_U.in.input.TLA_1_deg <= 35.0)))) {
      Autothrust_DWork.pMode = athr_mode_THR_MCT;
    } else if ((rtb_status == athr_status_ENGAGED_ACTIVE) && (Autothrust_U.in.input.mode_requested == 3.0) &&
               Autothrust_U.in.data.is_engine_operative_1 && Autothrust_U.in.data.is_engine_operative_2 && (max_TLA_tmp ==
                25.0)) {
      Autothrust_DWork.pMode = athr_mode_THR_CLB;
    } else {
      ATHR_ENGAGED_tmp_0 = (Autothrust_U.in.data.is_engine_operative_1 && Autothrust_U.in.data.is_engine_operative_2);
      if ((rtb_status == athr_status_ENGAGED_ACTIVE) && (Autothrust_U.in.input.mode_requested == 3.0) &&
          ((ATHR_ENGAGED_tmp_0 && (Autothrust_U.in.input.TLA_1_deg < 25.0) && (Autothrust_U.in.input.TLA_2_deg < 25.0)) ||
           (condition_AP_FD_ATHR_Specific && (Autothrust_U.in.input.TLA_1_deg < 35.0)) || (ATHR_ENGAGED_tmp &&
            (Autothrust_U.in.input.TLA_2_deg < 35.0)))) {
        Autothrust_DWork.pMode = athr_mode_THR_LVR;
      } else if ((rtb_status == athr_status_ENGAGED_ARMED) && ((ATHR_ENGAGED_tmp_0 && (max_TLA_tmp > 25.0) &&
                   (max_TLA_tmp < 35.0)) || ((max_TLA_tmp > 35.0) && (max_TLA_tmp < 45.0)))) {
        Autothrust_DWork.pMode = athr_mode_MAN_THR;
      } else if ((rtb_status == athr_status_ENGAGED_ACTIVE) && (Autothrust_U.in.input.mode_requested == 2.0)) {
        Autothrust_DWork.pMode = athr_mode_THR_IDLE;
      } else if ((rtb_status == athr_status_ENGAGED_ACTIVE) && (Autothrust_U.in.input.mode_requested == 1.0) &&
                 (!Autothrust_U.in.input.is_mach_mode_active)) {
        Autothrust_DWork.pMode = athr_mode_SPEED;
      } else if ((rtb_status == athr_status_ENGAGED_ACTIVE) && (Autothrust_U.in.input.mode_requested == 1.0) &&
                 Autothrust_U.in.input.is_mach_mode_active) {
        Autothrust_DWork.pMode = athr_mode_MACH;
      } else if (Autothrust_U.in.input.alpha_floor_condition) {
        Autothrust_DWork.pMode = athr_mode_A_FLOOR;
      } else {
        if ((!Autothrust_U.in.input.alpha_floor_condition) && Autothrust_DWork.pConditionAlphaFloor) {
          Autothrust_DWork.pMode = athr_mode_TOGA_LK;
        }
      }
    }
  }

  Autothrust_DWork.inhibitAboveThrustReductionAltitude = ((Autothrust_U.in.input.is_SRS_TO_mode_active &&
    (!Autothrust_DWork.was_SRS_TO_active) && (Autothrust_U.in.data.H_ind_ft >
    Autothrust_U.in.input.thrust_reduction_altitude)) || (Autothrust_U.in.input.is_SRS_GA_mode_active &&
    (!Autothrust_DWork.was_SRS_GA_active) && (Autothrust_U.in.data.H_ind_ft >
    Autothrust_U.in.input.thrust_reduction_altitude_go_around)) || ((Autothrust_U.in.input.is_SRS_TO_mode_active ||
    Autothrust_U.in.input.is_SRS_GA_mode_active) && Autothrust_DWork.inhibitAboveThrustReductionAltitude));
  ATHR_ENGAGED_tmp_0 = (Autothrust_U.in.data.is_engine_operative_1 && Autothrust_U.in.data.is_engine_operative_2);
  ATHR_ENGAGED_tmp_1 = (Autothrust_U.in.data.is_engine_operative_1 && (!Autothrust_U.in.data.is_engine_operative_2));
  condition_THR_LK_tmp = (Autothrust_U.in.data.is_engine_operative_2 && (!Autothrust_U.in.data.is_engine_operative_1));
  Autothrust_DWork.condition_THR_LK = (((rtb_status == athr_status_DISENGAGED) && (Autothrust_DWork.pStatus !=
    athr_status_DISENGAGED) && ((ATHR_ENGAGED_tmp_0 && (Autothrust_U.in.input.TLA_1_deg == 25.0) &&
    (Autothrust_U.in.input.TLA_2_deg == 25.0)) || (ATHR_ENGAGED_tmp_1 && (Autothrust_U.in.input.TLA_1_deg == 35.0) &&
    (Autothrust_U.in.input.TLA_2_deg <= 35.0)) || (condition_THR_LK_tmp && (Autothrust_U.in.input.TLA_2_deg == 35.0) &&
    (Autothrust_U.in.input.TLA_1_deg <= 35.0)))) || (((!Autothrust_DWork.condition_THR_LK) || ((!(rtb_status !=
    athr_status_DISENGAGED)) && ((Autothrust_U.in.input.TLA_1_deg == 25.0) || (Autothrust_U.in.input.TLA_2_deg == 25.0) ||
    (Autothrust_U.in.input.TLA_1_deg == 35.0) || (Autothrust_U.in.input.TLA_2_deg == 35.0)))) &&
    Autothrust_DWork.condition_THR_LK));
  Autothrust_DWork.pStatus = rtb_status;
  Autothrust_DWork.was_SRS_TO_active = Autothrust_U.in.input.is_SRS_TO_mode_active;
  Autothrust_DWork.was_SRS_GA_active = Autothrust_U.in.input.is_SRS_GA_mode_active;
  condition_AP_FD_ATHR_Specific = !Autothrust_U.in.data.is_engine_operative_1;
  ATHR_ENGAGED_tmp = !Autothrust_U.in.data.is_engine_operative_2;
  if ((rtb_on_ground == 0) || (condition_AP_FD_ATHR_Specific && ATHR_ENGAGED_tmp)) {
    if (max_TLA_tmp > 35.0) {
      Autothrust_Y.out.output.thrust_limit_type = athr_thrust_limit_type_TOGA;
      Autothrust_Y.out.output.thrust_limit_percent = rtb_BusAssignment.input.thrust_limit_TOGA_percent;
    } else if (max_TLA_tmp > 25.0) {
      if (!rtb_y_h) {
        Autothrust_Y.out.output.thrust_limit_type = athr_thrust_limit_type_MCT;
        Autothrust_Y.out.output.thrust_limit_percent = rtb_y_b;
      } else {
        Autothrust_Y.out.output.thrust_limit_type = athr_thrust_limit_type_FLEX;
        Autothrust_Y.out.output.thrust_limit_percent = rtb_y_b;
      }
    } else if (max_TLA_tmp >= 0.0) {
      Autothrust_Y.out.output.thrust_limit_type = athr_thrust_limit_type_CLB;
      Autothrust_Y.out.output.thrust_limit_percent = rtb_BusAssignment.input.thrust_limit_CLB_percent;
    } else if (max_TLA_tmp < 0.0) {
      Autothrust_Y.out.output.thrust_limit_type = athr_thrust_limit_type_REVERSE;
      Autothrust_Y.out.output.thrust_limit_percent = Autothrust_U.in.input.thrust_limit_REV_percent;
    } else {
      Autothrust_Y.out.output.thrust_limit_type = athr_thrust_limit_type_NONE;
      Autothrust_Y.out.output.thrust_limit_percent = 0.0;
    }
  } else if (max_TLA_tmp >= 0.0) {
    if ((!rtb_y_h) || (max_TLA_tmp > 35.0)) {
      Autothrust_Y.out.output.thrust_limit_type = athr_thrust_limit_type_TOGA;
      Autothrust_Y.out.output.thrust_limit_percent = rtb_BusAssignment.input.thrust_limit_TOGA_percent;
    } else {
      Autothrust_Y.out.output.thrust_limit_type = athr_thrust_limit_type_FLEX;
      Autothrust_Y.out.output.thrust_limit_percent = rtb_y_b;
    }
  } else if (max_TLA_tmp < 0.0) {
    Autothrust_Y.out.output.thrust_limit_type = athr_thrust_limit_type_REVERSE;
    Autothrust_Y.out.output.thrust_limit_percent = Autothrust_U.in.input.thrust_limit_REV_percent;
  } else {
    Autothrust_Y.out.output.thrust_limit_type = athr_thrust_limit_type_NONE;
    Autothrust_Y.out.output.thrust_limit_percent = 0.0;
  }

  max_TLA_tmp = rtb_Saturation;
  cUseAutoThrustControl = ((rtb_status == athr_status_ENGAGED_ACTIVE) && (((Autothrust_U.in.input.TLA_1_deg <= 35.0) &&
    (Autothrust_U.in.input.TLA_2_deg <= 35.0)) || Autothrust_U.in.input.alpha_floor_condition));
  Autothrust_DWork.pThrustMemoActive = ((((Autothrust_U.in.input.ATHR_push && (rtb_status != athr_status_DISENGAGED)) ||
    ((!cUseAutoThrustControl) && Autothrust_DWork.pUseAutoThrustControl)) && ((ATHR_ENGAGED_tmp_0 &&
    (Autothrust_U.in.input.TLA_1_deg == 25.0) && (Autothrust_U.in.input.TLA_2_deg == 25.0)) || (ATHR_ENGAGED_tmp_1 &&
    (Autothrust_U.in.input.TLA_1_deg == 35.0) && (Autothrust_U.in.input.TLA_2_deg <= 35.0)) || (condition_THR_LK_tmp &&
    (Autothrust_U.in.input.TLA_2_deg == 35.0) && (Autothrust_U.in.input.TLA_1_deg <= 35.0)))) ||
    (((Autothrust_U.in.input.TLA_1_deg == 25.0) || (Autothrust_U.in.input.TLA_1_deg == 35.0) ||
      (Autothrust_U.in.input.TLA_2_deg == 25.0) || (Autothrust_U.in.input.TLA_2_deg == 35.0)) &&
     Autothrust_DWork.pThrustMemoActive));
  Autothrust_DWork.pUseAutoThrustControl = cUseAutoThrustControl;
  if (Autothrust_U.in.data.is_engine_operative_1 && Autothrust_U.in.data.is_engine_operative_2) {
    rtb_Saturation = rtb_BusAssignment.input.thrust_limit_CLB_percent;
  } else {
    rtb_Saturation = rtb_y_b;
  }

  rtb_Switch_d = Autothrust_P.Gain1_Gain_c * Autothrust_U.in.data.alpha_deg;
  rtb_y_b = Autothrust_U.in.data.bz_m_s2 * std::sin(rtb_Switch_d);
  rtb_Switch_d = std::cos(rtb_Switch_d);
  rtb_Switch_d *= Autothrust_U.in.data.bx_m_s2;
  rtb_Sum2 = (rtb_y_b + rtb_Switch_d) * Autothrust_P.Gain_Gain_h * Autothrust_P.Gain_Gain_b * look1_binlxpw
    (static_cast<real_T>(Autothrust_U.in.input.is_approach_mode_active),
     Autothrust_P.ScheduledGain2_BreakpointsForDimension1, Autothrust_P.ScheduledGain2_Table, 1U) +
    (Autothrust_U.in.input.V_c_kn - Autothrust_U.in.data.V_ias_kn);
  rtb_Gain_f = Autothrust_P.DiscreteDerivativeVariableTs_Gain * rtb_Sum2;
  rtb_Divide = (rtb_Gain_f - Autothrust_DWork.Delay_DSTATE_g) / Autothrust_U.in.time.dt;
  rtb_Switch_d = Autothrust_U.in.time.dt * Autothrust_P.LagFilter_C1;
  rtb_y_b = rtb_Switch_d + Autothrust_P.Constant_Value;
  Autothrust_DWork.Delay1_DSTATE = 1.0 / rtb_y_b * (Autothrust_P.Constant_Value - rtb_Switch_d) *
    Autothrust_DWork.Delay1_DSTATE + (rtb_Divide + Autothrust_DWork.Delay_DSTATE_l) * (rtb_Switch_d / rtb_y_b);
  switch (static_cast<int32_T>(Autothrust_U.in.input.mode_requested)) {
   case 0:
    rtb_Switch_d = Autothrust_P.Constant1_Value;
    break;

   case 1:
    rtb_Switch_d = Autothrust_DWork.Delay1_DSTATE * look1_binlxpw(static_cast<real_T>
      (Autothrust_U.in.input.is_approach_mode_active), Autothrust_P.ScheduledGain1_BreakpointsForDimension1,
      Autothrust_P.ScheduledGain1_Table, 1U) + rtb_Sum2 * look1_binlxpw(static_cast<real_T>
      (Autothrust_U.in.input.is_approach_mode_active), Autothrust_P.ScheduledGain_BreakpointsForDimension1,
      Autothrust_P.ScheduledGain_Table, 1U);
    break;

   case 2:
    if (rtb_BusAssignment.data.commanded_engine_N1_1_percent > rtb_BusAssignment.data.commanded_engine_N1_2_percent) {
      rtb_y_b = rtb_BusAssignment.data.commanded_engine_N1_1_percent;
    } else {
      rtb_y_b = rtb_BusAssignment.data.commanded_engine_N1_2_percent;
    }

    rtb_Switch_d = (Autothrust_U.in.input.thrust_limit_IDLE_percent - rtb_y_b) * Autothrust_P.Gain_Gain;
    break;

   default:
    if (rtb_BusAssignment.data.commanded_engine_N1_1_percent > rtb_BusAssignment.data.commanded_engine_N1_2_percent) {
      rtb_y_b = rtb_BusAssignment.data.commanded_engine_N1_1_percent;
    } else {
      rtb_y_b = rtb_BusAssignment.data.commanded_engine_N1_2_percent;
    }

    rtb_Switch_d = (rtb_BusAssignment.input.thrust_limit_CLB_percent - rtb_y_b) * Autothrust_P.Gain_Gain_m;
    break;
  }

  rtb_Switch_d *= Autothrust_P.DiscreteTimeIntegratorVariableTsLimit_Gain;
  rtb_Switch_d *= Autothrust_U.in.time.dt;
  if (!(rtb_status == Autothrust_P.CompareToConstant_const_d)) {
    Autothrust_DWork.icLoad = 1U;
  }

  if (Autothrust_DWork.icLoad != 0) {
    if (rtb_BusAssignment.data.commanded_engine_N1_1_percent > rtb_BusAssignment.data.commanded_engine_N1_2_percent) {
      rtb_y_b = rtb_BusAssignment.data.commanded_engine_N1_1_percent;
    } else {
      rtb_y_b = rtb_BusAssignment.data.commanded_engine_N1_2_percent;
    }

    Autothrust_DWork.Delay_DSTATE_k = rtb_y_b - rtb_Switch_d;
  }

  Autothrust_DWork.Delay_DSTATE_k += rtb_Switch_d;
  if (Autothrust_DWork.Delay_DSTATE_k > rtb_Saturation) {
    Autothrust_DWork.Delay_DSTATE_k = rtb_Saturation;
  } else {
    if (Autothrust_DWork.Delay_DSTATE_k < Autothrust_U.in.input.thrust_limit_IDLE_percent) {
      Autothrust_DWork.Delay_DSTATE_k = Autothrust_U.in.input.thrust_limit_IDLE_percent;
    }
  }

  rtb_Switch_d = std::abs(Autothrust_P.Constant_Value_g);
  rtb_y_b = Autothrust_DWork.Delay_DSTATE_k - Autothrust_DWork.Delay_DSTATE;
  rtb_Saturation = rtb_Switch_d * Autothrust_U.in.time.dt;
  if (rtb_y_b < rtb_Saturation) {
    rtb_Saturation = rtb_y_b;
  }

  rtb_Switch_d *= Autothrust_P.Gain_Gain_p4;
  rtb_Switch_d *= Autothrust_U.in.time.dt;
  if (rtb_Saturation > rtb_Switch_d) {
    rtb_Switch_d = rtb_Saturation;
  }

  Autothrust_DWork.Delay_DSTATE += rtb_Switch_d;
  if (Autothrust_DWork.pUseAutoThrustControl) {
    if (Autothrust_DWork.Delay_DSTATE > max_TLA_tmp) {
      rtb_Sum2 = max_TLA_tmp;
    } else if (Autothrust_DWork.Delay_DSTATE < Autothrust_U.in.input.thrust_limit_IDLE_percent) {
      rtb_Sum2 = Autothrust_U.in.input.thrust_limit_IDLE_percent;
    } else {
      rtb_Sum2 = Autothrust_DWork.Delay_DSTATE;
    }

    if (Autothrust_DWork.Delay_DSTATE > rtb_Saturation1) {
      rtb_Switch_f_idx_1 = rtb_Saturation1;
    } else if (Autothrust_DWork.Delay_DSTATE < Autothrust_U.in.input.thrust_limit_IDLE_percent) {
      rtb_Switch_f_idx_1 = Autothrust_U.in.input.thrust_limit_IDLE_percent;
    } else {
      rtb_Switch_f_idx_1 = Autothrust_DWork.Delay_DSTATE;
    }
  } else if (Autothrust_DWork.pThrustMemoActive) {
    rtb_Sum2 = rtb_BusAssignment.data.commanded_engine_N1_1_percent;
    rtb_Switch_f_idx_1 = rtb_BusAssignment.data.commanded_engine_N1_2_percent;
  } else {
    rtb_Sum2 = max_TLA_tmp;
    rtb_Switch_f_idx_1 = rtb_Saturation1;
  }

  Autothrust_Y.out.output.N1_TLA_2_percent = rtb_Saturation1;
  rtb_Saturation = rtb_Sum2 - rtb_BusAssignment.data.commanded_engine_N1_1_percent;
  if (rtb_inReverse) {
    Autothrust_DWork.Delay_DSTATE_n = Autothrust_P.DiscreteTimeIntegratorVariableTs_InitialCondition;
  }

  Autothrust_DWork.Delay_DSTATE_n += Autothrust_P.Gain_Gain_d * rtb_Saturation *
    Autothrust_P.DiscreteTimeIntegratorVariableTs_Gain * Autothrust_U.in.time.dt;
  if (Autothrust_DWork.Delay_DSTATE_n > Autothrust_P.DiscreteTimeIntegratorVariableTs_UpperLimit) {
    Autothrust_DWork.Delay_DSTATE_n = Autothrust_P.DiscreteTimeIntegratorVariableTs_UpperLimit;
  } else {
    if (Autothrust_DWork.Delay_DSTATE_n < Autothrust_P.DiscreteTimeIntegratorVariableTs_LowerLimit) {
      Autothrust_DWork.Delay_DSTATE_n = Autothrust_P.DiscreteTimeIntegratorVariableTs_LowerLimit;
    }
  }

  if (!rtb_inReverse) {
    Autothrust_DWork.Delay_DSTATE_l2 = Autothrust_P.DiscreteTimeIntegratorVariableTs1_InitialCondition;
  }

  Autothrust_DWork.Delay_DSTATE_l2 += Autothrust_P.Gain1_Gain_h * rtb_Saturation *
    Autothrust_P.DiscreteTimeIntegratorVariableTs1_Gain * Autothrust_U.in.time.dt;
  if (Autothrust_DWork.Delay_DSTATE_l2 > Autothrust_P.DiscreteTimeIntegratorVariableTs1_UpperLimit) {
    Autothrust_DWork.Delay_DSTATE_l2 = Autothrust_P.DiscreteTimeIntegratorVariableTs1_UpperLimit;
  } else {
    if (Autothrust_DWork.Delay_DSTATE_l2 < Autothrust_P.DiscreteTimeIntegratorVariableTs1_LowerLimit) {
      Autothrust_DWork.Delay_DSTATE_l2 = Autothrust_P.DiscreteTimeIntegratorVariableTs1_LowerLimit;
    }
  }

  Autothrust_ThrustMode1(Autothrust_U.in.input.TLA_1_deg, &rtb_Saturation1);
  rtb_Switch_d = rtb_Switch_f_idx_1 - rtb_BusAssignment.data.commanded_engine_N1_2_percent;
  if (rtb_NOT1) {
    Autothrust_DWork.Delay_DSTATE_lz = Autothrust_P.DiscreteTimeIntegratorVariableTs_InitialCondition_n;
  }

  rtb_y_b = Autothrust_P.Gain_Gain_bf * rtb_Switch_d * Autothrust_P.DiscreteTimeIntegratorVariableTs_Gain_k *
    Autothrust_U.in.time.dt + Autothrust_DWork.Delay_DSTATE_lz;
  if (rtb_y_b > Autothrust_P.DiscreteTimeIntegratorVariableTs_UpperLimit_p) {
    Autothrust_DWork.Delay_DSTATE_lz = Autothrust_P.DiscreteTimeIntegratorVariableTs_UpperLimit_p;
  } else if (rtb_y_b < Autothrust_P.DiscreteTimeIntegratorVariableTs_LowerLimit_e) {
    Autothrust_DWork.Delay_DSTATE_lz = Autothrust_P.DiscreteTimeIntegratorVariableTs_LowerLimit_e;
  } else {
    Autothrust_DWork.Delay_DSTATE_lz = rtb_y_b;
  }

  if (!rtb_NOT1) {
    Autothrust_DWork.Delay_DSTATE_h = Autothrust_P.DiscreteTimeIntegratorVariableTs1_InitialCondition_e;
  }

  Autothrust_DWork.Delay_DSTATE_h += Autothrust_P.Gain1_Gain_g * rtb_Switch_d *
    Autothrust_P.DiscreteTimeIntegratorVariableTs1_Gain_l * Autothrust_U.in.time.dt;
  if (Autothrust_DWork.Delay_DSTATE_h > Autothrust_P.DiscreteTimeIntegratorVariableTs1_UpperLimit_o) {
    Autothrust_DWork.Delay_DSTATE_h = Autothrust_P.DiscreteTimeIntegratorVariableTs1_UpperLimit_o;
  } else {
    if (Autothrust_DWork.Delay_DSTATE_h < Autothrust_P.DiscreteTimeIntegratorVariableTs1_LowerLimit_h) {
      Autothrust_DWork.Delay_DSTATE_h = Autothrust_P.DiscreteTimeIntegratorVariableTs1_LowerLimit_h;
    }
  }

  Autothrust_ThrustMode1(Autothrust_U.in.input.TLA_2_deg, &rtb_y_b);
  Autothrust_Y.out.time = Autothrust_U.in.time;
  Autothrust_Y.out.data = rtb_BusAssignment.data;
  Autothrust_Y.out.data_computed.TLA_in_active_range = rtb_out;
  Autothrust_Y.out.data_computed.is_FLX_active = rtb_y_h;
  Autothrust_Y.out.data_computed.ATHR_push =
    rtb_BusConversion_InsertedFor_BusAssignment_at_inport_1_BusCreator1_g_ATHR_push;
  Autothrust_Y.out.data_computed.ATHR_disabled = Autothrust_DWork.Memory_PreviousInput;
  Autothrust_Y.out.input = rtb_BusAssignment.input;
  if (!rtb_inReverse) {
    Autothrust_Y.out.output.sim_throttle_lever_1_pos = Autothrust_DWork.Delay_DSTATE_n;
  } else {
    Autothrust_Y.out.output.sim_throttle_lever_1_pos = Autothrust_DWork.Delay_DSTATE_l2;
  }

  if (!rtb_NOT1) {
    Autothrust_Y.out.output.sim_throttle_lever_2_pos = Autothrust_DWork.Delay_DSTATE_lz;
  } else {
    Autothrust_Y.out.output.sim_throttle_lever_2_pos = Autothrust_DWork.Delay_DSTATE_h;
  }

  Autothrust_Y.out.output.sim_thrust_mode_1 = rtb_Saturation1;
  Autothrust_Y.out.output.sim_thrust_mode_2 = rtb_y_b;
  Autothrust_Y.out.output.N1_TLA_1_percent = max_TLA_tmp;
  Autothrust_Y.out.output.is_in_reverse_1 = rtb_inReverse;
  Autothrust_Y.out.output.is_in_reverse_2 = rtb_NOT1;
  Autothrust_Y.out.output.N1_c_1_percent = rtb_Sum2;
  Autothrust_Y.out.output.N1_c_2_percent = rtb_Switch_f_idx_1;
  Autothrust_Y.out.output.status = rtb_status;
  Autothrust_Y.out.output.mode = Autothrust_DWork.pMode;
  ATHR_ENGAGED_tmp_0 = (((!Autothrust_U.in.input.is_SRS_TO_mode_active) || ((Autothrust_U.in.data.H_ind_ft >=
    Autothrust_U.in.input.thrust_reduction_altitude) && (!Autothrust_DWork.inhibitAboveThrustReductionAltitude))) &&
                        ((!Autothrust_U.in.input.is_SRS_GA_mode_active) || ((Autothrust_U.in.data.H_ind_ft >=
    Autothrust_U.in.input.thrust_reduction_altitude_go_around) && (!Autothrust_DWork.inhibitAboveThrustReductionAltitude))));
  if ((rtb_status != athr_status_DISENGAGED) && (rtb_on_ground == 0) && ATHR_ENGAGED_tmp_0 &&
      Autothrust_U.in.data.is_engine_operative_1 && Autothrust_U.in.data.is_engine_operative_2 &&
      (((Autothrust_U.in.input.TLA_1_deg < 25.0) && (Autothrust_U.in.input.TLA_2_deg < 25.0)) ||
       (Autothrust_U.in.input.TLA_1_deg > 25.0) || (Autothrust_U.in.input.TLA_2_deg > 25.0))) {
    Autothrust_Y.out.output.mode_message = athr_mode_message_LVR_CLB;
  } else if ((rtb_status != athr_status_DISENGAGED) && (rtb_on_ground == 0) && ATHR_ENGAGED_tmp_0 &&
             (condition_AP_FD_ATHR_Specific || ATHR_ENGAGED_tmp) && (Autothrust_U.in.input.TLA_1_deg != 35.0) &&
             (Autothrust_U.in.input.TLA_2_deg != 35.0)) {
    Autothrust_Y.out.output.mode_message = athr_mode_message_LVR_MCT;
  } else if ((rtb_status == athr_status_ENGAGED_ACTIVE) && Autothrust_U.in.data.is_engine_operative_1 &&
             Autothrust_U.in.data.is_engine_operative_2 && (((Autothrust_U.in.input.TLA_1_deg == 25.0) &&
               (Autothrust_U.in.input.TLA_2_deg != 25.0)) || ((Autothrust_U.in.input.TLA_2_deg == 25.0) &&
               (Autothrust_U.in.input.TLA_1_deg != 25.0)))) {
    Autothrust_Y.out.output.mode_message = athr_mode_message_LVR_ASYM;
  } else if (Autothrust_DWork.condition_THR_LK) {
    Autothrust_Y.out.output.mode_message = athr_mode_message_THR_LK;
  } else {
    Autothrust_Y.out.output.mode_message = athr_mode_message_NONE;
  }

  Autothrust_DWork.Delay_DSTATE_g = rtb_Gain_f;
  Autothrust_DWork.Delay_DSTATE_l = rtb_Divide;
  Autothrust_DWork.icLoad = 0U;
}

void AutothrustModelClass::initialize()
{
  Autothrust_DWork.Memory_PreviousInput = Autothrust_P.SRFlipFlop_initial_condition;
  Autothrust_DWork.Delay_DSTATE_b = Autothrust_P.Delay_InitialCondition_a;
  Autothrust_DWork.Memory_PreviousInput_f = Autothrust_P.SRFlipFlop_initial_condition_a;
  Autothrust_DWork.Delay_DSTATE = Autothrust_P.RateLimiterDynamicEqualVariableTs_InitialCondition;
  Autothrust_DWork.Delay_DSTATE_g = Autothrust_P.DiscreteDerivativeVariableTs_InitialCondition;
  Autothrust_DWork.Delay_DSTATE_l = Autothrust_P.Delay_InitialCondition;
  Autothrust_DWork.Delay1_DSTATE = Autothrust_P.Delay1_InitialCondition;
  Autothrust_DWork.icLoad = 1U;
  Autothrust_DWork.Delay_DSTATE_n = Autothrust_P.DiscreteTimeIntegratorVariableTs_InitialCondition;
  Autothrust_DWork.Delay_DSTATE_l2 = Autothrust_P.DiscreteTimeIntegratorVariableTs1_InitialCondition;
  Autothrust_DWork.Delay_DSTATE_lz = Autothrust_P.DiscreteTimeIntegratorVariableTs_InitialCondition_n;
  Autothrust_DWork.Delay_DSTATE_h = Autothrust_P.DiscreteTimeIntegratorVariableTs1_InitialCondition_e;
  Autothrust_DWork.pMode = athr_mode_NONE;
  Autothrust_DWork.pStatus = athr_status_DISENGAGED;
}

void AutothrustModelClass::terminate()
{
}

AutothrustModelClass::AutothrustModelClass() :
  Autothrust_DWork(),
  Autothrust_U(),
  Autothrust_Y()
{
}

AutothrustModelClass::~AutothrustModelClass()
{
}
