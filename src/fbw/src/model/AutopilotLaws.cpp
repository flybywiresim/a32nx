#include "AutopilotLaws.h"
#include "AutopilotLaws_private.h"
#include "look1_binlxpw.h"
#include "rt_modd.h"

const uint8_T AutopilotLaws_IN_NO_ACTIVE_CHILD = 0U;
const uint8_T AutopilotLaws_IN_any = 1U;
const uint8_T AutopilotLaws_IN_left = 2U;
const uint8_T AutopilotLaws_IN_right = 3U;
const uint8_T AutopilotLaws_IN_NO_ACTIVE_CHILD_h = 0U;
const uint8_T AutopilotLaws_IN_any_o = 1U;
const uint8_T AutopilotLaws_IN_left_n = 2U;
const uint8_T AutopilotLaws_IN_right_i = 3U;
const uint8_T AutopilotLaws_IN_InAir = 1U;
const uint8_T AutopilotLaws_IN_NO_ACTIVE_CHILD_n = 0U;
const uint8_T AutopilotLaws_IN_OnGround = 2U;
const ap_laws_output AutopilotLaws_rtZap_laws_output = {
  {
    0.0,
    0.0
  },

  {
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    false,
    0.0,
    0.0,
    0.0,
    false,
    0.0,
    false,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    false,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0
  },

  {
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    false,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    false
  },

  {
    0.0,

    {
      0.0,
      0.0,
      0.0
    },

    {
      0.0,
      0.0,
      0.0
    }
  }
} ;

const ap_laws_input AutopilotLaws_rtZap_laws_input = { { 0.0, 0.0 }, { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false, 0.0, 0.0, 0.0, false, 0.0, false, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false } };

void AutopilotLawsModelClass::AutopilotLaws_Chart_Init(rtDW_Chart_AutopilotLaws_T *localDW)
{
  localDW->is_active_c3_AutopilotLaws = 0U;
  localDW->is_c3_AutopilotLaws = AutopilotLaws_IN_NO_ACTIVE_CHILD;
}

void AutopilotLawsModelClass::AutopilotLaws_Chart(real_T rtu_right, real_T rtu_left, real_T rtu_use_short_path, real_T
  *rty_out, rtDW_Chart_AutopilotLaws_T *localDW)
{
  real_T tmp;
  real_T tmp_0;
  if (localDW->is_active_c3_AutopilotLaws == 0U) {
    localDW->is_active_c3_AutopilotLaws = 1U;
    localDW->is_c3_AutopilotLaws = AutopilotLaws_IN_any;
    if (std::abs(rtu_left) < std::abs(rtu_right)) {
      *rty_out = rtu_left;
    } else {
      *rty_out = rtu_right;
    }
  } else {
    switch (localDW->is_c3_AutopilotLaws) {
     case AutopilotLaws_IN_any:
      tmp = std::abs(rtu_right);
      tmp_0 = std::abs(rtu_left);
      if ((rtu_use_short_path == 0.0) && (tmp < tmp_0) && (tmp >= 10.0) && (tmp <= 20.0)) {
        localDW->is_c3_AutopilotLaws = AutopilotLaws_IN_right;
        *rty_out = rtu_right;
      } else if ((rtu_use_short_path == 0.0) && (tmp_0 < tmp) && (tmp_0 >= 10.0) && (tmp_0 <= 20.0)) {
        localDW->is_c3_AutopilotLaws = AutopilotLaws_IN_left;
        *rty_out = rtu_left;
      } else if (tmp_0 < tmp) {
        *rty_out = rtu_left;
      } else {
        *rty_out = rtu_right;
      }
      break;

     case AutopilotLaws_IN_left:
      tmp = std::abs(rtu_left);
      tmp_0 = std::abs(rtu_right);
      if ((rtu_use_short_path != 0.0) || (tmp_0 < 10.0) || (tmp < 10.0)) {
        localDW->is_c3_AutopilotLaws = AutopilotLaws_IN_any;
        if (tmp < tmp_0) {
          *rty_out = rtu_left;
        } else {
          *rty_out = rtu_right;
        }
      } else {
        *rty_out = rtu_left;
      }
      break;

     default:
      tmp = std::abs(rtu_left);
      tmp_0 = std::abs(rtu_right);
      if ((rtu_use_short_path != 0.0) || (tmp_0 < 10.0) || (tmp < 10.0)) {
        localDW->is_c3_AutopilotLaws = AutopilotLaws_IN_any;
        if (tmp < tmp_0) {
          *rty_out = rtu_left;
        } else {
          *rty_out = rtu_right;
        }
      } else {
        *rty_out = rtu_right;
      }
      break;
    }
  }
}

void AutopilotLawsModelClass::AutopilotLaws_Chart_k_Init(rtDW_Chart_AutopilotLaws_m_T *localDW)
{
  localDW->is_active_c1_AutopilotLaws = 0U;
  localDW->is_c1_AutopilotLaws = AutopilotLaws_IN_NO_ACTIVE_CHILD_h;
}

void AutopilotLawsModelClass::AutopilotLaws_Chart_n(real_T rtu_right, real_T rtu_left, boolean_T rtu_use_short_path,
  real_T *rty_out, rtDW_Chart_AutopilotLaws_m_T *localDW)
{
  real_T tmp;
  real_T tmp_0;
  boolean_T tmp_1;
  if (localDW->is_active_c1_AutopilotLaws == 0U) {
    localDW->is_active_c1_AutopilotLaws = 1U;
    localDW->is_c1_AutopilotLaws = AutopilotLaws_IN_any_o;
    if (std::abs(rtu_left) < std::abs(rtu_right)) {
      *rty_out = rtu_left;
    } else {
      *rty_out = rtu_right;
    }
  } else {
    switch (localDW->is_c1_AutopilotLaws) {
     case AutopilotLaws_IN_any_o:
      tmp = std::abs(rtu_right);
      tmp_0 = std::abs(rtu_left);
      tmp_1 = !rtu_use_short_path;
      if (tmp_1 && (tmp < tmp_0) && (tmp >= 10.0) && (tmp <= 20.0)) {
        localDW->is_c1_AutopilotLaws = AutopilotLaws_IN_right_i;
        *rty_out = rtu_right;
      } else if (tmp_1 && (tmp_0 < tmp) && (tmp_0 >= 10.0) && (tmp_0 <= 20.0)) {
        localDW->is_c1_AutopilotLaws = AutopilotLaws_IN_left_n;
        *rty_out = rtu_left;
      } else if (tmp_0 < tmp) {
        *rty_out = rtu_left;
      } else {
        *rty_out = rtu_right;
      }
      break;

     case AutopilotLaws_IN_left_n:
      tmp = std::abs(rtu_left);
      tmp_0 = std::abs(rtu_right);
      if (rtu_use_short_path || (tmp_0 < 10.0) || (tmp < 10.0)) {
        localDW->is_c1_AutopilotLaws = AutopilotLaws_IN_any_o;
        if (tmp < tmp_0) {
          *rty_out = rtu_left;
        } else {
          *rty_out = rtu_right;
        }
      } else {
        *rty_out = rtu_left;
      }
      break;

     default:
      tmp = std::abs(rtu_left);
      tmp_0 = std::abs(rtu_right);
      if (rtu_use_short_path || (tmp_0 < 10.0) || (tmp < 10.0)) {
        localDW->is_c1_AutopilotLaws = AutopilotLaws_IN_any_o;
        if (tmp < tmp_0) {
          *rty_out = rtu_left;
        } else {
          *rty_out = rtu_right;
        }
      } else {
        *rty_out = rtu_right;
      }
      break;
    }
  }
}

void AutopilotLawsModelClass::AutopilotLaws_Voter1(real_T rtu_u1, real_T rtu_u2, real_T rtu_u3, real_T *rty_Y)
{
  real_T x[3];
  int32_T rtu_u1_0;
  x[0] = rtu_u1;
  x[1] = rtu_u2;
  x[2] = rtu_u3;
  if (rtu_u1 < rtu_u2) {
    if (rtu_u2 < rtu_u3) {
      rtu_u1_0 = 1;
    } else if (rtu_u1 < rtu_u3) {
      rtu_u1_0 = 2;
    } else {
      rtu_u1_0 = 0;
    }
  } else if (rtu_u1 < rtu_u3) {
    rtu_u1_0 = 0;
  } else if (rtu_u2 < rtu_u3) {
    rtu_u1_0 = 2;
  } else {
    rtu_u1_0 = 1;
  }

  *rty_Y = x[rtu_u1_0];
}

void AutopilotLawsModelClass::step()
{
  real_T result[3];
  real_T rtb_Minup;
  real_T rtb_Divide_e;
  real_T rtb_Divide_p;
  real_T rtb_GainTheta;
  real_T rtb_GainTheta1;
  real_T rtb_Saturation;
  real_T rtb_Saturation1;
  int32_T rtb_on_ground;
  int32_T rtb_BusAssignment_output_ap_on;
  real_T rtb_ManualSwitch;
  real_T rtb_Mod1_b;
  real_T rtb_Mod2;
  boolean_T rtb_Compare_b;
  boolean_T rtb_Compare_h;
  real_T rtb_Mod1_k;
  real_T rtb_Gain_ot;
  real_T rtb_Gain_ju3;
  real_T rtb_Sum2_d;
  real_T rtb_Gain_k;
  real_T rtb_Gain_or;
  real_T rtb_Gain4;
  real_T rtb_Sum2_ho;
  real_T rtb_Gain_o;
  real_T rtb_Y_j;
  real_T rtb_Sum2_l;
  real_T rtb_Y;
  real_T rtb_Sum_l1;
  real_T rtb_out;
  real_T rtb_out_co;
  real_T rtb_out_f;
  real_T rtb_out_gk;
  int32_T i;
  real_T tmp[9];
  uint32_T rtb_Divide_o;
  boolean_T rtb_Divide_g;
  real_T rtb_Sum_o5_tmp;
  rtb_GainTheta = AutopilotLaws_P.GainTheta_Gain * AutopilotLaws_U.in.data.Theta_deg;
  rtb_GainTheta1 = AutopilotLaws_P.GainTheta1_Gain * AutopilotLaws_U.in.data.Phi_deg;
  rtb_Saturation1 = 0.017453292519943295 * rtb_GainTheta;
  rtb_Mod1_k = 0.017453292519943295 * rtb_GainTheta1;
  rtb_out_gk = std::tan(rtb_Saturation1);
  rtb_Saturation = std::sin(rtb_Mod1_k);
  rtb_Mod1_k = std::cos(rtb_Mod1_k);
  tmp[0] = 1.0;
  tmp[3] = rtb_Saturation * rtb_out_gk;
  tmp[6] = rtb_Mod1_k * rtb_out_gk;
  tmp[1] = 0.0;
  tmp[4] = rtb_Mod1_k;
  tmp[7] = -rtb_Saturation;
  tmp[2] = 0.0;
  rtb_out_f = 1.0 / std::cos(rtb_Saturation1);
  tmp[5] = rtb_out_f * rtb_Saturation;
  tmp[8] = rtb_out_f * rtb_Mod1_k;
  rtb_Saturation = AutopilotLaws_P.Gain_Gain_de * AutopilotLaws_U.in.data.p_rad_s * AutopilotLaws_P.Gainpk_Gain;
  rtb_Saturation1 = AutopilotLaws_P.Gain_Gain_d * AutopilotLaws_U.in.data.q_rad_s * AutopilotLaws_P.Gainqk_Gain;
  rtb_out_gk = AutopilotLaws_P.Gain_Gain_m * AutopilotLaws_U.in.data.r_rad_s;
  for (rtb_on_ground = 0; rtb_on_ground < 3; rtb_on_ground++) {
    result[rtb_on_ground] = tmp[rtb_on_ground + 6] * rtb_out_gk + (tmp[rtb_on_ground + 3] * rtb_Saturation1 +
      tmp[rtb_on_ground] * rtb_Saturation);
  }

  rtb_Saturation = AutopilotLaws_P.Gain_Gain_n * AutopilotLaws_U.in.data.gear_strut_compression_1 -
    AutopilotLaws_P.Constant1_Value_b;
  if (rtb_Saturation > AutopilotLaws_P.Saturation_UpperSat) {
    rtb_Saturation = AutopilotLaws_P.Saturation_UpperSat;
  } else {
    if (rtb_Saturation < AutopilotLaws_P.Saturation_LowerSat) {
      rtb_Saturation = AutopilotLaws_P.Saturation_LowerSat;
    }
  }

  rtb_Saturation1 = AutopilotLaws_P.Gain1_Gain_ll * AutopilotLaws_U.in.data.gear_strut_compression_2 -
    AutopilotLaws_P.Constant1_Value_b;
  if (rtb_Saturation1 > AutopilotLaws_P.Saturation1_UpperSat_j) {
    rtb_Saturation1 = AutopilotLaws_P.Saturation1_UpperSat_j;
  } else {
    if (rtb_Saturation1 < AutopilotLaws_P.Saturation1_LowerSat_d) {
      rtb_Saturation1 = AutopilotLaws_P.Saturation1_LowerSat_d;
    }
  }

  if (AutopilotLaws_DWork.is_active_c5_AutopilotLaws == 0U) {
    AutopilotLaws_DWork.is_active_c5_AutopilotLaws = 1U;
    AutopilotLaws_DWork.is_c5_AutopilotLaws = AutopilotLaws_IN_OnGround;
    rtb_on_ground = 1;
  } else if (AutopilotLaws_DWork.is_c5_AutopilotLaws == AutopilotLaws_IN_InAir) {
    if ((rtb_Saturation > 0.05) || (rtb_Saturation1 > 0.05)) {
      AutopilotLaws_DWork.is_c5_AutopilotLaws = AutopilotLaws_IN_OnGround;
      rtb_on_ground = 1;
    } else {
      rtb_on_ground = 0;
    }
  } else {
    if ((rtb_Saturation == 0.0) && (rtb_Saturation1 == 0.0)) {
      AutopilotLaws_DWork.is_c5_AutopilotLaws = AutopilotLaws_IN_InAir;
      rtb_on_ground = 0;
    } else {
      rtb_on_ground = 1;
    }
  }

  rtb_BusAssignment_output_ap_on = ((AutopilotLaws_U.in.input.enabled_AP1 != 0.0) ||
    (AutopilotLaws_U.in.input.enabled_AP2 != 0.0));
  rtb_Saturation = AutopilotLaws_P.Gain2_Gain_b * AutopilotLaws_U.in.data.zeta_pos;
  rtb_Saturation1 = rt_modd((AutopilotLaws_U.in.data.nav_loc_deg - (AutopilotLaws_U.in.data.Psi_magnetic_deg +
    AutopilotLaws_P.Constant3_Value_o)) + AutopilotLaws_P.Constant3_Value_o, AutopilotLaws_P.Constant3_Value_o);
  AutopilotLaws_Chart(rtb_Saturation1, AutopilotLaws_P.Gain_Gain_f * rt_modd(AutopilotLaws_P.Constant3_Value_o -
    rtb_Saturation1, AutopilotLaws_P.Constant3_Value_o), AutopilotLaws_P.Constant2_Value, &rtb_out_gk,
                      &AutopilotLaws_DWork.sf_Chart_g2);
  if (AutopilotLaws_U.in.data.H_radio_ft <= AutopilotLaws_P.CompareToConstant_const) {
    rtb_Saturation1 = AutopilotLaws_P.Gain_Gain_a * rtb_out_gk;
  } else {
    rtb_Saturation1 = AutopilotLaws_P.Constant1_Value;
  }

  rtb_out_gk = AutopilotLaws_U.in.time.dt * AutopilotLaws_P.LagFilter1_C1;
  rtb_Minup = rtb_out_gk + AutopilotLaws_P.Constant_Value_l;
  AutopilotLaws_DWork.Delay1_DSTATE = 1.0 / rtb_Minup * (AutopilotLaws_P.Constant_Value_l - rtb_out_gk) *
    AutopilotLaws_DWork.Delay1_DSTATE + (rtb_Saturation1 + AutopilotLaws_DWork.Delay_DSTATE) * (rtb_out_gk / rtb_Minup);
  if (AutopilotLaws_P.ManualSwitch_CurrentSetting == 1) {
    rtb_ManualSwitch = AutopilotLaws_P.Constant_Value;
  } else {
    rtb_ManualSwitch = AutopilotLaws_U.in.input.lateral_law;
  }

  if (AutopilotLaws_U.in.data.nav_dme_nmi > AutopilotLaws_P.Saturation_UpperSat_o) {
    rtb_out_f = AutopilotLaws_P.Saturation_UpperSat_o;
  } else if (AutopilotLaws_U.in.data.nav_dme_nmi < AutopilotLaws_P.Saturation_LowerSat_o) {
    rtb_out_f = AutopilotLaws_P.Saturation_LowerSat_o;
  } else {
    rtb_out_f = AutopilotLaws_U.in.data.nav_dme_nmi;
  }

  rtb_out_gk = std::sin(AutopilotLaws_P.Gain1_Gain_or * AutopilotLaws_U.in.data.nav_loc_error_deg) * rtb_out_f *
    AutopilotLaws_P.Gain2_Gain_g;
  if (rtb_out_gk > AutopilotLaws_P.Saturation1_UpperSat_g) {
    rtb_out_gk = AutopilotLaws_P.Saturation1_UpperSat_g;
  } else {
    if (rtb_out_gk < AutopilotLaws_P.Saturation1_LowerSat_k) {
      rtb_out_gk = AutopilotLaws_P.Saturation1_LowerSat_k;
    }
  }

  if (rtb_ManualSwitch != AutopilotLaws_P.CompareToConstant_const_k) {
    AutopilotLaws_DWork.Delay_DSTATE_h = AutopilotLaws_P.DiscreteTimeIntegratorVariableTs_InitialCondition;
  }

  AutopilotLaws_DWork.Delay_DSTATE_h += AutopilotLaws_P.Gain6_Gain * rtb_out_gk *
    AutopilotLaws_P.DiscreteTimeIntegratorVariableTs_Gain * AutopilotLaws_U.in.time.dt;
  if (AutopilotLaws_DWork.Delay_DSTATE_h > AutopilotLaws_P.DiscreteTimeIntegratorVariableTs_UpperLimit) {
    AutopilotLaws_DWork.Delay_DSTATE_h = AutopilotLaws_P.DiscreteTimeIntegratorVariableTs_UpperLimit;
  } else {
    if (AutopilotLaws_DWork.Delay_DSTATE_h < AutopilotLaws_P.DiscreteTimeIntegratorVariableTs_LowerLimit) {
      AutopilotLaws_DWork.Delay_DSTATE_h = AutopilotLaws_P.DiscreteTimeIntegratorVariableTs_LowerLimit;
    }
  }

  rtb_Mod1_k = AutopilotLaws_U.in.data.nav_loc_error_deg + AutopilotLaws_U.in.data.nav_loc_deg;
  rtb_Mod1_b = rt_modd((AutopilotLaws_U.in.data.Psi_magnetic_deg - (rt_modd(rt_modd(rtb_Mod1_k,
    AutopilotLaws_P.Constant3_Value_ej) + AutopilotLaws_P.Constant3_Value_ej, AutopilotLaws_P.Constant3_Value_ej) +
    AutopilotLaws_P.Constant3_Value_k)) + AutopilotLaws_P.Constant3_Value_k, AutopilotLaws_P.Constant3_Value_k);
  rtb_Mod2 = rt_modd(AutopilotLaws_P.Constant3_Value_k - rtb_Mod1_b, AutopilotLaws_P.Constant3_Value_k);
  if (rtb_Mod1_b < rtb_Mod2) {
    rtb_Mod1_b *= AutopilotLaws_P.Gain1_Gain_o;
  } else {
    rtb_Mod1_b = AutopilotLaws_P.Gain_Gain_c * rtb_Mod2;
  }

  rtb_out_gk = rt_modd((rt_modd(rt_modd(((rtb_out_gk * look1_binlxpw(AutopilotLaws_U.in.data.V_gnd_kn,
    AutopilotLaws_P.ScheduledGain_BreakpointsForDimension1, AutopilotLaws_P.ScheduledGain_Table, 2U) +
    AutopilotLaws_DWork.Delay_DSTATE_h) + AutopilotLaws_P.Gain1_Gain_f * rtb_Mod1_b) +
    AutopilotLaws_U.in.data.Psi_magnetic_deg, AutopilotLaws_P.Constant3_Value_e) + AutopilotLaws_P.Constant3_Value_e,
    AutopilotLaws_P.Constant3_Value_e) - (AutopilotLaws_U.in.data.Psi_magnetic_deg + AutopilotLaws_P.Constant3_Value_n))
                       + AutopilotLaws_P.Constant3_Value_n, AutopilotLaws_P.Constant3_Value_n);
  AutopilotLaws_Chart(rtb_out_gk, AutopilotLaws_P.Gain_Gain_fr * rt_modd(AutopilotLaws_P.Constant3_Value_n - rtb_out_gk,
    AutopilotLaws_P.Constant3_Value_n), AutopilotLaws_P.Constant1_Value_e, &rtb_Mod1_b, &AutopilotLaws_DWork.sf_Chart_j);
  switch (static_cast<int32_T>(rtb_ManualSwitch)) {
   case 0:
    rtb_out_gk = AutopilotLaws_P.beta_Value_ed;
    break;

   case 1:
    rtb_out_gk = AutopilotLaws_P.beta_Value_e;
    break;

   case 2:
    rtb_out_gk = AutopilotLaws_P.beta_Value_b;
    break;

   case 3:
    rtb_out_gk = AutopilotLaws_P.beta_Value_m;
    break;

   case 4:
    rtb_out_gk = AutopilotLaws_P.beta_Value;
    break;

   case 5:
    rtb_out_gk = AutopilotLaws_DWork.Delay1_DSTATE;
    break;

   default:
    rtb_out_gk = (AutopilotLaws_P.Gain5_Gain * rtb_Mod1_b + AutopilotLaws_P.Gain_Gain_b * result[2]) + rtb_Saturation;
    break;
  }

  rtb_Mod2 = look1_binlxpw(AutopilotLaws_U.in.data.V_tas_kn, AutopilotLaws_P.ROLLLIM1_bp01Data,
    AutopilotLaws_P.ROLLLIM1_tableData, 4U);
  rtb_Mod1_b = rt_modd((AutopilotLaws_U.in.input.Psi_c_deg - (AutopilotLaws_U.in.data.Psi_magnetic_deg +
    AutopilotLaws_P.Constant3_Value_f)) + AutopilotLaws_P.Constant3_Value_f, AutopilotLaws_P.Constant3_Value_f);
  rtb_Compare_b = ((rtb_ManualSwitch == AutopilotLaws_P.CompareToConstant5_const) == static_cast<int32_T>
                   (AutopilotLaws_P.CompareToConstant_const_h));
  rtb_Divide_e = AutopilotLaws_P.Subsystem_Value / AutopilotLaws_U.in.time.dt;
  if (!rtb_Compare_b) {
    for (i = 0; i < 100; i++) {
      AutopilotLaws_DWork.Delay_DSTATE_lp[i] = AutopilotLaws_P.Delay_InitialCondition_h;
    }
  }

  if (rtb_Divide_e > 100.0) {
    rtb_Divide_o = 100U;
  } else {
    if (rtb_Divide_e < 0.0) {
      rtb_out_f = std::ceil(rtb_Divide_e);
    } else {
      rtb_out_f = std::floor(rtb_Divide_e);
    }

    rtb_out_f = std::fmod(rtb_out_f, 4.294967296E+9);
    rtb_Divide_o = rtb_out_f < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(static_cast<uint32_T>(-rtb_out_f))) :
      static_cast<uint32_T>(rtb_out_f);
  }

  if (rtb_Divide_e < 1.0) {
    rtb_Compare_h = rtb_Compare_b;
  } else {
    rtb_Compare_h = AutopilotLaws_DWork.Delay_DSTATE_lp[100U - rtb_Divide_o];
  }

  AutopilotLaws_Chart_n(rtb_Mod1_b, AutopilotLaws_P.Gain_Gain_a0 * rt_modd(AutopilotLaws_P.Constant3_Value_f -
    rtb_Mod1_b, AutopilotLaws_P.Constant3_Value_f), rtb_Compare_b != rtb_Compare_h, &rtb_Divide_e,
                        &AutopilotLaws_DWork.sf_Chart_n);
  rtb_Mod1_b = rt_modd((AutopilotLaws_U.in.input.Psi_c_deg - (AutopilotLaws_U.in.data.Psi_magnetic_track_deg +
    AutopilotLaws_P.Constant3_Value_o3)) + AutopilotLaws_P.Constant3_Value_o3, AutopilotLaws_P.Constant3_Value_o3);
  rtb_Compare_h = ((rtb_ManualSwitch == AutopilotLaws_P.CompareToConstant4_const) == static_cast<int32_T>
                   (AutopilotLaws_P.CompareToConstant_const_e));
  rtb_Divide_p = AutopilotLaws_P.Subsystem_Value_n / AutopilotLaws_U.in.time.dt;
  if (!rtb_Compare_h) {
    for (i = 0; i < 100; i++) {
      AutopilotLaws_DWork.Delay_DSTATE_h5[i] = AutopilotLaws_P.Delay_InitialCondition_bl;
    }
  }

  if (rtb_Divide_p > 100.0) {
    rtb_Divide_o = 100U;
  } else {
    if (rtb_Divide_p < 0.0) {
      rtb_out_f = std::ceil(rtb_Divide_p);
    } else {
      rtb_out_f = std::floor(rtb_Divide_p);
    }

    rtb_out_f = std::fmod(rtb_out_f, 4.294967296E+9);
    rtb_Divide_o = rtb_out_f < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(static_cast<uint32_T>(-rtb_out_f))) :
      static_cast<uint32_T>(rtb_out_f);
  }

  if (rtb_Divide_p < 1.0) {
    rtb_Divide_g = rtb_Compare_h;
  } else {
    rtb_Divide_g = AutopilotLaws_DWork.Delay_DSTATE_h5[100U - rtb_Divide_o];
  }

  AutopilotLaws_Chart_n(rtb_Mod1_b, AutopilotLaws_P.Gain_Gain_ab * rt_modd(AutopilotLaws_P.Constant3_Value_o3 -
    rtb_Mod1_b, AutopilotLaws_P.Constant3_Value_o3), rtb_Compare_h != rtb_Divide_g, &rtb_out,
                        &AutopilotLaws_DWork.sf_Chart_f);
  rtb_out_f = AutopilotLaws_P.Gain_Gain_nu * AutopilotLaws_U.in.data.flight_guidance_xtk_nmi;
  if (rtb_out_f > AutopilotLaws_P.Saturation_UpperSat_k) {
    rtb_out_f = AutopilotLaws_P.Saturation_UpperSat_k;
  } else {
    if (rtb_out_f < AutopilotLaws_P.Saturation_LowerSat_p) {
      rtb_out_f = AutopilotLaws_P.Saturation_LowerSat_p;
    }
  }

  rtb_Mod1_b = rt_modd((rt_modd(rt_modd((AutopilotLaws_P.Gain2_Gain_f * AutopilotLaws_U.in.data.flight_guidance_tae_deg
    + rtb_out_f) * AutopilotLaws_P.Gain1_Gain_nh + AutopilotLaws_U.in.data.Psi_magnetic_track_deg,
    AutopilotLaws_P.Constant3_Value_l) + AutopilotLaws_P.Constant3_Value_l, AutopilotLaws_P.Constant3_Value_l) -
                        (AutopilotLaws_U.in.data.Psi_magnetic_track_deg + AutopilotLaws_P.Constant3_Value_fy)) +
                       AutopilotLaws_P.Constant3_Value_fy, AutopilotLaws_P.Constant3_Value_fy);
  AutopilotLaws_Chart(rtb_Mod1_b, AutopilotLaws_P.Gain_Gain_o * rt_modd(AutopilotLaws_P.Constant3_Value_fy - rtb_Mod1_b,
    AutopilotLaws_P.Constant3_Value_fy), AutopilotLaws_P.Constant_Value_c, &rtb_out_co, &AutopilotLaws_DWork.sf_Chart);
  rtb_Mod1_k = rt_modd((AutopilotLaws_U.in.data.Psi_magnetic_track_deg - (rt_modd(rt_modd(rtb_Mod1_k,
    AutopilotLaws_P.Constant3_Value_dq) + AutopilotLaws_P.Constant3_Value_dq, AutopilotLaws_P.Constant3_Value_dq) +
    AutopilotLaws_P.Constant3_Value_dw)) + AutopilotLaws_P.Constant3_Value_dw, AutopilotLaws_P.Constant3_Value_dw);
  rtb_Mod1_b = rt_modd(AutopilotLaws_P.Constant3_Value_dw - rtb_Mod1_k, AutopilotLaws_P.Constant3_Value_dw);
  if (AutopilotLaws_U.in.data.nav_dme_nmi > AutopilotLaws_P.Saturation_UpperSat_m) {
    rtb_out_f = AutopilotLaws_P.Saturation_UpperSat_m;
  } else if (AutopilotLaws_U.in.data.nav_dme_nmi < AutopilotLaws_P.Saturation_LowerSat_k) {
    rtb_out_f = AutopilotLaws_P.Saturation_LowerSat_k;
  } else {
    rtb_out_f = AutopilotLaws_U.in.data.nav_dme_nmi;
  }

  rtb_out_f = std::sin(AutopilotLaws_P.Gain1_Gain_h * AutopilotLaws_U.in.data.nav_loc_error_deg) * rtb_out_f *
    look1_binlxpw(AutopilotLaws_U.in.data.nav_dme_nmi, AutopilotLaws_P.ScheduledGain_BreakpointsForDimension1_a,
                  AutopilotLaws_P.ScheduledGain_Table_p, 4U);
  if (rtb_out_f > AutopilotLaws_P.Saturation1_UpperSat_i) {
    rtb_out_f = AutopilotLaws_P.Saturation1_UpperSat_i;
  } else {
    if (rtb_out_f < AutopilotLaws_P.Saturation1_LowerSat_g) {
      rtb_out_f = AutopilotLaws_P.Saturation1_LowerSat_g;
    }
  }

  if (rtb_Mod1_k < rtb_Mod1_b) {
    rtb_Mod1_k *= AutopilotLaws_P.Gain1_Gain;
  } else {
    rtb_Mod1_k = AutopilotLaws_P.Gain_Gain * rtb_Mod1_b;
  }

  rtb_Mod1_k = rt_modd((rt_modd(rt_modd((rtb_out_f + rtb_Mod1_k) * AutopilotLaws_P.Gain3_Gain +
    AutopilotLaws_U.in.data.Psi_magnetic_track_deg, AutopilotLaws_P.Constant3_Value_h) +
    AutopilotLaws_P.Constant3_Value_h, AutopilotLaws_P.Constant3_Value_h) -
                        (AutopilotLaws_U.in.data.Psi_magnetic_track_deg + AutopilotLaws_P.Constant3_Value_p)) +
                       AutopilotLaws_P.Constant3_Value_p, AutopilotLaws_P.Constant3_Value_p);
  AutopilotLaws_Chart(rtb_Mod1_k, AutopilotLaws_P.Gain_Gain_fk * rt_modd(AutopilotLaws_P.Constant3_Value_p - rtb_Mod1_k,
    AutopilotLaws_P.Constant3_Value_p), AutopilotLaws_P.Constant_Value_e, &rtb_out_f, &AutopilotLaws_DWork.sf_Chart_g);
  rtb_Mod1_k = AutopilotLaws_P.DiscreteDerivativeVariableTs_Gain * AutopilotLaws_U.in.data.nav_loc_error_deg;
  rtb_Mod1_b = (rtb_Mod1_k - AutopilotLaws_DWork.Delay_DSTATE_hc) / AutopilotLaws_U.in.time.dt *
    AutopilotLaws_P.Gain3_Gain_i + AutopilotLaws_U.in.data.nav_loc_error_deg;
  rtb_Divide_p = AutopilotLaws_U.in.time.dt * AutopilotLaws_P.LagFilter_C1;
  rtb_Minup = rtb_Divide_p + AutopilotLaws_P.Constant_Value_o;
  AutopilotLaws_DWork.Delay1_DSTATE_a = 1.0 / rtb_Minup * (AutopilotLaws_P.Constant_Value_o - rtb_Divide_p) *
    AutopilotLaws_DWork.Delay1_DSTATE_a + (rtb_Mod1_b + AutopilotLaws_DWork.Delay_DSTATE_p) * (rtb_Divide_p / rtb_Minup);
  rtb_Divide_p = rt_modd((rt_modd(rt_modd(AutopilotLaws_DWork.Delay1_DSTATE_a * look1_binlxpw
    (AutopilotLaws_U.in.data.H_radio_ft, AutopilotLaws_P.ScheduledGain_BreakpointsForDimension1_e,
     AutopilotLaws_P.ScheduledGain_Table_pf, 4U) + AutopilotLaws_U.in.data.Psi_magnetic_track_deg,
    AutopilotLaws_P.Constant3_Value_g) + AutopilotLaws_P.Constant3_Value_g, AutopilotLaws_P.Constant3_Value_g) -
    (AutopilotLaws_U.in.data.Psi_magnetic_track_deg + AutopilotLaws_P.Constant3_Value_fa)) +
    AutopilotLaws_P.Constant3_Value_fa, AutopilotLaws_P.Constant3_Value_fa);
  AutopilotLaws_Chart(rtb_Divide_p, AutopilotLaws_P.Gain_Gain_e * rt_modd(AutopilotLaws_P.Constant3_Value_fa -
    rtb_Divide_p, AutopilotLaws_P.Constant3_Value_fa), AutopilotLaws_P.Constant_Value_p, &rtb_Minup,
                      &AutopilotLaws_DWork.sf_Chart_no);
  switch (static_cast<int32_T>(rtb_ManualSwitch)) {
   case 0:
    rtb_Divide_p = rtb_GainTheta1;
    break;

   case 1:
    rtb_Divide_p = rtb_Divide_e * look1_binlxpw(AutopilotLaws_U.in.data.V_tas_kn,
      AutopilotLaws_P.ScheduledGain_BreakpointsForDimension1_c, AutopilotLaws_P.ScheduledGain_Table_c, 5U);
    break;

   case 2:
    rtb_Divide_p = rtb_out * look1_binlxpw(AutopilotLaws_U.in.data.V_tas_kn,
      AutopilotLaws_P.ScheduledGain_BreakpointsForDimension1_g, AutopilotLaws_P.ScheduledGain_Table_m, 5U);
    break;

   case 3:
    rtb_Divide_p = rtb_out_co * look1_binlxpw(AutopilotLaws_U.in.data.V_tas_kn,
      AutopilotLaws_P.ScheduledGain_BreakpointsForDimension1_f, AutopilotLaws_P.ScheduledGain_Table_f, 5U) +
      AutopilotLaws_U.in.data.flight_guidance_phi_deg;
    break;

   case 4:
    rtb_Divide_p = rtb_out_f * look1_binlxpw(AutopilotLaws_U.in.data.V_tas_kn,
      AutopilotLaws_P.ScheduledGain_BreakpointsForDimension1_d, AutopilotLaws_P.ScheduledGain_Table_o, 5U);
    break;

   case 5:
    rtb_Divide_p = rtb_Minup * look1_binlxpw(AutopilotLaws_U.in.data.V_tas_kn,
      AutopilotLaws_P.ScheduledGain_BreakpointsForDimension1_l, AutopilotLaws_P.ScheduledGain_Table_ow, 5U);
    break;

   default:
    rtb_Divide_p = AutopilotLaws_P.Constant3_Value;
    break;
  }

  if (rtb_Divide_p > rtb_Mod2) {
    rtb_Divide_p = rtb_Mod2;
  } else {
    rtb_ManualSwitch = AutopilotLaws_P.Gain1_Gain_l * rtb_Mod2;
    if (rtb_Divide_p < rtb_ManualSwitch) {
      rtb_Divide_p = rtb_ManualSwitch;
    }
  }

  if ((AutopilotLaws_U.in.input.enabled_AP1 == 0.0) && (AutopilotLaws_U.in.input.enabled_AP2 == 0.0)) {
    AutopilotLaws_DWork.icLoad = 1U;
  }

  if (AutopilotLaws_DWork.icLoad != 0) {
    AutopilotLaws_DWork.Delay_DSTATE_hcy = rtb_GainTheta1;
  }

  rtb_out_f = rtb_Divide_p - AutopilotLaws_DWork.Delay_DSTATE_hcy;
  rtb_Y = AutopilotLaws_P.Constant2_Value_h * AutopilotLaws_U.in.time.dt;
  if (rtb_out_f < rtb_Y) {
    rtb_Y = rtb_out_f;
  }

  rtb_out_f = AutopilotLaws_P.Gain1_Gain_k * AutopilotLaws_P.Constant2_Value_h * AutopilotLaws_U.in.time.dt;
  if (rtb_Y > rtb_out_f) {
    rtb_out_f = rtb_Y;
  }

  AutopilotLaws_DWork.Delay_DSTATE_hcy += rtb_out_f;
  rtb_Y = AutopilotLaws_U.in.time.dt * AutopilotLaws_P.LagFilter_C1_l;
  rtb_Minup = rtb_Y + AutopilotLaws_P.Constant_Value_cq;
  AutopilotLaws_DWork.Delay1_DSTATE_l = 1.0 / rtb_Minup * (AutopilotLaws_P.Constant_Value_cq - rtb_Y) *
    AutopilotLaws_DWork.Delay1_DSTATE_l + (AutopilotLaws_DWork.Delay_DSTATE_hcy + AutopilotLaws_DWork.Delay_DSTATE_j) *
    (rtb_Y / rtb_Minup);
  rtb_out_f = static_cast<real_T>(rtb_BusAssignment_output_ap_on) - AutopilotLaws_DWork.Delay_DSTATE_a;
  rtb_Y = AutopilotLaws_P.RateLimiterVariableTs_up * AutopilotLaws_U.in.time.dt;
  if (rtb_out_f < rtb_Y) {
    rtb_Y = rtb_out_f;
  }

  rtb_out_f = AutopilotLaws_U.in.time.dt * AutopilotLaws_P.RateLimiterVariableTs_lo;
  if (rtb_Y > rtb_out_f) {
    rtb_out_f = rtb_Y;
  }

  AutopilotLaws_DWork.Delay_DSTATE_a += rtb_out_f;
  if (AutopilotLaws_DWork.Delay_DSTATE_a > AutopilotLaws_P.Saturation_UpperSat_a) {
    rtb_Sum2_l = AutopilotLaws_P.Saturation_UpperSat_a;
  } else if (AutopilotLaws_DWork.Delay_DSTATE_a < AutopilotLaws_P.Saturation_LowerSat_ov) {
    rtb_Sum2_l = AutopilotLaws_P.Saturation_LowerSat_ov;
  } else {
    rtb_Sum2_l = AutopilotLaws_DWork.Delay_DSTATE_a;
  }

  rtb_ManualSwitch = AutopilotLaws_DWork.Delay1_DSTATE_l * rtb_Sum2_l;
  rtb_Sum2_l = AutopilotLaws_P.Constant_Value_a - rtb_Sum2_l;
  rtb_Sum2_l *= rtb_GainTheta1;
  AutopilotLaws_Y.out.output.autopilot.Phi_c_deg = rtb_ManualSwitch + rtb_Sum2_l;
  AutopilotLaws_Y.out.output.flight_director.Beta_c_deg = rtb_out_gk;
  AutopilotLaws_Y.out.output.autopilot.Beta_c_deg = rtb_out_gk;
  AutopilotLaws_Y.out.output.flight_director.Phi_c_deg = rtb_Divide_p;
  if (AutopilotLaws_P.Constant1_Value_h > AutopilotLaws_P.Switch_Threshold_l) {
    rtb_Sum2_l = (AutopilotLaws_U.in.input.V_c_kn - AutopilotLaws_U.in.data.V_ias_kn) * AutopilotLaws_P.Gain1_Gain_b;
    if (rtb_Sum2_l > AutopilotLaws_P.Saturation1_UpperSat) {
      rtb_Sum2_l = AutopilotLaws_P.Saturation1_UpperSat;
    } else {
      if (rtb_Sum2_l < AutopilotLaws_P.Saturation1_LowerSat) {
        rtb_Sum2_l = AutopilotLaws_P.Saturation1_LowerSat;
      }
    }
  } else {
    rtb_Sum2_l = AutopilotLaws_P.Constant1_Value_h;
  }

  if (AutopilotLaws_P.ManualSwitch_CurrentSetting_b == 1) {
    rtb_Minup = AutopilotLaws_P.Constant_Value_m;
  } else {
    rtb_Minup = AutopilotLaws_U.in.input.vertical_law;
  }

  if (rtb_Minup != AutopilotLaws_P.CompareToConstant5_const_e) {
    AutopilotLaws_B.u = (AutopilotLaws_U.in.input.H_c_ft + AutopilotLaws_U.in.data.H_ft) -
      AutopilotLaws_U.in.data.H_ind_ft;
  }

  rtb_ManualSwitch = AutopilotLaws_B.u - AutopilotLaws_U.in.data.H_ft;
  rtb_Y = AutopilotLaws_U.in.time.dt * AutopilotLaws_P.LagFilter_C1_a;
  rtb_Divide_p = rtb_Y + AutopilotLaws_P.Constant_Value_h;
  AutopilotLaws_DWork.Delay1_DSTATE_g = 1.0 / rtb_Divide_p * (AutopilotLaws_P.Constant_Value_h - rtb_Y) *
    AutopilotLaws_DWork.Delay1_DSTATE_g + (rtb_ManualSwitch + AutopilotLaws_DWork.Delay_DSTATE_n) * (rtb_Y /
    rtb_Divide_p);
  rtb_Sum2_l += AutopilotLaws_P.Gain_Gain_ft * AutopilotLaws_DWork.Delay1_DSTATE_g;
  if (rtb_Sum2_l > AutopilotLaws_P.Saturation_UpperSat_n) {
    rtb_Sum2_l = AutopilotLaws_P.Saturation_UpperSat_n;
  } else {
    if (rtb_Sum2_l < AutopilotLaws_P.Saturation_LowerSat_d) {
      rtb_Sum2_l = AutopilotLaws_P.Saturation_LowerSat_d;
    }
  }

  rtb_Sum2_l -= AutopilotLaws_U.in.data.H_dot_ft_min;
  rtb_out_f = AutopilotLaws_P.kntoms_Gain * AutopilotLaws_U.in.data.V_tas_kn;
  if (rtb_out_f > AutopilotLaws_P.Saturation_UpperSat_h) {
    rtb_out_f = AutopilotLaws_P.Saturation_UpperSat_h;
  } else {
    if (rtb_out_f < AutopilotLaws_P.Saturation_LowerSat_g) {
      rtb_out_f = AutopilotLaws_P.Saturation_LowerSat_g;
    }
  }

  rtb_out_f = AutopilotLaws_P.ftmintoms_Gain * rtb_Sum2_l / rtb_out_f;
  if (rtb_out_f > 1.0) {
    rtb_out_f = 1.0;
  } else {
    if (rtb_out_f < -1.0) {
      rtb_out_f = -1.0;
    }
  }

  rtb_Gain_ot = AutopilotLaws_P.Gain_Gain_fv * std::asin(rtb_out_f);
  rtb_Y = AutopilotLaws_U.in.time.dt * AutopilotLaws_P.LagFilter1_C1_a;
  rtb_Divide_p = rtb_Y + AutopilotLaws_P.Constant_Value_n;
  AutopilotLaws_DWork.Delay1_DSTATE_i = 1.0 / rtb_Divide_p * (AutopilotLaws_P.Constant_Value_n - rtb_Y) *
    AutopilotLaws_DWork.Delay1_DSTATE_i + (AutopilotLaws_U.in.data.nav_gs_error_deg + AutopilotLaws_DWork.Delay_DSTATE_l)
    * (rtb_Y / rtb_Divide_p);
  rtb_Mod2 = AutopilotLaws_P.DiscreteDerivativeVariableTs_Gain_l * AutopilotLaws_DWork.Delay1_DSTATE_i;
  rtb_Divide_e = (rtb_Mod2 - AutopilotLaws_DWork.Delay_DSTATE_o) / AutopilotLaws_U.in.time.dt *
    AutopilotLaws_P.Gain3_Gain_o + AutopilotLaws_DWork.Delay1_DSTATE_i;
  rtb_Sum2_l = AutopilotLaws_U.in.time.dt * AutopilotLaws_P.LagFilter_C1_n;
  rtb_Y = rtb_Sum2_l + AutopilotLaws_P.Constant_Value_g;
  AutopilotLaws_DWork.Delay1_DSTATE_d = 1.0 / rtb_Y * (AutopilotLaws_P.Constant_Value_g - rtb_Sum2_l) *
    AutopilotLaws_DWork.Delay1_DSTATE_d + (rtb_Divide_e + AutopilotLaws_DWork.Delay_DSTATE_m) * (rtb_Sum2_l / rtb_Y);
  rtb_out_gk = rtb_GainTheta - std::cos(AutopilotLaws_P.Gain1_Gain_p * rtb_GainTheta1) *
    AutopilotLaws_U.in.data.alpha_deg;
  if ((AutopilotLaws_U.in.data.H_radio_ft > AutopilotLaws_P.CompareToConstant_const_n) &&
      AutopilotLaws_U.in.data.nav_gs_valid) {
    rtb_out_f = AutopilotLaws_DWork.Delay1_DSTATE_d * look1_binlxpw(AutopilotLaws_U.in.data.H_radio_ft,
      AutopilotLaws_P.ScheduledGain_BreakpointsForDimension1_h, AutopilotLaws_P.ScheduledGain_Table_i, 4U);
  } else {
    rtb_out_f = (AutopilotLaws_P.Constant3_Value_d - rtb_out_gk) * AutopilotLaws_P.Gain2_Gain;
  }

  AutopilotLaws_Voter1(rtb_out_f, AutopilotLaws_P.Gain1_Gain_nq * (AutopilotLaws_P.Constant1_Value_n - rtb_out_gk),
                       AutopilotLaws_P.Gain_Gain_p * (AutopilotLaws_P.Constant2_Value_n - rtb_out_gk), &rtb_Y_j);
  rtb_Sum_o5_tmp = AutopilotLaws_U.in.input.H_c_ft - AutopilotLaws_U.in.data.H_ind_ft;
  rtb_out_f = AutopilotLaws_P.kntoms_Gain_e * AutopilotLaws_U.in.data.V_tas_kn;
  if (rtb_Sum_o5_tmp < 0.0) {
    rtb_out_gk = -1.0;
  } else if (rtb_Sum_o5_tmp > 0.0) {
    rtb_out_gk = 1.0;
  } else {
    rtb_out_gk = rtb_Sum_o5_tmp;
  }

  if (rtb_out_f > AutopilotLaws_P.Saturation_UpperSat_nj) {
    rtb_out_f = AutopilotLaws_P.Saturation_UpperSat_nj;
  } else {
    if (rtb_out_f < AutopilotLaws_P.Saturation_LowerSat_c) {
      rtb_out_f = AutopilotLaws_P.Saturation_LowerSat_c;
    }
  }

  rtb_out_f = ((AutopilotLaws_P.Constant_Value_b * rtb_out_gk + rtb_Sum_o5_tmp) * AutopilotLaws_P.Gain_Gain_el -
               AutopilotLaws_U.in.data.H_dot_ft_min) * AutopilotLaws_P.ftmintoms_Gain_c / rtb_out_f;
  if (rtb_out_f > 1.0) {
    rtb_out_f = 1.0;
  } else {
    if (rtb_out_f < -1.0) {
      rtb_out_f = -1.0;
    }
  }

  rtb_Gain_ju3 = AutopilotLaws_P.Gain_Gain_ph * std::asin(rtb_out_f);
  rtb_Sum2_l = AutopilotLaws_P.Gain1_Gain_d * AutopilotLaws_U.in.data.alpha_deg;
  rtb_Y = AutopilotLaws_U.in.data.bz_m_s2 * std::sin(rtb_Sum2_l);
  rtb_Sum2_l = std::cos(rtb_Sum2_l);
  rtb_Sum2_l *= AutopilotLaws_U.in.data.bx_m_s2;
  rtb_Sum2_ho = AutopilotLaws_U.in.data.V_ias_kn - AutopilotLaws_U.in.input.V_c_kn;
  rtb_out_f = rtb_Sum2_ho * AutopilotLaws_P.Gain1_Gain_hn;
  if (rtb_out_f > AutopilotLaws_P.Saturation_UpperSat_j) {
    rtb_out_f = AutopilotLaws_P.Saturation_UpperSat_j;
  } else {
    if (rtb_out_f < AutopilotLaws_P.Saturation_LowerSat_b) {
      rtb_out_f = AutopilotLaws_P.Saturation_LowerSat_b;
    }
  }

  rtb_Sum2_d = (rtb_Y + rtb_Sum2_l) * AutopilotLaws_P.Gain_Gain_h * AutopilotLaws_P.Gain_Gain_g + rtb_out_f;
  rtb_out_f = AutopilotLaws_P.kntoms_Gain_i * AutopilotLaws_U.in.data.V_tas_kn;
  if (rtb_out_f > AutopilotLaws_P.Saturation_UpperSat_hs) {
    rtb_out_f = AutopilotLaws_P.Saturation_UpperSat_hs;
  } else {
    if (rtb_out_f < AutopilotLaws_P.Saturation_LowerSat_cj) {
      rtb_out_f = AutopilotLaws_P.Saturation_LowerSat_cj;
    }
  }

  rtb_out_f = (AutopilotLaws_P.Constant_Value_k - AutopilotLaws_U.in.data.H_dot_ft_min) *
    AutopilotLaws_P.ftmintoms_Gain_a / rtb_out_f;
  if (rtb_out_f > 1.0) {
    rtb_out_f = 1.0;
  } else {
    if (rtb_out_f < -1.0) {
      rtb_out_f = -1.0;
    }
  }

  rtb_Gain_k = AutopilotLaws_P.Gain_Gain_nq * std::asin(rtb_out_f);
  rtb_out_f = AutopilotLaws_P.kntoms_Gain_a * AutopilotLaws_U.in.data.V_tas_kn;
  if (rtb_out_f > AutopilotLaws_P.Saturation_UpperSat_g) {
    rtb_out_f = AutopilotLaws_P.Saturation_UpperSat_g;
  } else {
    if (rtb_out_f < AutopilotLaws_P.Saturation_LowerSat_bb) {
      rtb_out_f = AutopilotLaws_P.Saturation_LowerSat_bb;
    }
  }

  rtb_out_f = (AutopilotLaws_U.in.input.H_dot_c_fpm - AutopilotLaws_U.in.data.H_dot_ft_min) *
    AutopilotLaws_P.ftmintoms_Gain_aa / rtb_out_f;
  if (rtb_out_f > 1.0) {
    rtb_out_f = 1.0;
  } else {
    if (rtb_out_f < -1.0) {
      rtb_out_f = -1.0;
    }
  }

  rtb_Gain_or = AutopilotLaws_P.Gain_Gain_no * std::asin(rtb_out_f);
  rtb_Sum_l1 = AutopilotLaws_U.in.input.FPA_c_deg - (rtb_GainTheta - std::cos(AutopilotLaws_P.Gain1_Gain_de *
    rtb_GainTheta1) * AutopilotLaws_U.in.data.alpha_deg);
  rtb_Gain4 = (rtb_GainTheta - AutopilotLaws_P.Constant2_Value_f) * AutopilotLaws_P.Gain4_Gain;
  rtb_Y = AutopilotLaws_U.in.time.dt * AutopilotLaws_P.WashoutFilter_C1;
  rtb_Divide_p = rtb_Y + AutopilotLaws_P.Constant_Value_j;
  rtb_out_gk = AutopilotLaws_P.Constant_Value_j / rtb_Divide_p;
  rtb_out_f = AutopilotLaws_DWork.Delay_DSTATE_f * rtb_out_gk;
  rtb_out_gk *= AutopilotLaws_U.in.data.H_ind_ft;
  AutopilotLaws_DWork.Delay1_DSTATE_gz = 1.0 / rtb_Divide_p * (AutopilotLaws_P.Constant_Value_j - rtb_Y) *
    AutopilotLaws_DWork.Delay1_DSTATE_gz + (rtb_out_gk - rtb_out_f);
  rtb_Y = AutopilotLaws_U.in.time.dt * AutopilotLaws_P.LagFilter_C1_i;
  rtb_Divide_p = rtb_Y + AutopilotLaws_P.Constant_Value_ha;
  AutopilotLaws_DWork.Delay1_DSTATE_gs = 1.0 / rtb_Divide_p * (AutopilotLaws_P.Constant_Value_ha - rtb_Y) *
    AutopilotLaws_DWork.Delay1_DSTATE_gs + (AutopilotLaws_U.in.data.H_radio_ft + AutopilotLaws_DWork.Delay_DSTATE_g) *
    (rtb_Y / rtb_Divide_p);
  rtb_out = (AutopilotLaws_DWork.Delay1_DSTATE_gz + AutopilotLaws_DWork.Delay1_DSTATE_gs) *
    AutopilotLaws_P.DiscreteDerivativeVariableTs2_Gain;
  rtb_out_co = (rtb_out - AutopilotLaws_DWork.Delay_DSTATE_hd) / AutopilotLaws_U.in.time.dt *
    AutopilotLaws_P.Gain2_Gain_j;
  rtb_Sum2_l = AutopilotLaws_U.in.time.dt * AutopilotLaws_P.LagFilter3_C1;
  rtb_Y = rtb_Sum2_l + AutopilotLaws_P.Constant_Value_ca;
  AutopilotLaws_DWork.Delay1_DSTATE_b = 1.0 / rtb_Y * (AutopilotLaws_P.Constant_Value_ca - rtb_Sum2_l) *
    AutopilotLaws_DWork.Delay1_DSTATE_b + (rtb_out_co + AutopilotLaws_DWork.Delay_DSTATE_i) * (rtb_Sum2_l / rtb_Y);
  rtb_Y = AutopilotLaws_U.in.time.dt * AutopilotLaws_P.WashoutFilter1_C1;
  rtb_Divide_p = rtb_Y + AutopilotLaws_P.Constant_Value_f;
  rtb_out_gk = AutopilotLaws_P.Constant_Value_f / rtb_Divide_p;
  rtb_out_f = AutopilotLaws_DWork.Delay_DSTATE_b * rtb_out_gk;
  rtb_out_gk *= AutopilotLaws_U.in.data.H_dot_ft_min;
  AutopilotLaws_DWork.Delay1_DSTATE_ik = 1.0 / rtb_Divide_p * (AutopilotLaws_P.Constant_Value_f - rtb_Y) *
    AutopilotLaws_DWork.Delay1_DSTATE_ik + (rtb_out_gk - rtb_out_f);
  rtb_Y = AutopilotLaws_P.Gain_Gain_k0 * AutopilotLaws_U.in.data.H_radio_ft;
  rtb_out_f = AutopilotLaws_P.kntoms_Gain_av * AutopilotLaws_U.in.data.V_tas_kn;
  if (AutopilotLaws_P.Constant1_Value_d < rtb_Y) {
    rtb_Y = AutopilotLaws_P.Constant1_Value_d;
  }

  if (rtb_out_f > AutopilotLaws_P.Saturation_UpperSat_b) {
    rtb_out_f = AutopilotLaws_P.Saturation_UpperSat_b;
  } else {
    if (rtb_out_f < AutopilotLaws_P.Saturation_LowerSat_h) {
      rtb_out_f = AutopilotLaws_P.Saturation_LowerSat_h;
    }
  }

  rtb_out_f = (rtb_Y - (AutopilotLaws_DWork.Delay1_DSTATE_b + AutopilotLaws_DWork.Delay1_DSTATE_ik)) *
    AutopilotLaws_P.ftmintoms_Gain_n / rtb_out_f;
  if (rtb_out_f > 1.0) {
    rtb_out_f = 1.0;
  } else {
    if (rtb_out_f < -1.0) {
      rtb_out_f = -1.0;
    }
  }

  rtb_out_gk = AutopilotLaws_P.Gain_Gain_eu * std::asin(rtb_out_f);
  rtb_Divide_p = AutopilotLaws_P.Constant1_Value_dg - rtb_GainTheta;
  rtb_Sum2_l = AutopilotLaws_P.Gain1_Gain_nl * AutopilotLaws_U.in.data.alpha_deg;
  rtb_Y = AutopilotLaws_U.in.data.bz_m_s2 * std::sin(rtb_Sum2_l);
  rtb_Sum2_l = std::cos(rtb_Sum2_l);
  rtb_Sum2_l *= AutopilotLaws_U.in.data.bx_m_s2;
  rtb_out_f = rtb_Sum2_ho * AutopilotLaws_P.Gain1_Gain_fr;
  if (rtb_out_f > AutopilotLaws_P.Saturation_UpperSat_e) {
    rtb_out_f = AutopilotLaws_P.Saturation_UpperSat_e;
  } else {
    if (rtb_out_f < AutopilotLaws_P.Saturation_LowerSat_py) {
      rtb_out_f = AutopilotLaws_P.Saturation_LowerSat_py;
    }
  }

  rtb_Sum2_ho = (rtb_Y + rtb_Sum2_l) * AutopilotLaws_P.Gain_Gain_dc * AutopilotLaws_P.Gain_Gain_ov + rtb_out_f;
  rtb_out_f = AutopilotLaws_P.kntoms_Gain_f * AutopilotLaws_U.in.data.V_tas_kn;
  if (rtb_out_f > AutopilotLaws_P.Saturation_UpperSat_e4) {
    rtb_out_f = AutopilotLaws_P.Saturation_UpperSat_e4;
  } else {
    if (rtb_out_f < AutopilotLaws_P.Saturation_LowerSat_f) {
      rtb_out_f = AutopilotLaws_P.Saturation_LowerSat_f;
    }
  }

  rtb_out_f = (AutopilotLaws_P.Constant_Value_i - AutopilotLaws_U.in.data.H_dot_ft_min) *
    AutopilotLaws_P.ftmintoms_Gain_b / rtb_out_f;
  if (rtb_out_f > 1.0) {
    rtb_out_f = 1.0;
  } else {
    if (rtb_out_f < -1.0) {
      rtb_out_f = -1.0;
    }
  }

  rtb_Gain_o = AutopilotLaws_P.Gain_Gain_fl * std::asin(rtb_out_f);
  AutopilotLaws_Voter1(rtb_Divide_p, AutopilotLaws_P.Gain_Gain_b2 * rtb_Sum2_ho, AutopilotLaws_P.VS_Gain_c * rtb_Gain_o,
                       &rtb_Sum2_l);
  switch (static_cast<int32_T>(rtb_Minup)) {
   case 0:
    rtb_Sum2_l = rtb_GainTheta;
    break;

   case 1:
    rtb_Sum2_l = AutopilotLaws_P.VS_Gain * rtb_Gain_ot;
    break;

   case 2:
    rtb_Sum2_l = AutopilotLaws_P.VS_Gain_a * rtb_Gain_ju3;
    break;

   case 3:
    if (rtb_Sum_o5_tmp > AutopilotLaws_P.Switch_Threshold_n) {
      rtb_out_f = AutopilotLaws_P.Gain_Gain_k * rtb_Sum2_d;
      rtb_Sum2_l = AutopilotLaws_P.VS_Gain_j * rtb_Gain_k;
      if (rtb_out_f > rtb_Sum2_l) {
        rtb_Sum2_l = rtb_out_f;
      }
    } else {
      rtb_out_f = AutopilotLaws_P.Gain_Gain_k * rtb_Sum2_d;
      rtb_Sum2_l = AutopilotLaws_P.VS_Gain_j * rtb_Gain_k;
      if (rtb_out_f < rtb_Sum2_l) {
        rtb_Sum2_l = rtb_out_f;
      }
    }
    break;

   case 4:
    rtb_Sum2_l = AutopilotLaws_P.VS_Gain_h * rtb_Gain_or;
    break;

   case 5:
    rtb_Sum2_l = AutopilotLaws_P.Gain_Gain_c3 * rtb_Sum_l1;
    break;

   case 6:
    rtb_Sum2_l = rtb_Y_j;
    break;

   case 7:
    if (rtb_on_ground > AutopilotLaws_P.Switch_Threshold) {
      rtb_Sum2_l = rtb_Gain4;
    } else {
      rtb_Sum2_l = AutopilotLaws_P.VS_Gain_e * rtb_out_gk;
    }
    break;
  }

  rtb_Sum2_l += rtb_GainTheta;
  if (rtb_Sum2_l > AutopilotLaws_P.Constant1_Value_i) {
    rtb_Sum2_l = AutopilotLaws_P.Constant1_Value_i;
  } else {
    rtb_out_f = AutopilotLaws_P.Gain1_Gain_m * AutopilotLaws_P.Constant1_Value_i;
    if (rtb_Sum2_l < rtb_out_f) {
      rtb_Sum2_l = rtb_out_f;
    }
  }

  if (rtb_BusAssignment_output_ap_on == 0) {
    AutopilotLaws_DWork.icLoad_f = 1U;
  }

  if (AutopilotLaws_DWork.icLoad_f != 0) {
    AutopilotLaws_DWork.Delay_DSTATE_h2 = rtb_GainTheta;
  }

  rtb_Sum2_l -= AutopilotLaws_DWork.Delay_DSTATE_h2;
  rtb_Y = AutopilotLaws_P.Constant2_Value_h1 * AutopilotLaws_U.in.time.dt;
  if (rtb_Sum2_l >= rtb_Y) {
    rtb_Sum2_l = rtb_Y;
  }

  rtb_Y = AutopilotLaws_P.Gain1_Gain_i * AutopilotLaws_P.Constant2_Value_h1 * AutopilotLaws_U.in.time.dt;
  if (rtb_Sum2_l > rtb_Y) {
    rtb_Y = rtb_Sum2_l;
  }

  AutopilotLaws_DWork.Delay_DSTATE_h2 += rtb_Y;
  rtb_Sum2_l = AutopilotLaws_U.in.time.dt * AutopilotLaws_P.LagFilter_C1_id;
  rtb_Y = rtb_Sum2_l + AutopilotLaws_P.Constant_Value_d;
  AutopilotLaws_DWork.Delay1_DSTATE_c = 1.0 / rtb_Y * (AutopilotLaws_P.Constant_Value_d - rtb_Sum2_l) *
    AutopilotLaws_DWork.Delay1_DSTATE_c + (AutopilotLaws_DWork.Delay_DSTATE_h2 + AutopilotLaws_DWork.Delay_DSTATE_me) *
    (rtb_Sum2_l / rtb_Y);
  rtb_out_f = static_cast<real_T>(rtb_BusAssignment_output_ap_on) - AutopilotLaws_DWork.Delay_DSTATE_ah;
  rtb_Y = AutopilotLaws_P.RateLimiterVariableTs_up_i * AutopilotLaws_U.in.time.dt;
  if (rtb_out_f < rtb_Y) {
    rtb_Y = rtb_out_f;
  }

  rtb_out_f = AutopilotLaws_U.in.time.dt * AutopilotLaws_P.RateLimiterVariableTs_lo_o;
  if (rtb_Y > rtb_out_f) {
    rtb_out_f = rtb_Y;
  }

  AutopilotLaws_DWork.Delay_DSTATE_ah += rtb_out_f;
  if (AutopilotLaws_DWork.Delay_DSTATE_ah > AutopilotLaws_P.Saturation_UpperSat_ju) {
    rtb_Sum2_l = AutopilotLaws_P.Saturation_UpperSat_ju;
  } else if (AutopilotLaws_DWork.Delay_DSTATE_ah < AutopilotLaws_P.Saturation_LowerSat_dl) {
    rtb_Sum2_l = AutopilotLaws_P.Saturation_LowerSat_dl;
  } else {
    rtb_Sum2_l = AutopilotLaws_DWork.Delay_DSTATE_ah;
  }

  rtb_out_f = AutopilotLaws_DWork.Delay1_DSTATE_c * rtb_Sum2_l;
  rtb_Sum2_l = AutopilotLaws_P.Constant_Value_an - rtb_Sum2_l;
  rtb_Sum2_l *= rtb_GainTheta;
  AutopilotLaws_Y.out.output.autopilot.Theta_c_deg = rtb_out_f + rtb_Sum2_l;
  AutopilotLaws_Voter1(rtb_Divide_p, rtb_Sum2_ho, rtb_Gain_o, &rtb_Y);
  switch (static_cast<int32_T>(rtb_Minup)) {
   case 0:
    rtb_Y = rtb_GainTheta;
    break;

   case 1:
    rtb_Y = rtb_Gain_ot;
    break;

   case 2:
    rtb_Y = rtb_Gain_ju3;
    break;

   case 3:
    if (rtb_Sum_o5_tmp > AutopilotLaws_P.Switch_Threshold_k) {
      if (rtb_Sum2_d > rtb_Gain_k) {
        rtb_Y = rtb_Sum2_d;
      } else {
        rtb_Y = rtb_Gain_k;
      }
    } else if (rtb_Sum2_d < rtb_Gain_k) {
      rtb_Y = rtb_Sum2_d;
    } else {
      rtb_Y = rtb_Gain_k;
    }
    break;

   case 4:
    rtb_Y = rtb_Gain_or;
    break;

   case 5:
    rtb_Y = rtb_Sum_l1;
    break;

   case 6:
    rtb_Y = rtb_Y_j;
    break;

   case 7:
    if (rtb_on_ground > AutopilotLaws_P.Switch1_Threshold) {
      rtb_Y = AutopilotLaws_P.Gain2_Gain_h * rtb_Gain4;
    } else {
      rtb_Y = rtb_out_gk;
    }
    break;
  }

  rtb_out_gk = rtb_Y + rtb_GainTheta;
  if (rtb_out_gk > AutopilotLaws_P.Constant1_Value_i) {
    AutopilotLaws_Y.out.output.flight_director.Theta_c_deg = AutopilotLaws_P.Constant1_Value_i;
  } else {
    rtb_out_f = AutopilotLaws_P.Gain1_Gain_n * AutopilotLaws_P.Constant1_Value_i;
    if (rtb_out_gk < rtb_out_f) {
      AutopilotLaws_Y.out.output.flight_director.Theta_c_deg = rtb_out_f;
    } else {
      AutopilotLaws_Y.out.output.flight_director.Theta_c_deg = rtb_out_gk;
    }
  }

  AutopilotLaws_Y.out.time = AutopilotLaws_U.in.time;
  AutopilotLaws_Y.out.data.Theta_deg = rtb_GainTheta;
  AutopilotLaws_Y.out.data.Phi_deg = rtb_GainTheta1;
  AutopilotLaws_Y.out.data.qk_deg_s = result[1];
  AutopilotLaws_Y.out.data.rk_deg_s = result[2];
  AutopilotLaws_Y.out.data.pk_deg_s = result[0];
  AutopilotLaws_Y.out.data.V_ias_kn = AutopilotLaws_U.in.data.V_ias_kn;
  AutopilotLaws_Y.out.data.V_tas_kn = AutopilotLaws_U.in.data.V_tas_kn;
  AutopilotLaws_Y.out.data.V_mach = AutopilotLaws_U.in.data.V_mach;
  AutopilotLaws_Y.out.data.V_gnd_kn = AutopilotLaws_U.in.data.V_gnd_kn;
  AutopilotLaws_Y.out.data.alpha_deg = AutopilotLaws_U.in.data.alpha_deg;
  AutopilotLaws_Y.out.data.H_ft = AutopilotLaws_U.in.data.H_ft;
  AutopilotLaws_Y.out.data.H_ind_ft = AutopilotLaws_U.in.data.H_ind_ft;
  AutopilotLaws_Y.out.data.H_radio_ft = AutopilotLaws_U.in.data.H_radio_ft;
  AutopilotLaws_Y.out.data.H_dot_ft_min = AutopilotLaws_U.in.data.H_dot_ft_min;
  AutopilotLaws_Y.out.data.Psi_magnetic_deg = AutopilotLaws_U.in.data.Psi_magnetic_deg;
  AutopilotLaws_Y.out.data.Psi_magnetic_track_deg = AutopilotLaws_U.in.data.Psi_magnetic_track_deg;
  AutopilotLaws_Y.out.data.Psi_true_deg = AutopilotLaws_U.in.data.Psi_true_deg;
  AutopilotLaws_Y.out.data.bx_m_s2 = AutopilotLaws_U.in.data.bx_m_s2;
  AutopilotLaws_Y.out.data.by_m_s2 = AutopilotLaws_U.in.data.by_m_s2;
  AutopilotLaws_Y.out.data.bz_m_s2 = AutopilotLaws_U.in.data.bz_m_s2;
  AutopilotLaws_Y.out.data.nav_valid = AutopilotLaws_U.in.data.nav_valid;
  AutopilotLaws_Y.out.data.nav_loc_deg = AutopilotLaws_U.in.data.nav_loc_deg;
  AutopilotLaws_Y.out.data.nav_dme_valid = AutopilotLaws_U.in.data.nav_dme_valid;
  AutopilotLaws_Y.out.data.nav_dme_nmi = AutopilotLaws_U.in.data.nav_dme_nmi;
  AutopilotLaws_Y.out.data.nav_loc_valid = AutopilotLaws_U.in.data.nav_loc_valid;
  AutopilotLaws_Y.out.data.nav_loc_error_deg = AutopilotLaws_U.in.data.nav_loc_error_deg;
  AutopilotLaws_Y.out.data.nav_gs_valid = AutopilotLaws_U.in.data.nav_gs_valid;
  AutopilotLaws_Y.out.data.nav_gs_error_deg = AutopilotLaws_U.in.data.nav_gs_error_deg;
  AutopilotLaws_Y.out.data.flight_guidance_xtk_nmi = AutopilotLaws_U.in.data.flight_guidance_xtk_nmi;
  AutopilotLaws_Y.out.data.flight_guidance_tae_deg = AutopilotLaws_U.in.data.flight_guidance_tae_deg;
  AutopilotLaws_Y.out.data.flight_guidance_phi_deg = AutopilotLaws_U.in.data.flight_guidance_phi_deg;
  AutopilotLaws_Y.out.data.flight_phase = AutopilotLaws_U.in.data.flight_phase;
  AutopilotLaws_Y.out.data.V2_kn = AutopilotLaws_U.in.data.V2_kn;
  AutopilotLaws_Y.out.data.VAPP_kn = AutopilotLaws_U.in.data.VAPP_kn;
  AutopilotLaws_Y.out.data.VLS_kn = AutopilotLaws_U.in.data.VLS_kn;
  AutopilotLaws_Y.out.data.is_flight_plan_available = AutopilotLaws_U.in.data.is_flight_plan_available;
  AutopilotLaws_Y.out.data.altitude_constraint_ft = AutopilotLaws_U.in.data.altitude_constraint_ft;
  AutopilotLaws_Y.out.data.thrust_reduction_altitude = AutopilotLaws_U.in.data.thrust_reduction_altitude;
  AutopilotLaws_Y.out.data.thrust_reduction_altitude_go_around =
    AutopilotLaws_U.in.data.thrust_reduction_altitude_go_around;
  AutopilotLaws_Y.out.data.acceleration_altitude = AutopilotLaws_U.in.data.acceleration_altitude;
  AutopilotLaws_Y.out.data.acceleration_altitude_engine_out = AutopilotLaws_U.in.data.acceleration_altitude_engine_out;
  AutopilotLaws_Y.out.data.acceleration_altitude_go_around = AutopilotLaws_U.in.data.acceleration_altitude_go_around;
  AutopilotLaws_Y.out.data.cruise_altitude = AutopilotLaws_U.in.data.cruise_altitude;
  AutopilotLaws_Y.out.data.on_ground = rtb_on_ground;
  AutopilotLaws_Y.out.data.zeta_deg = rtb_Saturation;
  AutopilotLaws_Y.out.data.throttle_lever_1_pos = AutopilotLaws_U.in.data.throttle_lever_1_pos;
  AutopilotLaws_Y.out.data.throttle_lever_2_pos = AutopilotLaws_U.in.data.throttle_lever_2_pos;
  AutopilotLaws_Y.out.data.flaps_handle_index = AutopilotLaws_U.in.data.flaps_handle_index;
  AutopilotLaws_Y.out.input = AutopilotLaws_U.in.input;
  AutopilotLaws_Y.out.output.ap_on = rtb_BusAssignment_output_ap_on;
  AutopilotLaws_DWork.Delay_DSTATE = rtb_Saturation1;
  for (rtb_on_ground = 0; rtb_on_ground < 99; rtb_on_ground++) {
    AutopilotLaws_DWork.Delay_DSTATE_lp[rtb_on_ground] = AutopilotLaws_DWork.Delay_DSTATE_lp[rtb_on_ground + 1];
    AutopilotLaws_DWork.Delay_DSTATE_h5[rtb_on_ground] = AutopilotLaws_DWork.Delay_DSTATE_h5[rtb_on_ground + 1];
  }

  AutopilotLaws_DWork.Delay_DSTATE_lp[99] = rtb_Compare_b;
  AutopilotLaws_DWork.Delay_DSTATE_h5[99] = rtb_Compare_h;
  AutopilotLaws_DWork.Delay_DSTATE_hc = rtb_Mod1_k;
  AutopilotLaws_DWork.Delay_DSTATE_p = rtb_Mod1_b;
  AutopilotLaws_DWork.icLoad = 0U;
  AutopilotLaws_DWork.Delay_DSTATE_j = AutopilotLaws_DWork.Delay_DSTATE_hcy;
  AutopilotLaws_DWork.Delay_DSTATE_n = rtb_ManualSwitch;
  AutopilotLaws_DWork.Delay_DSTATE_l = AutopilotLaws_U.in.data.nav_gs_error_deg;
  AutopilotLaws_DWork.Delay_DSTATE_o = rtb_Mod2;
  AutopilotLaws_DWork.Delay_DSTATE_m = rtb_Divide_e;
  AutopilotLaws_DWork.Delay_DSTATE_f = AutopilotLaws_U.in.data.H_ind_ft;
  AutopilotLaws_DWork.Delay_DSTATE_g = AutopilotLaws_U.in.data.H_radio_ft;
  AutopilotLaws_DWork.Delay_DSTATE_hd = rtb_out;
  AutopilotLaws_DWork.Delay_DSTATE_i = rtb_out_co;
  AutopilotLaws_DWork.Delay_DSTATE_b = AutopilotLaws_U.in.data.H_dot_ft_min;
  AutopilotLaws_DWork.icLoad_f = 0U;
  AutopilotLaws_DWork.Delay_DSTATE_me = AutopilotLaws_DWork.Delay_DSTATE_h2;
}

void AutopilotLawsModelClass::initialize()
{
  (void) std::memset((static_cast<void *>(&AutopilotLaws_B)), 0,
                     sizeof(BlockIO_AutopilotLaws_T));
  (void) std::memset(static_cast<void *>(&AutopilotLaws_DWork), 0,
                     sizeof(D_Work_AutopilotLaws_T));
  AutopilotLaws_U.in = AutopilotLaws_rtZap_laws_input;
  AutopilotLaws_Y.out = AutopilotLaws_rtZap_laws_output;

  {
    int32_T i;
    AutopilotLaws_DWork.Delay_DSTATE = AutopilotLaws_P.Delay_InitialCondition;
    AutopilotLaws_DWork.Delay1_DSTATE = AutopilotLaws_P.Delay1_InitialCondition;
    AutopilotLaws_DWork.Delay_DSTATE_h = AutopilotLaws_P.DiscreteTimeIntegratorVariableTs_InitialCondition;
    for (i = 0; i < 100; i++) {
      AutopilotLaws_DWork.Delay_DSTATE_lp[i] = AutopilotLaws_P.Delay_InitialCondition_h;
      AutopilotLaws_DWork.Delay_DSTATE_h5[i] = AutopilotLaws_P.Delay_InitialCondition_bl;
    }

    AutopilotLaws_DWork.Delay_DSTATE_hc = AutopilotLaws_P.DiscreteDerivativeVariableTs_InitialCondition;
    AutopilotLaws_DWork.Delay_DSTATE_p = AutopilotLaws_P.Delay_InitialCondition_b;
    AutopilotLaws_DWork.Delay1_DSTATE_a = AutopilotLaws_P.Delay1_InitialCondition_e;
    AutopilotLaws_DWork.icLoad = 1U;
    AutopilotLaws_DWork.Delay_DSTATE_j = AutopilotLaws_P.Delay_InitialCondition_e;
    AutopilotLaws_DWork.Delay1_DSTATE_l = AutopilotLaws_P.Delay1_InitialCondition_p;
    AutopilotLaws_DWork.Delay_DSTATE_a = AutopilotLaws_P.RateLimiterVariableTs_InitialCondition;
    AutopilotLaws_DWork.Delay_DSTATE_n = AutopilotLaws_P.Delay_InitialCondition_f;
    AutopilotLaws_DWork.Delay1_DSTATE_g = AutopilotLaws_P.Delay1_InitialCondition_d;
    AutopilotLaws_DWork.Delay_DSTATE_l = AutopilotLaws_P.Delay_InitialCondition_g;
    AutopilotLaws_DWork.Delay1_DSTATE_i = AutopilotLaws_P.Delay1_InitialCondition_a;
    AutopilotLaws_DWork.Delay_DSTATE_o = AutopilotLaws_P.DiscreteDerivativeVariableTs_InitialCondition_h;
    AutopilotLaws_DWork.Delay_DSTATE_m = AutopilotLaws_P.Delay_InitialCondition_j;
    AutopilotLaws_DWork.Delay1_DSTATE_d = AutopilotLaws_P.Delay1_InitialCondition_dk;
    AutopilotLaws_DWork.Delay_DSTATE_f = AutopilotLaws_P.Delay_InitialCondition_c;
    AutopilotLaws_DWork.Delay1_DSTATE_gz = AutopilotLaws_P.Delay1_InitialCondition_m;
    AutopilotLaws_DWork.Delay_DSTATE_g = AutopilotLaws_P.Delay_InitialCondition_a;
    AutopilotLaws_DWork.Delay1_DSTATE_gs = AutopilotLaws_P.Delay1_InitialCondition_dd;
    AutopilotLaws_DWork.Delay_DSTATE_hd = AutopilotLaws_P.DiscreteDerivativeVariableTs2_InitialCondition;
    AutopilotLaws_DWork.Delay_DSTATE_i = AutopilotLaws_P.Delay_InitialCondition_i;
    AutopilotLaws_DWork.Delay1_DSTATE_b = AutopilotLaws_P.Delay1_InitialCondition_j;
    AutopilotLaws_DWork.Delay_DSTATE_b = AutopilotLaws_P.Delay_InitialCondition_d;
    AutopilotLaws_DWork.Delay1_DSTATE_ik = AutopilotLaws_P.Delay1_InitialCondition_jn;
    AutopilotLaws_DWork.icLoad_f = 1U;
    AutopilotLaws_DWork.Delay_DSTATE_me = AutopilotLaws_P.Delay_InitialCondition_fs;
    AutopilotLaws_DWork.Delay1_DSTATE_c = AutopilotLaws_P.Delay1_InitialCondition_i;
    AutopilotLaws_DWork.Delay_DSTATE_ah = AutopilotLaws_P.RateLimiterVariableTs_InitialCondition_p;
    AutopilotLaws_DWork.is_active_c5_AutopilotLaws = 0U;
    AutopilotLaws_DWork.is_c5_AutopilotLaws = AutopilotLaws_IN_NO_ACTIVE_CHILD_n;
    AutopilotLaws_Chart_Init(&AutopilotLaws_DWork.sf_Chart_g2);
    AutopilotLaws_Chart_Init(&AutopilotLaws_DWork.sf_Chart_j);
    AutopilotLaws_Chart_k_Init(&AutopilotLaws_DWork.sf_Chart_n);
    AutopilotLaws_Chart_k_Init(&AutopilotLaws_DWork.sf_Chart_f);
    AutopilotLaws_Chart_Init(&AutopilotLaws_DWork.sf_Chart);
    AutopilotLaws_Chart_Init(&AutopilotLaws_DWork.sf_Chart_g);
    AutopilotLaws_Chart_Init(&AutopilotLaws_DWork.sf_Chart_no);
    AutopilotLaws_B.u = AutopilotLaws_P.Y_Y0;
  }
}

void AutopilotLawsModelClass::terminate()
{
}

AutopilotLawsModelClass::AutopilotLawsModelClass()
{
}

AutopilotLawsModelClass::~AutopilotLawsModelClass()
{
}
