#ifndef RTW_HEADER_AutopilotStateMachine_h_
#define RTW_HEADER_AutopilotStateMachine_h_
#include <cmath>
#include "rtwtypes.h"
#include "AutopilotStateMachine_types.h"

#include "multiword_types.h"

class AutopilotStateMachineModelClass {
 public:
  typedef struct {
    ap_sm_output BusAssignment_g;
    ap_vertical_output out;
    ap_lateral_output out_g;
  } BlockIO_AutopilotStateMachine_T;

  typedef struct {
    ap_vertical Delay1_DSTATE;
    ap_lateral Delay_DSTATE;
    real_T Delay_DSTATE_d[100];
    real_T Delay_DSTATE_c[100];
    real_T Delay_DSTATE_l;
    real_T Delay1_DSTATE_b;
    real_T DelayInput1_DSTATE;
    real_T Delay_DSTATE_b;
    real_T Delay1_DSTATE_f;
    real_T Delay_DSTATE_a;
    real_T Delay1_DSTATE_i;
    real_T Delay_DSTATE_o;
    real_T Delay_DSTATE_ov;
    real_T Delay1_DSTATE_h;
    real_T Delay_DSTATE_c5;
    real_T Delay1_DSTATE_p;
    real_T Delay_DSTATE_f;
    real_T Delay_DSTATE_e;
    real_T local_H_fcu_ft;
    real_T local_H_constraint_ft;
    real_T eventTime;
    real_T eventTime_p;
    real_T eventTime_a;
    real_T eventTime_c;
    real_T newFcuAltitudeSelected;
    real_T newFcuAltitudeSelected_b;
    boolean_T DelayInput1_DSTATE_p;
    boolean_T DelayInput1_DSTATE_b;
    boolean_T DelayInput1_DSTATE_d;
    boolean_T DelayInput1_DSTATE_e;
    boolean_T DelayInput1_DSTATE_g;
    boolean_T DelayInput1_DSTATE_f;
    boolean_T DelayInput1_DSTATE_i;
    boolean_T DelayInput1_DSTATE_bd;
    boolean_T DelayInput1_DSTATE_a;
    boolean_T DelayInput1_DSTATE_fn;
    boolean_T DelayInput1_DSTATE_h;
    uint8_T is_active_c6_AutopilotStateMachine;
    uint8_T is_c6_AutopilotStateMachine;
    uint8_T is_ON;
    uint8_T is_GS;
    uint8_T is_active_c5_AutopilotStateMachine;
    uint8_T is_c5_AutopilotStateMachine;
    uint8_T is_active_c1_AutopilotStateMachine;
    uint8_T is_c1_AutopilotStateMachine;
    uint8_T is_ON_c;
    uint8_T is_LOC;
    boolean_T verticalSpeedCancelMode;
    boolean_T sAP1;
    boolean_T sAP2;
    boolean_T sLandModeArmedOrActive;
    boolean_T eventTime_not_empty;
    boolean_T eventTime_not_empty_p;
    boolean_T newFcuAltitudeSelected_c;
    boolean_T state;
    boolean_T eventTime_not_empty_m;
    boolean_T eventTime_not_empty_e;
    boolean_T sThrottleCondition;
    boolean_T state_h;
    boolean_T state_d;
    boolean_T state_j;
    boolean_T sDES;
    boolean_T sCLB;
  } D_Work_AutopilotStateMachine_T;

  typedef struct {
    ap_sm_input in;
  } ExternalInputs_AutopilotStateMachine_T;

  typedef struct {
    ap_sm_output out;
  } ExternalOutputs_AutopilotStateMachine_T;

  struct Parameters_AutopilotStateMachine_T {
    ap_sm_output ap_sm_output_MATLABStruct;
    real_T LagFilter_C1;
    real_T WashoutFilter_C1;
    real_T LagFilter_C1_n;
    real_T LagFilter3_C1;
    real_T WashoutFilter1_C1;
    real_T DiscreteDerivativeVariableTs2_Gain;
    real_T DiscreteDerivativeVariableTs2_InitialCondition;
    real_T RateLimiterDynamicVariableTs_InitialCondition;
    real_T RateLimiterDynamicVariableTs_InitialCondition_g;
    real_T Debounce_Value;
    real_T Debounce_Value_j;
    real_T CompareToConstant_const;
    real_T CompareToConstant_const_d;
    real_T CompareToConstant_const_da;
    real_T DetectDecrease_vinit;
    boolean_T DetectIncrease_vinit;
    boolean_T DetectIncrease1_vinit;
    boolean_T DetectIncrease2_vinit;
    boolean_T DetectIncrease3_vinit;
    boolean_T DetectIncrease4_vinit;
    boolean_T DetectIncrease5_vinit;
    boolean_T DetectIncrease6_vinit;
    boolean_T DetectIncrease7_vinit;
    boolean_T DetectIncrease8_vinit;
    boolean_T DetectIncrease9_vinit;
    boolean_T DetectIncrease10_vinit;
    ap_vertical Delay1_InitialCondition;
    ap_lateral Delay_InitialCondition;
    real_T Constant_Value;
    real_T Constant_Value_a;
    real_T GainTheta_Gain;
    real_T GainTheta1_Gain;
    real_T Gain_Gain;
    real_T Gainqk_Gain;
    real_T Gain_Gain_a;
    real_T Gain_Gain_k;
    real_T Gainpk_Gain;
    real_T Gain_Gain_af;
    real_T Constant1_Value;
    real_T Saturation_UpperSat;
    real_T Saturation_LowerSat;
    real_T Gain1_Gain;
    real_T Saturation1_UpperSat;
    real_T Saturation1_LowerSat;
    real_T Gain2_Gain;
    real_T Constant_Value_j;
    real_T Delay_InitialCondition_i;
    real_T Constant_Value_jq;
    real_T Delay_InitialCondition_m;
    real_T Delay_InitialCondition_c;
    real_T Constant_Value_b;
    real_T Delay1_InitialCondition_l;
    real_T Delay_InitialCondition_n;
    real_T Constant_Value_l;
    real_T Delay1_InitialCondition_d;
    real_T Delay_InitialCondition_b;
    real_T Constant_Value_f;
    real_T Delay1_InitialCondition_j;
    real_T Gain2_Gain_d;
    real_T Delay_InitialCondition_p;
    real_T Constant_Value_jm;
    real_T Delay1_InitialCondition_k;
    real_T Delay_InitialCondition_k;
    real_T Constant_Value_n;
    real_T Delay1_InitialCondition_n;
    real_T Raising_Value;
    real_T Falling_Value;
    real_T Raising_Value_c;
    real_T Falling_Value_a;
  };

  void initialize();
  void step();
  void terminate();
  AutopilotStateMachineModelClass();
  ~AutopilotStateMachineModelClass();
  void setExternalInputs(const ExternalInputs_AutopilotStateMachine_T* pExternalInputs_AutopilotStateMachine_T)
  {
    AutopilotStateMachine_U = *pExternalInputs_AutopilotStateMachine_T;
  }

  const AutopilotStateMachineModelClass::ExternalOutputs_AutopilotStateMachine_T & getExternalOutputs() const
  {
    return AutopilotStateMachine_Y;
  }

 private:
  static Parameters_AutopilotStateMachine_T AutopilotStateMachine_P;
  BlockIO_AutopilotStateMachine_T AutopilotStateMachine_B;
  D_Work_AutopilotStateMachine_T AutopilotStateMachine_DWork;
  ExternalInputs_AutopilotStateMachine_T AutopilotStateMachine_U;
  ExternalOutputs_AutopilotStateMachine_T AutopilotStateMachine_Y;
  static void AutopilotStateMachine_BitShift(real_T rtu_u, real_T *rty_y);
  static void AutopilotStateMachine_BitShift1(real_T rtu_u, real_T *rty_y);
  boolean_T AutopilotStateMachine_X_TO_OFF(const ap_sm_output *BusAssignment);
  boolean_T AutopilotStateMachine_X_TO_GA_TRK(const ap_sm_output *BusAssignment);
  boolean_T AutopilotStateMachine_ON_TO_HDG(const ap_sm_output *BusAssignment);
  boolean_T AutopilotStateMachine_ON_TO_NAV(const ap_sm_output *BusAssignment);
  void AutopilotStateMachine_NAV_entry(void);
  void AutopilotStateMachine_HDG_entry(const ap_sm_output *BusAssignment);
  boolean_T AutopilotStateMachine_ON_TO_LOC(const ap_sm_output *BusAssignment);
  void AutopilotStateMachine_HDG_during(const ap_sm_output *BusAssignment);
  void AutopilotStateMachine_LOC_CPT_entry(void);
  void AutopilotStateMachine_OFF_entry(void);
  void AutopilotStateMachine_ROLL_OUT_entry(void);
  void AutopilotStateMachine_FLARE_entry(void);
  boolean_T AutopilotStateMachine_LOC_TO_X(const ap_sm_output *BusAssignment);
  void AutopilotStateMachine_LOC_TRACK_entry(void);
  void AutopilotStateMachine_LAND_entry(void);
  boolean_T AutopilotStateMachine_NAV_TO_HDG(const ap_sm_output *BusAssignment);
  boolean_T AutopilotStateMachine_RWY_TO_RWY_TRK(const ap_sm_output *BusAssignment);
  void AutopilotStateMachine_RWY_TRK_entry(const ap_sm_output *BusAssignment);
  void AutopilotStateMachine_GA_TRK_entry(const ap_sm_output *BusAssignment);
  void AutopilotStateMachine_GA_TRK_during(void);
  boolean_T AutopilotStateMachine_OFF_TO_HDG(const ap_sm_output *BusAssignment);
  boolean_T AutopilotStateMachine_OFF_TO_NAV(const ap_sm_output *BusAssignment);
  boolean_T AutopilotStateMachine_OFF_TO_RWY(const ap_sm_output *BusAssignment);
  boolean_T AutopilotStateMachine_OFF_TO_RWY_TRK(const ap_sm_output *BusAssignment);
  void AutopilotStateMachine_RWY_entry(void);
  void AutopilotStateMachine_OFF_entry_p(void);
  void AutopilotStateMachine_ALT_CPT_entry(void);
  void AutopilotStateMachine_VS_entry(void);
  void AutopilotStateMachine_DES_entry(void);
  void AutopilotStateMachine_CLB_entry(void);
  void AutopilotStateMachine_OP_CLB_entry(void);
  void AutopilotStateMachine_OP_DES_entry(void);
  void AutopilotStateMachine_GS_CPT_entry(void);
  void AutopilotStateMachine_SRS_entry(void);
  boolean_T AutopilotStateMachine_X_TO_SRS_GA(void);
  void AutopilotStateMachine_VS_during(void);
  void AutopilotStateMachine_ALT_entry(void);
  void AutopilotStateMachine_VS(void);
  void AutopilotStateMachine_ALT_during(void);
  void AutopilotStateMachine_ALT_exit(void);
  void AutopilotStateMachine_ALT_CST_entry(void);
  void AutopilotStateMachine_ALT(void);
  void AutopilotStateMachine_ALT_CPT_during(void);
  void AutopilotStateMachine_ALT_CPT(void);
  void AutopilotStateMachine_ALT_CST(void);
  void AutopilotStateMachine_ALT_CST_CPT(void);
  void AutopilotStateMachine_CLB_during(void);
  void AutopilotStateMachine_ALT_CST_CPT_entry(void);
  void AutopilotStateMachine_DES_during(void);
  void AutopilotStateMachine_DES(void);
  void AutopilotStateMachine_ROLL_OUT_entry_o(void);
  boolean_T AutopilotStateMachine_GS_TO_X(void);
  void AutopilotStateMachine_GS_TRACK_entry(void);
  void AutopilotStateMachine_LAND_entry_i(void);
  void AutopilotStateMachine_FLARE_entry_g(void);
  void AutopilotStateMachine_GS(void);
  void AutopilotStateMachine_OP_CLB_during(void);
  void AutopilotStateMachine_OP_DES_during(void);
  void AutopilotStateMachine_SRS(void);
  void AutopilotStateMachine_exit_internal_ON(void);
  void AutopilotStateMachine_SRS_GA_entry(void);
  void AutopilotStateMachine_ON(void);
};

#endif

