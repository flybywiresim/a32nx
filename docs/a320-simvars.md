# A320neo Local SimVars

- A32NX_NO_SMOKING_MEMO
    - Boolean that determines whether the NO SMOKING annunciation should be visible on the ECAM memo
    - Also is used for knowing when to play the NO SMOKING chime sound

- A32NX_ADIRS_PFD_ALIGNED
    - Bool
    - 0 when ADIRS is not aligned
    - 1 when ADIRS is aligned or 3 minutes after it has started aligning

- A32NX_Neo_ADIRS_START_TIME
    - Seconds
    - Holds the start time in seconds that the ADIRS TIMER will count down from
    - Used to have certain things turn on based on a percentage of the total alignment time

- A32NX_ADIRS_1_FAULT
    - Bool
    - Whether the "FAULT" indication is shown on the OVHD ADIRS panel for ADIRS 1

- A32NX_ADIRS_2_FAULT
    - Bool
    - Whether the "FAULT" indication is shown on the OVHD ADIRS panel for ADIRS 2

- A32NX_ADIRS_3_FAULT
    - Bool
    - Whether the "FAULT" indication is shown on the OVHD ADIRS panel for ADIRS 3

- A32NX_BRAKE_TEMPERATURE_{1,2,3,4}
    - celsius
    - represents the brake temperature of the rear wheels
- A32NX_REPORTED_BRAKE_TEMPERATURE_{1,2,3,4}
    - celsius
    - represents the reported brake temperature of the rear wheels by the sensor.
    - It can be different from the brake temperature when the brake fan has been used, because the brake fan will cool the sensor more than the brakes
	- (which have much more energy to dissipate) therefore giving potentially erroneous readings that the pilots must take into account
- A32NX_BRAKE_FAN
	- boolean
	- whether or not the brake fan is running (brake fan button pressed AND left main landing gear down and locked)
- A32NX_BRAKE_FAN_BTN_PRESSED
	- boolean
	- whether or not the brake fan button is pressed
- A32NX_BRAKES_HOT
    - boolean
    - whether one of the brakes are hot (>300°C)
- XMLVAR_Auto
    - Used in the `.flt` files to set a default value for the ATC 3 way switch on the TCAS panel
    - Maps to the `I:XMLVAR_Auto` variable which is the actual backing var for the switch

- XMLVAR_ALT_MODE_REQUESTED
    - Used in the `.flt` files to set a default value for the ALT RPTG 2 way switch on the TCAS panel
    - Maps to the `I:XMLVAR_ALT_MODE_REQUESTED` variable which is the actual backing var for the switch

- A32NX_KNOB_OVHD_AIRCOND_XBLEED_Position
    - Position (0-2)
    - 0 is SHUT, 1 is AUTO, 2 is OPEN

- A32NX_KNOB_OVHD_AIRCOND_PACKFLOW_Position
    - Position (0-2)
    - 0 is LO, 1 is NORM, 2 is HI

- A32NX_AIRCOND_PACK1_FAULT
    - Bool
    - True if fault in pack 1

- A32NX_AIRCOND_PACK1_TOGGLE
    - Bool
    - True if pack 1 is on

- A32NX_AIRCOND_PACK2_FAULT
    - Bool
    - True if fault in pack 2

- A32NX_AIRCOND_PACK2_TOGGLE
    - Bool
    - True if pack 2 is on

- A32NX_AIRCOND_HOTAIR_FAULT
    - Bool
    - True if fault in hot air system

- A32NX_AIRCOND_HOTAIR_TOGGLE
    - Bool
    - True if hot air system is on

- A32NX_AIRCOND_RAMAIR_TOGGLE
    - Bool
    - True if ram air is on

- A32NX_CALLS_EMER_ON
    - Bool
    - True if emergency cabin call is on

- A32NX_OVHD_COCKPITDOORVIDEO_TOGGLE
    - Bool
    - True if cockpit door video system is on

- PUSH_DOORPANEL_VIDEO
    - Bool
    - True if pedestal door video button is being held

- A32NX_HYD_ENG1PUMP_FAULT
    - Bool
    - True if engine 1 hyd pump fault

- A32NX_HYD_ENG1PUMP_TOGGLE
    - Bool
    - True if engine 1 hyd pump is on

- A32NX_HYD_ENG2PUMP_FAULT
    - Bool
    - True if engine 2 hyd pump fault

- A32NX_HYD_ENG2PUMP_TOGGLE
    - Bool
    - True if engine 2 hyd pump is on

- A32NX_HYD_ELECPUMP_FAULT
    - Bool
    - True if elec hyd pump fault

- A32NX_HYD_ELECPUMP_TOGGLE
    - Bool
    - True if elec hyd pump is on/auto

- A32NX_HYD_PTU_FAULT
    - Bool
    - True if PTU fault

- A32NX_HYD_PTU_TOGGLE
    - Bool
    - True if PTU system on/auto

- A32NX_HYD_ELECPUMPY_FAULT
    - Bool
    - True if yellow elec hyd pump fault

- A32NX_HYD_ELECPUMPY_TOGGLE
    - Bool
    - True if yellow elec hyd pump is on/auto

- A32NX_ENGMANSTART1_TOGGLE
    - Bool
    - True if manual engine 1 start on

- A32NX_ENGMANSTART2_TOGGLE
    - Bool
    - True if manual engine 2 start on

- A32NX_VENTILATION_BLOWER_FAULT
    - Bool
    - True if ventilation blower fault

- A32NX_VENTILATION_BLOWER_TOGGLE
    - Bool
    - True if ventilation blower on

- A32NX_VENTILATION_EXTRACT_FAULT
    - Bool
    - True if ventilation extractor fault

- A32NX_VENTILATION_EXTRACT_TOGGLE
    - Bool
    - True if ventilation extractor on

- A32NX_VENTILATION_CABFANS_TOGGLE
    - Bool
    - True if cabin fans on/auto

- A32NX_PITOT_HEAT_AUTO
    - Bool
    - True if pitot heating auto

- A32NX_EMERELECPWR_GEN1_FAULT
    - Bool
    - True if generator 1 line fault

- A32NX_EMERELECPWR_GEN1_TOGGLE
    - Bool
    - True if generator 1 line is off

- A32NX_EMERELECPWR_RAT_FAULT
    - Bool
    - True if RAT elec power fault

- A32NX_EVAC_COMMAND_FAULT
    - Bool
    - True if evac command fault

- A32NX_EVAC_COMMAND_TOGGLE
    - Bool
    - True if evac command button is on

- PUSH_OVHD_EVAC_HORN
    - Bool
    - True if evac horn cutout button is being pressed

- A32NX_EVAC_CAPT_TOGGLE
    - Bool
    - True if evac switch set to CAPT

- A32NX_KNOB_SWITCHING_1_Position
    - ATT HDG
    - Position (0-2)
    - 0 is CAPT, 1 is NORM, 2 is F/O

- A32NX_KNOB_SWITCHING_2_Position
    - AIR DATA
    - Position (0-2)
    - 0 is CAPT, 1 is NORM, 2 is F/O

- A32NX_KNOB_SWITCHING_3_Position
    - EIS DMC
    - Position (0-2)
    - 0 is CAPT, 1 is NORM, 2 is F/O

- A32NX_KNOB_SWITCHING_4_Position
    - ECAM/ND XFR
    - Position (0-2)
    - 0 is CAPT, 1 is NORM, 2 is F/O

- A32NX_METRIC_ALT_TOGGLE
    - Bool
    - True if PFD metric altitude enabled

- A32NX_OVHD_HYD_BLUEPUMP_OVRD
    - Bool
    - True if "BLUE PUMP OVRD" switch is off

- A32NX_OVHD_HYD_LEAK_MEASUREMENT_G
    - Bool
    - True if "HYD LEAK MEASUREMENT G" switch is on

- A32NX_OVHD_HYD_LEAK_MEASUREMENT_G_LOCK
    - Bool
    - True if "HYD LEAK MEASUREMENT G" switch lock is down

- A32NX_OVHD_HYD_LEAK_MEASUREMENT_B
    - Bool
    - True if "HYD LEAK MEASUREMENT B" switch is on

- A32NX_OVHD_HYD_LEAK_MEASUREMENT_B_LOCK
    - Bool
    - True if "HYD LEAK MEASUREMENT B" switch lock is down

- A32NX_OVHD_HYD_LEAK_MEASUREMENT_Y
    - Bool
    - True if "HYD LEAK MEASUREMENT Y" switch is on

- A32NX_OVHD_HYD_LEAK_MEASUREMENT_Y_LOCK
    - Bool
    - True if "HYD LEAK MEASUREMENT Y" switch lock is down

- A32NX_LANDING_ELEVATION
    - Number in feet
    - Minimum -2000, maximum 15000

- A32NX_MAN_VS_CONTROL
    - Number, either 0,1 or 2
    - 0 if switch is in up position, 1 if switch is neutral, 2 if switch is down.

- A32NX_CAB_PRESS_MODE_MAN
    - Bool
    - True if CABIN PRESS MODE SEL is in manual mode

- A32NX_CAB_PRESS_SYS_FAULT
    - Bool
    - Determines if the FAULT light on the CABIN PRESS MODE SEL pushbutton
      should be on

- A32NX_DITCHING
    - Bool
    - True if DITCHING mode is enabled

- A32NX_FWC_FLIGHT_PHASE
    - Enum
    - Contains the numeric flight phase as determined by the FWC

- A32NX_FWC_SKIP_STARTUP
    - Bool
    - Set to true in a non-cold and dark flight phase to skip the initial memorization step

- A32NX_FWC_TOMEMO
    - Bool
    - True when the FWC decides that the takeoff memo should be shown

- A32NX_FWC_LDGMEMO
    - Bool
    - True when the FWC decides that the landing memo should be shown

- A32NX_FWC_INHIBOVRD
    - Bool
    - True when the FWC decides that flight phase inhibits should be overridden (and ignored)

- A32NX_VSPEEDS_VS
    - Number
    - Current config stall speed
    - is mach corrected

- A32NX_VSPEEDS_VLS
    - Number
    - Current config minimum selectable speed
    - is mach corrected

- A32NX_VSPEEDS_F
    - Number
    - F-Speed (approach)

- A32NX_VSPEEDS_S
    - Number
    - S-Speed (approach)

- A32NX_VSPEEDS_GD
    - Number
    - Green Dot speed (clean config or O)
    - is mach corrected

- A32NX_VSPEEDS_LANDING_CONF3
    - Bool
    - True if FLAPS 3 is selected in perf page

- A32NX_VSPEEDS_TO_CONF
    - Number
    - Flaps config for TakeOff, 1, 2 or 3

- A32NX_VSPEEDS_V2
    - Number
    - TakeOff V2 Speed calculated based on A32NX_VSPEEDS_TO_CONF config

- A32NX_VSPEEDS_VLS_APP
    - Number
    - vls calculated for config full whether A32NX_VSPEEDS_LANDING_CONF3 or not
    - is mach corrected

- A32NX_VSPEEDS_VAPP
    - Number
    - vapp calculated for config full  whether A32NX_VSPEEDS_LANDING_CONF3 or not
    - is mach corrected

- A32NX_TRK_FPA_MODE_ACTIVE
    - Bool
    - True if TRK/FPA mode is active

- A32NX_AUTOPILOT_TRACK_SELECTED
    - Degrees
    - The selected track in the FCU

- A32NX_AUTOPILOT_FPA_SELECTED
    - Degrees
    - The selected flight path angle in the FCU

- A32NX_APU_EGT_CAUTION
    - Celsius
    - The APU's exhaust gas temperature caution level, to be indicated in amber in the cockpit

- A32NX_APU_EGT_WARNING
    - Celsius
    - The APU's exhaust gas temperature warning level, to be indicated in red in the cockpit

- A32NX_APU_EGT
    - Celsius
    - The APU's exhaust gas temperature

- A32NX_APU_N
    - Percent
    - The APU's rotations per minute in percentage of the maximum RPM

- A32NX_APU_START_CONTACTOR_ENERGIZED
    - Bool
    - Indicates if the APU START contactor is energized

- A32NX_APU_BLEED_AIR_VALVE_OPEN
    - Bool
    - Indicates if the APU bleed air valve is open

- A32NX_APU_LOW_FUEL_PRESSURE_FAULT
    - Bool
    - Indicates if the APU has an active LOW FUEL PRESSURE fault

- A32NX_APU_IS_AUTO_SHUTDOWN
    - Bool
    - Indicates if the APU automatically shut down (for a reason other than fire)

- A32NX_APU_IS_EMERGENCY_SHUTDOWN
    - Bool
    - Indicates if the APU automatically shut down due to fire

- A32NX_ECAM_INOP_SYS_APU
    - Bool
    - Indicates if the APU is inoperable

- A32NX_APU_FLAP_OPEN_PERCENTAGE
    - Percent
    - Indicates the percentage the APU air intake flap is open

- A32NX_FIRE_BUTTON_APU
    - Bool
    - Indicates if the APU fire button is RELEASED

 - A32NX_RMP_L_TOGGLE_SWITCH
    - Boolean
    - Whether the left radio management panel toggle switch is on or off.

- A32NX_RMP_R_TOGGLE_SWITCH
    - Boolean
    - Whether the right radio management panel toggle switch is on or off.

- A32NX_RMP_L_SELECTED_MODE
    - Number
    - The current mode of the left radio management panel.

- A32NX_RMP_R_SELECTED_MODE
    - Number
    - The current mode of the right radio management panel.

- A32NX_RMP_L_VHF2_STANDBY
    - Hz
    - The VHF 2 standby frequency for the left RMP.

- A32NX_RMP_L_VHF3_STANDBY
    - Hz
    - The VHF 3 standby frequency for the left RMP.

- A32NX_RMP_R_VHF1_STANDBY
    - Hz
    - The VHF 1 standby frequency for the right RMP.

- A32NX_RMP_R_VHF3_STANDBY
    - Hz
    - The VHF 3 standby frequency for the right RMP.

- A32NX_TO_CONFIG_FLAPS_ENTERED
    - Bool
    - True if the pilot has entered a FLAPS value in the PERF TAKE OFF takeoff

- A32NX_TO_CONFIG_FLAPS
    - Enum
    - The pilot-entered FLAPS value in the PERF TAKE OFF page. 0 is a valid entry.

- A32NX_TO_CONFIG_THS_ENTERED
    - Bool
    - True if the pilot has entered a THS value in the PERF TAKEO FF takeoff

- A32NX_TO_CONFIG_THS
    - Degrees
    - The pilot-entered THS value in the PERF TAKE OFF page. 0 is a valid entry.

- A32NX_ENG_OUT_ACC_ALT
    - feet
    - The engine out acceleration altitude, set in the PERF TAKE OFF page.


- A32NX_RAIN_REPELLENT_RIGHT_ON
    -Bool
    -True if rain repellent is activated on the right windshield.

- A32NX_RAIN_REPELLENT_LEFT_ON
    -Bool
    -True if rain repellent is activated on the left windshield.

- A32NX_RCDR_TEST
    -Bool
    -True if RCDR being tested.

- A32NX_RADAR_MULTISCAN_AUTO
    -Bool
    -True if multiscan switch is set to AUTO.

- A32NX_RADAR_GCS_AUTO
    -Bool
    -True if GCS switch is set to AUTO.

-A32NX_OXYGEN_MASKS_DEPLOYED
    -Bool
    -True if cabin oxygen masks have been deployed.

-A32NX_RCDR_GROUND_CONTROL_ON
    -Bool
    -True if ground control is on.

-A32NX_EMERELECPWR_MAN_ON
    -Bool
    -True if Ram Air Turbine has been manually deployed.

-A32NX_EMERELECPWR_GEN_TEST
    -Bool
    -True if emergency generator is being tested.

-A32NX_OXYGEN_PASSENGER_LIGHT_ON
    -Bool
    -True if cabin oxygen mask doors open.

-A32NX_OXYGEN_TMR_RESET
    -Bool
    -True if oxygen timer is being reset.

-A32NX_OXYGEN_TMR_RESET_FAULT
    -Bool
    -True if fault with oxygen timer.

-A32NX_APU_AUTOEXITING_RESET
    -Bool
    -True if APU autoexiting is being reset.

-A32NX_ELT_TEST_RESET
    -Bool
    -True if ELT is being tested/reset.

-A32NX_ELT_ON
    -Bool
    -True if ELT is on.

-A32NX_DLS_ON
    -Bool
    -True if data loading selector is on.

-A32NX_CREW_HEAD_SET
    -Bool
    -True if CVR crew head set is being pressed.

-A32NX_SVGEINT_OVRD_ON
    -Bool
    -True if SVGE INT OVRD is on.

-A32NX_AVIONICS_COMPLT_ON
    -Bool
    -True if avionics comp lt is on.

-A32NX_CARGOSMOKE_FWD_DISCHARGED
    -Bool
    -True if cargosmoke one bottle is discharged

-A32NX_CARGOSMOKE_AFT_DISCHARGED
    -Bool
    -True if cargosmoke two bottle is discharged

-A32NX_AIDS_PRINT_ON
    -Bool
    -True if AIDS print is on.

-A32NX_DFDR_EVENT_ON
    -Bool
    -True if DFDR event is on.

-A32NX_APU_AUTOEXITING_TEST_ON
    -Bool
    -True if APU AUTOEXITING is being tested.

-A32NX_APU_AUTOEXITING_TEST_OK
    -Bool
    -True if APU AUTOEXITING TEST returns OK.

- A32NX_OVHD_{name}_PB_IS_AVAILABLE
    - Bool
    - True when the push button's AVAIL light should illuminate
    - {name}
        - APU_START

- A32NX_OVHD_{name}_PB_HAS_FAULT
    - Bool
    - Indicates if the push button's FAULT light should illuminate
    - {name}
        - APU_MASTER_SW
        - ELEC_BAT_10
        - ELEC_BAT_11
        - ELEC_IDG_1
        - ELEC_IDG_2
        - ELEC_ENG_GEN_1
        - ELEC_ENG_GEN_2
        - ELEC_AC_ESS_FEED
        - ELEC_GALY_AND_CAB
        - PNEU_APU_BLEED

- A32NX_OVHD_{name}_PB_IS_AUTO
    - Bool
    - True when the push button is AUTO
    - {name}
        - ELEC_BAT_10
        - ELEC_BAT_11
        - ELEC_BUS_TIE_PB
        - ELEC_GALY_AND_CAB

- A32NX_OVHD_{name}_PB_IS_RELEASED
    - Bool
    - True when the push button is RELEASED
    - {name}
        - ELEC_IDG_1
        - ELEC_IDG_2

- A32NX_OVHD_ELEC_AC_ESS_FEED_PB_IS_NORMAL
    - Bool
    - True when the AC ESS FEED push button is NORMAL

- A32NX_OVHD_{name}_PB_IS_ON
    - Bool
    - True when the push button is ON
    - {name}
        - APU_START
        - APU_MASTER_SW
        - ELEC_COMMERCIAL
        - PNEU_APU_BLEED

- A32NX_ELEC_CONTACTOR_{name}_IS_CLOSED
    - Bool
    - True when the contactor is CLOSED
    - {name}
        - 1PC1: DC BAT BUS feed contactor between DC BUS 1 and DC BAT BUS
        - 1PC2: DC BAT BUS feed contactor between DC BUS 2 and DC BAT BUS
        - 2XB1: Contactor between battery 1 and the static inverter
        - 2XB2: Contactor between battery 2 and the DC ESS BUS
        - 3XC1: AC ESS feed contactor between AC BUS 1 and AC ESS BUS
        - 3XC2: AC ESS feed contactor between AC BUS 2 and AC ESS BUS
        - 3PE: Transformer rectifier ESS contactor between TR ESS and DC ESS BUS
        - 2XE: Emergency generator contactor
        - 3XG: External power contactor
        - 3XS: APU generator contactor
        - 4PC: Contactor between DC BAT BUS and DC ESS BUS
        - 5PU1: Transformer rectifier 1 contactor between TR1 and DC BUS 1
        - 5PU2: Transformer rectifier 2 contactor between TR2 and DC BUS 2
        - 6PB1: Battery 1 contactor
        - 6PB2: Battery 2 contactor
        - 8PH: DC ESS SHED contactor
        - 8XH: AC ESS SHED contactor
        - 9XU1: Engine generator line contactor 1
        - 9XU2: Engine generator line contactor 2
        - 11XU1: AC BUS tie 1 contactor
        - 11XU2: AC BUS tie 2 contactor
        - 15XE1: Contactor between AC ESS BUS and TR ESS + EMER GEN
        - 15XE2: Contactor between the static inverter and AC ESS BUS

- A32NX_ELEC_{name}_BUS_IS_POWERED
    - Bool
    - True when the given bus is powered
    - {name}
        - AC_1
        - AC_2
        - AC_ESS
        - AC_ESS_SHED
        - AC_STAT_INV
        - DC_1
        - DC_2
        - DC_ESS
        - DC_ESS_SHED
        - DC_BAT
        - DC_HOT_1
        - DC_HOT_2


- A32NX_ELEC_{name}_POTENTIAL
    - Volts
    - The electric potential of the given element
    - {name}
        - APU_GEN_1
        - ENG_GEN_1
        - ENG_GEN_2
        - EXT_PWR
        - STAT_INV
        - EMER_GEN
        - TR_1
        - TR_2
        - TR_3: TR ESS
        - BAT_10: Battery 1
        - BAT_11: Battery 2

- A32NX_ELEC_{name}_POTENTIAL_NORMAL
    - Bool
    - Indicates if the potential is within the normal range
    - {name}
        - APU_GEN_1
        - ENG_GEN_1
        - ENG_GEN_2
        - EXT_PWR
        - STAT_INV
        - EMER_GEN
        - TR_1
        - TR_2
        - TR_3: TR ESS
        - BAT_10: Battery 1
        - BAT_11: Battery 2

- A32NX_ELEC_{name}_FREQUENCY:
    - Hertz
    - The frequency of the alternating current of the given element
    - {name}
        - APU_GEN_1
        - ENG_GEN_1
        - ENG_GEN_2
        - EXT_PWR
        - STAT_INV
        - EMER_GEN

- A32NX_ELEC_{name}_FREQUENCY_NORMAL
    - Hertz
    - Indicates if the frequency is within the normal range
    - {name}
        - APU_GEN_1
        - ENG_GEN_1
        - ENG_GEN_2
        - EXT_PWR
        - STAT_INV
        - EMER_GEN

- A32NX_ELEC_{name}_LOAD
    - Percent
    - The load the generator is providing compared to its maximum
    - {name}
        - APU_GEN_1
        - ENG_GEN_1
        - ENG_GEN_2

- A32NX_ELEC_{name}_LOAD_NORMAL
    - Percent
    - Indicates if the load is within the normal range
    - {name}
        - APU_GEN_1
        - ENG_GEN_1
        - ENG_GEN_2

- A32NX_ELEC_{name}_CURRENT
    - Ampere
    - The electric current flowing through the given element
    - {name}
        - TR_1
        - TR_2
        - TR_3: TR ESS
        - BAT_10: Battery 1
        - BAT_11: Battery 2

- A32NX_ELEC_{name}_CURRENT_NORMAL
    - Ampere
    - Indicates if the current is within the normal range
    - {name}
        - TR_1
        - TR_2
        - TR_3: TR ESS
        - BAT_10: Battery 1
        - BAT_11: Battery 2

- A32NX_ELEC_ENG_GEN_{number}_IDG_OIL_OUTLET_TEMPERATURE
    - Celsius
    - The integrated drive generator's oil outlet temperature
    - {number}
        - 1
        - 2

- A32NX_ELEC_ENG_GEN_{number}_IDG_IS_CONNECTED
    - Bool
    - Indicates if the given integrated drive generator is connected
    - {number}
        - 1
        - 2

- A32NX_FMGC_FLIGHT_PHASE
    - Enum
    - Holds the FMGCs current flight phase
    - Use FMGC_FLIGHT_PHASES to check for phases (import NXFMGCFlightPhases from A32NX_Utils)

- A32NX_SIDESTICK_POSITION_X
    - Number
    - Provides the direct sidestick position (lateral)

- A32NX_SIDESTICK_POSITION_Y
    - Number
    - Provides the direct sidestick position (longitudinal)
