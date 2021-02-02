# Documentation

## Custom Autopilot System

⚠️ This is work in progress, there are still issues, see section Known issues below!

### Known issues

⚠️ The custom autopilot system is not yet on study level. In order to achieve this level a longer effort is needed. The system is a large improvement over the default implementation and there is no reason to hold it back longer than necessary.

⚠️ When starting in flight, the default AP might be actived without seeing it. In that case you can disconnect it by using `AUTOPILOT OFF` event. Default keyboard mapping `Shift+Alt+Z`. Often this event is used for sidestick disconnect button.

ℹ️ Tuning is a large effort to be done for differnt flight conditions like speed, configuration, weight and center-of-gravity (CG). You can help by reporting issues in certain flight conditions. Please take note of important conditions mentioned before.

#### Not solved or missing

- :x: custom ATHR system is not yet available
- :x: FMA indications for ATHR system are missing
- :x: due to missing custom ATHR system, the (OP) CLB/DES modes might need manual thrust control
      -> a simple and hacky workaround has been added though
- :x: due to this workaround, the engine EGT can go into read area when in (OP) CLB/DES (see workaround above)
- :x: after a longer pause the custom autopilot system can go crazy
- :x: due to lack of VNAV, DES mode is currently only using SPD/MACH
- :x: Transitions might not be as they should
- :x: FD off/on does not deactivate all FMA items
- :x: Engagement of AP with FD off is incorrect
- :x: AP disconnect does not trigger master warning etc.
- :x: AP does not disconnect with rudder or sidestick input
- :x: NAV mode being armed might show dashes in the FCU instead of selected HDG
- :x: Engine out operations are not yet considered
- :x: AP performance when flying turbulence might not be satisfying
- :x: AP is not disconnected due to pilot input (sidestick, rudder) or turbulence
- :x: FMA or engagement of AP when changing position with slew might be wrong
- :x: Flight Director (FD) guidance in pitch is not satisfying yet

#### First implementation available
- :small_orange_diamond: Principle go-around mode has been added but not all conditions are respected yet
- :small_orange_diamond: NAV mode is for the time being using default flight plan manager until the custom is ready
- :small_orange_diamond: altitude constraints seem to work with CLB and DES (there are many situations out there, so there can still be unknown bugs)

#### Considered solved

- :heavy_check_mark: In manual approach LOC and G/S might be lost too fast with mode reversion to HDG + V/S

## Custom Fly-By-Wire System

⚠️ This is work in progress, there are still issues, see section Known issues below!

### Compatibility

- :x: incompatible with YourControls add-on

### Sensitivity and dead zones

ℹ️ It is recommended that the sidestick uses a linear sensitivity with only dead zone set appropriately.

### Throttle configuration

⚠️ It is needed to start the sim and load a flight using the A32NX with custom fly-by-wire to create those files.

⚠️ When throttle sensitivity is changed, the throttle configuration needs to be adapted in most cases.

ℹ️ The developer mode console shows input and output values. This can help to setup the throttle configuration.

In case of Microsoft Store version, the configuration of the throttle can be found here:
`%LOCALAPPDATA%\Packages\Microsoft.FlightSimulator_8wekyb3d8bbwe\LocalState\packages\<name of a32nx folder in community folder>\work\ThrottleConfiguration.ini`

In case of Steam version, the configuration of the throttle can be found here:
`%APPDATA%\Roaming\Microsoft Flight Simulator\Packages\<name of a32nx folder in community folder>\work\ThrottleConfiguration.ini`

This is the default configuration:

```
[Throttle]
Log = true
Enabled = true
ReverseOnAxis = false
ReverseIdle = true
DetentDeadZone = 2.0
DetentReverseFull = -1.00
DetentReverseIdle = -0.90
DetentIdle = -1.00
DetentClimb = 0.89
DetentFlexMct = 0.95
DetentTakeOffGoAround = 1.00
```

ℹ️ The configuration file is written with the default configuration when it's not found.

⚠️ The plane has to be reloaded to use a changed configured (reload in developer mode is sufficient).

The following values can be configured:

| Parameter|Comment|
|----------|-------------|
| Enabled |  Enabled or disables throttle handling completely |
| ReverseOnAxis | When true reverse will be mapped on throttle axis and parameter *DetentReverseFull* is used  |
| ReverseIdle | When true a reverse idle detent is added and parameter *DetentReverseIdle* is used. Only works with *ReverseOnAxis* enabled |
| DetentDeadZone | Deadzone around the detents, applies after mapping the values to range -20 to 100 |
| DetentReverseFull | Configures the throttle value for REV FULL detent |
| DetentReverseIdle | Configures the throttle value for REV IDLE detent |
| DetentIdle | Configures the throttle value for IDLE detent |
| DetentClimb | Configures the throttle value for CLB detent |
| DetentFlexMct | Configures the throttle value for FLX/MCT detent |
| DetentTakeOffGoAround | Configures the throttle value for TOGA detent |

#### Example for Thrustmaster TCA Sidestick Airbus Edition (1)

This example shows how the config looks like when the throttle axis is mapped *without* reverse.
It uses Mode 1 in the joystick configuration. The throttle all way down means idle.

```
[Throttle]
Log = true
Enabled = true
ReverseOnAxis = false
DetentReverseFull = -1.00
DetentIdle = -1.00
DetentClimb = 0.66
DetentFlexMct = 0.88
DetentTakeOffGoAround = 1.00
```

#### Example for Thrustmaster TCA Sidestick Airbus Edition (2)

This example shows how the config looks like when the throttle axis is mapped *with* reverse.
It uses Mode 1 in the joystick configuration. The throttle in the detent means idle, all way down maximum reverse.

```
[Throttle]
Log = true
Enabled = true
ReverseOnAxis = true
DetentReverseFull = -1.00
DetentIdle = -0.77
DetentClimb = 0.66
DetentFlexMct = 0.88
DetentTakeOffGoAround = 1.00
```

#### Example for Thrustmaster TCA Quadrant Airbus Edition

```
[Throttle]
Log = true
Enabled = true
ReverseOnAxis = true
DetentReverseFull = -1.00
DetentIdle = 0.00
DetentClimb = 0.66
DetentFlexMct = 0.88
DetentTakeOffGoAround = 1.00
```

#### Example for Thrustmaster TCA Quadrant Airbus Edition (2)

This example shows a calibrated hardware with linear sensitity in Microsoft Flight Simulator (all values 0% with reactivity 100%). It uses also the idle reverse detent.

```
[Throttle]
Log = true
Enabled = true
ReverseOnAxis = true
ReverseIdle = true
DetentDeadZone = 2.5
DetentReverseIdle = -0.90
DetentReverseFull = -1.00
DetentIdle = -0.42
DetentClimb = 0.06
DetentFlexMct = 0.53
DetentTakeOffGoAround = 1.00
```

### Known issues

⚠️ The custom fly-by-wire system is not yet on study level. In order to achieve this level a longer effort is needed. The system is a large improvement over the default implementation and there is no reason to hold it back longer than necessary.

ℹ️ Tuning is a large effort to be done for differnt flight conditions like speed, configuration, weight and center-of-gravity (CG). You can help by reporting issues in certain flight conditions. Please take note of important conditions mentioned before.

#### Not solved or missing

- :x: High speed protection
- :x: High angle of attack (AoA) protection
- :x: Alternative Law
- :x: Direct Law (in flight)
- :x: Simulation of hydraulic system missing -> when engines are off / electric pump is off control surfaces should not work

#### Considered solved

- :heavy_check_mark: Normal Law (Pitch) creates a too small pitch rate on low speed or g-load on higher speeds
- :heavy_check_mark: Rotation Law including tailstrike protection
- :heavy_check_mark: pitch normal law (C* law) sometimes oscillates on low speed
- :heavy_check_mark: yaw damper / rudder control missing
- :heavy_check_mark: pitch attitude protections can oscillate
- :heavy_check_mark: nose-down pitch attitude protection sometimes kicks-in too early
- :heavy_check_mark: transformation from ground to flight mode might take longer than intended (nose might drop after releasing the stick)
- :heavy_check_mark: auto-trim feature locks trim wheel completely
- :heavy_check_mark: flare mode might be stronger than expected, needs to be investigated
- :heavy_check_mark: after landing sometimes a slight pitch up moment is introduced, needs to be investigated
- :heavy_check_mark: strange interaction with default auto thrust system -> thrust lever sometimes does not move, fix is to manually disable ATHR
- :heavy_check_mark: after a longer pause the fbw system goes crazy
