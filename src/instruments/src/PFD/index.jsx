import ReactDOM from 'react-dom';
import { Component } from 'react';
import { Horizon } from './AttitudeIndicatorHorizon.jsx';
import { AttitudeIndicatorFixedUpper, AttitudeIndicatorFixedCenter } from './AttitudeIndicatorFixed.jsx';
import { LandingSystem } from './LandingSystemIndicator.jsx';
import { VerticalSpeedIndicator } from './VerticalSpeedIndicator.jsx';
import { HeadingOfftape, HeadingTape } from './HeadingIndicator.jsx';
import { AltitudeIndicatorOfftape, AltitudeIndicator } from './AltitudeIndicator.jsx';
import { AirspeedIndicatorOfftape, AirspeedIndicator } from './SpeedIndicator.jsx';
import { FMA } from './FMA.jsx';
import { getSimVar, setSimVar, renderTarget, createDeltaTimeCalculator } from '../util.js';
import { SmoothSin, LagFilter, RateLimiter } from './PFDUtils.jsx';
import './style.scss';

const AltModeSelected = () => {
    if (Simplane.getAutoPilotAltitudeManaged() && getSimVar('L:AP_CURRENT_TARGET_ALTITUDE_IS_CONSTRAINT', 'number') !== 0) {
        return false;
    }
    return true;
};

// eslint-disable-next-line react/prefer-stateless-function
class PFD extends Component {
    constructor(props) {
        super(props);

        const url = document.getElementsByTagName('a32nx-pfd-element')[0].getAttribute('url');
        this.dispIndex = parseInt(url.substring(url.length - 1), 10);

        this.deltaTime = 0;
        this.GetDeltaTime = createDeltaTimeCalculator();
        this.prevAirspeed = 0;

        this.VLs = 0;
        this.VAlphaProt = 0;
        this.VAlphaLim = 0;
        this.VS = 0;

        this.barTimer = 10;

        this.smoothFactor = 0.5;

        this.isAttExcessive = false;

        this.AirspeedAccFilter = new LagFilter(1.2);
        this.AirspeedAccRateLimiter = new RateLimiter(1.2, -1.2);

        this.LSButtonPressed = false;

        // for testing only
        setSimVar('L:A32NX_MachPreselVal', -1, 'mach');
        setSimVar('L:A32NX_SpeedPreselVal', -1, 'knots');
    }

    componentDidMount() {
        renderTarget.parentElement.addEventListener('update', () => {
            this.update(this.GetDeltaTime());
        });
        renderTarget.parentElement.addEventListener(`A320_Neo_PFD_BTN_LS_${this.dispIndex}`, () => {
            this.onLSButtonPressed();
        });
    }

    onLSButtonPressed() {
        this.LSButtonPressed = !this.LSButtonPressed;
        setSimVar(`L:BTN_LS_${this.dispIndex}_FILTER_ACTIVE`, this.LSButtonPressed, 'Bool');
    }

    smoothSpeeds(_dTime, _vls, _vaprot, _valim, _vs) {
        const seconds = _dTime / 1000;
        this.VLs = SmoothSin(this.VLs, _vls, this.smoothFactor, seconds);
        this.VAlphaProt = SmoothSin(this.VAlphaProt, _vaprot, this.smoothFactor, seconds);
        this.VAlphaLim = SmoothSin(this.VAlphaLim, _valim, this.smoothFactor, seconds);
        this.VS = SmoothSin(this.VS, _vs, this.smoothFactor, seconds);
    }

    update(_deltaTime) {
        this.deltaTime = _deltaTime;
        this.forceUpdate();
    }

    render() {
        const pitch = -getSimVar('PLANE PITCH DEGREES', 'degrees');
        const roll = getSimVar('PLANE BANK DEGREES', 'degrees');
        const heading = getSimVar('PLANE HEADING DEGREES MAGNETIC', 'degrees');

        if (!this.isAttExcessive && (pitch > 25 || pitch < -13 || Math.abs(roll) > 45)) {
            this.isAttExcessive = true;
        } else if (this.isAttExcessive && pitch < 22 && pitch > -10 && Math.abs(roll) < 40) {
            this.isAttExcessive = false;
        }

        const groundTrack = getSimVar('GPS GROUND MAGNETIC TRACK', 'degrees');

        const isOnGround = getSimVar('SIM ON GROUND', 'Bool');

        const radioAlt = getSimVar('PLANE ALT ABOVE GROUND MINUS CG', 'feet');
        const decisionHeight = getSimVar('L:AIRLINER_DECISION_HEIGHT', 'feet');

        const baroAlt = getSimVar('INDICATED ALTITUDE', 'feet');
        const mda = getSimVar('L:AIRLINER_MINIMUM_DESCENT_ALTITUDE', 'feet');

        const FlightPhase = getSimVar('L:A32NX_FWC_FLIGHT_PHASE', 'Enum');

        const pressureMode = Simplane.getPressureSelectedMode(Aircraft.A320_NEO);

        const clampedAirspeed = Math.max(getSimVar('AIRSPEED INDICATED', 'knots'), 30);
        const airspeedAcc = (clampedAirspeed - this.prevAirspeed) / this.deltaTime * 1000;
        this.prevAirspeed = clampedAirspeed;

        const rateLimitedAirspeedAcc = this.AirspeedAccRateLimiter.step(airspeedAcc, this.deltaTime / 1000);
        const filteredAirspeedAcc = this.AirspeedAccFilter.step(rateLimitedAirspeedAcc, this.deltaTime / 1000);

        const mach = getSimVar('AIRSPEED MACH', 'mach');

        const VS = getSimVar('L:A32NX_SPEEDS_VS', 'number');
        const VMax = getSimVar('L:A32NX_SPEEDS_VMAX', 'number');
        const VLs = getSimVar('L:A32NX_SPEEDS_VLS', 'number');

        let showSpeedBars = true;
        if (isOnGround) {
            showSpeedBars = false;
            this.barTimer = 0;
        } else if (this.barTimer < 10) {
            showSpeedBars = false;
            this.barTimer += this.deltaTime / 1000;
        }

        this.smoothSpeeds(this.deltaTime, VLs, VS * 1.1, VS * 1.03, VS);

        let targetAlt = Simplane.getAutoPilotDisplayedAltitudeLockValue();
        let isManaged = false;
        if (!AltModeSelected()) {
            const CSTAlt = getSimVar('L:A32NX_AP_CSTN_ALT', 'feet');
            if (Number.isFinite(CSTAlt)) {
                isManaged = true;
                targetAlt = CSTAlt;
            }
        }

        let targetSpeed = Simplane.getV2Airspeed();
        const isSelected = Simplane.getAutoPilotAirspeedSelected();
        const isMach = Simplane.getAutoPilotMachModeActive();
        if (isSelected) {
            if (isMach) {
                targetSpeed = SimVar.GetGameVarValue('FROM MACH TO KIAS', 'number', Simplane.getAutoPilotMachHoldValue());
            } else {
                targetSpeed = Simplane.getAutoPilotAirspeedHoldValue();
            }
        } if (targetSpeed < 0) {
            if (isMach) {
                targetSpeed = Simplane.getCurrentFlightPhase() === FlightPhase.FLIGHT_PHASE_APPROACH
                    ? getSimVar('L:A32NX_AP_APPVLS', 'knots')
                    : SimVar.GetGameVarValue('FROM MACH TO KIAS', 'number', Simplane.getAutoPilotMachHoldValue());
            } else {
                targetSpeed = Simplane.getCurrentFlightPhase() === FlightPhase.FLIGHT_PHASE_APPROACH
                    ? getSimVar('L:A32NX_AP_APPVLS', 'knots') : Simplane.getAutoPilotAirspeedHoldValue();
            }
        }

        const FDActive = getSimVar(`AUTOPILOT FLIGHT DIRECTOR ACTIVE:${this.dispIndex}`, 'Bool');

        let selectedHeading = NaN;
        if (getSimVar('L:A320_FCU_SHOW_SELECTED_HEADING', 'number')) {
            selectedHeading = Simplane.getAutoPilotSelectedHeadingLockValue(false);
        }

        let ILSCourse = NaN;
        if (this.LSButtonPressed) {
            ILSCourse = getSimVar('NAV LOCALIZER:3', 'degrees');
        }

        return (
            <svg className="pfd-svg" version="1.1" viewBox="0 0 158.75 158.75" xmlns="http://www.w3.org/2000/svg">
                <Horizon pitch={pitch} roll={roll} heading={heading} FDActive={FDActive} selectedHeading={selectedHeading} isOnGround={isOnGround} radioAlt={radioAlt} decisionHeight={decisionHeight} isAttExcessive={this.isAttExcessive} deltaTime={this.deltaTime} />
                <path
                    id="Mask1"
                    className="BackgroundFill"
                    d="m32.138 101.25c7.4164 13.363 21.492 21.652 36.768 21.652 15.277 0 29.352-8.2886 36.768-21.652v-40.859c-7.4164-13.363-21.492-21.652-36.768-21.652-15.277 0-29.352 8.2886-36.768 21.652zm-32.046 57.498h158.66v-158.75h-158.66z"
                />
                <HeadingTape heading={heading} ILSCourse={ILSCourse} />
                <AltitudeIndicator altitude={baroAlt} FWCFlightPhase={FlightPhase} />
                <AirspeedIndicator airspeed={clampedAirspeed} airspeedAcc={filteredAirspeedAcc} FWCFlightPhase={FlightPhase} altitude={baroAlt} VAlphaLim={this.VAlphaLim} VAlphaProt={this.VAlphaProt} VLs={this.VLs} VMax={VMax} showBars={showSpeedBars} />
                <path
                    id="Mask2"
                    className="BackgroundFill"
                    d="m32.138 145.34h73.536v10.382h-73.536zm0-44.092c7.4164 13.363 21.492 21.652 36.768 21.652 15.277 0 29.352-8.2886 36.768-21.652v-40.859c-7.4164-13.363-21.492-21.652-36.768-21.652-15.277 0-29.352 8.2886-36.768 21.652zm-32.046 57.498h158.66v-158.75h-158.66zm115.14-35.191v-85.473h15.609v85.473zm-113.33 0v-85.473h27.548v85.473z"
                />
                <LandingSystem LSButtonPressed={this.LSButtonPressed} deltaTime={this.deltaTime} />
                <AttitudeIndicatorFixedUpper />
                <AttitudeIndicatorFixedCenter isOnGround={isOnGround} FDActive={FDActive} isAttExcessive={this.isAttExcessive} />
                <VerticalSpeedIndicator radioAlt={radioAlt} />
                <HeadingOfftape ILSCourse={ILSCourse} groundTrack={groundTrack} heading={heading} selectedHeading={selectedHeading} />
                <AltitudeIndicatorOfftape altitude={baroAlt} radioAlt={radioAlt} MDA={mda} targetAlt={targetAlt} altIsManaged={isManaged} mode={pressureMode} />
                <AirspeedIndicatorOfftape airspeed={clampedAirspeed} mach={mach} airspeedAcc={filteredAirspeedAcc} targetSpeed={targetSpeed} speedIsManaged={!isSelected} />
                <FMA isAttExcessive={this.isAttExcessive} />
            </svg>
        );
    }
}

ReactDOM.render(<PFD />, renderTarget);
