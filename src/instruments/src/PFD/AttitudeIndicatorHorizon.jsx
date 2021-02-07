import {
    calculateHorizonOffsetFromPitch,
    calculateVerticalOffsetFromRoll,
    getSmallestAngle,
    HorizontalTape,
    LagFilter,
} from './PFDUtils.jsx';
import { getSimVar } from '../util.js';

const DisplayRange = 35;
const DistanceSpacing = 15;
const ValueSpacing = 10;
const SideslipIndicatorFilter = new LagFilter(0.8);

const TickFunction = (heading, offset) => (
    <path transform={`translate(${offset} 0)`} className="NormalStroke White" d="m68.906 80.823v1.8" />
);

const HeadingBug = (offset) => (
    <g id="HorizonHeadingBug" transform={`translate(${offset} 0)`}>
        <path className="ThickOutline" d="m68.906 80.823v-9.0213" />
        <path className="ThickStroke Cyan" d="m68.906 80.823v-9.0213" />
    </g>
);

export const Horizon = ({ pitch, roll, heading, isOnGround, radioAlt, decisionHeight, selectedHeading, FDActive, isAttExcessive, deltaTime }) => {
    if (!getSimVar('L:A32NX_ADIRS_PFD_ALIGNED_ATT', 'Bool')) {
        return null;
    }

    const headingAvail = getSimVar('L:A320_Neo_ADIRS_STATE', 'Enum') === 2;

    const yOffset = Math.max(Math.min(calculateHorizonOffsetFromPitch(pitch), 31.563), -31.563);

    const bugs = [];
    if (!Number.isNaN(selectedHeading) && !FDActive) {
        bugs.push([HeadingBug, selectedHeading]);
    }

    return (
        <g id="RollGroup" transform={`rotate(${roll} 68.814 80.730)`}>
            <g id="PitchGroup" transform={`translate(0 ${calculateHorizonOffsetFromPitch(pitch)})`}>
                <path d="m23.906 80.823v-160h90v160z" className="SkyFill" />
                <path d="m113.91 223.82h-90v-143h90z" className="EarthFill" />

                {/* If you're wondering why some paths have an "h0" appended, it's to work around a
            rendering bug in webkit, where paths with only one line is rendered blurry. */}

                <g className="NormalStroke White">
                    <path d="m66.406 85.323h5h0" />
                    <path d="m64.406 89.823h9h0" />
                    <path d="m66.406 94.073h5h0" />
                    <path d="m59.406 97.823h19h0" />
                    <path d="m64.406 103.82h9h0" />
                    <path d="m59.406 108.82h19h0" />
                    <path d="m55.906 118.82h26h0" />
                    <path d="m52.906 138.82h32h0" />
                    <path d="m47.906 168.82h42h0" />
                    <path d="m66.406 76.323h5h0" />
                    <path d="m64.406 71.823h9h0" />
                    <path d="m66.406 67.323h5h0" />
                    <path d="m59.406 62.823h19h0" />
                    <path d="m66.406 58.323h5h0" />
                    <path d="m64.406 53.823h9h0" />
                    <path d="m66.406 49.323h5h0" />
                    <path d="m59.406 44.823h19h0" />
                    <path d="m66.406 40.573h5h0" />
                    <path d="m64.406 36.823h9h0" />
                    <path d="m66.406 33.573h5h0" />
                    <path d="m55.906 30.823h26h0" />
                    <path d="m52.906 10.823h32h0" />
                    <path d="m47.906-19.177h42h0" />
                </g>

                <g id="PitchProtUpper" className="NormalStroke Green">
                    <path d="m51.506 31.523h4m-4-1.4h4" />
                    <path d="m86.306 31.523h-4m4-1.4h-4" />
                </g>
                <g id="PitchProtLostUpper" style={{ display: 'none' }} className="NormalStroke Amber">
                    <path d="m52.699 30.116 1.4142 1.4142m-1.4142 0 1.4142-1.4142" />
                    <path d="m85.114 31.53-1.4142-1.4142m1.4142 0-1.4142 1.4142" />
                </g>
                <g id="PitchProtLower" className="NormalStroke Green">
                    <path d="m59.946 104.52h4m-4-1.4h4" />
                    <path d="m77.867 104.52h-4m4-1.4h-4" />
                </g>
                <g id="PitchProtLostLower" style={{ display: 'none' }} className="NormalStroke Amber">
                    <path d="m61.199 103.12 1.4142 1.4142m-1.4142 0 1.4142-1.4142" />
                    <path d="m76.614 104.53-1.4142-1.4142m1.4142 0-1.4142 1.4142" />
                </g>

                <path d="m68.906 121.82-8.0829 14h2.8868l5.1962-9 5.1962 9h2.8868z" className="NormalStroke Red" />
                <path d="m57.359 163.82 11.547-20 11.547 20h-4.0414l-7.5056-13-7.5056 13z" className="NormalStroke Red" />
                <path d="m71.906 185.32v3.5h15l-18-18-18 18h15v-3.5h-6.5l9.5-9.5 9.5 9.5z" className="NormalStroke Red" />
                <path d="m60.824 13.823h2.8868l5.1962 9 5.1962-9h2.8868l-8.0829 14z" className="NormalStroke Red" />
                <path d="m61.401-13.177h-4.0414l11.547 20 11.547-20h-4.0414l-7.5056 13z" className="NormalStroke Red" />
                <path d="m68.906-26.177-9.5-9.5h6.5v-3.5h-15l18 18 18-18h-15v3.5h6.5z" className="NormalStroke Red" />

                <TailstrikeIndicator />

                <path d="m23.906 80.823h90h0" className="NormalOutline" />
                <path d="m23.906 80.823h90h0" className="NormalStroke White" />

                <g className="FontSmall White Fill EndAlign">
                    <text x="55.729935" y="64.812828">10</text>
                    <text x="88.618317" y="64.812714">10</text>
                    <text x="54.710766" y="46.931034">20</text>
                    <text x="89.564583" y="46.930969">20</text>
                    <text x="50.867237" y="32.910896">30</text>
                    <text x="93.408119" y="32.910839">30</text>
                    <text x="48.308414" y="12.690886">50</text>
                    <text x="96.054962" y="12.690853">50</text>
                    <text x="43.050652" y="-17.138285">80</text>
                    <text x="101.48304" y="-17.138248">80</text>
                    <text x="55.781109" y="99.81395">10</text>
                    <text x="88.669487" y="99.813919">10</text>
                    <text x="54.645519" y="110.8641">20</text>
                    <text x="89.892426" y="110.86408">20</text>
                    <text x="51.001217" y="120.96314">30</text>
                    <text x="93.280037" y="120.96311">30</text>
                    <text x="48.220913" y="140.69778">50</text>
                    <text x="96.090324" y="140.69786">50</text>
                    <text x="43.125065" y="170.80962">80</text>
                    <text x="101.38947" y="170.80959">80</text>
                </g>
            </g>
            <path d="m40.952 49.249v-20.562h55.908v20.562z" className="NormalOutline SkyFill" />
            <path d="m40.952 49.249v-20.562h55.908v20.562z" className="NormalStroke White" />
            <SideslipIndicator isOnGround={isOnGround} roll={roll} deltaTime={deltaTime} />
            <RisingGround radioAlt={radioAlt} pitch={pitch} />
            {headingAvail
            && <HorizontalTape graduationElementFunction={TickFunction} bugs={bugs} yOffset={yOffset} displayRange={DisplayRange} distanceSpacing={DistanceSpacing} valueSpacing={ValueSpacing} heading={heading} />}
            {!isAttExcessive
            && <RadioAltAndDH radioAlt={radioAlt} decisionHeight={decisionHeight} roll={roll} />}
            <FlightPathVector />
            {!isAttExcessive
            && <FlightPathDirector FDActive={FDActive} />}
        </g>
    );
};

const FlightPathVector = () => {
    if (!getSimVar('L:A32NX_TRK_FPA_MODE_ACTIVE', 'bool')) {
        return null;
    }

    const roll = getSimVar('PLANE BANK DEGREES', 'degrees');
    const pitch = -getSimVar('PLANE PITCH DEGREES', 'degrees');
    const AOA = getSimVar('INCIDENCE ALPHA', 'degrees');
    const FPA = pitch - (Math.cos(roll * Math.PI / 180) * AOA);
    const DA = getSmallestAngle(getSimVar('GPS GROUND TRUE TRACK', 'degrees'), getSimVar('GPS GROUND TRUE HEADING', 'degrees'));

    const xOffset = Math.max(Math.min(DA, 21), -21) * DistanceSpacing / ValueSpacing;
    const yOffset = calculateHorizonOffsetFromPitch(pitch) - calculateHorizonOffsetFromPitch(FPA);

    return (
        <g transform={`translate(${xOffset} ${yOffset})`}>
            <svg x="53.4" y="65.3" width="31px" height="31px" version="1.1" viewBox="0 0 31 31" xmlns="http://www.w3.org/2000/svg">
                <g transform={`rotate(${-roll} 15.5 15.5)`}>
                    <path className="NormalOutline" d="m17.766 15.501c8.59e-4 -1.2531-1.0142-2.2694-2.2665-2.2694-1.2524 0-2.2674 1.0163-2.2665 2.2694-8.57e-4 1.2531 1.0142 2.2694 2.2665 2.2694 1.2524 0 2.2674-1.0163 2.2665-2.2694z" />
                    <path className="ThickOutline" d="m17.766 15.501h5.0367m-9.5698 0h-5.0367m7.3033-2.2678v-2.5199" />
                    <path className="NormalStroke Green" d="m17.766 15.501c8.59e-4 -1.2531-1.0142-2.2694-2.2665-2.2694-1.2524 0-2.2674 1.0163-2.2665 2.2694-8.57e-4 1.2531 1.0142 2.2694 2.2665 2.2694 1.2524 0 2.2674-1.0163 2.2665-2.2694z" />
                    <path className="ThickStroke Green" d="m17.766 15.501h5.0367m-9.5698 0h-5.0367m7.3033-2.2678v-2.5199" />
                </g>
            </svg>
        </g>
    );
};

const FlightPathDirector = ({ FDActive }) => {
    if (!FDActive || !getSimVar('L:A32NX_TRK_FPA_MODE_ACTIVE', 'bool')) {
        return null;
    }

    const lateralAPMode = getSimVar('L:A32NX_FMA_LATERAL_MODE', 'number');
    const verticalAPMode = getSimVar('L:A32NX_FMA_VERTICAL_MODE', 'enum');
    const showLateralFD = lateralAPMode !== 0 && lateralAPMode !== 34 && lateralAPMode !== 40;
    const showVerticalFD = verticalAPMode !== 0 && verticalAPMode !== 34;

    if (!showVerticalFD && !showLateralFD) {
        return null;
    }

    const FDRollOrder = getSimVar('L:A32NX_FLIGHT_DIRECTOR_BANK', 'number');
    const currentRoll = getSimVar('PLANE BANK DEGREES', 'degrees');
    const FDRollOffset = -FDRollOrder * 0.77;

    const DA = getSmallestAngle(getSimVar('GPS GROUND TRUE TRACK', 'degrees'), getSimVar('GPS GROUND TRUE HEADING', 'degrees'));

    const xOffset = Math.max(Math.min(DA, 21), -21) * DistanceSpacing / ValueSpacing;

    const FDPitchOrder = getSimVar('L:A32NX_FLIGHT_DIRECTOR_PITCH', 'number');
    const currentPitch = -getSimVar('PLANE PITCH DEGREES', 'degrees');
    const AOA = getSimVar('INCIDENCE ALPHA', 'degrees');
    const FPA = currentPitch - (Math.cos(currentRoll * Math.PI / 180) * AOA);

    const yOffset = calculateHorizonOffsetFromPitch(currentPitch) - calculateHorizonOffsetFromPitch(FPA) + (FDPitchOrder + currentPitch) * 0.44;

    return (
        <g transform={`translate(${xOffset} ${yOffset})`}>
            <svg x="53.4" y="65.3" width="31px" height="31px" version="1.1" viewBox="0 0 31 31" xmlns="http://www.w3.org/2000/svg">
                <g transform={`rotate(${FDRollOffset} 15.5 15.5)`} className="CornerRound">
                    <path className="NormalOutline" d="m16.507 15.501a1.0074 1.008 0 1 0-2.0147 0 1.0074 1.008 0 1 0 2.0147 0zm7.5551 0 6.5478-1.5119v3.0238l-6.5478-1.5119m-17.125 0-6.5478-1.5119v3.0238l6.5478-1.5119h17.125" />
                    <path className="NormalStroke Green" d="m16.507 15.501a1.0074 1.008 0 1 0-2.0147 0 1.0074 1.008 0 1 0 2.0147 0zm7.5551 0 6.5478-1.5119v3.0238l-6.5478-1.5119m-17.125 0-6.5478-1.5119v3.0238l6.5478-1.5119h17.125" />
                </g>
            </svg>
        </g>
    );
};

const TailstrikeIndicator = () => {
    // should also not be displayed when thrust levers are at or above FLX/MCT, but I don't know if there is a simvar
    // for that
    if (getSimVar('PLANE ALT ABOVE GROUND MINUS CG', 'feet') > 400
        || getSimVar('AIRSPEED INDICATED', 'knots') < 50
        || getSimVar('GENERAL ENG THROTTLE LEVER POSITION:1', 'percent over 100') >= 0.95
        || getSimVar('GENERAL ENG THROTTLE LEVER POSITION:2', 'percent over 100') >= 0.95) {
        return null;
    }

    return (
        <path id="TailstrikeWarning" d="m72.682 50.223h2.9368l-6.7128 8-6.7128-8h2.9368l3.7759 4.5z" className="NormalStroke Amber" />
    );
};

const RadioAltAndDH = ({ radioAlt, decisionHeight, roll }) => {
    if (radioAlt <= 2500) {
        const verticalOffset = calculateVerticalOffsetFromRoll(roll);
        const size = (radioAlt > 400 ? 'FontLarge' : 'FontLargest');
        const DHValid = decisionHeight >= 0;
        const color = (radioAlt > 400 || (radioAlt > decisionHeight + 100 && DHValid) ? 'Green' : 'Amber');

        let text = '';

        if (radioAlt < 5) {
            text = Math.round(radioAlt).toString();
        } else if (radioAlt <= 50) {
            text = (Math.round(radioAlt / 5) * 5).toString();
        } else if (radioAlt > 50 || (radioAlt > decisionHeight + 100 && DHValid)) {
            text = (Math.round(radioAlt / 10) * 10).toString();
        }

        return (
            <g id="DHAndRAGroup" transform={`translate(0 ${-verticalOffset})`}>
                {radioAlt <= decisionHeight ? <text id="AttDHText" x="73.511879" y="113.19068" className="FontLargest Amber EndAlign Blink9Seconds">DH</text> : null}
                <text id="RadioAlt" x="68.803764" y="119.88165" className={`${size} ${color} MiddleAlign`}>{text}</text>
            </g>
        );
    }
    return null;
};

const SideslipIndicator = ({ isOnGround, roll, deltaTime }) => {
    let SIIndexOffset = 0;

    const verticalOffset = calculateVerticalOffsetFromRoll(roll);

    if (isOnGround) {
        // on ground, lateral g is indicated. max 0.3g, max deflection is 15mm
        const latAcc = getSimVar('ACCELERATION BODY X', 'G Force');
        const accInG = Math.min(0.3, Math.max(-0.3, latAcc));
        SIIndexOffset = -accInG * 15 / 0.3;
    } else {
        SIIndexOffset = Math.max(Math.min(getSimVar('INCIDENCE BETA', 'degrees'), 15), -15);
    }

    SIIndexOffset = SideslipIndicatorFilter.step(SIIndexOffset, deltaTime / 1000);

    return (
        <g id="RollTriangleGroup" transform={`translate(0 ${verticalOffset})`} className="NormalStroke Yellow CornerRound">
            <path d="m66.185 43.176h5.4437l-2.7218-3.427z" />
            <path id="SideSlipIndicator" transform={`translate(${SIIndexOffset} 0)`} d="m62.862 47.359 2.1615-2.7214h7.7653l2.1615 2.7214z" />
        </g>
    );
};

const RisingGround = ({ radioAlt, pitch }) => {
    const targetPitch = -0.1 * radioAlt;

    const targetOffset = Math.max(Math.min(calculateHorizonOffsetFromPitch(pitch - targetPitch) - 31.563, 0), -63.093);

    return (
        <g id="HorizonGroundRectangle" transform={`translate(0 ${targetOffset})`}>
            <path d="m113.95 157.74h-90.08v-45.357h90.08z" className="NormalOutline EarthFill" />
            <path d="m113.95 157.74h-90.08v-45.357h90.08z" className="NormalStroke White" />
        </g>
    );
};
