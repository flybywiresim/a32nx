import { VerticalTape } from './PFDUtils.jsx';
import { DigitalAltitudeReadout } from './DigitalAltitudeReadout.jsx';
import { getSimVar } from '../util.js';

const DisplayRange = 570;
const ValueSpacing = 100;
const DistanceSpacing = 7.5;

const GraduationElement = (alt, offset) => {
    let text = '';
    let isText = false;
    if (alt % 500 === 0) {
        isText = true;
        text = (Math.abs(alt) / 100).toString().padStart(3, '0');
    }

    return (
        <g transform={`translate(0 ${offset})`}>
            {isText
            && <path className="NormalStroke White" d="m115.79 81.889 1.3316-1.0783-1.3316-1.0783" />}
            <path className="NormalStroke White" d="m130.85 80.819h-2.0147" />
            <text className="FontMedium MiddleAlign White" x="122.98842" y="82.939713">{text}</text>
        </g>
    );
};

const LandingElevationIndicator = ({ altitude, FWCFlightPhase }) => {
    if (FWCFlightPhase !== 7 && FWCFlightPhase !== 8) {
        return null;
    }

    const landingElevation = getSimVar('C:fs9gps:FlightPlanDestinationAltitude', 'feet');
    const delta = altitude - landingElevation;
    if (delta > DisplayRange) {
        return null;
    }
    const offset = (delta - DisplayRange) * DistanceSpacing / ValueSpacing;

    return (
        <path id="AltTapeLandingElevation" className="EarthFill" d={`m130.85 123.56h-13.096v${offset}h13.096z`} />
    );
};

const RadioAltIndicator = ({ radioAlt }) => {
    if (radioAlt > DisplayRange) {
        return null;
    }
    const offset = (radioAlt - DisplayRange) * DistanceSpacing / ValueSpacing;

    return (
        <path id="AltTapeGroundReference" className="Fill Red" d={`m131.15 123.56h2.8709v${offset}h-2.8709z`} />
    );
};

export const AltitudeIndicator = ({ altitude, FWCFlightPhase }) => {
    if (!getSimVar('L:A32NX_ADIRS_PFD_ALIGNED_FIRST', 'Bool')) {
        return (
            <AltTapeBackground />
        );
    }

    const bugs = [];

    return (
        <g>
            <AltTapeBackground />
            <LandingElevationIndicator altitude={altitude} FWCFlightPhase={FWCFlightPhase} />
            <VerticalTape tapeValue={altitude} graduationElementFunction={GraduationElement} bugs={bugs} displayRange={DisplayRange + 30} valueSpacing={ValueSpacing} distanceSpacing={DistanceSpacing} lowerLimit={-1500} upperLimit={50000} />
        </g>
    );
};

export const AltitudeIndicatorOfftape = ({ altitude, MDA, targetAlt, altIsManaged, mode, radioAlt }) => {
    if (!getSimVar('L:A32NX_ADIRS_PFD_ALIGNED_FIRST', 'Bool')) {
        return (
            <>
                <path id="AltTapeOutline" className="NormalStroke Red" d="m117.75 123.56h13.096v-85.473h-13.096" />
                <path id="AltReadoutBackground" className="BlackFill" d="m131.35 85.308h-13.63v-8.9706h13.63z" />
                <text id="AltFailText" className="Blink9Seconds FontLargest Red EndAlign" x="131.16769" y="83.433167">ALT</text>
            </>
        );
    }

    return (
        <g>
            <path id="AltTapeOutline" className="NormalStroke White" d="m117.75 123.56h17.83m-4.7345-85.473v85.473m-13.096-85.473h17.83" />
            <LinearDeviationIndicator alt={altitude} linearDeviation={NaN} />
            <SelectedAltIndicator currentAlt={altitude} targetAlt={targetAlt} altIsManaged={altIsManaged} mode={mode} />
            <AltimeterIndicator mode={mode} />
            <MetricAltIndicator altitude={altitude} MDA={MDA} targetAlt={targetAlt} altIsManaged={altIsManaged} />
            <path id="AltReadoutBackground" className="BlackFill" d="m130.85 85.308h-13.13v-8.9706h13.13v-2.671h8.8647v14.313h-8.8647z" />
            <RadioAltIndicator radioAlt={radioAlt} />
            <DigitalAltitudeReadout altitude={altitude} MDA={MDA} />
        </g>
    );
};

const AltTapeBackground = () => (
    <path id="AltTapeBackground" d="m130.85 123.56h-13.096v-85.473h13.096z" className="TapeBackground" />
);

const SelectedAltIndicator = ({ currentAlt, targetAlt, altIsManaged, mode }) => {
    const color = altIsManaged ? 'Magenta' : 'Cyan';

    const isSTD = mode === 'STD';
    let boxLength = 19.14;
    let text = '';
    if (isSTD) {
        text = Math.round(targetAlt / 100).toString().padStart(3, '0');
        boxLength = 12.5;
    } else {
        text = Math.round(targetAlt).toString().padStart(5, ' ');
    }

    if (currentAlt - targetAlt > DisplayRange) {
        return (
            <g id="SelectedAltLowerGroup">
                <text id="SelectedAltLowerText" className={`FontMedium EndAlign ${color}`} x="135.41222" y="128.90233" xmlSpace="preserve">{text}</text>
                {isSTD
                && <text id="SelectedAltLowerFLText" className={`FontSmall MiddleAlign ${color}`} x="120.83108" y="128.97597">FL</text>}
            </g>
        );
    } if (currentAlt - targetAlt < -DisplayRange) {
        return (
            <g id="SelectedAltUpperGroup">
                <text id="SelectedAltUpperText" className={`FontMedium EndAlign ${color}`} x="135.41232" y="37.348804" xmlSpace="preserve">{text}</text>
                {isSTD
                && <text id="SelectedAltUpperFLText" className={`FontSmall MiddleAlign ${color}`} x="120.83106" y="37.337193">FL</text>}
            </g>
        );
    }
    const offset = (currentAlt - targetAlt) * DistanceSpacing / ValueSpacing;

    return (
        <g id="AltTapeTargetSymbol" transform={`translate(0 ${offset})`}>
            <path className="BlackFill" d={`m117.75 77.784h${boxLength}v6.0476h-${boxLength}z`} />
            <path className={`NormalStroke ${color}`} d="m122.79 83.831v6.5516h-7.0514v-8.5675l2.0147-1.0079m4.8441-3.0238v-6.5516h-6.8588v8.5675l2.0147 1.0079" />
            <text id="AltTapeTargetText" className={`FontMedium StartAlign ${color}`} x="118.12846" y="82.867332" xmlSpace="preserve">{text}</text>
        </g>
    );
};

const LinearDeviationIndicator = ({ linearDeviation, alt }) => {
    if (Number.isNaN(linearDeviation)) {
        return null;
    }
    const circleRadius = 30;
    if (alt - linearDeviation > DisplayRange - circleRadius) {
        return (
            <path id="VDevDotLower" className="Fill Green" d="m116.24 121.85c4.9e-4 0.83465 0.67686 1.511 1.511 1.511 0.83418 0 1.5105-0.67636 1.511-1.511h-1.511z" />
        );
    } if (alt - linearDeviation < -DisplayRange + circleRadius) {
        return (
            <path id="VDevDotUpper" className="Fill Green" d="m116.24 39.8c4.9e-4 -0.83466 0.67686-1.511 1.511-1.511 0.83418 0 1.5105 0.67635 1.511 1.511h-1.511z" />
        );
    }
    const offset = (alt - linearDeviation) * DistanceSpacing / ValueSpacing;

    return (
        <path id="VDevDot" className="Fill Green" transform={`translate(0 ${offset})`} d="m119.26 80.796a1.511 1.5119 0 1 0-3.022 0 1.511 1.5119 0 1 0 3.022 0z" />
    );
};

const AltimeterIndicator = ({ mode }) => {
    if (mode === 'STD') {
        return (
            <g id="STDAltimeterModeGroup">
                <path className="NormalStroke Yellow" d="m124.79 131.74h13.096v7.0556h-13.096z" />
                <text className="FontMedium Cyan AlignLeft" x="125.99706" y="137.20053">STD</text>
            </g>
        );
    }

    const units = Simplane.getPressureSelectedUnits();
    const pressure = Simplane.getPressureValue(units);
    let text;
    if (units === 'millibar') {
        text = Math.round(pressure).toString();
    } else {
        text = pressure.toFixed(2);
    }

    return (
        <g id="AltimeterGroup">
            {mode === 'QFE'
            && <path className="NormalStroke White" d="m 116.83686,133.0668 h 13.93811 v 5.8933 h -13.93811 z" />}
            <text id="AltimeterModeText" className="FontMedium White" x="118.29047" y="138.03368">{mode}</text>
            <text id="AltimeterSettingText" className="FontMedium MiddleAlign Cyan" x="140.86115" y="138.03368">{text}</text>
        </g>
    );
};

const MetricAltIndicator = ({ altitude, MDA, targetAlt, altIsManaged }) => {
    const currentMetricAlt = Math.round(altitude * 0.3048 / 10) * 10;

    const targetMetric = Math.round(targetAlt * 0.3048 / 10) * 10;
    const targetAltColor = altIsManaged ? 'Magenta' : 'Cyan';

    const currentMetricAltColor = altitude > MDA ? 'Green' : 'Amber';

    const showMetricAlt = getSimVar('L:A32NX_METRIC_ALT_TOGGLE', 'bool');
    if (!showMetricAlt) {
        return null;
    }

    return (
        <g id="MetricAltGroup">
            <path className="NormalStroke Yellow" d="m116.56 140.22h29.213v7.0556h-29.213z" />
            <text className="FontMedium Cyan MiddleAlign" x="141.78165" y="145.69975">M</text>
            <text id="MetricAltText" className={`FontMedium ${currentMetricAltColor} MiddleAlign`} x="128.23189" y="145.80269">{currentMetricAlt}</text>
            <g id="MetricAltTargetGroup">
                <text id="MetricAltTargetText" className={`FontSmallest ${targetAltColor} MiddleAlign`} x="93.670235" y="37.946552">{targetMetric}</text>
                <text className="FontSmallest Cyan MiddleAlign" x="105.15807" y="37.872921">M</text>
            </g>
        </g>
    );
};
