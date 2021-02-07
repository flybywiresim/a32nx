import { getSimVar } from '../util.js';
import { LagFilter } from './PFDUtils.jsx';

const filterLocalizerIndicator = new LagFilter(1.5);
const filterGlideslopeIndicator = new LagFilter(1.5);

export function LandingSystem({ LSButtonPressed, deltaTime }) {
    let showVDev = false;

    if (!LSButtonPressed) {
        showVDev = Simplane.getAutoPilotApproachLoaded() && Simplane.getAutoPilotApproachType() === 10;
    }

    return (
        <g id="LSAndDeviationGroup">
            <LandingSystemInfo displayed={LSButtonPressed} />
            {LSButtonPressed && (
                <g id="LSGroup">
                    <LocalizerIndicator deltaTime={deltaTime} />
                    <GlideslopeIndicator deltaTime={deltaTime} />
                    <MarkerBeaconIndicator />
                </g>
            )}
            {showVDev && (
                <g id="DeviationGroup">
                    <VDevIndicator />
                    <LDevIndicator />
                </g>
            )}
        </g>
    );
}

const LandingSystemInfo = ({ displayed }) => {
    if (!displayed || !getSimVar('NAV HAS LOCALIZER:3', 'Bool')) {
        return null;
    }

    // normally the ident and freq should be always displayed when an ILS freq is set, but currently it only show when we have a signal
    const identText = getSimVar('NAV IDENT:3', 'string');

    const freqTextSplit = (Math.round(getSimVar('NAV FREQUENCY:3', 'MHz') * 1000) / 1000).toString().split('.');
    const freqTextLeading = freqTextSplit[0];
    const freqTextTrailing = freqTextSplit[1].padEnd(2, '0');

    const hasDME = getSimVar('NAV HAS DME:3', 'Bool');

    let distLeading = '';
    let distTrailing = '';
    if (hasDME) {
        const dist = Math.round(getSimVar('NAV DME:3', 'nautical miles') * 10) / 10;

        if (dist < 20) {
            const distSplit = dist.toString().split('.');

            distLeading = distSplit[0];
            distTrailing = `.${distSplit.length > 1 ? distSplit[1] : '0'}`;
        } else {
            distLeading = Math.round(dist).toString();
            distTrailing = '';
        }
    }

    return (
        <g id="LSInfoGroup">
            <text id="ILSIdent" className="Magenta FontLarge AlignLeft" x="1.5204413" y="145.11539">{identText}</text>
            <text id="ILSFreqLeading" className="Magenta FontLarge AlignLeft" x="1.0791086" y="151.14395">{freqTextLeading}</text>
            <text id="ILSFreqTrailing" className="Magenta FontSmallest AlignLeft" x="14.297637" y="151.26903">{`.${freqTextTrailing}`}</text>
            {hasDME && (
                <g id="ILSDistGroup">
                    <text className="Magenta AlignLeft" x="0.69199055" y="157.26602">
                        <tspan id="ILSDistLeading" className="FontLarge">{distLeading}</tspan>
                        <tspan id="ILSDistTrailing" className="FontSmallest">{distTrailing}</tspan>
                    </text>
                    <text className="Cyan FontSmallest AlignLeft" x="17.303238" y="157.2491">NM</text>
                </g>
            )}
        </g>
    );
};

const LocalizerIndicator = ({ deltaTime }) => {
    const hasLoc = getSimVar('NAV HAS LOCALIZER:3', 'Bool');

    let diamond = null;

    if (hasLoc) {
        const deviation = filterLocalizerIndicator.step(getSimVar('NAV RADIAL ERROR:3', 'degrees'), deltaTime / 1000);
        const dots = deviation / 0.8;

        if (dots > 2) {
            diamond = <path id="LocDiamondRight" className="NormalStroke Magenta" d="m99.127 133.03 3.7776-2.5198-3.7776-2.5198" />;
        } else if (dots < -2) {
            diamond = <path id="LocDiamondLeft" className="NormalStroke Magenta" d="m38.686 133.03-3.7776-2.5198 3.7776-2.5198" />;
        } else {
            diamond = <path id="LocDiamond" className="NormalStroke Magenta" transform={`translate(${dots * 30.221 / 2} 0)`} d="m65.129 130.51 3.7776 2.5198 3.7776-2.5198-3.7776-2.5198z" />;
        }
    } else {
        filterLocalizerIndicator.reset();
    }

    return (
        <g id="LocalizerSymbolsGroup">
            <path className="NormalStroke White" d="m54.804 130.51a1.0073 1.0079 0 1 0-2.0147 0 1.0073 1.0079 0 1 0 2.0147 0z" />
            <path className="NormalStroke White" d="m39.693 130.51a1.0074 1.0079 0 1 0-2.0147 0 1.0074 1.0079 0 1 0 2.0147 0z" />
            <path className="NormalStroke White" d="m85.024 130.51a1.0073 1.0079 0 1 0-2.0147 0 1.0073 1.0079 0 1 0 2.0147 0z" />
            <path className="NormalStroke White" d="m100.13 130.51a1.0074 1.0079 0 1 0-2.0147 0 1.0074 1.0079 0 1 0 2.0147 0z" />
            {diamond}
            <path id="LocalizerNeutralLine" className="Yellow Fill" d="m68.098 134.5v-8.0635h1.5119v8.0635z" />
        </g>
    );
};

const GlideslopeIndicator = ({ deltaTime }) => {
    const hasGlideslope = getSimVar('NAV HAS GLIDE SLOPE:3', 'Bool');

    let diamond = null;

    if (hasGlideslope) {
        const deviation = filterGlideslopeIndicator.step(getSimVar('NAV GLIDE SLOPE ERROR:3', 'degrees'), deltaTime / 1000);
        const dots = deviation / 0.4;

        if (dots > 2) {
            diamond = <path id="GlideSlopeDiamondLower" className="NormalStroke Magenta" d="m107.19 111.06 2.5184 3.7798 2.5184-3.7798" />;
        } else if (dots < -2) {
            diamond = <path id="GlideSlopeDiamondUpper" className="NormalStroke Magenta" d="m107.19 50.585 2.5184-3.7798 2.5184 3.7798" />;
        } else {
            diamond = <path id="GlideSlopeDiamond" className="NormalStroke Magenta" transform={`translate(0 ${dots * 30.238 / 2})`} d="m109.7 77.043-2.5184 3.7798 2.5184 3.7798 2.5184-3.7798z" />;
        }
    } else {
        filterGlideslopeIndicator.reset();
    }

    return (
        <g id="GlideslopeSymbolsGroup">
            <path className="NormalStroke White" d="m110.71 50.585a1.0074 1.0079 0 1 0-2.0147 0 1.0074 1.0079 0 1 0 2.0147 0z" />
            <path className="NormalStroke White" d="m110.71 65.704a1.0074 1.0079 0 1 0-2.0147 0 1.0074 1.0079 0 1 0 2.0147 0z" />
            <path className="NormalStroke White" d="m110.71 95.942a1.0074 1.0079 0 1 0-2.0147 0 1.0074 1.0079 0 1 0 2.0147 0z" />
            <path className="NormalStroke White" d="m110.71 111.06a1.0074 1.0079 0 1 0-2.0147 0 1.0074 1.0079 0 1 0 2.0147 0z" />
            {diamond}
        </g>
    );
};

const VDevIndicator = () => {
    const deviation = getSimVar('GPS VERTICAL ERROR', 'feet');
    const dots = deviation / 100;

    let diamond;

    if (dots > 2) {
        diamond = <path id="VDevSymbolLower" className="NormalStroke Green" d="m107.19 111.06v2.0159h5.0368v-2.0159" />;
    } else if (dots < -2) {
        diamond = <path id="VDevSymbolUpper" className="NormalStroke Green" d="m107.19 50.585v-2.0159h5.0368v2.0159" />;
    } else {
        diamond = <path id="VDevSymbol" className="NormalStroke Green" transform={`translate(0 ${dots * 30.238 / 2})`} d="m112.22 78.807h-5.0368v4.0318h5.0368v-2.0159z" />;
    }

    return (
        <g id="VertDevSymbolsGroup">
            <text className="FontSmall AlignRight Green" x="95.022" y="43.126">V/DEV</text>
            <path className="NormalStroke White" d="m108.7 65.704h2.0147" />
            <path className="NormalStroke White" d="m108.7 50.585h2.0147" />
            <path className="NormalStroke White" d="m108.7 111.06h2.0147" />
            <path className="NormalStroke White" d="m108.7 95.942h2.0147" />
            {diamond}
        </g>
    );
};

// Not implemented on the FMS side I think, so this is just static and hidden for now
const LDevIndicator = () => (
    <g id="LatDeviationSymbolsGroup" style={{ display: 'none' }}>
        <text className="FontSmall AlignRight Green" x="30.888" y="122.639">L/DEV</text>
        <path className="NormalStroke White" d="m38.686 129.51v2.0158" />
        <path className="NormalStroke White" d="m53.796 129.51v2.0158" />
        <path className="NormalStroke White" d="m84.017 129.51v2.0158" />
        <path className="NormalStroke White" d="m99.127 129.51v2.0158" />
        <path id="LDevSymbolLeft" className="NormalStroke Green" d="m38.686 127.99h-2.0147v5.0397h2.0147" />
        <path id="LDevSymbolRight" className="NormalStroke Green" d="m99.127 127.99h2.0147v5.0397h-2.0147" />
        <path id="LDevSymbol" className="NormalStroke Green" d="m66.892 127.99v5.0397h4.0294v-5.0397h-2.0147z" />
    </g>
);

const MarkerBeaconIndicator = () => {
    const markerState = getSimVar('MARKER BEACON STATE', 'Enum');

    let classNames = '';
    let markerText = '';

    if (markerState === 0) {
        return null;
    } if (markerState === 1) {
        classNames = 'Cyan OuterMarkerBlink';
        markerText = 'OM';
    } else if (markerState === 2) {
        classNames = 'Amber MiddleMarkerBlink';
        markerText = 'MM';
    } else {
        classNames = 'White InnerMarkerBlink';
        markerText = 'IM';
    }

    return (
        <text id="ILSMarkerText" className={`FontLarge StartAlign ${classNames}`} x="98.339211" y="125.12898">{markerText}</text>
    );
};
