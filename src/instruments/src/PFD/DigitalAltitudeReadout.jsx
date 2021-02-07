const TensDigits = (value, offset, color) => {
    let text;
    if (value < 0) {
        text = (value + 100).toString();
    } else if (value >= 100) {
        text = (value - 100).toString().padEnd(2, '0');
    } else {
        text = value.toString().padEnd(2, '0');
    }

    return (
        <text transform={`translate(0 ${offset})`} className={`FontSmallest MiddleAlign ${color}`} x="4.3894" y="8.9133">{text}</text>
    );
};

const HundredsDigit = (value, offset, color) => {
    let text;
    if (value < 0) {
        text = (value + 1).toString();
    } else if (value >= 10) {
        text = (value - 10).toString();
    } else {
        text = value.toString();
    }

    return (
        <text transform={`translate(0 ${offset})`} className={`FontLargest MiddleAlign ${color}`} x="11.431" y="7.1">{text}</text>
    );
};
const ThousandsDigit = (value, offset, color) => {
    let text;
    if (!Number.isNaN(value)) {
        text = (value % 10).toString();
    } else {
        text = '';
    }
    return (
        <text transform={`translate(0 ${offset})`} className={`FontLargest MiddleAlign ${color}`} x="6.98" y="7.1">{text}</text>
    );
};
const TenThousandsDigit = (value, offset, color) => {
    let text;
    if (!Number.isNaN(value)) {
        text = value.toString();
    } else {
        text = '';
    }
    return (
        <text transform={`translate(0 ${offset})`} className={`FontLargest MiddleAlign ${color}`} x="2.298" y="7.1">{text}</text>
    );
};

export const DigitalAltitudeReadout = ({ altitude, MDA }) => {
    const isNegative = altitude < 0;

    const color = (MDA !== 0 && altitude < MDA) ? 'Amber' : 'Green';

    const absAlt = Math.abs(Math.max(Math.min(altitude, 50000), -1500));
    const tensDigits = absAlt % 100;

    const HundredsValue = Math.floor((absAlt / 100) % 10);
    let HundredsPosition = 0;
    if (tensDigits > 80) {
        HundredsPosition = tensDigits / 20 - 4;
    }

    const ThousandsValue = Math.floor((absAlt / 1000) % 10);
    let ThousandsPosition = 0;
    if (HundredsValue >= 9) {
        ThousandsPosition = HundredsPosition;
    }

    const TenThousandsValue = Math.floor((absAlt / 10000) % 10);
    let TenThousandsPosition = 0;
    if (ThousandsValue >= 9) {
        TenThousandsPosition = ThousandsPosition;
    }

    const showThousandsZero = TenThousandsValue !== 0;

    return (
        <g id="AltReadoutGroup">
            <g>
                <svg x="117.754" y="76.3374" width="13.5" height="8.9706" viewBox="0 0 13.5 8.9706">
                    <Drum position={TenThousandsPosition} value={TenThousandsValue} color={color} showZero={false} elementFunction={TenThousandsDigit} valueSpacing={1} distanceSpacing={7} displayRange={1} />
                    <Drum position={ThousandsPosition} value={ThousandsValue} color={color} showZero={showThousandsZero} elementFunction={ThousandsDigit} valueSpacing={1} distanceSpacing={7} displayRange={1} />
                    <Drum position={HundredsPosition} value={HundredsValue} color={color} elementFunction={HundredsDigit} valueSpacing={1} distanceSpacing={7} displayRange={1} />
                </svg>
                <svg x="130.85" y="73.6664" width="8.8647" height="14.313" viewBox="0 0 8.8647 14.313">
                    <Drum position={tensDigits} value={tensDigits} color={color} elementFunction={TensDigits} valueSpacing={20} distanceSpacing={4.7} displayRange={40} />
                </svg>
            </g>
            <path id="AltReadoutReducedAccurMarks" className="NormalStroke Amber" style={{ display: 'none' }} d="m132.61 81.669h4.7345m-4.7345-1.6933h4.7345" />
            <path id="AltReadoutOutline" className="NormalStroke Yellow" d="m117.75 76.337h13.096v-2.671h8.8647v14.313h-8.8647v-2.671h-13.096" />
            {isNegative
                && (
                    <g id="AltNegativeText" className="FontLarge EndAlign">
                        <text className="White" x="121.46731" y="78.156288">N</text>
                        <text className="White" x="121.49069" y="83.301224">E</text>
                        <text className="White" x="121.46731" y="88.446159">G</text>
                    </g>
                )}
        </g>
    );
};

const Drum = ({
    displayRange, valueSpacing, distanceSpacing, position, value, color, elementFunction, showZero = true,
}) => {
    const numTicks = Math.round(displayRange * 2 / valueSpacing);

    let highestPosition = Math.round((position + displayRange) / valueSpacing) * valueSpacing;
    if (highestPosition > position + displayRange) {
        highestPosition -= valueSpacing;
    }

    let highestValue = Math.round((value + displayRange) / valueSpacing) * valueSpacing;
    if (highestValue > value + displayRange) {
        highestValue -= valueSpacing;
    }

    const graduationElements = [];

    for (let i = 0; i < numTicks; i++) {
        const elementPosition = highestPosition - i * valueSpacing;
        const offset = -elementPosition * distanceSpacing / valueSpacing;

        let elementVal = highestValue - i * valueSpacing;
        if (!showZero && elementVal === 0) {
            elementVal = NaN;
        }

        graduationElements.push(elementFunction(elementVal, offset, color));
    }

    return (
        <g transform={`translate(0 ${position * distanceSpacing / valueSpacing})`}>
            {graduationElements}
        </g>
    );
};
