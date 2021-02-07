import { getSimVar } from '../util.js';

export const VerticalSpeedIndicator = ({ radioAlt }) => {
    if (!getSimVar('L:A32NX_ADIRS_PFD_ALIGNED_FIRST', 'Bool')) {
        return (
            <>
                <path className="TapeBackground" d="m151.84 131.72 4.1301-15.623v-70.556l-4.1301-15.623h-5.5404v101.8z" />
                <g id="VSpeedFailText">
                    <text className="Blink9Seconds FontLargest Red EndAlign" x="153.13206" y="77.501472">V</text>
                    <text className="Blink9Seconds FontLargest Red EndAlign" x="153.13406" y="83.211388">/</text>
                    <text className="Blink9Seconds FontLargest Red EndAlign" x="152.99374" y="88.870819">S</text>
                </g>
            </>
        );
    }

    // This represents inertial vertical speed
    const verticalSpeed = getSimVar('VELOCITY WORLD Y', 'feet per minute');

    let isAmber = false;
    if ((Math.abs(verticalSpeed) > 6000 || (radioAlt < 2500 && radioAlt > 1000 && verticalSpeed < -2000) || (radioAlt < 1000 && verticalSpeed < -1200))) {
        isAmber = true;
    }

    const absVSpeed = Math.abs(verticalSpeed);
    const sign = Math.sign(verticalSpeed);

    let yOffset = 0;

    if (absVSpeed < 1000) {
        yOffset = verticalSpeed / 1000 * -27.22;
    } else if (absVSpeed < 2000) {
        yOffset = (verticalSpeed - sign * 1000) / 1000 * -10.1 - sign * 27.22;
    } else if (absVSpeed < 6000) {
        yOffset = (verticalSpeed - sign * 2000) / 4000 * -10.1 - sign * 37.32;
    } else {
        yOffset = sign * -47.37;
    }

    return (
        <g>
            <path className="TapeBackground" d="m151.84 131.72 4.1301-15.623v-70.556l-4.1301-15.623h-5.5404v101.8z" />
            <g id="VerticalSpeedGroup">
                <g className="Fill White">
                    <path d="m149.92 54.339v-1.4615h1.9151v1.4615z" />
                    <path d="m149.92 44.26v-1.4615h1.9151v1.4615z" />
                    <path d="m149.92 34.054v-1.2095h1.9151v1.2095z" />
                    <path d="m151.84 107.31h-1.9151v1.4615h1.9151z" />
                    <path d="m151.84 117.39h-1.9151v1.4615h1.9151z" />
                    <path d="m151.84 127.59h-1.9151v1.2095h1.9151z" />
                </g>
                <g className="NormalStroke White">
                    <path d="m149.92 67.216h1.7135" />
                    <path d="m151.84 48.569h-1.9151" />
                    <path d="m151.84 38.489h-1.9151" />
                    <path d="m149.92 94.43h1.7135" />
                    <path d="m151.84 113.08h-1.9151" />
                    <path d="m151.84 123.16h-1.9151" />
                </g>
                <g className="FontSmallest MiddleAlign Fill White">
                    <text x="148.07108" y="109.72845">1</text>
                    <text x="148.14471" y="119.8801">2</text>
                    <text x="148.07108" y="129.90607">6</text>
                    <text x="148.09711" y="55.316456">1</text>
                    <text x="148.06529" y="45.356102">2</text>
                    <text x="148.11371" y="35.195072">6</text>
                </g>
                <path className="Fill Yellow" d="m145.79 80.067h6.0476v1.5119h-6.0476z" />
                <VSpeedNeedle isAmber={isAmber} yOffset={yOffset} />
                <VSpeedText yOffset={yOffset} isAmber={isAmber} VSpeed={verticalSpeed} />
            </g>
        </g>
    );
};

const VSpeedNeedle = ({ yOffset, isAmber }) => {
    const className = `HugeStroke ${isAmber ? 'Amber' : 'Green'}`;

    return (
        <>
            <path className="HugeOutline" d={`m162.74 80.822 l -12 ${yOffset}`} />
            <path className={className} id="VSpeedIndicator" d={`m162.74 80.822 l -12 ${yOffset}`} />
        </>
    );
};

const VSpeedText = ({ VSpeed, yOffset, isAmber }) => {
    const absVSpeed = Math.abs(VSpeed);
    const sign = Math.sign(VSpeed);

    if (absVSpeed < 200) {
        return null;
    }

    const textOffset = yOffset - sign * 2.4;

    const text = (Math.round(absVSpeed / 100) < 10 ? '0' : '') + Math.round(absVSpeed / 100).toString();
    const className = `FontSmallest MiddleAlign ${isAmber ? 'Amber' : 'Green'}`;

    return (
        <g id="VSpeedTextGroup" transform={`translate(0 ${textOffset})`}>
            <path className="BackgroundFill" d="m158.4 83.011h-7.0514v-4.3989h7.0514z" />
            <text id="VSpeedText" className={className} x="154.84036" y="82.554581">{text}</text>
        </g>
    );
};
