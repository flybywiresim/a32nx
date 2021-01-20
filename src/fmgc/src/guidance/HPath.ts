import { Degrees, NauticalMiles } from "../../../../typings/types"

interface GuidanceParameters {
    crossTrackError: NauticalMiles;
    trackAngleError: Degrees;
}

type SegmentType = "straightTrack" | "turn";

export const EARTH_RADIUS_NM = 3440.1;

/**
 * A better version of the % operator that can handle the negative wrap around.
 * @example mod(-10, 360) => 350
 */
const mod = (x: number, n: number) => x - Math.floor(x / n) * n;

export abstract class Segment {
    abstract get type(): SegmentType;

    constructor(readonly waypoint: WayPoint, public readonly from: LatLongAlt, public readonly to: LatLongAlt) {
        // pass
    }

    /**
     * Returns whether a position (usually the current position of the aircraft)
     * can be considered abeam to this segment.
     * @param ppos
     */
    abstract isAbeam(ppos: LatLongAlt): boolean;

    /**
     * Returns the absolute distance of this segment.
     */
    abstract get distance(): NauticalMiles;

    /**
     * Returns the distance to go for this segment.
     */
    abstract getDistanceToGo(ppos: LatLongAlt): NauticalMiles;

    /**
     *
     * @param ppos
     * @param trueTrack
     * @example
     * const a = SimVar.GetSimVarValue("PLANE LATITUDE", "degree latitude"),
     * const b = SimVar.GetSimVarValue("PLANE LONGITUDE", "degree longitude")
     * const ppos = new LatLongAlt(a, b);
     * const trueTrack = SimVar.GetSimVarValue("GPS GROUND TRUE TRACK", "degree");
     * calculateErrors(ppos, trueTrack);
     */
    calculateErrors(ppos: LatLongAlt, trueTrack: number): GuidanceParameters | null {
        return null;
    }

    abstract toString(): string;
}

export class StraightTrackSegment extends Segment {
    get type(): "straightTrack" {
        return "straightTrack";
    }

    get distance(): NauticalMiles {
        return Avionics.Utils.computeGreatCircleDistance(this.from, this.to);
    }

    get track(): Degrees {
        return Avionics.Utils.computeGreatCircleHeading(this.from, this.to);
    }

    isAbeam(ppos: LatLongAlt): boolean {
        const bearingAC = Avionics.Utils.computeGreatCircleHeading(this.from, ppos);
        const headingAC = Math.abs(Avionics.Utils.angleDiff(this.track, bearingAC));
        if (headingAC > 90) {
            // if we're even not abeam of the starting point
            return false;
        }
        const distanceAC = Avionics.Utils.computeDistance(this.from, ppos);
        const distanceAX = Math.cos(headingAC * Avionics.Utils.DEG2RAD) * distanceAC;
        // if we're too far away from the starting point to be still abeam of the ending point
        return distanceAX <= this.distance;
    }

    getDistanceToGo(ppos: LatLongAlt): NauticalMiles {
        // @todo find abeam point
        return Avionics.Utils.computeGreatCircleDistance(ppos, this.to);
    }

    toString(): string {
        return `Straight Track: ${this.from.toString()} => ${this.to.toString()}`;
    }

    calculateErrors(ppos: LatLongAlt, trueTrack: number): GuidanceParameters | null {
        const desiredTrack = this.track;
        const trackAngleError = mod(desiredTrack - trueTrack + 180, 360) - 180;

        // crosstrack error
        const bearingAC = Avionics.Utils.computeGreatCircleHeading(
            this.from,
            ppos
        );
        const bearingAB = desiredTrack;
        const distanceAC = Avionics.Utils.computeDistance(
            this.from,
            ppos
        );

        const desiredOffset = 0;
        const actualOffset =
            (Math.asin(
                Math.sin(Avionics.Utils.DEG2RAD * (distanceAC / EARTH_RADIUS_NM)) *
                Math.sin(Avionics.Utils.DEG2RAD * (bearingAC - bearingAB))
                ) /
                Avionics.Utils.DEG2RAD) *
            EARTH_RADIUS_NM;
        const crossTrackError = desiredOffset - actualOffset;

        return {
            trackAngleError,
            crossTrackError,
        }
    }
}

export class TurnSegment extends Segment {
    ABEAM_TOLERANCE_DEG = 1.0;

    constructor(waypoint: WayPoint, from: LatLongAlt, to: LatLongAlt, public readonly radius: number, public readonly clockwise: boolean) {
        super(waypoint, from, to);
    }

    get type(): "turn" {
        return "turn";
    }

    /**
     * Returns the center of the turning circle, in radius distance from both
     * from and to, i.e. distance(from, center) = distance(to, center) = radius.
     */
    get center(): LatLongAlt {
        const { lat: lat1, long: lon1 } = this.from;
        const x1: [number, number, number] = [
            Math.cos(lon1 * Avionics.Utils.DEG2RAD) * Math.cos(lat1 * Avionics.Utils.DEG2RAD),
            Math.sin(lon1 * Avionics.Utils.DEG2RAD) * Math.cos(lat1 * Avionics.Utils.DEG2RAD),
            Math.sin(lat1 * Avionics.Utils.DEG2RAD),
        ]

        const { lat: lat2, long: lon2 } = this.to;
        const x2: [number, number, number] = [
            Math.cos(lon2 * Avionics.Utils.DEG2RAD) * Math.cos(lat2 * Avionics.Utils.DEG2RAD),
            Math.sin(lon2 * Avionics.Utils.DEG2RAD) * Math.cos(lat2 * Avionics.Utils.DEG2RAD),
            Math.sin(lat2 * Avionics.Utils.DEG2RAD),
        ]

        const r = this.radius * Avionics.Utils.DEG2RAD * (1 / 60);

        const cross = ([ax, ay, az], [bx, by, bz]) => ([
            ay * bz - az * by,
            az * bx - ax * bz,
            ax * by - ay * bx,
        ]);
        const dot = <T extends number[]>(vec1: T, vec2: T) => vec1.reduce<number>(
            (x, _, i) => x + vec1[i] * vec2[i],
            0
        );

        const q = dot(x1, x2);
        const a = (Math.cos(r) - Math.cos(r) * q) / (1 - Math.pow(q, 2));
        const b = (Math.cos(r) - Math.cos(r) * q) / (1 - Math.pow(q, 2));
        // const a = (- Math.cos(r) * (q - 1)) / (1 - Math.pow(q, 2));
        // const b = (- Math.cos(r) * (q - 1)) / (1 - Math.pow(q, 2));

        const n = cross(x1, x2);
        const x0 = x1.map((_, i) => a * x1[i] + b * x2[i]);

        const t = Math.sqrt((1 - dot(x0, x0)) / dot(n, n));

        let point = !this.clockwise ?
            x0.map((x, i) => x + (t * n[i])) :
            x0.map((x, i) => x - (t * n[i]));

        const lon = Avionics.Utils.RAD2DEG * Math.atan2(point[1], point[0]);
        const lat = Avionics.Utils.RAD2DEG * Math.asin(point[2]);

        return new LatLongAlt(lat, lon, 0);
    }

    isAbeam(ppos: LatLongAlt): boolean {
        const center = this.center;
        const bearingFrom = Avionics.Utils.computeGreatCircleHeading(
            center,
            this.from,
        );
        const bearingPpos = Avionics.Utils.computeGreatCircleHeading(
            center,
            ppos,
        );
        const lowerBound = this.clockwise ? bearingFrom : bearingFrom - this.angle;
        const upperBound = this.clockwise ? bearingFrom + this.angle : bearingFrom;
        return bearingPpos >= lowerBound && bearingPpos <= upperBound;
    }

    toString(): string {
        return `Turn: ${this.from.toString()} @ r=${this.radius} => ${this.to.toString()}`;
    }

    get angle(): number {
        const center = this.center;
        const bearingFrom = Avionics.Utils.computeGreatCircleHeading(
            center,
            this.from,
        );
        const bearingTo = Avionics.Utils.computeGreatCircleHeading(
            center,
            this.to,
        );
        return Math.abs(Avionics.Utils.angleDiff(bearingFrom, bearingTo));
    }

    get distance(): NauticalMiles {
        const circumference = 2 * Math.PI * this.radius;
        return circumference / 360 * this.angle;
    }

    getDistanceToGo(ppos: LatLongAlt): NauticalMiles {
        //return Avionics.Utils.computeDistance(ppos, this.to.latlongalt);
        return 0;
    }

    calculateErrors(ppos: LatLongAlt, trueTrack: number): GuidanceParameters | null {
        const center = this.center;

        const bearingPpos = Avionics.Utils.computeGreatCircleHeading(
            center,
            ppos,
        );

        const desiredTrack = mod(
            this.clockwise ? bearingPpos + 90 : bearingPpos - 90,
            360
        );
        const trackAngleError = mod(desiredTrack - trueTrack, 360);

        const distanceFromCenter = Avionics.Utils.computeGreatCircleDistance(
            center,
            ppos,
        );
        const crossTrackError = this.clockwise ?
            distanceFromCenter - this.radius :
            this.radius - distanceFromCenter;

        return {
            trackAngleError,
            crossTrackError,
        }
    }
}
