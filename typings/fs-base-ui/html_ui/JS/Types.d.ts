import {Feet, Latitude, Longitude} from "../../../types";

declare global {
    class LatLong {
        constructor(latitude: Latitude, longitude: Longitude);
        constructor(data: { lat: Latitude, long: Longitude });

        lat: Latitude;
        long: Longitude;

        /**
         * @returns A string formatted as "40.471000, 73.580000".
         */
        toStringFloat(): string;

        /**
         * @returns A string formatted as "lat 40.47, long 73.58".
         */
        toString(): string;

        /**
         * @returns A string formatted as "4047.1N".
         */
        latToDegreeString(): string;

        /**
         * @returns A string formatted as "07358.0W".
         */
        longToDegreeString(): string;

        /**
         * @returns A string formatted as "N40°47.1 W073°58.0".
         */
        toDegreeString(): string;

        /**
         * @returns A string formatted as "4047.1N07358.0W".
         */
        toShortDegreeString(): string;

        /**
         * Parses a string into a LatLong or LatLongAlt
         * @param str A string formatted as either "0.0,0.0" or "0.0,0.0,0.0".
         * When the string contains spaces around the numbers, those are ignored.
         * @returns An instance of LatLong or LatLongAlt, depending on the number of values in the string. Null when
         * the string doesn't contain a comma.
         */
        static fromStringFloat(str: string): LatLong | LatLongAlt | null;
    }

    class LatLongAlt {
        constructor();
        constructor(latitude: Latitude, longitude: Longitude);
        constructor(latitude: Latitude, longitude: Longitude, alt: Feet);
        constructor(data: { lat: Latitude, long: Longitude, alt: Feet });

        lat: Latitude;
        long: Longitude;
        alt: Feet;

        /**
         * @returns A LatLong instance containing the latitude and longitude of this instance.
         */
        toLatLong(): LatLong;

        /**
         * @returns A string formatted as "52.370216, 4.895168, 1500.0".
         */
        toStringFloat(): string;

        /**
         * @returns A string formatted as "lat 52.37, long 4.90, alt 1500.00".
         */
        toString(): string;

        /**
         * @returns A string formatted as "5222.2N".
         */
        latToDegreeString(): string;

        /**
         * @returns A string formatted as "00453.7E".
         */
        longToDegreeString(): string;

        /**
         * @returns A string formatted as "N52°22.2 E004°53.7".
         */
        toDegreeString(): string;
    }

    class PitchBankHeading {
        constructor(data: { pitchDegree: number, bankDegree: number, headingDegree: number });

        pitchDegree: number;
        bankDegree: number;
        headingDegree: number;

        /**
         * @returns A string formatted as "p 2.40, b 10.60, h 240.20".
         */
        toString(): string;
    }

    class LatLongAltPBH {
        constructor(data: { lla: LatLongAlt, pbh: PitchBankHeading });

        lla: LatLongAlt;
        pbh: PitchBankHeading;

        /**
         * @returns A string formatted as "lla lat 52.37, long 4.90, alt 1500.00, pbh p 2.40, b 10.60, h 240.20".
         */
        toString(): string;
    }

    class PID_STRUCT {
        constructor(data: { pid_p: number; pid_i: number; pid_i2: number; pid_d: number;
            i_boundary: number; i2_boundary: number; d_boundary: number; });

        pid_p: number;
        pid_i: number;
        pid_i2: number;
        pid_d: number;
        i_boundary: number;
        i2_boundary: number;
        d_boundary: number;

        /**
         * @returns A string formatted as "pid_p 123.46, pid_i 123.46, pid_i2 123.46, pid_d 123.46, i_boundary 123.46, i2_boundary 123.46, d_boundary 123.46".
         */
        toString(): string;
    }

    class XYZ {
        constructor(data: { x: number; y: number; z: number; });

        x: number;
        y: number;
        z: number;

        /**
         * @returns A string formatted as "x 123.00, y 456.00, z 789.13".
         */
        toString(): string;
    }

    class DataDictionaryEntry {
        constructor(data: { key: any, data: any });

        /**
         * @returns A string formatted as "key: " + key + ", data: " + data
         */
        toString(): string;
    }

    class POIInfo {
        constructor(data: { distance: any, angle: any, isSelected: any });

        /**
         * @returns A string formatted as "Distance: " + distance + ", angle: " + angle + ", selected: " + isSelected
         */
        toString(): string;
    }

    class KeyActionParams {
        /**
         * Parses the JSON string and sets the properties on the newly created instance.
         */
        constructor(json: string | null);

        bReversed: boolean;
        static sKeyDelimiter: string;
    }

    /**
     * The Simvar class is not to be confused with the SimVar namespace.
     */
    class Simvar {
    }

    class Attribute {
    }
}

export {};
