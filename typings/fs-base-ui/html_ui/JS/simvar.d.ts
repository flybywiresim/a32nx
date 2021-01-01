/// <reference path="./Types.d.ts" />
/// <reference path="./Simplane.d.ts" />
/// <reference path="../../../types.d.ts" />

type NumberVarUnit = ("number" | "Number") | "position 32k" | ("SINT32") | ("bool" | "Bool" | "Boolean" | "boolean") | "Enum" | "lbs" | "kg" | ("Degrees" | "degree" | "Radians")
    | "radians" | ("Percent" | "percent") | ("Feet" | "feet" | "feets" | "Feets") | "Volts" | "Amperes" | "Hertz" | "PSI" | "celsius" | "degree latitude"
    | "degree longitude" | "Meters per second" | "Position" | ("Knots" | "knots") | "Seconds" | "seconds" | "kilograms per second" | "nautical miles" | "degrees"

type TextVarUnit = "Text" | "string"

type VarUnit = NumberVarUnit | TextVarUnit | LatLongAltVarUnit | LatLongAltPBHVarUnit
    | PitchBankHeadingVarUnit | PID_STRUCTVarUnit | XYZVarUnit;

type LatLongAltVarUnit = "latlonalt";
type LatLongAltPBHVarUnit = "latlonaltpbh";
type PitchBankHeadingVarUnit = "pbh";
type PID_STRUCTVarUnit = "pid_struct";
type XYZVarUnit = "xyz"
type POIListVarUnit = "poilist";
type FuelLevelsVarUnit = "FuelLevels";
type GlassCockpitSettingsVarUnit = "GlassCockpitSettings";

type SimVarBatchType = "string" | "number";

declare global {
    namespace SimVar {
        class SimVarValue {
            constructor(name?: string, unit?: VarUnit, type?: string);
            name?: string;
            unit?: string;
            type?: string;
        }

        class SimVarBatch {
            constructor(simVarCount: string, simVarIndex: string);
            add(name: string, unit: NumberVarUnit | TextVarUnit, type?: SimVarBatchType): void;
            getCount(): string;
            getIndex(): string;
            getNames(): string[];
            getUnits(): (NumberVarUnit | TextVarUnit)[];
            getTypes(): SimVarBatchType[];
        }

        /**
         * Determines if SimVar is ready.
         * @returns Something when ready, null when not ready.
         */
        function IsReady(): any | null;

        /**
         * When the SimVarValueHistory is stored (disabled by default and currently not exposed),
         * logs per SimVar (slowest per number of invocations first):
         * - The amount of invocations.
         * - The average invocation time in milliseconds.
         * - The total invocation time in milliseconds.
         * - The slowest invocation time in milliseconds.
         * - The average invocation time per frame in milliseconds.
         * - The average amount of invocations per frame.
         *
         *  Also logs the total invokes per frame and total time in milliseconds taken per frame.
         */
        function LogSimVarValueHistory(): void;

        /**
         * When the SimVarValueHistory is stored (disabled by default and currently not exposed),
         * logs per SimVar (slowest per frame first):
         * - The amount of invocations.
         * - The average invocation time in milliseconds.
         * - The total invocation time in milliseconds.
         * - The slowest invocation time in milliseconds.
         * - The average invocation time per frame in milliseconds.
         * - The average amount of invocations per frame.
         *
         *  Also logs the total invokes per frame and total time in milliseconds taken per frame.
         */
        function LogSimVarValueHistoryByTimePerFrame(): void;

        function GetSimVarArrayValues(batch: SimVarBatch, callback: (results: any[][]) => void, dataSource?: string): void;

        function GetSimVarValue(name: string, unit: NumberVarUnit, dataSource?: string): number | null;
        function GetSimVarValue(name: string, unit: TextVarUnit, dataSource?: string): string | null;
        function GetSimVarValue(name: string, unit: LatLongAltVarUnit, dataSource?: string): LatLongAlt | null;
        function GetSimVarValue(name: string, unit: LatLongAltPBHVarUnit, dataSource?: string): LatLongAltPBH | null;
        function GetSimVarValue(name: string, unit: PitchBankHeadingVarUnit, dataSource?: string): PitchBankHeading | null;
        function GetSimVarValue(name: string, unit: PID_STRUCTVarUnit, dataSource?: string): PID_STRUCT | null;
        function GetSimVarValue(name: string, unit: XYZVarUnit, dataSource?: string): XYZ | null;

        function SetSimVarValue(name: string, unit: NumberVarUnit, value: number, dataSource?: string): Promise<void>;
        function SetSimVarValue(name: string, unit: TextVarUnit, value: string, dataSource?: string): Promise<void>;
        function SetSimVarValue(name: string, unit: LatLongAltVarUnit, value: LatLongAlt, dataSource?: string): Promise<void>;
        function SetSimVarValue(name: string, unit: LatLongAltPBHVarUnit, value: LatLongAltPBH, dataSource?: string): Promise<void>;
        function SetSimVarValue(name: string, unit: PitchBankHeadingVarUnit, value: PitchBankHeading, dataSource?: string): Promise<void>;
        function SetSimVarValue(name: string, unit: PID_STRUCTVarUnit, value: PID_STRUCT, dataSource?: string): Promise<void>;
        function SetSimVarValue(name: string, unit: XYZVarUnit, value: XYZ, dataSource?: string): Promise<void>;

        function GetGlobalVarValue(name: string, unit: string): any | null;

        function GetGameVarValue(name: string, unit: NumberVarUnit, param1?: number, param2?: number): number | null;
        function GetGameVarValue(name: string, unit: TextVarUnit): string | null;
        function GetGameVarValue(name: string, unit: XYZVarUnit): XYZ | null;
        function GetGameVarValue(name: string, unit: POIListVarUnit): any | null;
        function GetGameVarValue(name: string, unit: FuelLevelsVarUnit): FuelLevels | null;
        function GetGameVarValue(name: string, unit: GlassCockpitSettingsVarUnit): GlassCockpitSettings | null;
        function GetGameVarValue(name: string, unit: string): any | null;

        function SetGameVarValue(name: string, unit: NumberVarUnit, value: number): Promise<void>;
        /**
         * Doesn't do anything.
         */
        function SetGameVarValue(name: string, unit: "string" | XYZVarUnit | POIListVarUnit | FuelLevelsVarUnit | GlassCockpitSettingsVarUnit, value: any): Promise<void>;
    }
}

export {};
