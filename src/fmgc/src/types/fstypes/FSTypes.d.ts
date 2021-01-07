/*
 * MIT License
 *
 * Copyright (c) 2020-2021 Working Title, FlyByWire Simulations
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/// <reference path="../../../../../typings/fs-base-ui/html_ui/JS/simvar.d.ts" />

declare class WayPoint {
    constructor(_baseInstrument: BaseInstrument);

    icao: string;
    ident: string;
    endsInDiscontinuity?: boolean;
    discontinuityCanBeCleared?: boolean;
    isVectors?: boolean;
    isRunway?: boolean;
    infos: WayPointInfo;
    type: string;
    bearingInFP: number;
    distanceInFP: number;
    cumulativeDistanceInFP: number;
    instrument: BaseInstrument;
    altDesc: number;
    altitude1: number;
    altitude2: number;
    legAltitudeDescription: number;
    legAltitude1: number;
    legAltitude2: number;
    additionalData: { [key: string]: any }
    _svgElements: any;
}

declare class BaseInstrument {
    addEventListener(type: string, listener: EventListenerOrEventListenerObject, options?: boolean | AddEventListenerOptions): void;

    facilityLoader: FacilityLoader;
    instrumentIdentifier: string;
}

declare class NavSystem {
}

declare class FacilityLoader {
    getFacilityRaw(icao: string, timeout?: number): Promise<any>;
}

declare class WayPointInfo {
    constructor(_instrument: BaseInstrument);

    coordinates: LatLongAlt;
    icao: string;
    ident: string;
    airwayIn: string;
    airwayOut: string;
    routes: any[];
    instrument: BaseInstrument;
    magneticVariation?: number;
    _svgElements: any;

    UpdateInfos(_CallBack?, loadFacilitiesTransitively?);

    CopyBaseInfosFrom(_WP: WayPoint);
}

interface RawApproachTransitionData {
    legs: {}[],
    name: string,
    __Type: "JS_ApproachTransition",
}

interface RawApproachData {
    finalLegs: {}[],
    missedLegs: {}[],
    icaos: any[],
    index: number,
    name?: string,
    runway: string,
    transitions: RawApproachTransitionData[],
    __Type: "JS_Approach",
}

declare class AirportInfo extends WayPointInfo {
    constructor(_instrument: BaseInstrument);

    frequencies: any[];
    namedFrequencies: any[];
    departures: any[];
    approaches: RawApproachData[];
    arrivals: any[];
    runways: any[];
    oneWayRunways: OneWayRunway[];

    UpdateNamedFrequencies(icao?: string): Promise<void>
}

declare class IntersectionInfo extends WayPointInfo {
    constructor(_instrument: BaseInstrument);
}

declare class VORInfo extends WayPointInfo {
    constructor(_instrument: BaseInstrument);
}

declare class NDBInfo extends WayPointInfo {
    constructor(_instrument: BaseInstrument);
}

declare interface OneWayRunway {
    designation: string;
    direction: number;
    beginningCoordinates: LatLongAlt;
    endCoordinates: LatLongAlt;
    elevation: number;
}

declare interface RunwayTransition {
    runwayNumber: number;
    runwayDesignation: number;
}

declare interface EnrouteTransition {
    legs: ProcedureLeg[];
}

declare class Runway {
}

declare class Avionics {
    static Utils: Utils;
}

declare class Utils {
    computeGreatCircleHeading(coords1: LatLongAlt, coords2: LatLongAlt): number;

    computeGreatCircleDistance(coords1: LatLongAlt, coords2: LatLongAlt): number;

    bearingDistanceToCoordinates(bearing: number, distanceInNM: number, lat: number, long: number): LatLongAlt;

    fmod(value: number, moduloBy: number): number;

    computeDistance(coords1: LatLongAlt, coords2: LatLongAlt);

    angleDiff(degrees1: number, degrees2: number);

    lerpAngle(from: number, to: number, d: number);

    DEG2RAD: number;
    RAD2DEG: number;
}

declare interface ProcedureLeg {
    type: number;
    fixIcao: string;
    originIcao: string;
    altDesc: number;
    altitude1: number;
    altitude2: number;
    course: number;
    distance: number;
    rho: number;
    theta: number;
}


declare class EmptyCallback {
    static Void: () => void;
    static Boolean: (boolean) => void;
}

declare class Coherent {
    static call(handler: string, ...params: any[]): Promise<any>
}

declare function RegisterViewListener(handler: string): void
