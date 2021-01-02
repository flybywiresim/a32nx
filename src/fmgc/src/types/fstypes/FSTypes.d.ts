/*
 * A32NX
 * Copyright (C) 2020-2021 FlyByWire Simulations and its contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/// <reference path="../../../../../typings/fs-base-ui/html_ui/JS/simvar.d.ts" />

declare class WayPoint {
    constructor(_baseInstrument: BaseInstrument);

    icao: string;
    ident: string;
    endsInDiscontinuity?: boolean;
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

declare class AirportInfo extends WayPointInfo {
    constructor(_instrument: BaseInstrument);

    frequencies: any[];
    namedFrequencies: any[];
    departures: any[];
    approaches: any[];
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
