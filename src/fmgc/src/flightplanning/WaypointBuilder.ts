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

import { FlightPlanManager } from "../wtsdk";
import { GeoMath } from "./GeoMath";

/**
 * Creating a new waypoint to be added to a flight plan.
 */
export class WaypointBuilder {


  /**
   * Builds a WayPoint from basic data.
   * @param ident The ident of the waypoint to be created.
   * @param coordinates The coordinates of the waypoint.
   * @param instrument The base instrument instance.
   * @returns The built waypoint.
   */
  public static fromCoordinates(ident: string, coordinates: LatLongAlt, instrument: BaseInstrument): WayPoint {
    const waypoint = new WayPoint(instrument);
    waypoint.type = 'W';

    waypoint.infos = new IntersectionInfo(instrument);
    waypoint.infos.coordinates = coordinates;

    waypoint.ident = ident;
    waypoint.infos.ident = ident;

    return waypoint;
  }

  /**
   * Builds a WayPoint from a refrence waypoint.
   * @param ident The ident of the waypoint to be created.
   * @param placeCoordinates The coordinates of the reference waypoint.
   * @param bearing The bearing from the reference waypoint.
   * @param distance The distance from the reference waypoint.
   * @param instrument The base instrument instance.
   * @returns The built waypoint.
   */
  public static fromPlaceBearingDistance(ident: string, placeCoordinates: LatLongAlt, bearing: number, distance: number, instrument: BaseInstrument): WayPoint {
    let magneticBearing = bearing + GeoMath.getMagvar(placeCoordinates.lat, placeCoordinates.long);
    magneticBearing = magneticBearing < 0 ? 360 + magneticBearing : magneticBearing;

    const coordinates = Avionics.Utils.bearingDistanceToCoordinates(magneticBearing, distance, placeCoordinates.lat, placeCoordinates.long);

    return WaypointBuilder.fromCoordinates(ident, coordinates, instrument);
  }

  /**
   * Builds a WayPoint at a distance from an existing waypoint along the flight plan.
   * @param ident The ident of the waypoint to be created.
   * @param placeIndex The index of the reference waypoint in the flight plan.
   * @param distance The distance from the reference waypoint.
   * @param instrument The base instrument instance.
   * @param fpm The flightplanmanager instance.
   * @returns The built waypoint.
   */
  public static fromPlaceAlongFlightPlan(ident: string, placeIndex: number, distance: number, instrument: BaseInstrument, fpm: FlightPlanManager): WayPoint {
    console.log("running fromPlaceAlongFlightPlan");
    console.log("destination? " + fpm.getDestination() ? "True" : "False");
    const destinationDistanceInFlightplan = fpm.getDestination().cumulativeDistanceInFP;
    console.log("destinationDistanceInFlightplan " + destinationDistanceInFlightplan);

    const placeDistanceFromDestination = fpm.getWaypoint(placeIndex, 0, true).cumulativeDistanceInFP;
    console.log("placeDistanceFromDestination " + placeDistanceFromDestination);

    const distanceFromDestination = destinationDistanceInFlightplan - placeDistanceFromDestination - distance;
    console.log("distanceFromDestination " + distanceFromDestination);


    const coordinates = fpm.getCoordinatesAtNMFromDestinationAlongFlightPlan(distanceFromDestination);

    return WaypointBuilder.fromCoordinates(ident, coordinates, instrument);
  }

}