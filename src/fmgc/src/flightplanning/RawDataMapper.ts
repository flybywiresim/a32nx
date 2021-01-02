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

/**
 * A class for mapping raw facility data to WayPoints.
 */
export class RawDataMapper {

  /**
   * Maps a raw facility record to a WayPoint.
   * @param facility The facility record to map.
   * @param instrument The instrument to attach to the WayPoint.
   * @returns The mapped waypoint.
   */
  public static toWaypoint(facility: any, instrument: BaseInstrument): WayPoint {
    const waypoint = new WayPoint(instrument);

    waypoint.ident = facility.icao.substring(7, 12).trim();
    waypoint.icao = facility.icao;
    waypoint.type = facility.icao[0];

    switch (waypoint.type) {
      case 'A': {
        const info = new AirportInfo(instrument);
        info.CopyBaseInfosFrom(waypoint);
        info.UpdateNamedFrequencies();

        info.approaches = facility.approaches;
        info.approaches.forEach(approach =>
          approach.transitions.forEach(trans => trans.name = trans.legs[0].fixIcao.substring(7, 12).trim()));

        info.departures = facility.departures;
        info.departures.forEach(departure =>
          departure.runwayTransitions.forEach(trans => trans.name = RawDataMapper.generateRunwayTransitionName(trans)));
        info.departures.forEach(departure =>
          departure.enRouteTransitions.forEach(trans => trans.name = RawDataMapper.generateDepartureEnRouteTransitionName(trans)));

        info.arrivals = facility.arrivals;
        info.arrivals.forEach(arrival =>
          arrival.runwayTransitions.forEach(trans => trans.name = RawDataMapper.generateRunwayTransitionName(trans)));
        info.arrivals.forEach(arrival =>
          arrival.enRouteTransitions.forEach(trans => trans.name = RawDataMapper.generateArrivalTransitionName(trans)));

        info.runways = facility.runways;

        info.oneWayRunways = [];
        facility.runways.forEach(runway => info.oneWayRunways.push(...Object.assign(new Runway(), runway).splitIfTwoWays()));

        info.oneWayRunways.sort(RawDataMapper.sortRunways);
        waypoint.infos = info;
      }
        break;
      case 'V':
        waypoint.infos = new VORInfo(instrument);
        break;
      case 'N':
        waypoint.infos = new NDBInfo(instrument);
        break;
      case 'W':
        waypoint.infos = new IntersectionInfo(instrument);
        break;
      default:
        waypoint.infos = new WayPointInfo(instrument);
        break;
    }
    if(waypoint.type !== 'A'){
      waypoint.infos.CopyBaseInfosFrom(waypoint);
      waypoint.infos.routes = facility.routes;
    }

    waypoint.infos.coordinates = new LatLongAlt(facility.lat, facility.lon);
    return waypoint;
  };

  /**
   * A comparer for sorting runways by number, and then by L, C, and R.
   * @param r1 The first runway to compare.
   * @param r2 The second runway to compare.
   * @returns -1 if the first is before, 0 if equal, 1 if the first is after.
   */
  public static sortRunways(r1: OneWayRunway, r2: OneWayRunway): number {
    if (parseInt(r1.designation) === parseInt(r2.designation)) {
      let v1 = 0;
      if (r1.designation.indexOf("L") != -1) {
        v1 = 1;
      }
      else if (r1.designation.indexOf("C") != -1) {
        v1 = 2;
      }
      else if (r1.designation.indexOf("R") != -1) {
        v1 = 3;
      }
      let v2 = 0;
      if (r2.designation.indexOf("L") != -1) {
        v2 = 1;
      }
      else if (r2.designation.indexOf("C") != -1) {
        v2 = 2;
      }
      else if (r2.designation.indexOf("R") != -1) {
        v2 = 3;
      }
      return v1 - v2;
    }
    return parseInt(r1.designation) - parseInt(r2.designation);
  };

  /**
   * Generates a runway transition name from the designated runway in the transition data.
   * @param runwayTransition The runway transition to generate the name for.
   * @returns The runway transition name.
   */
  public static generateRunwayTransitionName(runwayTransition: RunwayTransition): string {
    let name = `RW${runwayTransition.runwayNumber}`;

    switch (runwayTransition.runwayDesignation) {
      case 1:
        name += "L";
        break;
      case 2:
        name += "R";
        break;
      case 3:
        name += "C";
        break;
    }

    return name;
  };

  /**
   * Generates an arrival transition name from a provided arrival enroute transition.
   * @param enrouteTransition The enroute transition to generate a name for.
   * @returns The generated transition name.
   */
  public static generateArrivalTransitionName(enrouteTransition: EnrouteTransition): string {
    return enrouteTransition.legs[0].fixIcao.substring(7, 12).trim();
  }

  /**
   * Generates a departure transition name from a provided departure enroute transition.
   * @param enrouteTransition The enroute transition to generate a name for.
   * @returns The generated transition name.
   */
  public static generateDepartureEnRouteTransitionName(enrouteTransition: EnrouteTransition): string {
    return enrouteTransition.legs[enrouteTransition.legs.length - 1].fixIcao.substring(7, 12).trim();
  }
}

