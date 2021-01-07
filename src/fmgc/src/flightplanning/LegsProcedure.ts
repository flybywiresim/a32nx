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

import { GPS } from '../wtsdk';
import { FixNamingScheme } from './FixNamingScheme';
import { GeoMath } from './GeoMath';
import { RawDataMapper } from './RawDataMapper';

/**
 * Creates a collection of waypoints from a legs procedure.
 */
export class LegsProcedure {

  /** The current index in the procedure. */
  private _currentIndex = 0;

  /** Whether or not there is a discontinuity pending to be mapped. */
  private _isDiscontinuityPending = false;

  /** A collection of the loaded facilities needed for this procedure. */
  private _facilities = new Map<string, any>();

  /** Whether or not the facilities have completed loading. */
  private _facilitiesLoaded = false;

  /**The collection of facility promises to await on first load. */
  private _facilitiesToLoad = new Map();

  /** Whether or not a non initial-fix procedure start has been added to the procedure. */
  private _addedProcedureStart = false;

  /** A normalization factor for calculating distances from triangular ratios. */
  public static distanceNormalFactorNM = (21639 / 2) * Math.PI;

  /** A collection of filtering rules for filtering ICAO data to pre-load for the procedure. */
  private legFilteringRules: ((icao: string) => boolean)[] = [
    icao => icao.trim() !== '', //Icao is not empty
    icao => icao[0] !== 'R', //Icao is not runway icao, which is not searchable
    icao => icao[0] !== 'A', //Icao is not airport icao, which can be skipped
    icao => icao.substr(1, 2) !== '  ', //Icao is not missing a region code
    icao => !this._facilitiesToLoad.has(icao) //Icao is not already being loaded
  ];

  /**
   * Creates an instance of a LegsProcedure.
   * @param legs The legs that are part of the procedure.
   * @param startingPoint The starting point for the procedure.
   * @param instrument The instrument that is attached to the flight plan.
   */
  constructor(private _legs: ProcedureLeg[], private _previousFix: WayPoint, private _instrument: BaseInstrument) {

    for (var leg of this._legs) {
      if (this.isIcaoValid(leg.fixIcao)) {
        this._facilitiesToLoad.set(leg.fixIcao, this._instrument.facilityLoader.getFacilityRaw(leg.fixIcao, 2000));
      }

      if (this.isIcaoValid(leg.originIcao)) {
        this._facilitiesToLoad.set(leg.originIcao, this._instrument.facilityLoader.getFacilityRaw(leg.originIcao, 2000));
      }
    }
  }

  /**
   * Checks whether or not there are any legs remaining in the procedure.
   * @returns True if there is a next leg, false otherwise.
   */
  public hasNext(): boolean {
    return this._currentIndex < this._legs.length || this._isDiscontinuityPending;
  }

  /**
   * Gets the next mapped leg from the procedure.
   * @returns The mapped waypoint from the leg of the procedure.
   */
  public async getNext(): Promise<WayPoint> {
    let isLegMappable = false;
    let mappedLeg: WayPoint;

    if (!this._facilitiesLoaded) {
      const facilityResults = await Promise.all(this._facilitiesToLoad.values());
      for (var facility of facilityResults.filter(f => f !== undefined)) {
        this._facilities.set(facility.icao, facility);
      }

      this._facilitiesLoaded = true;
    }

    while (!isLegMappable && this._currentIndex < this._legs.length) {
      const currentLeg = this._legs[this._currentIndex];
      isLegMappable = true;

      //Some procedures don't start with 15 (initial fix) but instead start with a heading and distance from
      //a fix: the procedure then starts with the fix exactly
      if (this._currentIndex === 0 && currentLeg.type === 10 && !this._addedProcedureStart) {
        mappedLeg = this.mapExactFix(currentLeg);
        this._addedProcedureStart = true;
      }
      else {
        try {
          switch (currentLeg.type) {
            case 3:
              mappedLeg = this.mapHeadingUntilDistanceFromOrigin(currentLeg, this._previousFix);
              break;
            case 4:
              //Only map if the fix is itself not a runway fix to avoid double
              //adding runway fixes
              if (currentLeg.fixIcao === '' || currentLeg.fixIcao[0] !== 'R') {
                mappedLeg = this.mapOriginRadialForDistance(currentLeg, this._previousFix);
              }
              else {
                isLegMappable = false;
              }
              break;
            case 5:
            case 21:
              mappedLeg = this.mapHeadingToInterceptNextLeg(currentLeg, this._previousFix, this._legs[this._currentIndex + 1]);
              break;
            case 6:
            case 23:
              mappedLeg = this.mapHeadingUntilRadialCrossing(currentLeg, this._previousFix);
              break;
            case 9:
            case 10:
              mappedLeg = this.mapBearingAndDistanceFromOrigin(currentLeg);
              break;
            case 11:
            case 22:
              mappedLeg = this.mapVectors(currentLeg, this._previousFix);
              break;
            case 15: {
              if (currentLeg.fixIcao[0] !== 'A') {
                const leg = this.mapExactFix(currentLeg);
                const prevLeg = this._previousFix;

                //If a type 15 (initial fix) comes up in the middle of a plan
                if (leg.icao === prevLeg.icao && leg.infos.coordinates.lat === prevLeg.infos.coordinates.lat
                  && leg.infos.coordinates.long === prevLeg.infos.coordinates.long) {
                  isLegMappable = false;
                }
                else {
                  mappedLeg = leg;
                }
              }
              //If type 15 is an airport itself, we don't need to map it (and the data is generally wrong)
              else {
                isLegMappable = false;
              }
            }
              break;
            case 7:
            case 17:
            case 18:
              mappedLeg = this.mapExactFix(currentLeg);
              break;
            case 2:
            case 19:
              mappedLeg = this.mapHeadingUntilAltitude(currentLeg, this._previousFix);
              break;
            default:
              isLegMappable = false;
              break;
          }
        }
        catch (err) {
          console.log(`LegsProcedure: Unexpected unmappable leg: ${err}`);
        }

        if (mappedLeg !== undefined) {
          mappedLeg.legAltitudeDescription = currentLeg.altDesc;
          mappedLeg.legAltitude1 = currentLeg.altitude1 * 3.28084;
          mappedLeg.legAltitude2 = currentLeg.altitude2 * 3.28084;
        }

        this._currentIndex++;
      }
    }

    if (mappedLeg !== undefined) {
      this._previousFix = mappedLeg;
      return mappedLeg;
    }
    else {
      return undefined;
    }
  }

  /**
   * Maps a heading until distance from origin leg.
   * @param leg The procedure leg to map.
   * @param prevLeg The previously mapped waypoint in the procedure.
   * @returns The mapped leg.
   */
  public mapHeadingUntilDistanceFromOrigin(leg: ProcedureLeg, prevLeg: WayPoint): WayPoint {
    const origin = this._facilities.get(leg.originIcao);
    const originIdent = origin.icao.substring(7, 12).trim();

    const bearingToOrigin = Avionics.Utils.computeGreatCircleHeading(prevLeg.infos.coordinates, new LatLongAlt(origin.lat, origin.lon));
    const distanceToOrigin = Avionics.Utils.computeGreatCircleDistance(prevLeg.infos.coordinates, new LatLongAlt(origin.lat, origin.lon)) / LegsProcedure.distanceNormalFactorNM;

    const deltaAngle = this.deltaAngleRadians(bearingToOrigin, leg.course);
    const targetDistance = (leg.distance / 1852) / LegsProcedure.distanceNormalFactorNM;

    const distanceAngle = Math.asin((Math.sin(distanceToOrigin) * Math.sin(deltaAngle)) / Math.sin(targetDistance));
    const inverseDistanceAngle = Math.PI - distanceAngle;

    const legDistance1 = 2 * Math.atan(Math.tan(0.5 * (targetDistance - distanceToOrigin)) * (Math.sin(0.5 * (deltaAngle + distanceAngle))
      / Math.sin(0.5 * (deltaAngle - distanceAngle))));

    const legDistance2 = 2 * Math.atan(Math.tan(0.5 * (targetDistance - distanceToOrigin)) * (Math.sin(0.5 * (deltaAngle + inverseDistanceAngle))
      / Math.sin(0.5 * (deltaAngle - inverseDistanceAngle))));

    const legDistance = targetDistance > distanceToOrigin ? legDistance1 : Math.min(legDistance1, legDistance2);
    const course = leg.course + GeoMath.getMagvar(prevLeg.infos.coordinates.lat, prevLeg.infos.coordinates.long);
    const coordinates = Avionics.Utils.bearingDistanceToCoordinates(course, legDistance * LegsProcedure.distanceNormalFactorNM, prevLeg.infos.coordinates.lat, prevLeg.infos.coordinates.long);

    return this.buildWaypoint(`${originIdent}${Math.trunc(legDistance * LegsProcedure.distanceNormalFactorNM)}`, coordinates);
  }

  /**
   * Maps a bearing/distance fix in the procedure.
   * @param leg The procedure leg to map.
   * @returns The mapped leg.
   */
  public mapBearingAndDistanceFromOrigin(leg: ProcedureLeg): WayPoint {
    const origin = this._facilities.get(leg.originIcao);
    const originIdent = origin.icao.substring(7, 12).trim();

    const course = leg.course + GeoMath.getMagvar(origin.lat, origin.lon);
    const coordinates = Avionics.Utils.bearingDistanceToCoordinates(leg.course, leg.distance / 1852, origin.lat, origin.lon);

    return this.buildWaypoint(`${originIdent}${Math.trunc(leg.distance / 1852)}`, coordinates);
  }

  /**
   * Maps a radial on the origin for a specified distance leg in the procedure.
   * @param leg The procedure leg to map.
   * @param prevLeg The previously mapped leg.
   * @returns The mapped leg.
   */
  public mapOriginRadialForDistance(leg: ProcedureLeg, prevLeg: WayPoint): WayPoint {
    if (leg.fixIcao.trim() !== '') {
      return this.mapExactFix(leg);
    }
    else {
      const origin = this._facilities.get(leg.originIcao);
      const originIdent = origin.icao.substring(7, 12).trim();

      const course = leg.course + GeoMath.getMagvar(prevLeg.infos.coordinates.lat, prevLeg.infos.coordinates.long);
      const coordinates = Avionics.Utils.bearingDistanceToCoordinates(course, leg.distance / 1852, prevLeg.infos.coordinates.lat, prevLeg.infos.coordinates.long);

      const distanceFromOrigin = Avionics.Utils.computeGreatCircleDistance(new LatLongAlt(origin.lat, origin.lon), coordinates);
      return this.buildWaypoint(`${originIdent}${Math.trunc(distanceFromOrigin / 1852)}`, coordinates);
    }
  }

  /**
   * Maps a heading turn to intercept the next leg in the procedure.
   * @param leg The procedure leg to map.
   * @param prevLeg The previously mapped leg.
   * @param nextLeg The next leg in the procedure to intercept.
   * @returns The mapped leg.
   */
  public mapHeadingToInterceptNextLeg(leg: ProcedureLeg, prevLeg: WayPoint, nextLeg: ProcedureLeg): WayPoint {
    let referenceCoordinates;
    let courseToIntercept;
    let referenceFix;

    switch (nextLeg.type) {
      case 4:
      case 7:
      case 15:
      case 17:
      case 18:
        referenceFix = this._facilities.get(nextLeg.fixIcao);
        referenceCoordinates = new LatLongAlt(referenceFix.lat, referenceFix.lon);
        courseToIntercept = nextLeg.course - 180;
        if (courseToIntercept < 0) {
          courseToIntercept += 360;
        }
        break;
      case 9:
        referenceFix = this._facilities.get(nextLeg.originIcao);
        referenceCoordinates = new LatLongAlt(referenceFix.lat, referenceFix.lon);
        courseToIntercept = nextLeg.course;
        break;
    }

    if (referenceCoordinates !== undefined && courseToIntercept !== undefined) {
      const distanceFromOrigin = Avionics.Utils.computeGreatCircleDistance(prevLeg.infos.coordinates, referenceCoordinates);
      const bearingToOrigin = Avionics.Utils.computeGreatCircleHeading(prevLeg.infos.coordinates, referenceCoordinates);
      const bearingFromOrigin = Avionics.Utils.computeGreatCircleHeading(referenceCoordinates, prevLeg.infos.coordinates);

      let ang1 = this.deltaAngleRadians(bearingToOrigin, leg.course);
      let ang2 = this.deltaAngleRadians(bearingFromOrigin, courseToIntercept);
      let ang3 = Math.acos(Math.sin(ang1) * Math.sin(ang2) * Math.cos(distanceFromOrigin / LegsProcedure.distanceNormalFactorNM) - Math.cos(ang1) * Math.cos(ang2));

      let legDistance = Math.acos((Math.cos(ang1) + Math.cos(ang2) * Math.cos(ang3)) / (Math.sin(ang2) * Math.sin(ang3))) * LegsProcedure.distanceNormalFactorNM;
      const course = leg.course + GeoMath.getMagvar(prevLeg.infos.coordinates.lat, prevLeg.infos.coordinates.long);
      const coordinates = Avionics.Utils.bearingDistanceToCoordinates(course, legDistance, prevLeg.infos.coordinates.lat, prevLeg.infos.coordinates.long);

      return this.buildWaypoint(FixNamingScheme.courseToIntercept(course), coordinates);
    }
  }

  /**
   * Maps flying a heading until crossing a radial of a reference fix.
   * @param leg The procedure leg to map.
   * @param prevLeg The previously mapped leg.
   * @returns The mapped leg.
   */
  public mapHeadingUntilRadialCrossing(leg: ProcedureLeg, prevLeg: WayPoint) {
    const origin = this._facilities.get(leg.originIcao);
    const originCoordinates = new LatLongAlt(origin.lat, origin.lon);

    const originToCoordinates = Avionics.Utils.computeGreatCircleHeading(originCoordinates, prevLeg.infos.coordinates);
    const coordinatesToOrigin = Avionics.Utils.computeGreatCircleHeading(prevLeg.infos.coordinates, new LatLongAlt(origin.lat, origin.lon));
    const distanceToOrigin = Avionics.Utils.computeGreatCircleDistance(prevLeg.infos.coordinates, originCoordinates) / LegsProcedure.distanceNormalFactorNM;

    const alpha = this.deltaAngleRadians(coordinatesToOrigin, leg.course);
    const beta = this.deltaAngleRadians(originToCoordinates, leg.theta);

    const gamma = Math.acos(Math.sin(alpha) * Math.sin(beta) * Math.cos(distanceToOrigin) - Math.cos(alpha) * Math.cos(beta));
    const legDistance = Math.acos((Math.cos(beta) + Math.cos(alpha) * Math.cos(gamma)) / (Math.sin(alpha) * Math.sin(gamma)));

    const course = leg.course + GeoMath.getMagvar(prevLeg.infos.coordinates.lat, prevLeg.infos.coordinates.long);
    const coordinates = Avionics.Utils.bearingDistanceToCoordinates(course, legDistance * LegsProcedure.distanceNormalFactorNM, prevLeg.infos.coordinates.lat, prevLeg.infos.coordinates.long);

    return this.buildWaypoint(`${this.getIdent(origin.icao)}${leg.theta}`, coordinates);
  }

  /**
   * Maps flying a heading until a proscribed altitude.
   * @param leg The procedure leg to map.
   * @param prevLeg The previous leg in the procedure.
   * @returns The mapped leg.
   */
  public mapHeadingUntilAltitude(leg: ProcedureLeg, prevLeg: WayPoint) {
    const altitudeFeet = (leg.altitude1 * 3.2808399);
    const distanceInNM = altitudeFeet / 500.0;

    const course = leg.course + GeoMath.getMagvar(prevLeg.infos.coordinates.lat, prevLeg.infos.coordinates.long);
    const coordinates = GeoMath.relativeBearingDistanceToCoords(course, distanceInNM, prevLeg.infos.coordinates);

    return this.buildWaypoint(FixNamingScheme.headingUntilAltitude(altitudeFeet), coordinates, prevLeg.infos.magneticVariation);
  }

  /**
   * Maps a vectors instruction.
   * @param leg The procedure leg to map.
   * @param prevLeg The previous leg in the procedure.
   * @returns The mapped leg.
   */
  public mapVectors(leg: ProcedureLeg, prevLeg: WayPoint) {
    const course = leg.course + GeoMath.getMagvar(prevLeg.infos.coordinates.lat, prevLeg.infos.coordinates.long);
    const coordinates = GeoMath.relativeBearingDistanceToCoords(course, 5, prevLeg.infos.coordinates);

    const waypoint = this.buildWaypoint(FixNamingScheme.vector(), coordinates);
    waypoint.isVectors = true;
    waypoint.endsInDiscontinuity = true;
    waypoint.discontinuityCanBeCleared = false;

    return waypoint;
  }

  /**
   * Maps an exact fix leg in the procedure.
   * @param leg The procedure leg to map.
   * @returns The mapped leg.
   */
  public mapExactFix(leg: ProcedureLeg): WayPoint {
    const facility = this._facilities.get(leg.fixIcao);
    if (facility) {
      return RawDataMapper.toWaypoint(facility, this._instrument);
    }
    else {
      const origin = this._facilities.get(leg.originIcao);
      const originIdent = origin.icao.substring(7, 12).trim();

      const coordinates = Avionics.Utils.bearingDistanceToCoordinates(leg.theta, leg.rho / 1852, origin.lat, origin.lon);
      return this.buildWaypoint(`${originIdent}${Math.trunc(leg.rho / 1852)}`, coordinates);
    }
  }

  /**
   * Gets the difference between two headings in zero north normalized radians.
   * @param a The degrees of heading a.
   * @param b The degrees of heading b.
   * @returns The difference between the two headings in zero north normalized radians.
   */
  private deltaAngleRadians(a: number, b: number): number {
    return Math.abs((Avionics.Utils.fmod((a - b) + 180, 360) - 180) * Avionics.Utils.DEG2RAD);
  }

  /**
   * Gets an ident from an ICAO.
   * @param icao The icao to pull the ident from.
   * @returns The parsed ident.
   */
  private getIdent(icao: string): string {
    return icao.substring(7, 12).trim();
  }

  /**
   * Checks if an ICAO is valid to load.
   * @param icao The icao to check.
   * @returns Whether or not the ICAO is valid.
   */
  private isIcaoValid(icao): boolean {
    for (var rule of this.legFilteringRules) {
      if (!rule(icao)) {
        return false;
      }
    }

    return true;
  }

  /**
   * Builds a WayPoint from basic data.
   * @param ident The ident of the waypoint.
   * @param coordinates The coordinates of the waypoint.
   * @param magneticVariation The magnetic variation of the waypoint, if any.
   * @returns The built waypoint.
   */
  public buildWaypoint(ident: string, coordinates: LatLongAlt, magneticVariation?: number): WayPoint {
    const waypoint = new WayPoint(this._instrument);
    waypoint.type = 'W';

    waypoint.infos = new IntersectionInfo(this._instrument);
    waypoint.infos.coordinates = coordinates;
    waypoint.infos.magneticVariation = magneticVariation;

    waypoint.ident = ident;
    waypoint.infos.ident = ident;

    return waypoint;
  }
}