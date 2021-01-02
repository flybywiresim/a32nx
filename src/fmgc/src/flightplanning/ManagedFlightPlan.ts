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

import { SegmentType, FlightPlanSegment } from './FlightPlanSegment';
import { LegsProcedure } from './LegsProcedure';
import { RawDataMapper } from './RawDataMapper';
import { GPS } from './GPS';
import { ProcedureDetails } from './ProcedureDetails';
import { DirectTo } from './DirectTo';
import { GeoMath } from './GeoMath';

/**
 * A flight plan managed by the FlightPlanManager.
 */
export class ManagedFlightPlan {

  /** Whether or not the flight plan has an origin airfield. */
  public originAirfield?: WayPoint;

  /** Whether or not the flight plan has a destination airfield. */
  public destinationAirfield?: WayPoint;

  /** The cruise altitude for this flight plan. */
  public cruiseAltitude: number = 0;

  /** The index of the currently active waypoint. */
  public activeWaypointIndex: number = 0;

  /** The details for selected procedures on this flight plan. */
  public procedureDetails: ProcedureDetails = new ProcedureDetails();

  /** The details of any direct-to procedures on this flight plan. */
  public directTo: DirectTo = new DirectTo();

  /** The departure segment of the flight plan. */
  public get departure(): FlightPlanSegment { return this.getSegment(SegmentType.Departure); }

  /** The enroute segment of the flight plan. */
  public get enroute(): FlightPlanSegment { return this.getSegment(SegmentType.Enroute); }

  /** The arrival segment of the flight plan. */
  public get arrival(): FlightPlanSegment { return this.getSegment(SegmentType.Arrival); }

  /** The approach segment of the flight plan. */
  public get approach(): FlightPlanSegment { return this.getSegment(SegmentType.Approach); }

  /** The approach segment of the flight plan. */
  public get missed(): FlightPlanSegment { return this.getSegment(SegmentType.Missed); }

  /** Whether the flight plan has an origin airfield. */
  public get hasOrigin() { return this.originAirfield; }

  /** Whether the flight plan has a destination airfield. */
  public get hasDestination() { return this.destinationAirfield; }

  /** The currently active waypoint. */
  public get activeWaypoint() { return this.waypoints[this.activeWaypointIndex]; }

  /** The parent instrument this flight plan is attached to locally. */
  private _parentInstrument?: BaseInstrument;

  /** The current active segments of the flight plan. */
  private _segments: FlightPlanSegment[] = [new FlightPlanSegment(SegmentType.Enroute, 0, [])];

  /** The waypoints of the flight plan. */
  public get waypoints(): WayPoint[] {
    const waypoints: WayPoint[] = [];
    if (this.originAirfield) {
      waypoints.push(this.originAirfield);
    }

    for (var segment of this._segments) {
      waypoints.push(...segment.waypoints);
    }

    if (this.destinationAirfield) {
      waypoints.push(this.destinationAirfield);
    }

    return waypoints;
  }

  public get segments(): FlightPlanSegment[] {
      return this._segments;
  }

  /** The length of the flight plan. */
  public get length(): number {
    const lastSeg = this._segments[this._segments.length - 1];
    return lastSeg.offset + lastSeg.waypoints.length + (this.hasDestination ? 1 : 0);
  }

  public get checksum():number {
    let checksum = 0;
    const waypoints = this.waypoints;
    for( let i = 0; i < waypoints.length; i++)
        checksum += waypoints[i].infos.coordinates.lat;
    return checksum;
  }

  /** The non-approach waypoints of the flight plan. */
  public get nonApproachWaypoints(): WayPoint[] {
    const waypoints = [];
    if (this.originAirfield) {
      waypoints.push(this.originAirfield);
    }

    for (var segment of this._segments.filter(s => s.type < SegmentType.Approach)) {
      waypoints.push(...segment.waypoints);
    }

    if (this.destinationAirfield) {
      waypoints.push(this.destinationAirfield);
    }

    return waypoints;
  }

  /**
   * Sets the parent instrument that the flight plan is attached to locally.
   * @param instrument The instrument that the flight plan is attached to.
   */
  public setParentInstrument(instrument: BaseInstrument): void {
    this._parentInstrument = instrument;
  }

  /**
   * Clears the flight plan.
   */
  public async clearPlan(): Promise<void> {

    this.originAirfield = undefined;
    this.destinationAirfield = undefined;

    this.cruiseAltitude = 0;
    this.activeWaypointIndex = 0;

    this.procedureDetails = new ProcedureDetails();
    this.directTo = new DirectTo();

    await GPS.clearPlan();
    this._segments = [new FlightPlanSegment(SegmentType.Enroute, 0, [])];
  }

  /**
   * Syncs the flight plan to FS9GPS.
   */
  public async syncToGPS(): Promise<void> {
    await GPS.clearPlan();
    for (var i = 0; i < this.waypoints.length; i++) {
      const waypoint = this.waypoints[i];

      if (waypoint.icao && waypoint.icao.trim() !== '') {
        await GPS.addIcaoWaypoint(waypoint.icao, i);
      }
      else {
        await GPS.addUserWaypoint(waypoint.infos.coordinates.lat, waypoint.infos.coordinates.long, i, waypoint.ident);
      }

      if (waypoint.endsInDiscontinuity) {
        break;
      }
    }

    await GPS.setActiveWaypoint(this.activeWaypointIndex);
    await GPS.logCurrentPlan();
  }

  /**
   * Adds a waypoint to the flight plan.
   * @param waypoint The waypoint to add.
   * @param index The index to add the waypoint at. If ommitted the waypoint will
   * be appended to the end of the flight plan.
   * @param segmentType The type of segment to add the waypoint to.
   */
  public addWaypoint(waypoint: WayPoint, index?: number | undefined, segmentType?: SegmentType): void {

    const mappedWaypoint = (waypoint instanceof WayPoint) ? waypoint : RawDataMapper.toWaypoint(waypoint, this._parentInstrument);
    if (mappedWaypoint.type === 'A' && index === 0) {
      this.originAirfield = mappedWaypoint;

      this.procedureDetails.departureIndex = -1;
      this.procedureDetails.departureRunwayIndex = -1;
      this.procedureDetails.departureTransitionIndex = -1;
      this.procedureDetails.originRunwayIndex = -1;

      this.reflowSegments();
      this.reflowDistances();
    }
    else if (mappedWaypoint.type === 'A' && index === undefined) {
      this.destinationAirfield = mappedWaypoint;

      this.procedureDetails.arrivalIndex = -1;
      this.procedureDetails.arrivalRunwayIndex = -1;
      this.procedureDetails.arrivalTransitionIndex = -1;
      this.procedureDetails.approachIndex = -1;
      this.procedureDetails.approachTransitionIndex = -1;

      this.reflowSegments();
      this.reflowDistances();
    }
    else {
      let segment = segmentType !== undefined
        ? this.getSegment(segmentType)
        : this.findSegmentByWaypointIndex(index);

      // hitting first waypoint in segment > enroute
      if (segment.type > SegmentType.Enroute && index == segment.offset) {
        const segIdx = this._segments.findIndex((seg) => { return seg.type == segment.type });
        // is prev segment enroute?
        const prevSeg = this._segments[segIdx - 1];
        if (prevSeg.type == SegmentType.Enroute) {
          segment = prevSeg;
        }
      }

      if (segment) {

        if (index > this.length) {
          index = undefined;
        }

        if (index !== undefined) {
          const segmentIndex = index - segment.offset;
          if (segmentIndex < segment.waypoints.length) {
            segment.waypoints.splice(segmentIndex, 0, mappedWaypoint);
          }
          else {
            segment.waypoints.push(mappedWaypoint);
          }
        }
        else {
          segment.waypoints.push(mappedWaypoint);
        }

        this.reflowSegments();
        this.reflowDistances();

        if (this.activeWaypointIndex === 0 && this.length > 1) {
          this.activeWaypointIndex = 1;
        }
        else if (this.activeWaypointIndex === 1 && waypoint.isRunway && segment.type === SegmentType.Departure) {
          this.activeWaypointIndex = 2;
        }
      }
    }
  }

  /**
   * Removes a waypoint from the flight plan.
   * @param index The index of the waypoint to remove.
   */
  public removeWaypoint(index: number): void {

    if (this.originAirfield && index === 0) {
      this.originAirfield = undefined;

      this.reflowSegments();
      this.reflowDistances();
    }
    else if (this.destinationAirfield && index === this.length - 1) {
      this.destinationAirfield = undefined;
    }
    else {
      const segment = this.findSegmentByWaypointIndex(index);
      if (segment) {
        segment.waypoints.splice(index - segment.offset, 1);

        if (segment.waypoints.length === 0 && segment.type !== SegmentType.Enroute) {
          this.removeSegment(segment.type);
        }

        this.reflowSegments();
        this.reflowDistances();
      }
    }

    if (index < this.activeWaypointIndex) {
      this.activeWaypointIndex--;
    }
  }

  /**
   * Gets a waypoint by index from the flight plan.
   * @param index The index of the waypoint to get.
   */
  public getWaypoint(index: number): WayPoint {
    if (this.originAirfield && index === 0) {
      return this.originAirfield;
    }

    if (this.destinationAirfield && index === this.length - 1) {
      return this.destinationAirfield;
    }

    const segment = this.findSegmentByWaypointIndex(index);
    if (segment) {
      return segment.waypoints[index - segment.offset];
    }
  }

  /**
   * Adds a plan segment to the flight plan.
   * @param type The type of the segment to add.
   */
  public addSegment(type: SegmentType): FlightPlanSegment {
    const segment = new FlightPlanSegment(type, 0, []);
    this._segments.push(segment);

    this._segments.sort((a, b) => a.type - b.type);
    this.reflowSegments();

    return segment;
  }

  /**
   * Removes a plan segment from the flight plan.
   * @param type The type of plan segment to remove.
   */
  public removeSegment(type: SegmentType): void {
    const segmentIndex = this._segments.findIndex(s => s.type === type);
    if (segmentIndex > -1) {
      this._segments.splice(segmentIndex, 1);
    }
  }

  /**
   * Reflows waypoint index offsets accross plans segments.
   */
  public reflowSegments(): void {
    let index = 0;
    if (this.originAirfield) {
      index = 1;
    }

    for (var segment of this._segments) {
      segment.offset = index;
      index += segment.waypoints.length;
    }
  }

  /**
   * Gets a flight plan segment of the specified type.
   * @param type The type of flight plan segment to get.
   * @returns The found segment, or FlightPlanSegment.Empty if not found.
   */
  public getSegment(type: number): FlightPlanSegment {
    const segment = this._segments.find(s => s.type === type);
    return segment !== undefined ? segment : FlightPlanSegment.Empty;
  }

  /**
   * Finds a flight plan segment by waypoint index.
   * @param index The index of the waypoint to find the segment for.
   * @returns The located segment, if any.
   */
  public findSegmentByWaypointIndex(index: number): FlightPlanSegment {
    for (var i = 0; i < this._segments.length; i++) {
      const segMaxIdx = this._segments[i].offset + this._segments[i].waypoints.length;
      if (segMaxIdx > index) {
        return this._segments[i];
      }
    }

    return this._segments[this._segments.length - 1];
  }

  /**
   * Recalculates all waypoint bearings and distances in the flight plan.
   */
  public reflowDistances(): void {
    let cumulativeDistance = 0;
    let waypoints = this.waypoints;

    for (var i = 0; i < waypoints.length; i++) {
      if (i > 0) {

        //If there's an approach selected and this is the last approach waypoint, use the destination waypoint for coordinates
        //Runway waypoints do not have coordinates
        const referenceWaypoint = waypoints[i];
        const prevWaypoint = waypoints[i - 1];

        const trueCourseToWaypoint = Avionics.Utils.computeGreatCircleHeading(prevWaypoint.infos.coordinates, referenceWaypoint.infos.coordinates);
        referenceWaypoint.bearingInFP = trueCourseToWaypoint - GeoMath.getMagvar(prevWaypoint.infos.coordinates.lat, prevWaypoint.infos.coordinates.long);
        referenceWaypoint.bearingInFP = referenceWaypoint.bearingInFP < 0 ? 360 + referenceWaypoint.bearingInFP : referenceWaypoint.bearingInFP;
        referenceWaypoint.distanceInFP = Avionics.Utils.computeGreatCircleDistance(prevWaypoint.infos.coordinates, referenceWaypoint.infos.coordinates);

        cumulativeDistance += referenceWaypoint.distanceInFP;
        referenceWaypoint.cumulativeDistanceInFP = cumulativeDistance;
      }
    }
  }

  /**
   * Copies a sanitized version of the flight plan for shared data storage.
   * @returns The sanitized flight plan.
   */
  public serialize(): any {
    const planCopy = new ManagedFlightPlan();
    const copyWaypoint = (waypoint: WayPoint) => ({
      icao: waypoint.icao,
      ident: waypoint.ident,
      type: waypoint.type,
      legAltitudeDescription: waypoint.legAltitudeDescription,
      legAltitude1: waypoint.legAltitude1,
      legAltitude2: waypoint.legAltitude2,
      isVectors: waypoint.isVectors,
      endsInDiscontinuity: waypoint.endsInDiscontinuity,
      distanceInFP: waypoint.distanceInFP,
      cumulativeDistanceInFP: waypoint.cumulativeDistanceInFP,
      isRunway: waypoint.isRunway,
      infos: {
        icao: waypoint.infos.icao,
        ident: waypoint.infos.ident,
        airwayIn: waypoint.infos.airwayIn,
        airwayOut: waypoint.infos.airwayOut,
        routes: waypoint.infos.routes,
        coordinates: {
          lat: waypoint.infos.coordinates.lat,
          long: waypoint.infos.coordinates.long,
          alt: waypoint.infos.coordinates.alt
        }
      }
    });

    const copyAirfield = (airfield: WayPoint): WayPoint => {
      const copy = Object.assign(new WayPoint(undefined), airfield);
      copy.infos = Object.assign(new AirportInfo(undefined), copy.infos);

      delete copy.instrument;
      delete copy.infos.instrument;
      delete copy._svgElements;
      delete copy.infos._svgElements;

      return copy;
    }

    planCopy.activeWaypointIndex = this.activeWaypointIndex;
    planCopy.destinationAirfield = this.destinationAirfield && copyAirfield(this.destinationAirfield);
    planCopy.originAirfield = this.originAirfield && copyAirfield(this.originAirfield);

    planCopy.procedureDetails = Object.assign({}, this.procedureDetails);
    planCopy.directTo = Object.assign({}, this.directTo);
    planCopy.directTo.interceptPoints = planCopy.directTo.interceptPoints?.map(w => copyWaypoint(w) as WayPoint);

    const copySegments = [];
    for (var segment of this._segments) {
      const copySegment = new FlightPlanSegment(segment.type, segment.offset, []);
      for (var waypoint of segment.waypoints) {
        copySegment.waypoints.push(copyWaypoint(waypoint) as WayPoint);
      }

      copySegments.push(copySegment);
    }

    planCopy._segments = copySegments;
    return planCopy;
  }

  /**
   * Copies the flight plan.
   * @returns The copied flight plan.
   */
  public copy(): ManagedFlightPlan {
    let newFlightPlan = Object.assign(new ManagedFlightPlan(), this);
    newFlightPlan.setParentInstrument(this._parentInstrument);

    newFlightPlan._segments = [];
    for (let i = 0; i < this._segments.length; i++) {
      const seg = this._segments[i];
      newFlightPlan._segments[i] = Object.assign(new FlightPlanSegment(seg.type, seg.offset, []), seg);
      newFlightPlan._segments[i].waypoints = [...seg.waypoints];
    }

    newFlightPlan.procedureDetails = Object.assign(new ProcedureDetails(), this.procedureDetails);
    newFlightPlan.directTo = Object.assign(new DirectTo(), this.directTo);
    newFlightPlan.directTo.interceptPoints = this.directTo.interceptPoints !== undefined ? [...this.directTo.interceptPoints] : undefined;

    return newFlightPlan;
  }

  /**
   * Reverses the flight plan.
   */
  public reverse(): void {
    //TODO: Fix flight plan indexes after reversal
    //this._waypoints.reverse();
  }

  /**
   * Goes direct to the specified waypoint index in the flight plan.
   * @param index The waypoint index to go direct to.
   */
  public addDirectTo(index: number): void {
    const interceptPoints = this.calculateDirectIntercept(this.getWaypoint(index));
    this.addWaypoint(interceptPoints[0], index);

    this.activeWaypointIndex = index + 1;

    this.directTo.isActive = true;
    this.directTo.waypointIsInFlightPlan = true;
    this.directTo.planWaypointIndex = index + 1;
    this.directTo.interceptPoints = interceptPoints;
  }

  /**
   * Calculates an intercept path to a direct-to waypoint.
   * @param waypoint The waypoint to calculate the path to.
   * @returns The waypoints that make up the intercept path.
   */
  public calculateDirectIntercept(waypoint: WayPoint): WayPoint[] {
    const lat = SimVar.GetSimVarValue("PLANE LATITUDE", "degree latitude");
    const long = SimVar.GetSimVarValue("PLANE LONGITUDE", "degree longitude");

    const planeCoords = new LatLongAlt(lat, long);

    const groundSpeed = SimVar.GetSimVarValue("GPS GROUND SPEED", "knots");
    const planeHeading = SimVar.GetSimVarValue("PLANE HEADING DEGREES TRUE", "Radians") * Avionics.Utils.RAD2DEG;

    const headingToFix = Avionics.Utils.computeGreatCircleHeading(planeCoords, waypoint.infos.coordinates);
    let angleDiff = Math.abs(Avionics.Utils.angleDiff(planeHeading, headingToFix));

    const turnDurationSeconds = (angleDiff / 3) + 6;
    const interceptDistance = (groundSpeed / 60 / 60) * turnDurationSeconds * 1.25;

    const createInterceptPoint = (coords: LatLongAlt) => {
      const interceptWaypoint = new WayPoint(this._parentInstrument);
      interceptWaypoint.ident = '$DIR';

      interceptWaypoint.infos = new IntersectionInfo(this._parentInstrument);
      interceptWaypoint.infos.coordinates = coords;

      return interceptWaypoint;
    };

    const coords = Avionics.Utils.bearingDistanceToCoordinates(planeHeading, Math.min(interceptDistance, 1.0), lat, long);
    return [createInterceptPoint(coords)];

    //TODO: Work out better direct to intercept waypoint(s)
    /*
    if (angleDiff < 90 && angleDiff > -90) {
      const coords = Avionics.Utils.bearingDistanceToCoordinates(planeHeading, interceptDistance, lat, long);
      return [createInterceptPoint(planeCoords), createInterceptPoint(coords)];
    }
    else {
      const coords1 = Avionics.Utils.bearingDistanceToCoordinates(planeHeading, interceptDistance / 2, lat, long);
      const coords2 = Avionics.Utils.bearingDistanceToCoordinates(planeHeading + (angleDiff / 2), interceptDistance / 2, coords1.lat, coords1.long);

      return [createInterceptPoint(planeCoords), createInterceptPoint(coords1), createInterceptPoint(coords2)];
    }
    */
  }

  /**
   * Builds a departure into the flight plan from indexes in the departure airport information.
   */
  public async buildDeparture(): Promise<void> {
    const legs = [];
    const origin = this.originAirfield;

    const departureIndex = this.procedureDetails.departureIndex;
    const runwayIndex = this.procedureDetails.departureRunwayIndex;
    const transitionIndex = this.procedureDetails.departureTransitionIndex;

    const selectedOriginRunwayIndex = this.procedureDetails.originRunwayIndex;

    const airportInfo = origin.infos as AirportInfo;

    if (departureIndex !== -1 && runwayIndex !== -1) {
      const runwayTransition = airportInfo.departures[departureIndex].runwayTransitions[runwayIndex];
      legs.push(...runwayTransition.legs);
    }

    if (departureIndex !== -1) {
      legs.push(...airportInfo.departures[departureIndex].commonLegs);
    }

    if (transitionIndex !== -1 && departureIndex !== -1) {
      // TODO: are enroutetransitions working?
      if (airportInfo.departures[departureIndex].enRouteTransitions.length > 0) {
        const transition = airportInfo.departures[departureIndex].enRouteTransitions[transitionIndex].legs;
        legs.push(...transition);
      }
    }

    let segment = this.departure;
    if (segment !== FlightPlanSegment.Empty) {
      for (var i = 0; i < segment.waypoints.length; i++) {
        this.removeWaypoint(segment.offset);
      }

      this.removeSegment(segment.type);
    }

    if (legs.length > 0 || selectedOriginRunwayIndex !== -1 || (departureIndex !== -1 && runwayIndex !== -1)) {
      segment = this.addSegment(SegmentType.Departure);
      let procedure = new LegsProcedure(legs, origin, this._parentInstrument);

      let runway;
      if (selectedOriginRunwayIndex !== -1) {
        runway = airportInfo.oneWayRunways[selectedOriginRunwayIndex];
      }
      else if (runwayIndex !== -1) {
        runway = this.getRunway(airportInfo.oneWayRunways, airportInfo.departures[departureIndex].runwayTransitions[runwayIndex].name);
      }

      if (runway) {
        const selectedRunwayMod = runway.designation.slice(-1);
        let selectedRunwayOutput = undefined;
        if (selectedRunwayMod == "L" || selectedRunwayMod == "C" || selectedRunwayMod == "R") {
          if (runway.designation.length == 2) {
            selectedRunwayOutput = "0" + runway.designation;
          } else {
            selectedRunwayOutput = runway.designation;
          }
        } else {
          if (runway.designation.length == 2) {
            selectedRunwayOutput = runway.designation;
          } else {
            selectedRunwayOutput = "0" + runway.designation;
          }
        }
        const runwayWaypoint = procedure.buildWaypoint(`RW${selectedRunwayOutput}`, runway.endCoordinates);
        runwayWaypoint.legAltitudeDescription = 1;
        runwayWaypoint.legAltitude1 = (runway.elevation * 3.28084) + 50;
        runwayWaypoint.isRunway = true;

        this.addWaypoint(runwayWaypoint, undefined, segment.type);

        procedure = new LegsProcedure(legs, runwayWaypoint, this._parentInstrument);
      }

      let waypointIndex = segment.offset;
      while (procedure.hasNext()) {
        const waypoint = await procedure.getNext();
        if (waypoint !== undefined) {
          this.addWaypoint(waypoint, ++waypointIndex, segment.type);
        }
      }
    }
  }

  /**
   * Builds an arrival into the flight plan from indexes in the arrival airport information.
   */
  public async buildArrival(): Promise<void> {
    const legs = [];
    const destination = this.destinationAirfield;

    const arrivalIndex = this.procedureDetails.arrivalIndex;
    const arrivalRunwayIndex = this.procedureDetails.arrivalRunwayIndex;
    const arrivalTransitionIndex = this.procedureDetails.arrivalTransitionIndex;

    const destinationInfo = destination.infos as AirportInfo;

    if (arrivalIndex !== -1 && arrivalTransitionIndex !== -1) {
      const transition = destinationInfo.arrivals[arrivalIndex].enRouteTransitions[arrivalTransitionIndex];
      if (transition !== undefined) {
        legs.push(...transition.legs);
      }
    }

    if (arrivalIndex !== -1) {
      legs.push(...destinationInfo.arrivals[arrivalIndex].commonLegs);
    }

    if (arrivalIndex !== -1 && arrivalRunwayIndex !== -1) {
      const runwayTransition = destinationInfo.arrivals[arrivalIndex].runwayTransitions[arrivalRunwayIndex];
      legs.push(...runwayTransition.legs);
    }

    let { startIndex, segment } = this.truncateSegment(SegmentType.Arrival);

    if (legs.length > 0) {
      if (segment === FlightPlanSegment.Empty) {
        segment = this.addSegment(SegmentType.Arrival);
        startIndex = segment.offset;
      }

      const procedure = new LegsProcedure(legs, this.getWaypoint(segment.offset - 1), this._parentInstrument);

      let waypointIndex = segment.offset;
      while (procedure.hasNext()) {
        const waypoint = await procedure.getNext();
        if (waypoint) {
          this.addWaypoint(waypoint, ++waypointIndex, segment.type);
        }
      }
    }
  }

  /**
   * Builds an approach into the flight plan from indexes in the arrival airport information.
   */
  public async buildApproach(): Promise<void> {
    const legs = [];
    const destination = this.destinationAirfield;

    const approachIndex = this.procedureDetails.approachIndex;
    const approachTransitionIndex = this.procedureDetails.approachTransitionIndex;
    const destinationRunwayIndex = this.procedureDetails.destinationRunwayIndex;
    const destinationRunwayExtension = this.procedureDetails.destinationRunwayExtension;

    const destinationInfo = destination.infos as AirportInfo;

    if (approachIndex !== -1 && approachTransitionIndex !== -1) {
      const transition = destinationInfo.approaches[approachIndex].transitions[approachTransitionIndex].legs;
      legs.push(...transition);
    }

    if (approachIndex !== -1) {
      legs.push(...destinationInfo.approaches[approachIndex].finalLegs);
    }

    let { startIndex, segment } = this.truncateSegment(SegmentType.Approach);

    if (legs.length > 0 || approachIndex !== -1 || destinationRunwayIndex !== -1) {

      if (segment === FlightPlanSegment.Empty) {
        segment = this.addSegment(SegmentType.Approach);
        startIndex = segment.offset;

        const prevWaypointIndex = segment.offset - 1;
        if (prevWaypointIndex > 0) {
          this.getWaypoint(segment.offset - 1).endsInDiscontinuity = true;
        }
      }

      const procedure = new LegsProcedure(legs, this.getWaypoint(startIndex - 1), this._parentInstrument);

      let waypointIndex = startIndex;
      while (procedure.hasNext()) {
        const waypoint = await procedure.getNext();
        if (waypoint !== undefined) {
          this.addWaypoint(waypoint, ++waypointIndex, segment.type);
        }
      }

      let runway: OneWayRunway;
      if (approachIndex !== -1) {
        runway = this.getRunway(destinationInfo.oneWayRunways, destinationInfo.approaches[approachIndex].runway);
      }
      else if (destinationRunwayIndex !== -1) {
        runway = destinationInfo.oneWayRunways[destinationRunwayIndex];
      }

      if (runway) {
        const selectedRunwayMod = runway.designation.slice(-1);
        let selectedRunwayOutput = undefined;
        if (selectedRunwayMod == "L" || selectedRunwayMod == "C" || selectedRunwayMod == "R") {
          if (runway.designation.length == 2) {
            selectedRunwayOutput = "0" + runway.designation;
          } else {
            selectedRunwayOutput = runway.designation;
          }
        } else {
          if (runway.designation.length == 2) {
            selectedRunwayOutput = runway.designation;
          } else {
            selectedRunwayOutput = "0" + runway.designation;
          }
        }
        if (approachIndex === -1 && destinationRunwayIndex !== -1 && destinationRunwayExtension !== -1) {
          const runwayExtensionWaypoint = procedure.buildWaypoint(`RX${selectedRunwayOutput}`,
            Avionics.Utils.bearingDistanceToCoordinates(runway.direction + 180, destinationRunwayExtension, runway.beginningCoordinates.lat, runway.beginningCoordinates.long));
          this.addWaypoint(runwayExtensionWaypoint);
        }

        const runwayWaypoint = procedure.buildWaypoint(`RW${selectedRunwayOutput}`, runway.beginningCoordinates);
        runwayWaypoint.legAltitudeDescription = 1;
        runwayWaypoint.legAltitude1 = (runway.elevation * 3.28084) + 50;
        runwayWaypoint.isRunway = true;

        this.addWaypoint(runwayWaypoint);
      }
    }
  }

  /**
   * Truncates a flight plan segment. If the active waypoint index is current in the segment,
   * a discontinuity will be added at the end of the active waypoint and the startIndex will
   * point to the next waypoint in the segment after the active.
   * @param type The type of segment to truncate.
   * @returns A segment to add to and a starting waypoint index.
   */
  public truncateSegment(type: SegmentType): { startIndex: number, segment: FlightPlanSegment } {
    let segment = this.getSegment(type);
    const startIndex = this.findSegmentByWaypointIndex(this.activeWaypointIndex) === segment
      ? this.activeWaypointIndex + 1
      : segment.offset;

    if (segment !== FlightPlanSegment.Empty) {
      const finalIndex = segment.offset + segment.waypoints.length;
      if(startIndex < finalIndex){
        for (var i = startIndex; i < finalIndex; i++) {
          this.removeWaypoint(startIndex);
        }
      }
    }

    if (segment.waypoints.length === 0) {
      this.removeSegment(segment.type);
      segment = FlightPlanSegment.Empty;
    }
    else {
      segment.waypoints[Math.max((startIndex - 1) - segment.offset, 0)].endsInDiscontinuity = true;
    }

    return { startIndex, segment };
  }

  /**
   * Gets the runway information from a given runway name.
   * @param runways The collection of runways to search.
   * @param runwayName The runway name.
   * @returns The found runway, if any.
   */
  public getRunway(runways: OneWayRunway[], runwayName: string): OneWayRunway {
    if (runways.length > 0) {
      let runwayIndex;
      runwayName = runwayName.replace('RW', '');
      const runwayLetter = runwayName[runwayName.length - 1];
      if (runwayLetter === ' ' || runwayLetter === 'C') {
        const runwayDirection = runwayName.trim();
        runwayIndex = runways.findIndex(r => r.designation === runwayDirection || r.designation === `${runwayDirection}C`);
      }
      else {
        runwayIndex = runways.findIndex(r => r.designation === runwayName);
      }

      if (runwayIndex !== -1) {
        return runways[runwayIndex];
      }
    }
  }

  /**
   * Converts a plain object into a ManagedFlightPlan.
   * @param flightPlanObject The object to convert.
   * @param parentInstrument The parent instrument attached to this flight plan.
   * @returns The converted ManagedFlightPlan.
   */
  public static fromObject(flightPlanObject: any, parentInstrument: BaseInstrument): ManagedFlightPlan {
    let plan = Object.assign(new ManagedFlightPlan(), flightPlanObject);
    plan.setParentInstrument(parentInstrument);

    plan.directTo = Object.assign(new DirectTo(), plan.directTo);

    const mapObject = (obj: any, parentType?: string): any => {
      if (obj && obj.infos) {
        obj = Object.assign(new WayPoint(parentInstrument), obj);
      }

      if (obj && obj.coordinates) {
        switch (parentType) {
          case 'A':
            obj = Object.assign(new AirportInfo(parentInstrument), obj);
            break;
          case 'W':
            obj = Object.assign(new IntersectionInfo(parentInstrument), obj);
            break;
          case 'V':
            obj = Object.assign(new VORInfo(parentInstrument), obj);
            break;
          case 'N':
            obj = Object.assign(new NDBInfo(parentInstrument), obj);
            break;
          default:
            obj = Object.assign(new WayPointInfo(parentInstrument), obj);
        }

        obj.coordinates = Object.assign(new LatLongAlt(), obj.coordinates);
      }

      return obj;
    };

    const visitObject = (obj: any): any => {
      for (var key in obj) {
        if (typeof obj[key] === 'object' && obj[key] && obj[key].scroll === undefined) {
          if (Array.isArray(obj[key])) {
            visitArray(obj[key]);
          }
          else {
            visitObject(obj[key]);
          }

          obj[key] = mapObject(obj[key], obj.type);
        }
      }
    };

    const visitArray = (array) => {
      array.forEach((item, index) => {
        if (Array.isArray(item)) {
          visitArray(item);
        }
        else if (typeof item === 'object') {
          visitObject(item);
        }

        array[index] = mapObject(item);
      });
    };

    visitObject(plan);
    return plan;
  }
}
