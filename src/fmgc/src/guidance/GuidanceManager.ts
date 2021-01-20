import { FlightPlanManager } from "../flightplanning/FlightPlanManager"
import { Segment, StraightTrackSegment, TurnSegment } from "./HPath"

/**
 * This class will guide the aircraft by predicting a flight path and
 * calculating the autopilot inputs to follow the predicted flight path.
 */
export class GuidanceManager {
    public flightPlanManager: FlightPlanManager;

    constructor(flightPlanManager: FlightPlanManager) {
        this.flightPlanManager = flightPlanManager;
    }

    /**
     * Returns a list of all continuous segments from the flight plan.
     * A continuous segment is terminated either the the destination airport or
     * by a discontinuity.
     */
    public getSegments(): Segment[][] {
        const waypointSegments = this.flightPlanManager.getContinuousSegments();

        return waypointSegments.map(points => {
            const segments: Segment[] = [];

            if (points.length > 2) {
                let aircraftAt = null;
                for (let i = 1; i < points.length - 1; i++) {
                    const lastSegment = segments[segments.length - 1];
                    const pPrev = points[i - 1];
                    const p = points[i];
                    const pNext = points[i + 1];

                    const hdgPrev = Avionics.Utils.computeGreatCircleHeading(
                        pPrev.infos.coordinates,
                        p.infos.coordinates,
                    );
                    const hdgNext = Avionics.Utils.computeGreatCircleHeading(
                        p.infos.coordinates,
                        pNext.infos.coordinates,
                    );
                    const mod = (x, n) => x - Math.floor(x / n) * n;
                    const deltaPc = mod(hdgNext - hdgPrev + 180, 360) - 180;

                    const kts = Math.max(SimVar.GetSimVarValue("GPS GROUND SPEED", "knots"), 220); // knots, i.e. nautical miles per hour
                    const nms = kts / 60 / 60; // nautical miles per second

                    const degPerSec = 2.5;
                    const xKr = nms / degPerSec / Avionics.Utils.DEG2RAD; // turn radius
                    const xs = xKr * Math.tan(Avionics.Utils.DEG2RAD * Math.abs(deltaPc / 2));

                    const { lat: pLat, long: pLong } = p.infos.coordinates;
                    const turnStart = Avionics.Utils.bearingDistanceToCoordinates(mod(hdgPrev + 180, 360), xs, pLat, pLong);
                    const turnEnd = Avionics.Utils.bearingDistanceToCoordinates(hdgNext, xs, pLat, pLong);

                    // optimize very trivial turns away
                    if (Math.abs(deltaPc) < 1) {
                        segments.push(
                            new StraightTrackSegment(
                                p,
                                lastSegment ? lastSegment.to : pPrev.infos.coordinates,
                                new LatLongAlt(turnEnd.lat, turnEnd.long, 0),
                            ),
                        )
                        aircraftAt = turnEnd;
                        continue;
                    }

                    const tp = Avionics.Utils.bearingDistanceToCoordinates(
                        deltaPc >= 0 ? mod(hdgPrev + 90, 360) : mod(hdgPrev - 90, 360),
                        xKr,
                        turnStart.lat,
                        turnStart.long
                    ).toLatLong();

                    segments.push(
                        new StraightTrackSegment(
                            p,
                            lastSegment ? lastSegment.to : pPrev.infos.coordinates,
                            new LatLongAlt(turnStart.lat, turnStart.long, 0),
                        ),
                    )

                    const bisectorLatLong = Avionics.Utils.bearingDistanceToCoordinates(
                        mod(
                            (deltaPc >= 0 ? mod(hdgPrev - 90, 360) : mod(hdgPrev + 90, 360)) +
                            deltaPc/ 2,
                        360),
                        xKr,
                        tp.lat,
                        tp.long
                    ).toLatLong();

                    // this will be the curve
                    const bisector = new LatLongAlt(bisectorLatLong.lat, bisectorLatLong.long, 0);
                    const cw = deltaPc >= 0;
                    segments.push(
                        new TurnSegment(
                            p,
                            turnStart,
                            bisector,
                            xKr,
                            cw,
                        ),
                        new TurnSegment(
                            pNext,
                            bisector,
                            turnEnd,
                            xKr,
                            cw,
                        ),
                    );

                    aircraftAt = turnEnd;
                }

                // terminate the last straight segment
                const lastPoint = points[points.length - 1];
                segments.push(
                    new StraightTrackSegment(
                        lastPoint,
                        aircraftAt,
                        lastPoint.infos.coordinates, // todo alt
                    ),
                )
            }
            else if (points.length == 2) {
                const [from, to] = points;
                segments.push(new StraightTrackSegment(
                    points[1],
                    from.infos.coordinates,
                    to.infos.coordinates,
                ));
            }

            return segments;
        });
    }

    public getActiveSegment(): Segment | null {
        const segments = this.getSegmentsForActiveLeg();

        // handle the trivial cases first
        if (!segments.length) {
            return null;
        }
        if (segments.length === 1) {
            return segments[0];
        }

        const ppos = new LatLongAlt(
            SimVar.GetSimVarValue("PLANE LATITUDE", "degree latitude"),
            SimVar.GetSimVarValue("PLANE LONGITUDE", "degree longitude"),
            0,
        );
        const abeam = segments.find(s => s.isAbeam(ppos));
        if (abeam) {
            return abeam;
        }

        // check if we're possibly abeam of the first segment of the next leg,
        // and sequencing is about to happen
        const nextSegments = this.getSegmentsForLeg(this.flightPlanManager.getActiveWaypointIndex() + 1);
        if (nextSegments.length && nextSegments[0].isAbeam(ppos)) {
            return nextSegments[0];
        }

        return null;
    }

    public getSegmentsForActiveLeg(): Segment[] {
        const activeWaypointIndex = this.flightPlanManager.getActiveWaypointIndex();
        return this.getSegmentsForLeg(activeWaypointIndex);
    }

    public getSegmentsForLeg(waypointIndex: number): Segment[] {
        const segmentGroups = this.getSegments();
        if (!segmentGroups.length) {
            return [];
        }
        const waypoint = this.flightPlanManager.getWaypoint(waypointIndex);
        if (!waypoint) {
            return [];
        }
        const ident = waypoint.infos ? waypoint.infos.ident : waypoint.ident;
        const segments = segmentGroups.find(g => g.some(s => s.waypoint.infos.ident === ident));
        if (!segments) {
            return [];
        }
        return segments.filter(s => s.waypoint.infos.ident === ident);
    }

    public shouldSequence(): boolean {
        const current = this.getSegmentsForActiveLeg();
        const ppos = new LatLongAlt(
            SimVar.GetSimVarValue("PLANE LATITUDE", "degree latitude"),
            SimVar.GetSimVarValue("PLANE LONGITUDE", "degree longitude"),
            0,
        );
        const trueTrack = SimVar.GetSimVarValue("GPS GROUND TRUE TRACK", "degree");
        if (current.some(s => s.isAbeam(ppos))) {
            return false;
        }

        const activeWaypointIndex = this.flightPlanManager.getActiveWaypointIndex();
        const next = this.getSegmentsForLeg(activeWaypointIndex + 1);
        for(const segment of next) {
            if (!segment.isAbeam(ppos)) {
                continue;
            }
            const { crossTrackError } = segment.calculateErrors(ppos, trueTrack);
            // we need to be at least within 5nm to sequence
            if (Math.abs(crossTrackError) < 5.0) {
                return true;
            }
        }
        return false;
    }
}