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

import { ManagedFlightPlan } from "./ManagedFlightPlan";
import { FlightPlanSegment } from "./FlightPlanSegment";

/**
 * A segmented flight plan as defined by the reference.
 *
 * This includes two additional properties - {@link originAirport} and {@link destinationAirport} - that are not present
 * in the reference. This is purely done for convenience reasons.
 *
 * For now, {@link enrouteSegments} is inaccurate - since the flight plan manager only outputs waypoints and not airways.
 *
 * @see getSegmentedFlightPlan
 *
 * ## Reference
 *
 * AMM - 22-71-00 PB001, page 4
 */
type SegmentedFlightPlan = {
    originAirport: string,
    departureSegment?: DepartureSegment,
    enrouteSegments: WayPoint[],
    arrivalSegment: ArrivalSegment,
    destinationAirport: string
};

/**
 * The "Departure" segment of a segmented flight plan
 */
type DepartureSegment = Partial<{
    sid: FlightPlanSegment,
    sidEnrouteTransitionIndex: number,
}>;

/**
 * An "Enroute" segment of a segmented flight plan
 */
type EnrouteSegment = Partial<{
    /**
     * Only exists if there is no departure segment
     */
    initialFix: WayPoint,
    airway: WayPoint[],
    direct: WayPoint,
}>;

/**
 * The "Arrival" segment of a segmented flight plan
 */
type ArrivalSegment = Partial<{
    starEnrouteTransitionIndex: number,
    star: FlightPlanSegment,
    approachTransitionIndex: number,
    approach: FlightPlanSegment,
    missedApproach: FlightPlanSegment,
}>;

/**
 * Discontinuity
 */
type Discontinuity = null;

/**
 * Element of a strung flight plan
 */
type StringItem = WayPoint | Discontinuity;

/**
 * List of strung elements from a segmented flight plan
 */
type StrungSegments = StringItem[];

/**
 * Constructs a {@link SegmentedFlightPlan} from a {@link ManagedFlightPlan}
 *
 * @return a {@link SegmentedFlightPlan} if it could be constructed with the info in the {@link ManagedFlightPlan}, `null` otherwise.
 */
export function getSegmentedFlightPlan(flightPlan: ManagedFlightPlan): SegmentedFlightPlan | null {
    if (!flightPlan.originAirfield || !flightPlan.destinationAirfield) {
        return null;
    }

    const segmented: Partial<SegmentedFlightPlan> = {};

    const planDeparture = flightPlan.departure;

    if (planDeparture !== FlightPlanSegment.Empty) {
        // We have a departure - add a DepartureSegment
        segmented.departureSegment = {
            sid: planDeparture,
            sidEnrouteTransitionIndex: flightPlan.procedureDetails.departureTransitionIndex
        };
    }

    // TODO actually implemented enroute segments - for now this only adds fixes
    // Add our "enroute segments"
    segmented.enrouteSegments = flightPlan.enroute.waypoints;

    // Add an ArrivalSegment
    segmented.arrivalSegment = {
        star: flightPlan.arrival,
        starEnrouteTransitionIndex: flightPlan.procedureDetails.arrivalTransitionIndex,
        missedApproach: flightPlan.missed
    };

    return {
        originAirport: flightPlan.originAirfield.ident,
        departureSegment: segmented.departureSegment,
        enrouteSegments: segmented.enrouteSegments,
        arrivalSegment: segmented.arrivalSegment,
        destinationAirport: flightPlan.destinationAirfield.ident,
    };
}

/**
 * Strings a {@link SegmentedFlightPlan} according to the logic defined in the referencce
 *
 * ## Reference
 *
 * AMM - 22-71-00 PB001, page 4-5
 */
export function stringSegmentedFlightPlan(segmentedFlightPlan: SegmentedFlightPlan): StrungSegments {
    return null;
}
