import { FlightPlanManager } from "../flightplanning/FlightPlanManager"
import { Leg, Geometry, TFLeg, Type1Transition } from "./Geometry";

const mod = (x: number, n: number) => x - Math.floor(x / n) * n;

/**
 * This class will guide the aircraft by predicting a flight path and
 * calculating the autopilot inputs to follow the predicted flight path.
 */
export class GuidanceManager {
    public flightPlanManager: FlightPlanManager;

    constructor(flightPlanManager: FlightPlanManager) {
        this.flightPlanManager = flightPlanManager;
    }

    getActiveLeg(): TFLeg | null {
        const activeIndex = this.flightPlanManager.getActiveWaypointIndex();
        const from = this.flightPlanManager.getWaypoint(activeIndex - 1);
        const to = this.flightPlanManager.getWaypoint(activeIndex);

        if (!from || !to) {
            return null;
        }

        return new TFLeg(from, to);
    }

    getNextLeg(): TFLeg | null {
        const activeIndex = this.flightPlanManager.getActiveWaypointIndex();
        const from = this.flightPlanManager.getWaypoint(activeIndex);
        const to = this.flightPlanManager.getWaypoint(activeIndex + 1);

        if (!from || !to) {
            return null;
        }

        return new TFLeg(from, to);
    }

    /**
     * The active leg path geometry, used for immediate autoflight.
     */
    getActiveLegPathGeometry(): Geometry | null {
        const activeLeg = this.getActiveLeg();
        const nextLeg = this.getNextLeg();

        if (!activeLeg) {
            return null;
        }

        const transitions = [];
        if (nextLeg) {
            const kts = Math.max(SimVar.GetSimVarValue('AIRSPEED TRUE', 'knots'), 150); // knots, i.e. nautical miles per hour

            // bank angle limits
            let bankAngle = 25;
            if (kts < 150) {
                bankAngle = 15 + Math.min(kts / 150, 1) * (25 - 15);
            } else if (kts > 300) {
                bankAngle = 25 - Math.min((kts - 300) / 150, 1) * (25 - 19);
            }

            // turn rate
            const degPerSec = 1091 * Math.tan(bankAngle * Avionics.Utils.DEG2RAD) / kts;
            const nms = kts / 60 / 60; // nautical miles per second
            const xKr = nms / degPerSec / Avionics.Utils.DEG2RAD; // turn radius

            const deltaPc = mod(nextLeg.bearing - activeLeg.bearing + 180, 360) - 180;
            const cw = deltaPc >= 0;

            transitions.push(new Type1Transition(
                activeLeg,
                nextLeg,
                xKr,
                cw,
            ));
        }

        return new Geometry(activeLeg, nextLeg, transitions);
    }

    /**
     * The full leg path geometry, used for the ND and F-PLN page.
     */
    getMultipleLegGeometry(): Geometry | null {
        return null;
    }
}
