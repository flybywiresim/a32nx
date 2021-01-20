// Lives here as a Util for now so it can be cleanly separate from FMCMainDisplay.
// Needs access to the FlightPlanManager

class NXFlightGuidance {
    constructor(mcdu) {
        this.mcdu = mcdu;
        this.lastAvail = null;
        this.lastXTE = null;
        this.lastTAE = null;
    }

    update(_deltaTime) {
        let available = false;

        let wpIndex = this.mcdu.flightPlanManager.getActiveWaypointIndex();
        if (this.mcdu.guidanceManager.shouldSequence()) {
            this.mcdu.flightPlanManager.setActiveWaypointIndex(++wpIndex);
        }

        const activeSegment = this.mcdu.guidanceManager.getActiveSegment();

        if (activeSegment) {
            const ppos = new LatLong(
                SimVar.GetSimVarValue("PLANE LATITUDE", "degree latitude"),
                SimVar.GetSimVarValue("PLANE LONGITUDE", "degree longitude"),
                0,
            );
            const trueTrack = SimVar.GetSimVarValue("GPS GROUND TRUE TRACK", "degree");

            const result = activeSegment.calculateErrors(ppos, trueTrack);
            if (result) {
                let modified = false;
                if (result.crossTrackError !== this.lastXTE) {
                    SimVar.SetSimVarValue("L:A32NX_FG_CROSS_TRACK_ERROR", "nautical miles", result.crossTrackError);
                    this.lastXTE = result.crossTrackError;
                    modified = true;
                }
                if (result.trackAngleError !== this.lastTAE) {
                    SimVar.SetSimVarValue("L:A32NX_FG_TRACK_ANGLE_ERROR", "degree", result.trackAngleError);
                    this.lastTAE = result.trackAngleError;
                    modified = true;
                }
                if (this.lastAvail !== true) {
                    SimVar.SetSimVarValue("L:A32NX_FG_AVAIL", "bool", true);
                    this.lastAvail = true;
                }
                if (modified) {
                    console.log(`> Guidance: XTE=${this.lastXTE.toFixed(3)}, TAE=${this.lastTAE.toFixed(1)}`);
                }
                available = true;
            }
        }

        if (!available && this.lastAvail !== false) {
            SimVar.SetSimVarValue("L:A32NX_FG_AVAIL", "bool", false);
            SimVar.SetSimVarValue("L:A32NX_FG_CROSS_TRACK_ERROR", "nautical miles", 0);
            SimVar.SetSimVarValue("L:A32NX_FG_TRACK_ANGLE_ERROR", "degree", 0);
            this.lastAvail = false;
            this.lastTAE = null;
            this.lastXTE = null;
        }
    }
}
