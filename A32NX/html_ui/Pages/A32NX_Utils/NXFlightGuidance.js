// Lives here as a Util for now so it can be cleanly separate from FMCMainDisplay.
// Needs access to the FlightPlanManager

class NXFlightGuidance {
    constructor(mcdu) {
        this.mcdu = mcdu;
        this.lastAvail = null;
        this.lastXTE = null;
        this.lastTAE = null;
        this.lastPhi = null;
    }

    update(_deltaTime) {
        let available = false;

        const geometry = this.mcdu.guidanceManager.getActiveLegPathGeometry();
        if (geometry !== null) {
            const ppos = new LatLong(
                SimVar.GetSimVarValue("PLANE LATITUDE", "degree latitude"),
                SimVar.GetSimVarValue("PLANE LONGITUDE", "degree longitude"),
                0,
            );
            const trueTrack = SimVar.GetSimVarValue("GPS GROUND TRUE TRACK", "degree");

            const params = geometry.getGuidanceParameters(ppos, trueTrack);
            if (params) {
                switch (params.law) {
                    case 3:
                        // Lateral Path
                        const {
                            crossTrackError,
                            trackAngleError,
                            phiCommand
                        } = params;
                        if (!this.lastAvail) {
                            SimVar.SetSimVarValue("L:A32NX_FG_AVAIL", "bool", true);
                            this.lastAvail = true;
                        }
                        if (crossTrackError !== this.lastXTE) {
                            SimVar.SetSimVarValue("L:A32NX_FG_CROSS_TRACK_ERROR", "nautical miles", crossTrackError);
                            this.lastXTE = crossTrackError;
                        }
                        if (trackAngleError !== this.lastTAE) {
                            SimVar.SetSimVarValue("L:A32NX_FG_TRACK_ANGLE_ERROR", "degree", trackAngleError);
                            this.lastTAE = trackAngleError;
                        }
                        if (phiCommand !== this.lastPhi) {
                            SimVar.SetSimVarValue("L:A32NX_FG_PHI_COMMAND", "degree", phiCommand);
                            this.lastPhi = phiCommand;
                        }
                        /*console.log(
                            `XTE=${crossTrackError} TAE=${trackAngleError} phi=${phiCommand}`
                        );*/
                        break;
                }

                available = true;
            }

            if (geometry.shouldSequenceLeg(ppos)) {
                let wpIndex = this.mcdu.flightPlanManager.getActiveWaypointIndex();
                this.mcdu.flightPlanManager.setActiveWaypointIndex(++wpIndex);
            }
        }


        if (!available && this.lastAvail !== false) {
            SimVar.SetSimVarValue("L:A32NX_FG_AVAIL", "bool", false);
            SimVar.SetSimVarValue("L:A32NX_FG_CROSS_TRACK_ERROR", "nautical miles", 0);
            SimVar.SetSimVarValue("L:A32NX_FG_TRACK_ANGLE_ERROR", "degree", 0);
            SimVar.SetSimVarValue("L:A32NX_FG_PHI_COMMAND", "degree", 0);
            this.lastAvail = false;
            this.lastTAE = null;
            this.lastXTE = null;
            this.lastPhi = null;
        }
    }
}
