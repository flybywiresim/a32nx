// Lives here as a Util for now so it can be cleanly separate from FMCMainDisplay.
// Needs access to the FlightPlanManager

class NXFlightGuidance {
    constructor(mcdu) {
        this.mcdu = mcdu;
        this.lastLeg = null;
        this.currentLeg = null;
        this.nextLeg = null;
        this.lastWaypoint = null;
        this.fromWaypoint = null;
        this.toWaypoint = null;
        this.nextToWaypoint = null;
        this.crossTrackError = null;
        this.trackAngleError = null;
    }

    update(_deltaTime) {
        const updatedWaypoints = this.updateWaypoints();
        if (updatedWaypoints) {
            this.updateLegs();
            console.log(`WPs: (${this.lastWaypoint ? this.lastWaypoint.ident : "null"}) - ${this.fromWaypoint ? this.fromWaypoint.ident : null} - ${this.toWaypoint ? this.toWaypoint.ident : null} - (${this.nextToWaypoint ? this.nextToWaypoint.ident : null})`);
            console.log(`Tracks: (${Math.round(lastLegTrack)}) - ${Math.round(activeLegTrack)} - (${Math.round(nextLegTrack)})`);
        }
        this.updateErrors();

        SimVar.SetSimVarValue("L:A32NX_FG_CROSS_TRACK_ERROR", "nautical miles", this.crossTrackError || 0);
        SimVar.SetSimVarValue("L:A32NX_FG_TRACK_ANGLE_ERROR", "degree", this.trackAngleError || 0);

        console.log(`XTE: ${Math.round(this.crossTrackError * 100) / 100}, TKE: ${Math.round(this.trackAngleError * 100) / 100}`);
    }

    updateWaypoints() {
        const waypointCount = this.mcdu.flightPlanManager.getWaypointsCount();
        const activeIndex = this.mcdu.flightPlanManager.getActiveWaypointIndex();
        if (waypointCount < activeIndex + 1 || activeIndex < 1) {
            const wasSet = this.fromWaypoint !== null;
            this.lastWaypoint = null;
            this.fromWaypoint = null;
            this.toWaypoint = null;
            this.nextToWaypoint = null;
            return wasSet;
        }

        const lastWaypoint = activeIndex > 0 ? this.mcdu.flightPlanManager.getWaypoint(activeIndex - 2) || null : null;
        const fromWaypoint = this.mcdu.flightPlanManager.getWaypoint(activeIndex - 1) || null;
        const toWaypoint = this.mcdu.flightPlanManager.getWaypoint(activeIndex) || null;
        const nextToWaypoint = waypointCount >= activeIndex + 3 ? this.mcdu.flightPlanManager.getWaypoint(activeIndex + 1) || null : null;

        if (fromWaypoint && this.fromWaypoint !== null && fromWaypoint.ident === this.fromWaypoint.ident) {
            // nothing has changed
            return false;
        }

        this.lastWaypoint = lastWaypoint;
        this.fromWaypoint = fromWaypoint;
        this.toWaypoint = toWaypoint;
        this.nextToWaypoint = nextToWaypoint;

        return true;
    }

    updateLegs() {
        if (this.fromWaypoint === null || this.toWaypoint === null) {
            this.lastLeg = null;
            this.currentLeg = null;
            this.nextLeg = null;
            return;
        }

        const activeLegTrack = this._getTrackFromWaypoints(this.fromWaypoint, this.toWaypoint);

        let lastLegTrack = null;
        if (this.lastWaypoint) {
            lastLegTrack = this._getTrackFromWaypoints(this.lastWaypoint, this.fromWaypoint);
        }

        let nextLegTrack = null;
        if (this.nextToWaypoint) {
            nextLegTrack = this._getTrackFromWaypoints(this.toWaypoint, this.nextToWaypoint);
        }
    }

    _getTrackFromWaypoints(from, to) {
        return Avionics.Utils.computeGreatCircleHeading(
            from.infos.coordinates,
            to.infos.coordinates
        );
    }

    updateErrors(_deltaTime) {
        if (this.fromWaypoint === null || this.toWaypoint === null) {
            this.crossTrackError = null;
            return;
        }

        const ppos = new LatLong(SimVar.GetSimVarValue("PLANE LATITUDE", "degree latitude"), SimVar.GetSimVarValue("PLANE LONGITUDE", "degree longitude"));

        // angle error
        const trueTrack = SimVar.GetSimVarValue("GPS GROUND TRUE TRACK", "degree");
        const desiredTrack = Avionics.Utils.computeGreatCircleHeading(
            this.fromWaypoint.infos.coordinates,
            this.toWaypoint.infos.coordinates,
        );
        const mod = (x, n) => x - Math.floor(x / n) * n;
        this.trackAngleError = mod((desiredTrack - trueTrack + 180), 360) - 180;

        // crosstrack error
        const bearingAC = Avionics.Utils.computeGreatCircleHeading(this.fromWaypoint.infos.coordinates, ppos);
        const bearingAB = Avionics.Utils.computeGreatCircleHeading(
            this.fromWaypoint.infos.coordinates,
            this.toWaypoint.infos.coordinates,
        );
        const distanceAC = Avionics.Utils.computeDistance(
            this.fromWaypoint.infos.coordinates,
            ppos,
        );
        const earthRadius = 3440.1;
        const deg2rad = Math.PI / 180;

        const desiredOffset = 0;
        const actualOffset = Math.asin(
            Math.sin(deg2rad * (distanceAC / earthRadius)) *
            Math.sin(deg2rad * (bearingAC - bearingAB))
        ) / deg2rad * earthRadius;
        this.crossTrackError = desiredOffset - actualOffset;
    }
}
