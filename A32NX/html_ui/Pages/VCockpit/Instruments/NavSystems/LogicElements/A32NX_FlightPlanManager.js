class FlightPlanManager {
    constructor(_instrument) {
        this._waypoints = [[], []];
        this._approachWaypoints = [];
        this._departureWaypointSize = 0;
        this._arrivalWaypointSize = 0;
        this._approachWaypointSize = 0;
        this._activeWaypointIndex = 0;
        this._onFlightPlanUpdateCallbacks = [];
        this.decelPrevIndex = -1;
        this._lastDistanceToPreviousActiveWaypoint = 0;
        this._isGoingTowardPreviousActiveWaypoint = false;
        this._resetTimer = 0;
        this._updateTimer = 0;
        this._isRegistered = false;
        this._isRegisteredAndLoaded = false;
        this._currentFlightPlanIndex = 0;
        this._activeWaypointIdentHasChanged = false;
        this._waypointReachedAt = SimVar.GetGlobalVarValue("ZULU TIME", "seconds");
        this._decelDistance = 32; // TODO: properly calculate this value
        this._decelReached = false;
        this._lastWaypointData = 0;
        this._timeLastSimVarCall = 0;
        this._gpsActiveWaypointIndexHasChanged = false;
        this._timeLastActiveWaypointIndexSimVarCall = 0;
        this._isLoadedApproachTimeLastSimVarCall = 0;
        this._isActiveApproachTimeLastSimVarCall = 0;
        this._approachActivated = false;
        this._currentFlightPlanVersion = -1;
        this._newFlightPlanVersion = -1;
        this._currentFlightPlanApproachVersion = -1;
        FlightPlanManager.DEBUG_INSTANCE = this;
        this.instrument = _instrument;
        this.registerListener();
    }
    addHardCodedConstraints(wp) {
        return;
        const icao = wp.icao;
        if (icao.indexOf("D0") != -1) {
            wp.legAltitude1 = 500;
        } else if (icao.indexOf("BOANE") != -1) {
            wp.legAltitude1 = 11000;
            wp.speedConstraint = 250;
        } else if (icao.indexOf("NEHOS") != -1) {
            wp.legAltitude1 = 8000;
            wp.speedConstraint = 230;
        } else if (icao.indexOf("GRIFY") != -1) {
            wp.legAltitude1 = 6000;
            wp.speedConstraint = 210;
        } else if (icao.indexOf("WK1KSEAHELZR") != -1) {
            wp.legAltitude1 = 4000;
        } else if (icao.indexOf("WK1KSEAKARFO") != -1) {
            wp.legAltitude1 = 3200;
        } else if (icao.indexOf("WK1KSEADGLAS") != -1) {
            wp.legAltitude1 = 1900;
        }
    }

    updateWaypointDistances(approach) {

        // TODO: This should share code with _loadWaypoints but since flight plan manager is rewritten in any case soonly,
        // this wouldn't be worth the effort.
        const activeIdent = this.getActiveWaypointIdent();
        const groundSpeed = SimVar.GetSimVarValue("GPS GROUND SPEED", "knots") < 100 ? 400 : SimVar.GetSimVarValue("GPS GROUND SPEED", "knots");
        const utcTime = SimVar.GetGlobalVarValue("ZULU TIME", "seconds");
        const waypoints = approach ? this._approachWaypoints : this._waypoints[this._currentFlightPlanIndex];
        const activeIndex = waypoints.findIndex(wp => {
            return wp && wp.ident === activeIdent;
        });
        const planeCoord = new LatLong(SimVar.GetSimVarValue("PLANE LATITUDE", "degree latitude"), SimVar.GetSimVarValue("PLANE LONGITUDE", "degree longitude"));
        const lastWaypoint = this._waypoints[this._currentFlightPlanIndex][this._waypoints[this._currentFlightPlanIndex].length - 2];
        for (let i = 0; i < waypoints.length; i++) {
            const waypoint = waypoints[i];
            if (waypoint.ident === activeIdent) {
                waypoint.liveDistanceTo = Avionics.Utils.computeGreatCircleDistance(planeCoord, waypoint.infos.coordinates);
                waypoint.liveETATo = waypoint.liveDistanceTo / groundSpeed * 3600;
                waypoint.liveUTCTo = utcTime + waypoint.liveETATo;
                if (approach) {
                    const prevWp = (i > 1 ? waypoints[i - 1] : lastWaypoint);
                    waypoint.distance = Avionics.Utils.computeGreatCircleDistance(prevWp.infos.coordinates, waypoint.infos.coordinates);
                    waypoint.cumulativeDistanceInFP = prevWp.cumulativeDistanceInFP + waypoint.distance;
                }
            } else if (!approach && activeIndex >= 0 && i > activeIndex) {
                const prevWp = waypoints[i - 1];
                waypoint.distance = Avionics.Utils.computeGreatCircleDistance(prevWp.infos.coordinates, waypoint.infos.coordinates);
                waypoint.liveDistanceTo = prevWp.liveDistanceTo + waypoint.distance;
                waypoint.liveETATo = waypoint.liveDistanceTo / groundSpeed * 3600;
                waypoint.liveUTCTo = utcTime + waypoint.liveETATo;
            } else if (approach) {
                const prevWp = (i > 1 ? waypoints[i - 1] : lastWaypoint);
                waypoint.distance = Avionics.Utils.computeGreatCircleDistance(prevWp.infos.coordinates, waypoint.infos.coordinates);
                if (waypoint.ident != "USER") {
                    waypoint.cumulativeDistanceInFP = prevWp.cumulativeDistanceInFP + waypoint.distance;
                }
                waypoint.bearing = Avionics.Utils.computeGreatCircleHeading(prevWp.infos.coordinates, waypoint.infos.coordinates);
                if (activeIndex < 0 || (activeIndex >= 0 && i > activeIndex)) {
                    waypoint.liveDistanceTo = prevWp.liveDistanceTo + waypoint.distance;
                    waypoint.liveETATo = waypoint.liveDistanceTo / groundSpeed * 3600;
                    waypoint.liveUTCTo = utcTime + waypoint.liveETATo;
                }
                if (i === waypoints.length - 1) {
                    const destWp = this.getWaypoint(this.getWaypointsCount() - 1);
                    destWp.distanceInFP = Avionics.Utils.computeGreatCircleDistance(waypoint.infos.coordinates , destWp.infos.coordinates);
                }
            } else {
                waypoint.liveDistanceTo = 0;
                waypoint.liveETATo = 0;
                waypoint.liveUTCTo = 0;
            }
        }
        const destination = this.getDestination();
        if (destination && approach) {
            if (waypoints.length > 0) {
                const lastWaypoint = waypoints[waypoints.length - 1];
                if (lastWaypoint) {
                    const distance = Math.round(Avionics.Utils.computeGreatCircleDistance(lastWaypoint.infos.coordinates, destination.infos.coordinates));
                    destination.cumulativeDistanceInFP = lastWaypoint.cumulativeDistanceInFP + distance;
                    destination.liveDistanceTo = lastWaypoint.liveDistanceTo + distance;
                    destination.liveETATo = lastWaypoint.liveETATo + (distance / groundSpeed * 3600);
                    destination.liveUTCTo = utcTime + destination.liveETATo;
                }
            }
            if (!this.getApproachWaypointsCount() || (this.getApproachWaypointsCount() > 0 && approach)) {
                if (this.decelWaypoint && this.decelWaypoint.prevWp) {
                    const prevWp = this.decelWaypoint.prevWp;
                    const dist = Avionics.Utils.computeGreatCircleDistance(planeCoord, this.decelWaypoint.infos.coordinates);
                    this.decelWaypoint.liveDistanceTo = prevWp.liveDistanceTo ? prevWp.liveDistanceTo + this.decelWaypoint.distanceInFP : dist;
                    this.decelWaypoint.liveETATo = (this._decelReached ? this._waypointReachedAt : this.decelWaypoint.liveDistanceTo / groundSpeed * 3600);
                    this.decelWaypoint.liveUTCTo = utcTime + this.decelWaypoint.liveETATo;
                }
            }
        }
    }
    update(_deltaTime) {
        if (this._resetTimer > 0) {
            this._resetTimer -= _deltaTime;
            if (this._resetTimer <= 0) {
                this._resetTimer = 0;
                this._activeWaypointIdentHasChanged = false;
                this._gpsActiveWaypointIndexHasChanged = false;
            }
        }
        this._updateTimer += _deltaTime;
        if (this._updateTimer >= 1000) {
            this._updateTimer = 0;
            const prevWaypoint = this.getPreviousActiveWaypoint();
            if (prevWaypoint) {
                const planeCoordinates = new LatLong(SimVar.GetSimVarValue("PLANE LATITUDE", "degree latitude"), SimVar.GetSimVarValue("PLANE LONGITUDE", "degree longitude"));
                if (isFinite(planeCoordinates.lat) && isFinite(planeCoordinates.long)) {
                    const dist = Avionics.Utils.computeGreatCircleDistance(planeCoordinates, prevWaypoint.infos.coordinates);
                    if (isFinite(dist)) {
                        if (dist < this._lastDistanceToPreviousActiveWaypoint) {
                            this._isGoingTowardPreviousActiveWaypoint = true;
                        } else {
                            this._isGoingTowardPreviousActiveWaypoint = false;
                        }
                        this._lastDistanceToPreviousActiveWaypoint = dist;
                        if ((this._activeWaypointIdentHasChanged || this._gpsActiveWaypointIndexHasChanged) && this._resetTimer <= 0) {
                            this._resetTimer = 3000;
                        }
                        return;
                    }
                }
            }
            if ((this._activeWaypointIdentHasChanged || this._gpsActiveWaypointIndexHasChanged) && this._resetTimer <= 0) {
                this._resetTimer = 3000;
            }
            this._isGoingTowardPreviousActiveWaypoint = false;
        }
    }
    onCurrentGameFlightLoaded(_callback) {
        if (this._isRegisteredAndLoaded) {
            _callback();
            return;
        }
        this._onCurrentGameFlightLoaded = _callback;
    }
    registerListener() {
        if (this._isRegistered) {
            return;
        }
        this._isRegistered = true;
        RegisterViewListener("JS_LISTENER_FLIGHTPLAN");
        setTimeout(() => {
            Coherent.call("LOAD_CURRENT_GAME_FLIGHT");
            Coherent.call("LOAD_CURRENT_ATC_FLIGHTPLAN");
            setTimeout(() => {
                this._isRegisteredAndLoaded = true;
                if (this._onCurrentGameFlightLoaded) {
                    this._onCurrentGameFlightLoaded();
                }
            }, 200);
        }, 200);
    }

    _getWaypointLocalStorageKey(waypoint) {
        return `${waypoint.ident}_${this.indexOfWaypoint(waypoint)}`;
    }

    setOrGetLegAltitudeDescription(waypoint, newValue) {
        // Create a unique key for the current waypoint
        const key = this._getWaypointLocalStorageKey(waypoint);
        // Save if not saved otherwise use the saved one
        if (!localStorage.getItem(key)) {
            localStorage.setItem(key, newValue);
            waypoint.legAltitudeDescription = newValue;
        }
        const val = localStorage.getItem(key);
        waypoint.legAltitudeDescription = parseInt(val);
    }

    setLegAltitudeDescription(waypoint, newValue) {
        const key = this._getWaypointLocalStorageKey(waypoint);
        localStorage.setItem(key, newValue);
        waypoint.legAltitudeDescription = newValue;
    }

    _loadWaypoints(data, currentWaypoints, approach, callback) {
        const waypoints = [];
        const todo = data.length;
        let done = 0;
        const groundSpeed = SimVar.GetSimVarValue("GPS GROUND SPEED", "knots") < 100 ? 400 : SimVar.GetSimVarValue("GPS GROUND SPEED", "knots");
        const utcTime = SimVar.GetGlobalVarValue("ZULU TIME", "seconds");
        const activeIdent = this.getActiveWaypointIdent();
        const activeIndex = data.findIndex(wp => {
            return wp && wp.ident === activeIdent;
        });
        if (data.length > 1 && !approach) {
            this._lastWaypointData = data[data.length - 2];
        }
        for (let i = 0; i < data.length; i++) {
            const currData = data[i];
            if (currData.ident === activeIdent) {
                const planeCoord = new LatLong(SimVar.GetSimVarValue("PLANE LATITUDE", "degree latitude"), SimVar.GetSimVarValue("PLANE LONGITUDE", "degree longitude"));
                currData.liveDistanceTo = Avionics.Utils.computeGreatCircleDistance(planeCoord, currData.lla);
                currData.liveETATo = currData.liveDistanceTo / groundSpeed * 3600;
                currData.liveUTCTo = utcTime + currData.liveETATo;
                if (approach) {
                    const prevData = (i > 1 ? data[i - 1] : this._lastWaypointData);
                    currData.distance = Avionics.Utils.computeGreatCircleDistance(prevData.lla, currData.lla);
                    currData.cumulativeDistance = prevData.cumulativeDistance + currData.distance;
                }
            } else if (!approach && activeIndex >= 0 && i > activeIndex) {
                const prevData = data[i - 1];
                currData.distance = Avionics.Utils.computeGreatCircleDistance(prevData.lla, currData.lla);
                currData.liveDistanceTo = prevData.liveDistanceTo + currData.distance;
                currData.liveETATo = currData.liveDistanceTo / groundSpeed * 3600;
                currData.liveUTCTo = utcTime + currData.liveETATo;
            } else if (approach) {
                const prevData = (i > 1 ? data[i - 1] : this._lastWaypointData);
                currData.distance = Avionics.Utils.computeGreatCircleDistance(prevData.lla, currData.lla);
                if (currData.ident != "USER") {
                    currData.cumulativeDistance = prevData.cumulativeDistance + currData.distance;
                }
                currData.bearing = Avionics.Utils.computeGreatCircleHeading(prevData.lla, currData.lla);
                if (activeIndex < 0 || (activeIndex >= 0 && i > activeIndex)) {
                    currData.liveDistanceTo = prevData.liveDistanceTo + currData.distance;
                    currData.liveETATo = currData.liveDistanceTo / groundSpeed * 3600;
                    currData.liveUTCTo = utcTime + currData.liveETATo;
                }
                if (i === data.length - 1) {
                    const destWp = this.getWaypoint(this.getWaypointsCount() - 1);
                    destWp.distanceInFP = Avionics.Utils.computeGreatCircleDistance(currData.lla , destWp.infos.coordinates);
                }
            } else {
                currData.liveDistanceTo = 0;
                currData.liveETATo = 0;
                currData.liveUTCTo = 0;
            }
            const waypointData = data[i];
            const ii = i;
            if (waypointData.icao[0] === " " || waypointData.icao[0] == "U" || waypointData.icao[0] == "R" || waypointData.ident === "CUSTD") {
                const wp = new WayPoint(this.instrument);
                wp.infos = new IntersectionInfo(this.instrument);
                wp.icao = "U " + waypointData.ident;
                wp.infos.icao = wp.icao;
                wp.ident = waypointData.ident;
                wp.infos.ident = waypointData.ident;
                wp.infos.coordinates = new LatLongAlt(waypointData.lla);
                wp.latitudeFP = waypointData.lla.lat;
                wp.longitudeFP = waypointData.lla.long;
                wp.altitudeinFP = waypointData.lla.alt * 3.2808;
                wp.altitudeModeinFP = waypointData.altitudeMode;
                wp.bearingInFP = isFinite(waypointData.heading) ? waypointData.heading : 0;
                wp.distanceInFP = waypointData.distance;
                wp.cumulativeDistanceInFP = waypointData.cumulativeDistance;
                wp.infos.totalDistInFP = waypointData.cumulativeDistance;
                wp.estimatedTimeOfArrivalFP = waypointData.estimatedTimeOfArrival;
                wp.estimatedTimeEnRouteFP = waypointData.estimatedTimeEnRoute;
                wp.cumulativeEstimatedTimeEnRouteFP = waypointData.cumulativeEstimatedTimeEnRoute;
                wp.infos.totalTimeInFP = waypointData.estimatedTimeEnRoute;
                wp.infos.airwayIdentInFP = waypointData.airwayIdent;
                wp.speedConstraint = waypointData.speedConstraint;
                wp.transitionLLas = waypointData.transitionLLas;
                wp.liveDistanceTo = waypointData.liveDistanceTo;
                wp.liveETATo = waypointData.liveETATo;
                wp.liveUTCTo = waypointData.liveUTCTo;
                if (wp.speedConstraint > 0) {
                }
                if (wp.speedConstraint > 400) {
                    wp.speedConstraint = -1;
                }
                if ((ii > 0 && ii <= this.getDepartureWaypointsCount()) && (wp.altitudeinFP >= 500)) {
                    this.setOrGetLegAltitudeDescription(wp, 2);
                    wp.legAltitude1 = wp.altitudeinFP;
                } else if ((ii < (data.length - 1) && ii >= (data.length - 1 - this.getArrivalWaypointsCount())) && (wp.altitudeinFP >= 500)) {
                    this.setOrGetLegAltitudeDescription(wp, 2);
                    wp.legAltitude1 = wp.altitudeinFP;
                } else if (ii > 0 && ii < data.length - 1) {
                    this.setOrGetLegAltitudeDescription(wp, 2);
                    wp.legAltitude1 = wp.altitudeinFP;
                }
                this.addHardCodedConstraints(wp);
                waypoints[ii] = wp;
                done++;
            } else {
                if (currentWaypoints[ii] &&
                    currentWaypoints[ii].infos &&
                    waypointData.icao[0] != "U" &&
                    currentWaypoints[ii].infos.icao === waypointData.icao) {
                    const v = currentWaypoints[ii];
                    waypoints[ii] = v;
                    v.bearingInFP = isFinite(waypointData.heading) ? waypointData.heading : 0;
                    v.distanceInFP = waypointData.distance;
                    v.altitudeinFP = waypointData.lla.alt * 3.2808;
                    v.altitudeModeinFP = waypointData.altitudeMode;
                    v.estimatedTimeOfArrivalFP = waypointData.estimatedTimeOfArrival;
                    v.estimatedTimeEnRouteFP = waypointData.estimatedTimeEnRoute;
                    v.cumulativeEstimatedTimeEnRouteFP = waypointData.cumulativeEstimatedTimeEnRoute;
                    v.cumulativeDistanceInFP = waypointData.cumulativeDistance;
                    v.infos.totalDistInFP = waypointData.cumulativeDistance;
                    v.infos.totalTimeInFP = waypointData.estimatedTimeEnRoute;
                    v.infos.airwayIdentInFP = waypointData.airwayIdent;
                    v.speedConstraint = waypointData.speedConstraint;
                    v.transitionLLas = waypointData.transitionLLas;
                    v.liveDistanceTo = waypointData.liveDistanceTo;
                    v.liveETATo = waypointData.liveETATo;
                    v.liveUTCTo = waypointData.liveUTCTo;
                    if (v.speedConstraint > 0) {
                    }
                    if (v.speedConstraint > 400) {
                        v.speedConstraint = -1;
                    }
                    if ((ii > 0 && ii <= this.getDepartureWaypointsCount()) && (v.altitudeinFP >= 500)) {
                        this.setOrGetLegAltitudeDescription(v, 2);
                        v.legAltitude1 = v.altitudeinFP;
                    } else if ((ii < (data.length - 1) && ii >= (data.length - 1 - this.getArrivalWaypointsCount())) && (v.altitudeinFP >= 500)) {
                        this.setOrGetLegAltitudeDescription(v, 3);
                        v.legAltitude1 = v.altitudeinFP;
                    } else if (ii > 0 && ii < data.length - 1) {
                        this.setOrGetLegAltitudeDescription(v, 0);
                        v.legAltitude1 = v.altitudeinFP;
                    }
                    this.addHardCodedConstraints(v);
                    done++;
                } else {
                    this.instrument.facilityLoader.getFacility(waypointData.icao).then((v) => {
                        done++;
                        waypoints[ii] = v;
                        if (v) {
                            v.infos.icao = v.icao;
                            v.infos.ident = v.ident;
                            v.infos.UpdateAirways();
                            const matchingCurrentWaypoint = currentWaypoints.find(wp => wp.infos.icao === v.infos.icao);
                            if (matchingCurrentWaypoint) {
                                v.infos.airwayIn = matchingCurrentWaypoint.infos.airwayIn;
                                v.infos.airwayOut = matchingCurrentWaypoint.infos.airwayOut;
                            }
                            v.latitudeFP = waypointData.lla.lat;
                            v.longitudeFP = waypointData.lla.long;
                            v.altitudeinFP = waypointData.lla.alt * 3.2808;
                            v.altitudeModeinFP = waypointData.altitudeMode;
                            v.bearingInFP = isFinite(waypointData.heading) ? waypointData.heading : 0;
                            v.distanceInFP = waypointData.distance;
                            v.cumulativeDistanceInFP = waypointData.cumulativeDistance;
                            v.infos.totalDistInFP = waypointData.cumulativeDistance;
                            v.estimatedTimeOfArrivalFP = waypointData.estimatedTimeOfArrival;
                            v.estimatedTimeEnRouteFP = waypointData.estimatedTimeEnRoute;
                            v.cumulativeEstimatedTimeEnRouteFP = waypointData.cumulativeEstimatedTimeEnRoute;
                            v.infos.totalTimeInFP = waypointData.estimatedTimeEnRoute;
                            v.infos.airwayIdentInFP = waypointData.airwayIdent;
                            v.speedConstraint = waypointData.speedConstraint;
                            v.transitionLLas = waypointData.transitionLLas;
                            v.liveDistanceTo = waypointData.liveDistanceTo;
                            v.liveETATo = waypointData.liveETATo;
                            v.liveUTCTo = waypointData.liveUTCTo;
                            if (v.speedConstraint > 0) {
                            }
                            if (v.speedConstraint > 400) {
                                v.speedConstraint = -1;
                            }
                            if ((ii > 0 && ii <= this.getDepartureWaypointsCount()) && (v.altitudeinFP >= 500)) {
                                this.setOrGetLegAltitudeDescription(v, 2);
                                v.legAltitude1 = v.altitudeinFP;
                            } else if ((ii < (data.length - 1) && ii >= (data.length - 1 - this.getArrivalWaypointsCount())) && (v.altitudeinFP >= 500)) {
                                this.setOrGetLegAltitudeDescription(v, 3);
                                v.legAltitude1 = v.altitudeinFP;
                            } else if (ii > 0 && ii < data.length - 1) {
                                this.setOrGetLegAltitudeDescription(v, 0);
                                v.legAltitude1 = v.altitudeinFP;
                            }
                            this.addHardCodedConstraints(v);
                        }
                    });
                }
            }
        }
        const destination = this.getDestination();
        if (destination) {
            if (data.length > 0) {
                const lastWaypointData = data[data.length - 1];
                if (lastWaypointData) {
                    const distance = Math.round(Avionics.Utils.computeGreatCircleDistance(lastWaypointData.lla, destination.infos.coordinates));
                    destination.cumulativeDistanceInFP = lastWaypointData.cumulativeDistance + distance;
                    destination.liveDistanceTo = lastWaypointData.liveDistanceTo + distance;
                    destination.liveETATo = lastWaypointData.liveETATo + (distance / groundSpeed * 3600);
                    destination.liveUTCTo = utcTime + destination.liveETATo;
                }
            }
            if (SimVar.GetSimVarValue("L:FLIGHTPLAN_USE_DECEL_WAYPOINT", "number") === 1) {
                if (!this.getApproachWaypointsCount() || (this.getApproachWaypointsCount() > 0 && approach)) {
                    setTimeout(() => {
                        if (!this.decelWaypoint) {
                            this.decelWaypoint = new WayPoint(this.instrument);
                            this.decelWaypoint.infos = new IntersectionInfo(this.instrument);
                        }
                        this.decelWaypoint.icao = "";
                        this.decelWaypoint.infos.icao = this.decelWaypoint.icao;
                        this.decelWaypoint.ident = "(DECEL)";
                        this.decelWaypoint.infos.name = "(DECEL)";
                        this.decelWaypoint.infos.ident = this.decelWaypoint.ident;
                        const decelPosition = this.getCoordinatesAtNMFromDestinationAlongFlightPlan(this._decelDistance);
                        if (decelPosition) {
                            const decelCoordinates = decelPosition.lla;
                            this.decelWaypoint.infos.coordinates = new LatLongAlt(decelCoordinates.lat, decelCoordinates.long);
                            this.decelWaypoint.latitudeFP = this.decelWaypoint.infos.coordinates.lat;
                            this.decelWaypoint.longitudeFP = this.decelWaypoint.infos.coordinates.long;
                            this.decelWaypoint.altitudeinFP = decelPosition.alt;
                            this.decelWaypoint.cumulativeDistanceInFP = decelPosition.cumulativeDistance;
                            this.decelWaypoint.prevWp = decelPosition.prevWp;
                            this.decelPrevIndex = decelPosition.prevIndex;
                            const prevWaypoint = decelPosition.prevWp;
                            if (prevWaypoint) {
                                this.decelWaypoint.legAltitude1 = decelPosition.alt;
                                this.decelWaypoint.legAltitudeDescription = 1;
                                this.decelWaypoint.distanceInFP = decelPosition.distance;
                                const planeCoord = new LatLong(SimVar.GetSimVarValue("PLANE LATITUDE", "degree latitude"), SimVar.GetSimVarValue("PLANE LONGITUDE", "degree longitude"));
                                const dist = Avionics.Utils.computeGreatCircleDistance(planeCoord, this.decelWaypoint.infos.coordinates);
                                this.decelWaypoint.liveDistanceTo = prevWaypoint.liveDistanceTo ? prevWaypoint.liveDistanceTo + this.decelWaypoint.distanceInFP : dist;
                                this.decelWaypoint.liveETATo = (this._decelReached ? this._waypointReachedAt : this.decelWaypoint.liveDistanceTo / groundSpeed * 3600);
                                this.decelWaypoint.liveUTCTo = utcTime + this.decelWaypoint.liveETATo;
                            }
                        }
                    }, 300);
                }
            }
        }
        const delayCallback = () => {
            if (done === todo) {
                if (callback) {
                    callback(waypoints);
                }
            } else {
                this.instrument.requestCall(delayCallback);
            }
        };
        delayCallback();
    }
    updateWaypointIndex() {
        Coherent.call("GET_ACTIVE_WAYPOINT_INDEX").then((waypointIndex) => {
            this._activeWaypointIndex = waypointIndex;
        });
    }

    _syncFlightPlanVersion() {
        // Each FPM instance tracks its own local version to guarantee an
        // increment call will trigger an update, but other instances can also
        // change the version, so check both the SimVar and the member and take
        // the newest version
        const simVarVersion = SimVar.GetSimVarValue("L:A32NX_FLIGHT_PLAN_VERSION", 'number');
        if (this._newFlightPlanVersion < simVarVersion) {
            this._newFlightPlanVersion = simVarVersion;
        }
    }

    _incrementFlightPlanVersion() {
        this._syncFlightPlanVersion();
        this._newFlightPlanVersion++;
        SimVar.SetSimVarValue("L:A32NX_FLIGHT_PLAN_VERSION", 'number', this._newFlightPlanVersion);
    }

    updateFlightPlan(callback = () => { }, log = false) {
        this._syncFlightPlanVersion();
        if (this._newFlightPlanVersion === this._currentFlightPlanVersion) {
            if (callback) {
                callback();
            }
            return;
        }
        const first = this._currentFlightPlanVersion === -1;
        this._currentFlightPlanVersion = this._newFlightPlanVersion;
        const t0 = performance.now();
        Coherent.call("GET_FLIGHTPLAN").then((flightPlanData) => {
            const t1 = performance.now();
            if (log) {
            }
            const index = flightPlanData.flightPlanIndex;
            this._cruisingAltitude = flightPlanData.cruisingAltitude;
            this._activeWaypointIndex = flightPlanData.activeWaypointIndex;
            this._departureWaypointSize = Math.max(0, flightPlanData.departureWaypointsSize);
            this._runwayIndex = flightPlanData.originRunwayIndex;
            this._departureRunwayIndex = flightPlanData.departureRunwayIndex;
            this._departureProcIndex = flightPlanData.departureProcIndex;
            this._departureEnRouteTransitionIndex = flightPlanData.departureEnRouteTransitionIndex;
            this._departureDiscontinuity = flightPlanData.departureDiscontinuity;
            this._arrivalWaypointSize = Math.max(0, flightPlanData.arrivalWaypointsSize);
            this._arrivalProcIndex = flightPlanData.arrivalProcIndex;
            this._arrivalTransitionIndex = flightPlanData.arrivalEnRouteTransitionIndex;
            this._arrivalDiscontinuity = flightPlanData.arrivalDiscontinuity;
            this._approachWaypointSize = Math.max(0, this._approachWaypoints.length);
            this._approachIndex = flightPlanData.approachIndex;
            this._approachTransitionIndex = flightPlanData.approachTransitionIndex;
            this._lastIndexBeforeApproach = flightPlanData.lastIndexBeforeApproach;
            this._isDirectTo = flightPlanData.isDirectTo;
            if (!this._directToTarget) {
                this._directToTarget = new WayPoint(this.instrument);
                this._directToTarget.infos = new IntersectionInfo(this.instrument);
            }
            this._directToTarget.icao = flightPlanData.directToTarget.icao;
            this._directToTarget.infos.icao = this._directToTarget.icao;
            this._directToTarget.ident = flightPlanData.directToTarget.ident;
            if (!this._directToTarget.ident) {
                this._directToTarget.ident = this._directToTarget.icao.substr(7);
            }
            this._directToTarget.infos.ident = this._directToTarget.ident;
            this._directToTarget.infos.coordinates = new LatLongAlt(flightPlanData.directToTarget.lla);
            this._directToOrigin = new LatLongAlt(flightPlanData.directToOrigin);
            if (!this._waypoints[index]) {
                this._waypoints[index] = [];
            }
            this._loadWaypoints(flightPlanData.waypoints, this._waypoints[index], false, (wps) => {
                this._waypoints[index] = wps;
                const t2 = performance.now();
                if (log) {
                }

                // HACK: Initial call to load approach will fail because flight plan isn't loaded yet,
                // so force it to load now as we have the flight plan ready.
                if (first) {
                    this.updateCurrentApproach(callback, false, true);
                } else if (callback) {
                    callback();
                }
            });
        });
    }
    updateCurrentApproach(callback = () => { }, log = false, force = false) {
        this._syncFlightPlanVersion();
        if (!force && this._newFlightPlanVersion === this._currentFlightPlanApproachVersion) {
            if (callback) {
                callback();
            }
            return;
        }
        this._currentFlightPlanApproachVersion = this._newFlightPlanVersion;
        const t0 = performance.now();
        Coherent.call("GET_APPROACH_FLIGHTPLAN").then((flightPlanData) => {
            this._loadWaypoints(flightPlanData.waypoints, this._approachWaypoints, true, (wps) => {
                this._approachWaypoints = wps;
                let previousWaypoint = this.getWaypoint(this.getWaypointsCount() - 2);
                for (let i = 0; i < this._approachWaypoints.length; i++) {
                    const waypoint = this._approachWaypoints[i];
                    if (waypoint) {
                        this.addHardCodedConstraints(waypoint);
                        previousWaypoint = waypoint;
                    }
                }
            });
        });
        Coherent.call("GET_CURRENT_APPROACH").then((approachData) => {
            const t1 = performance.now();
            if (log) {
                console.log("Approach Data loaded from FlightPlanManager in " + (t1 - t0).toFixed(2) + " ms.");
                console.log(approachData);
            }
            if (!this._approach) {
                this._approach = new Approach();
            }
            this._approach.name = approachData.name;
            this._approach.runway = approachData.name.split(" ")[1];
            this._approach.transitions = [];
            for (let i = 0; i < approachData.transitions.length; i++) {
                const transitionData = approachData.transitions[i];
                const transition = new Transition();
                let previousWaypoint = this.getWaypoint(this.getWaypointsCount() - 2);
                for (let j = 1; j < transitionData.waypoints.length; j++) {
                    const waypointData = transitionData.waypoints[j];
                    const waypoint = new WayPoint(this.instrument);
                    waypoint.infos = new IntersectionInfo(this.instrument);
                    waypoint.icao = waypointData.icao;
                    waypoint.infos.icao = waypoint.icao;
                    waypoint.ident = waypointData.ident;
                    if (!waypoint.ident) {
                        waypoint.ident = waypoint.icao.substr(7);
                    }
                    waypoint.infos.ident = waypoint.ident;
                    waypoint.infos.coordinates = new LatLongAlt(waypointData.lla);
                    waypoint.latitudeFP = waypointData.lla.lat;
                    waypoint.longitudeFP = waypointData.lla.lon;
                    waypoint.altitudeinFP = waypointData.lla.alt * 3.2808;
                    waypoint.altitudeModeinFP = waypointData.altitudeMode;
                    waypoint.transitionLLas = waypointData.transitionLLas;
                    const altitudeConstraintInFeet = waypoint.altitudeinFP;
                    if (altitudeConstraintInFeet >= 500) {
                        this.setOrGetLegAltitudeDescription(waypoint, 1);
                        waypoint.legAltitude1 = altitudeConstraintInFeet;
                    }
                    waypoint.speedConstraint = waypointData.speedConstraint;
                    if (waypoint.speedConstraint > 0) {
                    }
                    if (waypoint.speedConstraint > 400) {
                        waypoint.speedConstraint = -1;
                    }
                    if (previousWaypoint) {
                        waypoint.distanceInFP = Avionics.Utils.computeGreatCircleDistance(previousWaypoint.infos.coordinates, waypoint.infos.coordinates);
                        waypoint.cumulativeDistanceInFP = previousWaypoint.cumulativeDistanceInFP + waypoint.distanceInFP;
                        waypoint.bearingInFP = Avionics.Utils.computeGreatCircleHeading(previousWaypoint.infos.coordinates, waypoint.infos.coordinates);
                    }
                    transition.waypoints.push(waypoint);
                    previousWaypoint = waypoint;
                }
                transition.waypoints.push(this._waypoints[this._currentFlightPlanIndex][this._waypoints[this._currentFlightPlanIndex].length - 1]);
                this._approach.transitions.push(transition);
            }
            if (log) {
                console.log("FlightPlanManager now");
                console.log(this);
            }
            callback();
        });
    }
    get cruisingAltitude() {
        return this._cruisingAltitude;
    }
    getCurrentFlightPlanIndex() {
        return this._currentFlightPlanIndex;
    }
    setCurrentFlightPlanIndex(index, callback = EmptyCallback.Boolean) {
        Coherent.call("SET_CURRENT_FLIGHTPLAN_INDEX", index).then(() => {
            let attempts = 0;
            const checkTrueFlightPlanIndex = () => {
                Coherent.call("GET_CURRENT_FLIGHTPLAN_INDEX").then((value) => {
                    attempts++;
                    if (value === index) {
                        console.log("setCurrentFlightPlanIndex : Values matching, return after " + attempts + " attempts");
                        this._currentFlightPlanIndex = index;
                        this._incrementFlightPlanVersion();
                        this.updateFlightPlan(() => {
                            callback(true);
                        });
                        return;
                    } else {
                        if (attempts < 60) {
                            console.log("setCurrentFlightPlanIndex : Values mistmatch, retrying");
                            this.instrument.requestCall(checkTrueFlightPlanIndex);
                            return;
                        } else {
                            console.log("setCurrentFlightPlanIndex : Values mistmatched too long, aborting");
                            return callback(false);
                        }
                    }
                });
            };
            checkTrueFlightPlanIndex();
        });
    }
    createNewFlightPlan(callback = EmptyCallback.Void) {
        Coherent.call("CREATE_NEW_FLIGHTPLAN").then(() => {
            this.instrument.requestCall(callback);
        });
    }
    copyCurrentFlightPlanInto(index, callback = EmptyCallback.Void) {
        Coherent.call("COPY_CURRENT_FLIGHTPLAN_TO", index).then(() => {
            this.instrument.requestCall(callback);
        });
    }
    copyFlightPlanIntoCurrent(index, callback = EmptyCallback.Void) {
        Coherent.call("COPY_FLIGHTPLAN_TO_CURRENT", index).then(() => {
            this.instrument.requestCall(callback);
        });
    }
    clearFlightPlan(callback = EmptyCallback.Void) {
        Coherent.call("CLEAR_CURRENT_FLIGHT_PLAN").then(() => {
            this._incrementFlightPlanVersion();
            this.updateFlightPlan(() => {
                this.updateCurrentApproach(() => {
                    this.instrument.requestCall(callback);
                });
            });
        });
    }
    getOrigin() {
        if (this._waypoints.length > 0) {
            return this._waypoints[this._currentFlightPlanIndex][0];
        } else {
            return null;
        }
    }
    setOrigin(icao, callback = () => { }) {
        // NXDataStore instead of Simvar, because local string SimVars are not possible.
        NXDataStore.set("PLAN_ORIGIN", icao.replace("A      ", "").trim());

        Coherent.call("SET_ORIGIN", icao).then(() => {
            this._incrementFlightPlanVersion();
            this.updateFlightPlan(callback);
        });
    }
    getActiveWaypointIndex(forceSimVarCall = false, useCorrection = false) {
        if (useCorrection && this._isGoingTowardPreviousActiveWaypoint) {
            return this.getActiveWaypointIndex(forceSimVarCall, false) - 1;
        }
        const ident = this.getActiveWaypointIdent(forceSimVarCall);
        if (this.isActiveApproach()) {
            const waypointIndex = this.getApproachWaypoints().findIndex(w => {
                return w && w.ident === ident;
            });
            return waypointIndex;
        }
        let waypointIndex = this.getWaypoints().findIndex(w => {
            return w && w.ident === ident;
        });
        if (waypointIndex === -1) {
            waypointIndex = this.getApproachWaypoints().findIndex(w => {
                return w && w.ident === ident;
            });
        }
        if (useCorrection && (this._activeWaypointIdentHasChanged || this._gpsActiveWaypointIndexHasChanged)) {
            return waypointIndex - 1;
        }
        return waypointIndex;
    }
    setActiveWaypointIndex(index, callback = EmptyCallback.Void) {
        Coherent.call("SET_ACTIVE_WAYPOINT_INDEX", index).then(() => {
            this._incrementFlightPlanVersion();
            this.updateFlightPlan(callback);
        });
    }
    recomputeActiveWaypointIndex(callback = EmptyCallback.Void) {
        Coherent.call("RECOMPUTE_ACTIVE_WAYPOINT_INDEX").then(() => {
            this._incrementFlightPlanVersion();
            this.updateFlightPlan(callback);
        });
    }
    getPreviousActiveWaypoint(forceSimVarCall = false) {
        const ident = this.getActiveWaypointIdent(forceSimVarCall);
        if (this.isActiveApproach()) {
            const waypointIndex = this.getApproachWaypoints().findIndex(w => {
                return (w && w.ident === ident);
            });
            return this.getApproachWaypoints()[waypointIndex - 1];
        }
        let waypointIndex = this.getWaypoints().findIndex(w => {
            return (w && w.ident === ident);
        });
        if (waypointIndex === -1) {
            waypointIndex = this.getApproachWaypoints().findIndex(w => {
                return (w && w.ident === ident);
            });
        }
        return this.getWaypoints()[waypointIndex - 1];
    }
    getActiveWaypointIdent(forceSimVarCall = false) {
        let doSimVarCall = false;
        let t = 0;
        if (forceSimVarCall || this._activeWaypointIdent === undefined) {
            doSimVarCall = true;
        } else {
            t = performance.now();
            if (t - this._timeLastSimVarCall > 1000) {
                doSimVarCall = true;
            }
        }
        if (doSimVarCall) {
            const activeWaypointIdent = SimVar.GetSimVarValue("GPS WP NEXT ID", "string");
            if (this._activeWaypointIdent != activeWaypointIdent) {
                this._waypointReachedAt = SimVar.GetGlobalVarValue("ZULU TIME", "seconds");
                this._activeWaypointIdentHasChanged = true;
                this._activeWaypointIdent = activeWaypointIdent;
            }
            this._timeLastSimVarCall = t;
        }
        return this._activeWaypointIdent;
    }
    getGPSActiveWaypointIndex(forceSimVarCall = false) {
        let doSimVarCall = false;
        let t = 0;
        if (forceSimVarCall || this._gpsActiveWaypointIndex === undefined) {
            doSimVarCall = true;
        } else {
            t = performance.now();
            if (t - this._timeLastActiveWaypointIndexSimVarCall > 1000) {
                doSimVarCall = true;
            }
        }
        if (doSimVarCall) {
            const gpsActiveWaypointIndex = SimVar.GetSimVarValue("C:fs9gps:FlightPlanActiveWaypoint", "number");
            if (this._gpsActiveWaypointIndex != gpsActiveWaypointIndex) {
                this._gpsActiveWaypointIndexHasChanged = true;
                this._gpsActiveWaypointIndex = gpsActiveWaypointIndex;
            }
            this._timeLastActiveWaypointIndexSimVarCall = t;
        }
        return this._gpsActiveWaypointIndex;
    }
    getActiveWaypoint(forceSimVarCall = false, useCorrection = false) {
        if (useCorrection && this._isGoingTowardPreviousActiveWaypoint) {
            return this.getPreviousActiveWaypoint(forceSimVarCall);
        }
        const ident = this.getActiveWaypointIdent(forceSimVarCall);
        if (!this.isActiveApproach()) {
            const waypoint = this.getWaypoints().find(w => {
                return (w && w.ident === ident);
            });
            if (waypoint) {
                if (useCorrection && (this._activeWaypointIdentHasChanged || this._gpsActiveWaypointIndexHasChanged)) {
                    return this.getPreviousActiveWaypoint(forceSimVarCall);
                }
                return waypoint;
            }
        }
        if (this.isActiveApproach()) {
            const waypoint = this.getApproachWaypoints().find(w => {
                return (w && w.ident === ident);
            });
            return waypoint;
        }
        let waypoint = this.getWaypoints().find(w => {
            return (w && w.ident === ident);
        });
        if (!waypoint) {
            waypoint = this.getApproachWaypoints().find(w => {
                return (w && w.ident === ident);
            });
        }
        if (!waypoint && this._directToTarget && ident != "" && ident === this._directToTarget.ident) {
            waypoint = this._directToTarget;
        }
        if (useCorrection && (this._activeWaypointIdentHasChanged || this._gpsActiveWaypointIndexHasChanged)) {
            return this.getPreviousActiveWaypoint(forceSimVarCall);
        }
        return waypoint;
    }
    getNextActiveWaypoint(forceSimVarCall = false) {
        const ident = this.getActiveWaypointIdent(forceSimVarCall);
        if (this.isActiveApproach()) {
            const waypointIndex = this.getApproachWaypoints().findIndex(w => {
                return (w && w.ident === ident);
            });
            return this.getApproachWaypoints()[waypointIndex + 1];
        }
        let waypointIndex = this.getWaypoints().findIndex(w => {
            return (w && w.ident === ident);
        });
        if (waypointIndex === -1) {
            waypointIndex = this.getApproachWaypoints().findIndex(w => {
                return (w && w.ident === ident);
            });
        }
        return this.getWaypoints()[waypointIndex + 1];
    }
    getDistanceToActiveWaypoint() {
        const lat = SimVar.GetSimVarValue("PLANE LATITUDE", "degree latitude");
        const long = SimVar.GetSimVarValue("PLANE LONGITUDE", "degree longitude");
        const ll = new LatLong(lat, long);
        const waypoint = this.getActiveWaypoint();
        if (waypoint && waypoint.infos) {
            return Avionics.Utils.computeDistance(ll, waypoint.infos.coordinates);
        }
        return 0;
    }
    getBearingToActiveWaypoint() {
        const lat = SimVar.GetSimVarValue("PLANE LATITUDE", "degree latitude");
        const long = SimVar.GetSimVarValue("PLANE LONGITUDE", "degree longitude");
        const ll = new LatLong(lat, long);
        const waypoint = this.getActiveWaypoint();
        if (waypoint && waypoint.infos) {
            return Avionics.Utils.computeGreatCircleHeading(ll, waypoint.infos.coordinates);
        }
        return 0;
    }
    getETEToActiveWaypoint() {
        const lat = SimVar.GetSimVarValue("PLANE LATITUDE", "degree latitude");
        const long = SimVar.GetSimVarValue("PLANE LONGITUDE", "degree longitude");
        const ll = new LatLong(lat, long);
        const waypoint = this.getActiveWaypoint();
        if (waypoint && waypoint.infos) {
            const dist = Avionics.Utils.computeDistance(ll, waypoint.infos.coordinates);
            let groundSpeed = SimVar.GetSimVarValue("GPS GROUND SPEED", "knots");
            if (groundSpeed < 50) {
                groundSpeed = 50;
            }
            if (groundSpeed > 0.1) {
                return dist / groundSpeed * 3600;
            }
        }
        return 0;
    }
    getDestination() {
        if (this._waypoints[this._currentFlightPlanIndex].length > 1) {
            return this._waypoints[this._currentFlightPlanIndex][this._waypoints[this._currentFlightPlanIndex].length - 1];
        } else {
            return null;
        }
    }
    getDeparture() {
        const origin = this.getOrigin();
        if (origin) {
            const originInfos = origin.infos;
            if (originInfos instanceof AirportInfo) {
                return originInfos.departures[this._departureProcIndex];
            }
        }
    }
    getArrival() {
        const destination = this.getDestination();
        if (destination) {
            const destinationInfos = destination.infos;
            if (destinationInfos instanceof AirportInfo) {
                return destinationInfos.arrivals[this._arrivalProcIndex];
            }
        }
    }
    getAirportApproach() {
        const destination = this.getDestination();
        if (destination) {
            const destinationInfos = destination.infos;
            if (destinationInfos instanceof AirportInfo) {
                return destinationInfos.approaches[this._approachIndex];
            }
        }
    }
    getDepartureWaypoints() {
        const departureWaypoints = [];
        const origin = this.getOrigin();
        if (origin) {
            const originInfos = origin.infos;
            if (originInfos instanceof AirportInfo) {
                const departure = originInfos.departures[this._departureProcIndex];
                if (departure) {
                    let runwayTransition = departure.runwayTransitions[0];
                    if (departure.runwayTransitions.length > 0) {
                        runwayTransition = departure.runwayTransitions[this._departureRunwayIndex];
                    }
                    if (runwayTransition && runwayTransition.legs) {
                        for (let i = 0; i < runwayTransition.legs.length; i++) {
                            const wp = new WayPoint(this.instrument);
                            wp.icao = runwayTransition.legs[i].fixIcao;
                            wp.ident = wp.icao.substr(7);
                            departureWaypoints.push(wp);
                        }
                    }
                    if (departure && departure.commonLegs) {
                        for (let i = 0; i < departure.commonLegs.length; i++) {
                            const wp = new WayPoint(this.instrument);
                            wp.icao = departure.commonLegs[i].fixIcao;
                            wp.ident = wp.icao.substr(7);
                            departureWaypoints.push(wp);
                        }
                    }
                    let enRouteTransition = departure.enRouteTransitions[0];
                    if (departure.enRouteTransitions.length > 0) {
                        enRouteTransition = departure.enRouteTransitions[this._departureRunwayIndex];
                    }
                    if (enRouteTransition && enRouteTransition.legs) {
                        for (let i = 0; i < enRouteTransition.legs.length; i++) {
                            const wp = new WayPoint(this.instrument);
                            wp.icao = enRouteTransition.legs[i].fixIcao;
                            wp.ident = wp.icao.substr(7);
                            departureWaypoints.push(wp);
                        }
                    }
                }
            }
        }
        return departureWaypoints;
    }
    getDepartureWaypointsMap() {
        const departureWaypoints = [];
        for (let i = 1; i < this._departureWaypointSize + 1; i++) {
            departureWaypoints.push(this._waypoints[this._currentFlightPlanIndex][i]);
        }
        return departureWaypoints;
    }
    getEnRouteWaypoints(outFPIndex) {
        const enRouteWaypoints = [];
        for (let i = (1 + this._departureWaypointSize); i < this._waypoints[this._currentFlightPlanIndex].length - 1 - this._arrivalWaypointSize; i++) {
            enRouteWaypoints.push(this._waypoints[this._currentFlightPlanIndex][i]);
            if (outFPIndex) {
                outFPIndex.push(i);
            }
        }
        return enRouteWaypoints;
    }
    getEnRouteWaypointsLastIndex() {
        return this.getDepartureWaypointsCount() + this.getEnRouteWaypoints().length;
    }
    getArrivalWaypoints() {
        const arrivalWaypoints = [];
        const destination = this.getDestination();
        if (destination) {
            const destinationInfos = destination.infos;
            if (destinationInfos instanceof AirportInfo) {
                const arrival = destinationInfos.arrivals[this._arrivalProcIndex];
                if (arrival) {
                    const enRouteTransition = arrival.enRouteTransitions[0];
                    if (arrival.enRouteTransitions.length > 0) {
                    }
                    if (enRouteTransition && enRouteTransition.legs) {
                        for (let i = 0; i < enRouteTransition.legs.length; i++) {
                            const wp = new WayPoint(this.instrument);
                            wp.icao = enRouteTransition.legs[i].fixIcao;
                            wp.ident = wp.icao.substr(7);
                            arrivalWaypoints.push(wp);
                        }
                    }
                    if (arrival && arrival.commonLegs) {
                        for (let i = 0; i < arrival.commonLegs.length; i++) {
                            const wp = new WayPoint(this.instrument);
                            wp.icao = arrival.commonLegs[i].fixIcao;
                            wp.ident = wp.icao.substr(7);
                            arrivalWaypoints.push(wp);
                        }
                    }
                    const runwayTransition = arrival.runwayTransitions[0];
                    if (arrival.runwayTransitions.length > 0) {
                    }
                    if (runwayTransition && runwayTransition.legs) {
                        for (let i = 0; i < runwayTransition.legs.length; i++) {
                            const wp = new WayPoint(this.instrument);
                            wp.icao = runwayTransition.legs[i].fixIcao;
                            wp.ident = wp.icao.substr(7);
                            arrivalWaypoints.push(wp);
                        }
                    }
                }
            }
        }
        return arrivalWaypoints;
    }
    getArrivalWaypointsMap() {
        const arrivalWaypoints = [];
        for (let i = this._waypoints[this._currentFlightPlanIndex].length - 1 - this._arrivalWaypointSize; i < this._waypoints[this._currentFlightPlanIndex].length - 1; i++) {
            arrivalWaypoints.push(this._waypoints[this._currentFlightPlanIndex][i]);
        }
        return arrivalWaypoints;
    }
    getWaypointsWithAltitudeConstraints() {
        const waypointsWithAltitudeConstraints = [];
        for (let i = 0; i < this._waypoints[0].length; i++) {
            const wp = this._waypoints[0][i];
            if (wp.legAltitudeDescription >= 1 && wp.legAltitude1 < 20000) {
                waypointsWithAltitudeConstraints.push(wp);
            }
        }
        const approachWaypoints = this.getApproachWaypoints();
        for (let i = 0; i < approachWaypoints.length; i++) {
            const apprWp = approachWaypoints[i];
            if (apprWp.legAltitudeDescription >= 1 && apprWp.legAltitude1 < 20000) {
                waypointsWithAltitudeConstraints.push(apprWp);
            }
        }
        return waypointsWithAltitudeConstraints;
    }
    setDestination(icao, callback = () => { }) {
        // NXDataStore instead of Simvar, because local string SimVars are not possible.
        NXDataStore.set("PLAN_DESTINATION", icao.replace("A      ", "").trim());

        Coherent.call("SET_DESTINATION", icao).then(() => {
            this._incrementFlightPlanVersion();
            this.updateFlightPlan(callback);
        });
    }
    addWaypoint(icao, index = Infinity, callback = () => { }, setActive = true) {
        if (index === Infinity) {
            index = this._waypoints.length;
        }
        Coherent.call("ADD_WAYPOINT", icao, index, setActive).then(() => {
            this._incrementFlightPlanVersion();
            this.updateFlightPlan(callback);
        });
    }
    setWaypointAltitude(altitude, index, callback = () => { }) {
        Coherent.call("SET_WAYPOINT_ALTITUDE", altitude, index).then(() => {
            this._incrementFlightPlanVersion();
            this.updateFlightPlan(callback);
        });
    }
    setWaypointAdditionalData(index, key, value, callback = () => { }) {
        Coherent.call("SET_WAYPOINT_ADDITIONAL_DATA", index, key, value).then(() => {
            this._incrementFlightPlanVersion();
            this.updateFlightPlan(callback);
        });
    }
    getWaypointAdditionalData(index, key, callback = () => { }) {
        Coherent.call("GET_WAYPOINT_ADDITIONAL_DATA", index, key).then((value) => {
            callback(value);
        });
    }
    invertActiveFlightPlan(callback = () => { }) {
        Coherent.call("INVERT_ACTIVE_FLIGHT_PLAN").then(() => {
            this._incrementFlightPlanVersion();
            this.updateFlightPlan(callback);
        });
    }
    getApproachIfIcao(callback = () => { }) {
        Coherent.call("GET_IF_ICAO").then((value) => {
            callback(value);
        });
    }
    addFlightPlanUpdateCallback(_callback) {
        this._onFlightPlanUpdateCallbacks.push(_callback);
    }
    addWaypointByIdent(ident, index = Infinity, callback = EmptyCallback.Void) {
        SimVar.SetSimVarValue("C:fs9gps:IcaoSearchStartCursor", "string", "WANV", "FMC").then(() => {
            this.instrument.requestCall(() => {
                SimVar.SetSimVarValue("C:fs9gps:IcaoSearchEnterChar", "string", ident, "FMC").then(() => {
                    SimVar.SetSimVarValue("C:fs9gps:IcaoSearchMatchedIcao", "number", 0, "FMC").then(() => {
                        const icao = SimVar.GetSimVarValue("C:fs9gps:IcaoSearchCurrentIcao", "string", "FMC");
                        this.addWaypoint(icao, index, callback);
                    });
                });
            });
        });
    }
    removeWaypoint(index, thenSetActive = false, callback = () => { }) {
        Coherent.call("REMOVE_WAYPOINT", index, thenSetActive).then(() => {
            this._incrementFlightPlanVersion();
            this.updateFlightPlan(callback);
        });
    }
    indexOfWaypoint(waypoint) {
        return this._waypoints[this._currentFlightPlanIndex].indexOf(waypoint);
    }
    getWaypointsCount(flightPlanIndex = NaN) {
        if (isNaN(flightPlanIndex)) {
            flightPlanIndex = this._currentFlightPlanIndex;
        }
        if (this._waypoints && this._waypoints[flightPlanIndex]) {
            return this._waypoints[flightPlanIndex].length;
        }
        return 0;
    }
    getDepartureWaypointsCount() {
        return this._departureWaypointSize;
    }
    getArrivalWaypointsCount() {
        return this._arrivalWaypointSize;
    }
    getApproachWaypointsCount() {
        return this._approachWaypointSize;
    }
    getWaypoint(i, flightPlanIndex = NaN, considerApproachWaypoints) {
        if (isNaN(flightPlanIndex)) {
            flightPlanIndex = this._currentFlightPlanIndex;
        }
        if (!considerApproachWaypoints || i < this.getWaypointsCount() - 1) {
            return this._waypoints[flightPlanIndex][i];
        } else {
            const approachWaypoints = this.getApproachWaypoints();
            const apprWp = approachWaypoints[i - (this.getWaypointsCount() - 1)];
            if (apprWp) {
                return apprWp;
            }
            return this.getDestination();
        }
    }
    getWaypoints(flightPlanIndex = NaN) {
        if (isNaN(flightPlanIndex)) {
            flightPlanIndex = this._currentFlightPlanIndex;
        }
        return this._waypoints[flightPlanIndex];
    }
    getDepartureRunwayIndex() {
        return this._departureRunwayIndex;
    }
    getDepartureRunway() {
        const origin = this.getOrigin();
        if (origin) {
            const departure = this.getDeparture();
            const infos = origin.infos;
            if (infos instanceof AirportInfo) {
                if (departure) {
                    const fullRwy = departure.runwayTransitions[this.getDepartureRunwayIndex()];
                    if (fullRwy) {
                        const depRunway = fullRwy.name.replace("RW", "");
                        const runway = infos.oneWayRunways.find(r => {
                            return r.designation.indexOf(depRunway) !== -1;
                        });
                        if (runway) {
                            return runway;
                        }
                    }
                    return undefined;
                }
                if (this._runwayIndex >= 0) {
                    return infos.oneWayRunways[this._runwayIndex];
                }
            }
        }
    }
    getDetectedCurrentRunway() {
        const origin = this.getOrigin();
        if (origin && origin.infos instanceof AirportInfo) {
            const runways = origin.infos.oneWayRunways;
            if (runways && runways.length > 0) {
                const direction = Simplane.getHeadingMagnetic();
                let bestRunway = runways[0];
                let bestDeltaAngle = Math.abs(Avionics.Utils.angleDiff(direction, bestRunway.direction));
                for (let i = 1; i < runways.length; i++) {
                    const deltaAngle = Math.abs(Avionics.Utils.angleDiff(direction, runways[i].direction));
                    if (deltaAngle < bestDeltaAngle) {
                        bestDeltaAngle = deltaAngle;
                        bestRunway = runways[i];
                    }
                }
                return bestRunway;
            }
        }
        return undefined;
    }
    getDepartureProcIndex() {
        return this._departureProcIndex;
    }
    setDepartureProcIndex(index, callback = () => { }) {
        Coherent.call("SET_DEPARTURE_PROC_INDEX", index).then(() => {
            this._incrementFlightPlanVersion();
            this.updateFlightPlan(callback);
        });
    }
    setDepartureRunwayIndex(index, callback = EmptyCallback.Void) {
        Coherent.call("SET_DEPARTURE_RUNWAY_INDEX", index).then(() => {
            this._incrementFlightPlanVersion();
            this.updateFlightPlan(callback);
        });
    }
    setOriginRunwayIndex(index, callback = EmptyCallback.Void) {
        Coherent.call("SET_ORIGIN_RUNWAY_INDEX", index).then(() => {
            this._incrementFlightPlanVersion();
            this.updateFlightPlan(callback);
        });
    }
    getDepartureEnRouteTransitionIndex() {
        return this._departureEnRouteTransitionIndex;
    }
    setDepartureEnRouteTransitionIndex(index, callback = EmptyCallback.Void) {
        Coherent.call("SET_DEPARTURE_ENROUTE_TRANSITION_INDEX", index).then(() => {
            this._incrementFlightPlanVersion();
            this.updateFlightPlan(callback);
        });
    }
    getDepartureDiscontinuity() {
        return this._departureDiscontinuity;
    }
    clearDepartureDiscontinuity(callback = EmptyCallback.Void) {
        Coherent.call("CLEAR_DEPARTURE_DISCONTINUITY").then(() => {
            this._incrementFlightPlanVersion();
            this.updateFlightPlan(callback);
        });
    }
    removeDeparture(callback = () => { }) {
        Coherent.call("REMOVE_DEPARTURE_PROC").then(() => {
            this._incrementFlightPlanVersion();
            this.updateFlightPlan(callback);
        });
    }
    getArrivalProcIndex() {
        return this._arrivalProcIndex;
    }
    getArrivalTransitionIndex() {
        return this._arrivalTransitionIndex;
    }
    setArrivalProcIndex(index, callback = () => { }) {
        Coherent.call("SET_ARRIVAL_PROC_INDEX", index).then(() => {
            this._incrementFlightPlanVersion();
            this.updateFlightPlan(callback);
        });
    }
    getArrivalDiscontinuity() {
        return this._arrivalDiscontinuity;
    }
    clearArrivalDiscontinuity(callback = EmptyCallback.Void) {
        Coherent.call("CLEAR_ARRIVAL_DISCONTINUITY").then(() => {
            this._incrementFlightPlanVersion();
            this.updateFlightPlan(callback);
        });
    }
    setArrivalEnRouteTransitionIndex(index, callback = () => { }) {
        Coherent.call("SET_ARRIVAL_ENROUTE_TRANSITION_INDEX", index).then(() => {
            this._incrementFlightPlanVersion();
            this.updateFlightPlan(callback);
        });
    }
    setArrivalRunwayIndex(index, callback = () => { }) {
        Coherent.call("SET_ARRIVAL_RUNWAY_INDEX", index).then(() => {
            this._incrementFlightPlanVersion();
            this.updateFlightPlan(callback);
        });
    }
    getApproachIndex() {
        return this._approachIndex;
    }
    setApproachIndex(index, callback = () => { }, transition = 0) {
        Coherent.call("SET_APPROACH_INDEX", index).then(() => {
            Coherent.call("SET_APPROACH_TRANSITION_INDEX", transition).then(() => {
                this._incrementFlightPlanVersion();
                this.updateFlightPlan(() => {
                    this.updateCurrentApproach(() => {
                        callback();
                    });
                });
            });
        });
        SimVar.SetSimVarValue("C:fs9gps:FlightPlanNewApproachAirport", "string", this.getDestination().icao);
        SimVar.SetSimVarValue("C:fs9gps:FlightPlanNewApproachApproach", "number", index);
        SimVar.SetSimVarValue("C:fs9gps:FlightPlanNewApproachTransition", "number", transition);
        SimVar.SetSimVarValue("C:fs9gps:FlightPlanLoadApproach", "number", 1);
    }
    isLoadedApproach(forceSimVarCall = false) {
        let doSimVarCall = false;
        let t = 0;
        if (forceSimVarCall || this._isLoadedApproach === undefined) {
            doSimVarCall = true;
        } else {
            t = performance.now();
            if (t - this._isLoadedApproachTimeLastSimVarCall > 1000) {
                doSimVarCall = true;
            }
        }
        if (doSimVarCall) {
            this._isLoadedApproach = SimVar.GetSimVarValue("C:fs9gps:FlightPlanIsLoadedApproach", "Bool");
            this._isLoadedApproachTimeLastSimVarCall = t;
        }
        return this._isLoadedApproach;
    }
    isActiveApproach(forceSimVarCall = false) {
        let doSimVarCall = false;
        let t = 0;
        if (forceSimVarCall || this._isActiveApproach === undefined) {
            doSimVarCall = true;
        } else {
            t = performance.now();
            if (t - this._isActiveApproachTimeLastSimVarCall > 1000) {
                doSimVarCall = true;
            }
        }
        if (doSimVarCall) {
            this._isActiveApproach = SimVar.GetSimVarValue("C:fs9gps:FlightPlanIsActiveApproach", "Bool");
            this._isActiveApproachTimeLastSimVarCall = t;
        }
        return this._isActiveApproach;
    }
    activateApproach(callback = EmptyCallback.Void) {
        if (this.isActiveApproach() || !this.isLoadedApproach()) {
            if (this.isActiveApproach) {
                callback();
            }
            return;
        }
        Coherent.call("ACTIVATE_APPROACH").then(() => {
            this._approachActivated = true;
            this.updateCurrentApproach(() => {
                callback();
            });
        });
    }
    deactivateApproach() {
        Coherent.call("DEACTIVATE_APPROACH").then(() => {
            this._approachActivated = false;
        });
    }
    tryAutoActivateApproach() {
        Coherent.call("TRY_AUTOACTIVATE_APPROACH").then(() => {
            this._approachActivated = this.isActiveApproach();
        });
    }
    getApproachActiveWaypointIndex() {
        return this._approachActivated ? this.getActiveWaypointIndex() : -1;
    }
    getApproach() {
        if (!this._approach) {
            this._approach = new Approach();
        }
        return this._approach;
    }
    getApproachNavFrequency() {
        if (this._approachIndex >= 0) {
            const destination = this.getDestination();
            const approachName = this.getApproach().runway;
            if (destination) {
                if (destination.infos instanceof AirportInfo) {
                    const frequency = destination.infos.frequencies.find(f => {
                        return f.name.replace("RW0", "").replace("RW", "").indexOf(approachName) !== -1;
                    });
                    if (frequency) {
                        return frequency.mhValue;
                    }
                }
            }
        }
        return NaN;
    }
    getApproachTransitionIndex() {
        return this._approachTransitionIndex;
    }
    getLastIndexBeforeApproach() {
        return this._lastIndexBeforeApproach;
    }
    getApproachRunway() {
        const destination = this.getDestination();
        if (destination) {
            const infos = destination.infos;
            if (infos instanceof AirportInfo) {
                const approach = infos.approaches[this._approachIndex];
                if (approach) {
                    const approachRunway = approach.runway.replace(" ", "");
                    const runways = infos.oneWayRunways.filter(r => {
                        return r.designation.indexOf(approachRunway) !== -1;
                    });
                    if (runways.length > 1 && approachRunway.match(/\d$/)) {
                        let runway = runways.find(rw => {
                            return rw.designation.replace(" ", "") === approachRunway;
                        });
                        if (runway) {
                            return runway;
                        } else {
                            const approachRunwayC = approachRunway + 'C';
                            runway = runways.find(rw => {
                                return rw.designation.replace(" ", "") === approachRunwayC;
                            });
                            if (runway) {
                                return runway;
                            }
                        }
                    }
                    return runways[0];
                }
            }
        }
    }
    getApproachWaypoints() {
        return this._approachWaypoints;
        const waypoints = [];
        const airportApproach = this.getApproach();
        let transition;
        if (airportApproach) {
            const transitionIndex = this.getApproachTransitionIndex();
            transition = airportApproach.transitions[transitionIndex];
            if (!transition) {
                transition = airportApproach.transitions[0];
            }
        }
        if (airportApproach && transition) {
            for (let i = (this.getArrivalProcIndex() == -1 ? 0 : 1); i < transition.waypoints.length - 1; i++) {
                waypoints.push(transition.waypoints[i]);
            }
        }
        return waypoints;
    }
    setApproachTransitionIndex(index, callback = () => { }) {
        Coherent.call("SET_APPROACH_TRANSITION_INDEX", index).then(() => {
            this._incrementFlightPlanVersion();
            this.updateFlightPlan(callback);
        });
    }
    removeArrival(callback = () => { }) {
        Coherent.call("REMOVE_ARRIVAL_PROC").then(() => {
            this._incrementFlightPlanVersion();
            this.updateFlightPlan(callback);
        });
    }
    activateDirectTo(icao, callback = EmptyCallback.Void) {
        Coherent.call("ACTIVATE_DIRECT_TO", icao).then(() => {
            this._incrementFlightPlanVersion();
            this.updateFlightPlan(callback);
        });
    }
    cancelDirectTo(callback = EmptyCallback.Void) {
        Coherent.call("CANCEL_DIRECT_TO").then(() => {
            this._incrementFlightPlanVersion();
            this.updateFlightPlan(callback);
        });
    }
    getIsDirectTo() {
        return this._isDirectTo;
    }
    getDirectToTarget() {
        return this._directToTarget;
    }
    getDirecToOrigin() {
        return this._directToOrigin;
    }
    getCoordinatesHeadingAtDistanceAlongFlightPlan(distance) {
        let prevWaypoint = this.getPreviousActiveWaypoint();
        let nextWaypoint = this.getActiveWaypoint();
        if (prevWaypoint && nextWaypoint) {
            const planeCoordinates = new LatLong(SimVar.GetSimVarValue("PLANE LATITUDE", "degree latitude"), SimVar.GetSimVarValue("PLANE LONGITUDE", "degree longitude"));
            const a = Avionics.Utils.computeGreatCircleDistance(planeCoordinates, prevWaypoint.infos.coordinates);
            const b = Avionics.Utils.computeGreatCircleDistance(planeCoordinates, nextWaypoint.infos.coordinates);
            const f = a / (a + b);
            const dActiveLeg = (1 - f) * Avionics.Utils.computeGreatCircleDistance(prevWaypoint.infos.coordinates, nextWaypoint.infos.coordinates);
            if (distance <= dActiveLeg) {
                const ff = distance / dActiveLeg;
                const startLat = Avionics.Utils.lerpAngle(prevWaypoint.infos.lat, nextWaypoint.infos.lat, f);
                const startLong = Avionics.Utils.lerpAngle(prevWaypoint.infos.long, nextWaypoint.infos.long, f);
                const targetLat = Avionics.Utils.lerpAngle(startLat, nextWaypoint.infos.lat, ff);
                const targetLong = Avionics.Utils.lerpAngle(startLong, nextWaypoint.infos.long, ff);
                return { lla: new LatLong(targetLat, targetLong), heading: nextWaypoint.bearingInFP };
            }
            distance -= dActiveLeg;
            let index = this.getActiveWaypointIndex() + 1;
            let done = false;
            let currentLegLength = NaN;
            while (!done) {
                nextWaypoint = this.getWaypoint(index);
                prevWaypoint = this.getWaypoint(index - 1);
                if (nextWaypoint && prevWaypoint) {
                    currentLegLength = Avionics.Utils.computeGreatCircleDistance(prevWaypoint.infos.coordinates, nextWaypoint.infos.coordinates);
                    if (currentLegLength < distance) {
                        distance -= currentLegLength;
                        index++;
                    } else {
                        done = true;
                    }
                } else {
                    done = true;
                }
            }
            if (nextWaypoint && prevWaypoint && isFinite(currentLegLength)) {
                const ff = distance / currentLegLength;
                const targetLat = Avionics.Utils.lerpAngle(prevWaypoint.infos.lat, nextWaypoint.infos.lat, ff);
                const targetLong = Avionics.Utils.lerpAngle(prevWaypoint.infos.long, nextWaypoint.infos.long, ff);
                return { lla: new LatLong(targetLat, targetLong), heading: nextWaypoint.bearingInFP };
            }
            return { lla: new LatLong(this.getDestination().infos.coordinates), heading: 0 };
        }
        return undefined;
    }
    getCoordinatesAtNMFromDestinationAlongFlightPlan(distance) {
        const waypoints = this.getWaypoints();
        let allWaypoints = [...waypoints];
        const last = allWaypoints.pop();
        allWaypoints.push(...this.getApproachWaypoints());
        for (let i = 0; i < allWaypoints.length; i++) {
            const waypoint = allWaypoints[i];
            waypoint.alt = waypoint.legAltitude1;
            waypoint.real = true;
            if (waypoints.length > 1 && waypoint.transitionLLas) {
                let fromWaypoint = waypoints[waypoints.length - 2];
                for (let j = 0; j < waypoint.transitionLLas.length; j++) {
                    const coord = new LatLong(waypoint.transitionLLas[j].lat, waypoint.transitionLLas[j].long);
                    const dist = Avionics.Utils.computeGreatCircleDistance(fromWaypoint.infos.coordinates, coord);
                    const wp = {
                        ident: waypoint.ident + j,
                        cumulativeDistanceInFP: Math.min(fromWaypoint.cumulativeDistanceInFP + dist, waypoint.cumulativeDistanceInFP),
                        infos: { coordinates: {lat: waypoint.transitionLLas[j].lat, long: waypoint.transitionLLas[j].long}},
                        legAltitude1: waypoint.transitionLLas[j].alt,
                        real: false
                    };
                    allWaypoints.splice(i, 0, wp);
                    fromWaypoint = wp;
                    i++;
                }
            }
        }
        allWaypoints = allWaypoints.filter(function (wp) {
            return wp.ident !== "USER";
        });
        allWaypoints.push(last);
        const destination = this.getDestination();
        if (destination) {
            const fromStartDistance = destination.cumulativeDistanceInFP - distance;
            let prevIndex = 0;
            let prevReal;
            let prev;
            let next;
            let alt = 0;
            for (let i = 0; i < allWaypoints.length - 1; i++) {
                prev = allWaypoints[i];
                if (prev.real) {
                    prevReal = prev;
                    prevIndex++;
                }
                alt = prev.altitudeinFP ? prev.altitudeinFP : (prev.legAltitude1 ? prev.legAltitude1 : 0);
                next = allWaypoints[i + 1];
                if (prev.cumulativeDistanceInFP < fromStartDistance && next.cumulativeDistanceInFP > fromStartDistance) {
                    break;
                }
            }
            if (!next) {
                next = destination;
            }
            const output = new LatLong();
            const prevCD = prev ? prev.cumulativeDistanceInFP : 0;
            const nextCD = prev ? next.cumulativeDistanceInFP : 0;
            const d = (fromStartDistance - prevCD) / (nextCD - prevCD);
            const lat = (!prev ? SimVar.GetSimVarValue("PLANE LATITUDE", "degree latitude") : prev.infos.coordinates.lat);
            const long = (!prev ? SimVar.GetSimVarValue("PLANE LONGITUDE", "degree longitude") : prev.infos.coordinates.long);
            output.lat = Avionics.Utils.lerpAngle(lat, next.infos.coordinates.lat, Math.abs(d));
            output.long = Avionics.Utils.lerpAngle(long, next.infos.coordinates.long, Math.abs(d));
            const dist = prev ? Avionics.Utils.computeGreatCircleDistance(prev.infos.coordinates, output) : 0;
            return {
                lla: output,
                prevIndex: Math.max(0, prevIndex - 1),
                prevWp: prevReal,
                alt: 100 * Math.floor((alt + 50) / 100),
                cumulativeDistance: prev ? prev.cumulativeDistanceInFP : 0 + dist,
                distance: prevReal ? Avionics.Utils.computeGreatCircleDistance(prevReal.infos.coordinates, output) : 0
            };
        }
    }
}
