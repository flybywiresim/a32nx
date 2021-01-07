/*
 * A32NX
 * Copyright (C) 2020-2021 FlyByWire Simulations and its contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

class CDUFlightPlanPage {
    static ShowPage(mcdu, offset = 0) {
        mcdu.flightPlanManager.updateWaypointDistances(false /* approach */);
        mcdu.flightPlanManager.updateWaypointDistances(true /* approach */);
        mcdu.clearDisplay();
        mcdu.page.Current = mcdu.page.FlightPlanPage;
        mcdu.activeSystem = 'FMGC';
        CDUFlightPlanPage._timer = 0;
        mcdu.pageUpdate = () => {
            CDUFlightPlanPage._timer++;
            if (CDUFlightPlanPage._timer >= 100) {
                CDUFlightPlanPage.ShowPage(mcdu, offset);
            }
        };
        const isFlying = Simplane.getAltitudeAboveGround() > 10 ||
            Simplane.getEngineThrottleMode(0) >= ThrottleMode.FLEX_MCT && Simplane.getEngineThrottleMode(1) >= ThrottleMode.FLEX_MCT;
        let originIdentCell = "----";
        let runway = null;
        let showFrom = false;
        let showTMPY = false;

        const fpm = mcdu.flightPlanManager;

        if (fpm.getOrigin()) {
            originIdentCell = fpm.getOrigin().ident;
            runway = fpm.getDepartureRunway();
            if (runway) {
                originIdentCell += Avionics.Utils.formatRunway(runway.designation);
            }
        }
        const utcTime = SimVar.GetGlobalVarValue("ZULU TIME", "seconds");
        let originTimeCell = "----";
        if (fpm.getOrigin()) {
            if (isFlying) {
                originTimeCell = FMCMainDisplay.secondsToUTC(fpm._waypointReachedAt);
            } else {
                originTimeCell = "0000";
                fpm._waypointReachedAt = utcTime;
            }
        }
        let destCell = "----";
        let approachRunway = null;
        if (fpm.getDestination()) {
            destCell = fpm.getDestination().ident;
            approachRunway = fpm.getApproachRunway();
            if (approachRunway) {
                destCell += Avionics.Utils.formatRunway(approachRunway.designation);
            }
        }
        let rows = [[""], [""], [""], [""], [""], [""], [""], [""], [""], [""], [""], [""],];
        let rowsCount = 5;
        if (fpm.getCurrentFlightPlanIndex() === 1) {
            rowsCount = 5;
            rows[10] = [" ", " "];
            rows[11] = ["{ERASE[color]amber", "INSERT*[color]amber"];
            showTMPY = true;
            mcdu.onLeftInput[5] = async () => {
                mcdu.eraseTemporaryFlightPlan(() => {
                    CDUFlightPlanPage.ShowPage(mcdu, 0);
                });
            };
            mcdu.onRightInput[5] = async () => {
                mcdu.insertTemporaryFlightPlan(() => {
                    CDUFlightPlanPage.ShowPage(mcdu, 0);
                });
            };
        } else {
            let destTimeCell = "----";
            let destDistCell = "---";
            let destEFOBCell = "---";
            if (fpm.getDestination()) {
                destDistCell = fpm.getDestination().liveDistanceTo.toFixed(0);
                destEFOBCell = (mcdu.getDestEFOB(isFlying) * mcdu._conversionWeight).toFixed(1);
                if (isFlying) {
                    destTimeCell = FMCMainDisplay.secondsToUTC(fpm.getDestination().liveUTCTo);
                } else {
                    destTimeCell = FMCMainDisplay.secondsTohhmm(fpm.getDestination().liveETATo);
                }
            }
            if (!CDUInitPage.fuelPredConditionsMet(mcdu)) {
                destEFOBCell = "---";
            }
            rows[10] = ["\xa0DEST", "DIST EFOB", isFlying ? "UTC{sp}" : "TIME{sp}{sp}" ];//set last row
            rows[11] = [destCell, destDistCell + " " + destEFOBCell, destTimeCell + "{sp}{sp}"];
            mcdu.leftInputDelay[5] = () => {
                return mcdu.getDelaySwitchPage();
            };
            mcdu.onLeftInput[5] = () => {
                CDULateralRevisionPage.ShowPage(mcdu, fpm.getDestination(), fpm.getWaypointsCount() - 1);
            };
        }
        const waypointsWithDiscontinuities = [];
        const routeFirstWaypointIndex = 1 + fpm.getDepartureWaypointsCount();
        const routeLastWaypointIndex = fpm.getWaypointsCount() - 2 - fpm.getArrivalWaypointsCount();
        let first = 0;
        if (fpm.isActiveApproach()) {
            first = fpm.getWaypointsCount() - 1;
        } else {
            first = Math.max(0, fpm.getActiveWaypointIndex() - 1);
        }
        if (mcdu.currentFlightPhase <= FlightPhase.FLIGHT_PHASE_TAKEOFF) {
            first = 0;
        }
        for (let i = first; i < fpm.getWaypointsCount(); i++) {
            const prev = waypointsWithDiscontinuities[waypointsWithDiscontinuities.length - 1];
            const wp = fpm.getWaypoint(i);
            if (!prev || (prev.wp && prev.wp.ident != wp.ident)) {
                waypointsWithDiscontinuities.push({ wp: fpm.getWaypoint(i), fpIndex: i });
            }
        }
        const destination = waypointsWithDiscontinuities.pop();
        if (destination) {
            waypointsWithDiscontinuities.push(destination);
        }
        if (fpm.decelWaypoint) {
            const idx = waypointsWithDiscontinuities.length > 1 ?
                waypointsWithDiscontinuities.findIndex((e) => e.wp.cumulativeDistanceInFP > fpm.decelWaypoint.cumulativeDistanceInFP) : 0;
            if (idx >= 0 && idx < waypointsWithDiscontinuities.length) {
                waypointsWithDiscontinuities.splice(idx, 0, {
                    wp: fpm.decelWaypoint,
                    fpIndex: -42
                });
            }
        }
        if (waypointsWithDiscontinuities.length === 0) {
            rowsCount = 0;
            originIdentCell = "";
            rows = [
                ["", "SPD/ALT", "TIME{sp}{sp}"],
                ["PPOS[color]green", "---/ -----", "----{sp}{sp}"],
                [""],
                ["---F-PLN DISCONTINUITY---"],
                [""],
                ["------END OF F-PLN-------"],
                [""],
                ["-----NO ALTN F-PLN-------"],
                [""],
                [""],
                ["\xa0DEST", "DIST EFOB", "TIME{sp}{sp}"],
                ["------", "---- ----", "----{sp}{sp}"]
            ];
        } else {
            if (offset === 0) {
                showFrom = true;
            }
            rows[0] = ["", "SPD/ALT\xa0\xa0\xa0", isFlying ? "UTC{sp}" : "TIME{sp}{sp}"];
        }
        let iWaypoint = offset;
        let lastAltitudeConstraint = "";
        let lastSpeedConstraint = "";
        const activeIdent = fpm.getActiveWaypointIdent();
        const activeIndex = waypointsWithDiscontinuities.findIndex(w => {
            return w.wp && w.wp.ident === activeIdent;
        });

        let discontinuityCount = 0;
        for (let i = 0; i < (rowsCount > waypointsWithDiscontinuities.length ? waypointsWithDiscontinuities.length + 1 : rowsCount); i++) {
            const fixRow = i + discontinuityCount;
            let color = "green";
            if (fpm.getCurrentFlightPlanIndex() === 1) {
                color = "yellow";
            }
            if (waypointsWithDiscontinuities.length > 0) {
                iWaypoint = iWaypoint % (waypointsWithDiscontinuities.length + 1);
            }
            const index = iWaypoint;
            iWaypoint++;
            let depAltColor = "white";
            let depAlt = "-----";
            if (runway) {
                depAlt = (runway.elevation * 3.280).toFixed(0).toString();
                depAltColor = color;
            }
            depAlt = depAlt.padStart(6,"\xa0");
            if (index === 0 && first === 0) {
                rows[2 * fixRow + 1] = [`${originIdentCell}[color]${color}`, `{white}---{end}{${depAltColor}}/${depAlt}{end}[s-text]`, `${originTimeCell}[color]{sp}{sp}${color}[s-text]`];
                mcdu.leftInputDelay[fixRow] = () => {
                    return mcdu.getDelaySwitchPage();
                };
                mcdu.onLeftInput[fixRow] = async (value) => {
                    if (value === "") {
                        CDULateralRevisionPage.ShowPage(mcdu, fpm.getOrigin(), 0);
                    }
                };
            } else if (index === waypointsWithDiscontinuities.length - 1) {
                let destTimeCell = "----";
                let destDistCell = "---";
                let apprElev = "-----";
                let apprColor = "white";
                if (approachRunway) {
                    apprElev = (approachRunway.elevation * 3.280).toFixed(0).toString();
                    apprColor = color;
                }
                apprElev = apprElev.padStart(6,"\xa0");
                if (fpm.getDestination()) {
                    destDistCell = fpm.getDestination().liveDistanceTo.toFixed(0);
                    if (isFlying) {
                        destTimeCell = FMCMainDisplay.secondsToUTC(fpm.getDestination().liveUTCTo);
                    } else {
                        destTimeCell = FMCMainDisplay.secondsTohhmm(fpm.getDestination().liveETATo);
                    }
                    mcdu.leftInputDelay[fixRow] = () => {
                        return mcdu.getDelaySwitchPage();
                    };
                    mcdu.onLeftInput[fixRow] = (value) => {
                        if (value === "") {
                            CDULateralRevisionPage.ShowPage(mcdu, fpm.getDestination(), fpm.getWaypointsCount() - 1);
                        } else if (value === FMCMainDisplay.clrValue) {
                        } else if (value.length > 0) {
                            mcdu.insertWaypoint(value, fpm.getWaypointsCount() - 1, () => {
                                CDUFlightPlanPage.ShowPage(mcdu, offset);
                            });
                        }
                    };
                }
                rows[2 * fixRow + 1] = [destCell + "[color]" + color, "{white}---{end}{" + apprColor + "}/" + apprElev + "{end}" + "[s-text]", destTimeCell + "{sp}{sp}[color]" + color + "[s-text]"];
            } else if (index === waypointsWithDiscontinuities.length) {
                rows[2 * fixRow + 1] = ["------END OF F-PLN-------"]; // if last point then print the FPLN end
            } else if (index < waypointsWithDiscontinuities.length - 1) {
                let prevWaypoint;
                let waypoint;
                let fpIndex = 0;
                if (waypointsWithDiscontinuities[index]) {
                    waypoint = waypointsWithDiscontinuities[index].wp;
                    if (waypointsWithDiscontinuities[index - 1]) {
                        prevWaypoint = waypointsWithDiscontinuities[index - 1].wp;
                    }
                    fpIndex = waypointsWithDiscontinuities[index].fpIndex;
                }
                if (!waypoint) {
                    console.error("Should not reach.");
                } else {
                    let timeCell = "----";
                    if (isFlying) {
                        if (isFinite(waypoint.liveUTCTo) || isFinite(fpm._waypointReachedAt)) {
                            timeCell = FMCMainDisplay.secondsToUTC((index >= activeIndex || waypoint.ident === "(DECEL)" ? waypoint.liveUTCTo : fpm._waypointReachedAt)) + "[s-text]";
                        }
                    } else {
                        if (isFinite(waypoint.liveETATo)) {
                            timeCell = FMCMainDisplay.secondsTohhmm(index >= activeIndex || waypoint.ident === "(DECEL)" ? waypoint.liveETATo : 0) + "[s-text]";
                        }
                    }
                    if (fpIndex > fpm.getDepartureWaypointsCount()) {
                        if (fpIndex < fpm.getWaypointsCount() - fpm.getArrivalWaypointsCount()) {
                            if (waypoint.infos.airwayIdentInFP === "") {
                                const prevWaypointWithDiscontinuity = waypointsWithDiscontinuities[index - 1];
                                if (prevWaypointWithDiscontinuity) {
                                    prevWaypoint = prevWaypointWithDiscontinuity.wp;
                                }
                            }
                        }
                    }

                    const maxLineCount = rowsCount - discontinuityCount;

                    if (i < maxLineCount) { // enough space left before DEST line
                        let color = "green";
                        if (fpm.getCurrentFlightPlanIndex() === 1) {
                            color = "yellow";
                        } else if (waypoint === fpm.getActiveWaypoint()) {
                            color = "white";
                        }
                        let airwayName = "";
                        if (prevWaypoint && waypoint) {
                            let airway = undefined;
                            if (prevWaypoint.infos.airwayOut && prevWaypoint.infos.airwayOut === waypoint.infos.airwayIn) {
                                airway = {name: prevWaypoint.infos.airwayOut };
                            } else if (waypoint.infos.airwayIn && prevWaypoint.infos.airwayOut === undefined) {
                                airway = {name: waypoint.infos.airwayIn };
                            } else {
                                // vanilla Behavior that should not be working this way. (base don pilot feedback)
                                // airway = IntersectionInfo.GetCommonAirway(prevWaypoint, waypoint);
                            }
                            if (airway) {
                                airwayName = "\xa0" + airway.name;
                            }
                        }
                        const distance = (waypoint === fpm.getActiveWaypoint() ? waypoint.liveDistanceTo : waypoint.distanceInFP);
                        let dstnc;
                        if (i === 1) {
                            dstnc = distance.toFixed(0).toString() + "NM";
                        } else {
                            dstnc = distance.toFixed(0).toString() + "\xa0\xa0";
                        }
                        for (let z = 0; z < 9 - dstnc.length; z++) {
                            dstnc = dstnc + "\xa0";
                        }
                        dstnc = dstnc + "[color]" + color;
                        if (index === 0 && offset == 0) {
                            showFrom = true;
                        }

                        // Setup fix header

                        let fixAnnotation;
                        if (index === 0 && offset === 0) {
                            fixAnnotation = "\xa0FROM";
                        } else if (fpm.getDepartureProcIndex() !== -1 && fpm.getDepartureWaypoints().some(fix => fix === waypoint)) {
                            fixAnnotation = fpm.getDeparture().name;
                        } else if (fpm.getArrivalProcIndex() !== -1 && fpm.getArrivalWaypoints().some(fix => fix === waypoint)) {
                            fixAnnotation = fpm.getArrival().name;
                        } else {
                            fixAnnotation = airwayName;
                        }

                        // Render fix header

                        rows[2 * fixRow] = [`{sp}${fixAnnotation}`, ((index >= activeIndex || waypoint.ident === "(DECEL)") && i != 0 ? dstnc : i === 0 ? "SPD/ALT\xa0\xa0\xa0" : ""), i === 0 ? (isFlying ? "\xa0UTC{sp}" : "TIME{sp}{sp}") : ""];

                        let speedConstraint = "---";
                        if (waypoint.speedConstraint > 10) {
                            speedConstraint = "{magenta}*{end}" + waypoint.speedConstraint.toFixed(0);
                            if (speedConstraint === lastSpeedConstraint) {
                                speedConstraint = "\xa0\"\xa0";
                            } else {
                                lastSpeedConstraint = speedConstraint;
                            }
                        }

                        let altitudeConstraint = "-----";
                        let altPrefix = "\xa0";
                        const isDepartureWayPoint = routeFirstWaypointIndex > 1 && fpm.getDepartureWaypoints().indexOf(waypointsWithDiscontinuities[index]) !== -1;

                        if (mcdu.transitionAltitude >= 100 && waypoint.legAltitude1 > mcdu.transitionAltitude) {
                            altitudeConstraint = (waypoint.legAltitude1 / 100).toFixed(0).toString();
                            altitudeConstraint = "FL" + altitudeConstraint.padStart(3,"0");
                        } else {
                            altitudeConstraint = waypoint.legAltitude1.toFixed(0).toString().padStart(5,"\xa0");
                        }
                        if (waypoint.legAltitudeDescription !== 0 && waypoint.ident !== "(DECEL)") {
                            altPrefix = "{magenta}*{end}";
                            if (waypoint.legAltitudeDescription === 4) {
                                altitudeConstraint = ((waypoint.legAltitude1 + waypoint.legAltitude2) * 0.5).toFixed(0).toString();
                                altitudeConstraint = altitudeConstraint.padStart(5,"\xa0");
                            }
                        //predict altitude for STAR when constraints are missing
                        } else if (isDepartureWayPoint) {
                            altitudeConstraint = Math.floor(waypoint.cumulativeDistanceInFP * 0.14 * 6076.118 / 10);
                            altitudeConstraint = altitudeConstraint.padStart(5,"\xa0");
                        //waypoint is the first or the last of the actual route
                        } else if ((index === routeFirstWaypointIndex - 1) || (index === routeLastWaypointIndex + 1)) {
                            altitudeConstraint = "FL" + mcdu.cruiseFlightLevel.toString().padStart(3,"0"); ;
                        //waypoint is in between on the route
                        } else if (index <= routeLastWaypointIndex && index >= routeFirstWaypointIndex) {
                            altitudeConstraint = "FL" + mcdu.cruiseFlightLevel.toString().padStart(3,"0"); ;

                        }
                        if (altitudeConstraint === lastAltitudeConstraint) {
                            altitudeConstraint = "\xa0\xa0\"\xa0\xa0";
                        } else {
                            lastAltitudeConstraint = altitudeConstraint;
                        }
                        if (fpIndex !== -42) {
                            mcdu.leftInputDelay[fixRow] = (value) => {
                                if (value === "") {
                                    if (waypoint) {
                                        return mcdu.getDelaySwitchPage();
                                    }
                                }
                                return mcdu.getDelayBasic();
                            };
                            mcdu.onLeftInput[fixRow] = async (value) => {
                                if (value === "") {
                                    if (waypoint) {
                                        CDULateralRevisionPage.ShowPage(mcdu, waypoint, fpIndex);
                                    }
                                } else if (value === FMCMainDisplay.clrValue) {
                                    mcdu.removeWaypoint(fpIndex, () => {
                                        CDUFlightPlanPage.ShowPage(mcdu, offset);
                                    });
                                } else if (value.length > 0) {
                                    mcdu.insertWaypoint(value, fpIndex, () => {
                                        CDUFlightPlanPage.ShowPage(mcdu, offset);
                                    });
                                }
                            };
                            mcdu.rightInputDelay[fixRow] = () => {
                                return mcdu.getDelaySwitchPage();
                            };
                            mcdu.onRightInput[fixRow] = async () => {
                                if (waypoint) {
                                    CDUVerticalRevisionPage.ShowPage(mcdu, waypoint);
                                }
                            };
                        }

                        if (mcdu.activeHold && mcdu.activeHold.has(waypoint.ident)) {
                            const holdRows = [
                                [waypoint.ident + "[color]" + color, speedConstraint + "/" + altitudeConstraint + "[s-text][color]" + color, timeCell + "{sp}{sp}[color]" + color],
                                ["" , "IMM[color]cyan" , "HOLD"],
                                ["HOLD " + mcdu.activeHold.get(waypoint.ident).turn + "[color]" + color, "EXIT*[color]cyan", "SPD " + mcdu.activeHold.get(waypoint.ident).speed],
                                ["C" + mcdu.activeHold.get(waypoint.ident).course.toFixed(0) + "°"],
                                [waypoint.ident + "[color]" + color, speedConstraint + "/" + altitudeConstraint + "[s-text][color]" + color, timeCell + "{sp}{sp}[color]" + color]
                            ];

                            mcdu.rightInputDelay[fixRow + 1] = () => {
                                return mcdu.getDelaySwitchPage();
                            };
                            // place the button input on the HOLD line to clear the hold
                            mcdu.onRightInput[fixRow + 1] = async () => {
                                // TODO remove any active waypoints and route to active waypoint
                                mcdu.activeHold = null;
                                CDUFlightPlanPage.ShowPage(mcdu, offset);
                            };

                            for (let j = 0; i < rowsCount; i++) {
                                rows[2 * i + 1] = holdRows[j++];
                                if (i == rowsCount - 1 || j >= holdRows.length) {
                                    break;
                                }

                                rows[2 * i + 2] = holdRows[j++];
                            }
                        } else {
                            let spdColor = color;
                            let altColor = color;
                            if (speedConstraint === "---") {
                                spdColor = "white";
                            }
                            if (altitudeConstraint === "-----") {
                                altColor = "white";
                            }
                            rows[2 * fixRow + 1] = [
                                `${waypoint.ident}[color]${color}`,
                                `{${spdColor}}${speedConstraint}{end}{${altColor}}/${altPrefix}${altitudeConstraint}{end}[s-text]`,
                                `${timeCell}{sp}{sp}[color]white`
                            ];

                            if (waypoint.endsInDiscontinuity) {
                                if (i + 1 < maxLineCount) {
                                    rows[2 * fixRow + 2] = [""];
                                    rows[2 * fixRow + 3] = ["---F-PLN DISCONTINUITY--"];

                                    mcdu.onLeftInput[fixRow + 1] = (value) => {
                                        if (value === FMCMainDisplay.clrValue && waypoint.discontinuityCanBeCleared) {
                                            waypoint.endsInDiscontinuity = false;

                                            CDUFlightPlanPage.ShowPage(mcdu, offset);
                                        }
                                    };
                                }
                                discontinuityCount++;
                            }
                        }
                    } else {
                        let destTimeCell = "----";
                        let destDistCell = "---";
                        if (fpm.getDestination()) {
                            destDistCell = fpm.getDestination().infos.totalDistInFP.toFixed(0);
                            if (isFlying) {
                                destTimeCell = FMCMainDisplay.secondsToUTC(fpm.getDestination().estimatedTimeOfArrivalFP);
                            } else {
                                destTimeCell = FMCMainDisplay.secondsTohhmm(fpm.getDestination().cumulativeEstimatedTimeEnRouteFP);
                            }
                        }

                        mcdu.leftInputDelay[fixRow] = () => mcdu.getDelaySwitchPage();

                        mcdu.onLeftInput[fixRow] = () => {
                            CDULateralRevisionPage.ShowPage(mcdu, fpm.getDestination(), fpm.getWaypointsCount() - 1);
                        };
                    }
                }
            }
        }
        mcdu.currentFlightPlanWaypointIndex = offset + first;
        const wpCount = fpm.getWaypointsCount();
        if (wpCount > 0) {
            while (mcdu.currentFlightPlanWaypointIndex < 0) {
                mcdu.currentFlightPlanWaypointIndex += wpCount;
            }
            while (mcdu.currentFlightPlanWaypointIndex >= wpCount) {
                mcdu.currentFlightPlanWaypointIndex -= wpCount;
            }
        }
        mcdu.setTemplate([
            [`{left}{small}{sp}${showFrom ? "FROM" : "{sp}{sp}{sp}{sp}"}{end}{yellow}{sp}${showTMPY ? "TMPY" : ""}{end}{end}{right}{small}${SimVar.GetSimVarValue("ATC FLIGHT NUMBER", "string", "FMC")}{sp}{sp}{sp}{end}{end}`],
            ...rows
        ]);
        const allowScroll = waypointsWithDiscontinuities.length > 4;
        if (allowScroll) {//scroll only if there are more than 5 points
            mcdu.onDown = () => {//on page down decrement the page offset.
                if (offset > 0) { // if page not on top
                    offset--;
                } else { // else go to the bottom
                    offset = waypointsWithDiscontinuities.length;
                }
                CDUFlightPlanPage.ShowPage(mcdu, offset);
            };
            mcdu.onUp = () => {
                if (offset < waypointsWithDiscontinuities.length) { // if page not on bottom
                    offset++;
                } else { // else go on top
                    offset = 0;
                }
                CDUFlightPlanPage.ShowPage(mcdu, offset);
            };
        }
        mcdu.setArrows(allowScroll, allowScroll, true, true);
    }
}
CDUFlightPlanPage._timer = 0;
