/*
 * A32NX
 * Copyright (C) 2020 FlyByWire Simulations and its contributors
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

class CDUDirectToPage {
    static ShowPage(mcdu, directWaypoint, wptsListIndex = 0) {
        mcdu.clearDisplay();
        mcdu.page.Current = mcdu.page.DirectToPage;
        mcdu.activeSystem = 'FMGC';
        let directWaypointCell = "";
        if (directWaypoint) {
            directWaypointCell = directWaypoint.ident;
        } else if (mcdu.flightPlanManager.getDirectToTarget()) {
            directWaypointCell = mcdu.flightPlanManager.getDirectToTarget().ident;
        }
        const waypointsCell = ["", "", "", "", ""];
        let iMax = 5;
        let eraseLabel = "";
        if (directWaypoint) {
            iMax--;
            eraseLabel = "\xa0DIR TO[color]amber";
            waypointsCell[4] = "{ERASE[color]amber";
            mcdu.onLeftInput[5] = () => {
                SimVar.SetSimVarValue("L:A320_NEO_PREVIEW_DIRECT_TO", "number", 0);
                CDUDirectToPage.ShowPage(mcdu);
            };
        }
        mcdu.onLeftInput[0] = (value) => {
            mcdu.getOrSelectWaypointByIdent(value, (w) => {
                if (w) {
                    SimVar.SetSimVarValue("L:A320_NEO_PREVIEW_DIRECT_TO", "number", 1);
                    SimVar.SetSimVarValue("L:A320_NEO_PREVIEW_DIRECT_TO_LAT_0", "number", SimVar.GetSimVarValue("PLANE LATITUDE", "degree latitude"));
                    SimVar.SetSimVarValue("L:A320_NEO_PREVIEW_DIRECT_TO_LONG_0", "number", SimVar.GetSimVarValue("PLANE LONGITUDE", "degree longitude"));
                    SimVar.SetSimVarValue("L:A320_NEO_PREVIEW_DIRECT_TO_LAT_1", "number", w.infos.coordinates.lat);
                    SimVar.SetSimVarValue("L:A320_NEO_PREVIEW_DIRECT_TO_LONG_1", "number", w.infos.coordinates.long);
                    CDUDirectToPage.ShowPage(mcdu, w, wptsListIndex);
                }
            });
        };
        let i = 0;
        wptsListIndex = Math.max(wptsListIndex, mcdu.flightPlanManager.getActiveWaypointIndex());
        const totalWaypointsCount = mcdu.flightPlanManager.getWaypointsCount() + mcdu.flightPlanManager.getArrivalWaypointsCount() + mcdu.flightPlanManager.getApproachWaypoints().length;
        while (i < totalWaypointsCount && i + wptsListIndex < totalWaypointsCount && i < iMax) {
            const waypoint = mcdu.flightPlanManager.getWaypoint(i + wptsListIndex, NaN, true);
            if (waypoint) {
                waypointsCell[i] = "{" + waypoint.ident + "[color]cyan";
                if (waypointsCell[i]) {
                    mcdu.onLeftInput[i + 1] = () => {
                        SimVar.SetSimVarValue("L:A320_NEO_PREVIEW_DIRECT_TO", "number", 1);
                        SimVar.SetSimVarValue("L:A320_NEO_PREVIEW_DIRECT_TO_LAT_0", "number", SimVar.GetSimVarValue("PLANE LATITUDE", "degree latitude"));
                        SimVar.SetSimVarValue("L:A320_NEO_PREVIEW_DIRECT_TO_LONG_0", "number", SimVar.GetSimVarValue("PLANE LONGITUDE", "degree longitude"));
                        SimVar.SetSimVarValue("L:A320_NEO_PREVIEW_DIRECT_TO_LAT_1", "number", waypoint.infos.coordinates.lat);
                        SimVar.SetSimVarValue("L:A320_NEO_PREVIEW_DIRECT_TO_LONG_1", "number", waypoint.infos.coordinates.long);
                        CDUDirectToPage.ShowPage(mcdu, waypoint, wptsListIndex);
                    };
                }
            } else {
                waypointsCell[i] = "----";
            }
            i++;
        }
        if (i < iMax) {
            waypointsCell[i] = "--END--";
        }
        let insertLabel = "";
        let insertLine = "";
        if (directWaypoint) {
            insertLabel = "\xa0TMPY[color]amber";
            insertLine = "DIRECT*[color]amber";
            mcdu.onRightInput[5] = () => {
                mcdu.activateDirectToWaypoint(directWaypoint, () => {
                    SimVar.SetSimVarValue("L:A320_NEO_PREVIEW_DIRECT_TO", "number", 0);
                    CDUFlightPlanPage.ShowPage(mcdu);
                });
            };
        }
        mcdu.setTemplate([
            ["DIR TO"],
            ["\xa0WAYPOINT", "DIST\xa0", "UTC"],
            ["*[" + (directWaypointCell ? directWaypointCell : "\xa0\xa0\xa0\xa0\xa0") + "][color]cyan", "---", "----"],
            ["\xa0F-PLN WPTS"],
            [waypointsCell[0], "DIRECT TO[color]cyan"],
            ["", "WITH\xa0"],
            [waypointsCell[1], "ABEAM PTS[color]cyan"],
            ["", "RADIAL IN\xa0"],
            [waypointsCell[2], "[ ]°[color]cyan"],
            ["", "RADIAL OUT\xa0"],
            [waypointsCell[3], "[ ]°[color]cyan"],
            [eraseLabel, insertLabel],
            [waypointsCell[4], insertLine]
        ]);
        let up = false;
        let down = false;
        if (wptsListIndex < totalWaypointsCount - 5) {
            mcdu.onUp = () => {
                wptsListIndex++;
                CDUDirectToPage.ShowPage(mcdu, directWaypoint, wptsListIndex);
            };
            up = true;
        }
        if (wptsListIndex > 0) {
            mcdu.onDown = () => {
                wptsListIndex--;
                CDUDirectToPage.ShowPage(mcdu, directWaypoint, wptsListIndex);
            };
            down = true;
        }
        mcdu.setArrows(up, down, false ,false);
    }
}
