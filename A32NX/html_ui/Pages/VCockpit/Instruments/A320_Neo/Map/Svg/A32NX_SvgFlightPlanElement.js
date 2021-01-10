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

class SvgFlightPlanElement extends SvgMapElement {
    constructor() {
        super(...arguments);
        this.flightPlanIndex = NaN;
        this.highlightActiveLeg = true;
        this.points = [];
        this.latLong = new LatLong();
        this._debugTransitionIndex = 0;
        this._lastP0X = NaN;
        this._lastP0Y = NaN;
        this.hideReachedWaypoints = true;
        this._highlightedLegIndex = -1;
        this._isDashed = false;
    }
    id(map) {
        return "flight-plan-" + this.flightPlanIndex + "-map-" + map.index;
        ;
    }
    createDraw(map) {
        const container = document.createElementNS(Avionics.SVG.NS, "svg");
        container.id = this.id(map);
        container.setAttribute("overflow", "visible");
        if (map.config.flightPlanNonActiveLegStrokeWidth > 0) {
            this._outlinePath = document.createElementNS(Avionics.SVG.NS, "path");
            this._outlinePath.setAttribute("stroke", map.config.flightPlanNonActiveLegStrokeColor);
            this._outlinePath.setAttribute("fill", "none");
            const outlinePathWidth = fastToFixed((map.config.flightPlanNonActiveLegStrokeWidth + map.config.flightPlanNonActiveLegWidth), 0);
            this._outlinePath.setAttribute("stroke-width", outlinePathWidth);
            this._outlinePath.setAttribute("stroke-linecap", "square");
            container.appendChild(this._outlinePath);
            this._outlineActive = document.createElementNS(Avionics.SVG.NS, "path");
            this._outlineActive.setAttribute("stroke", map.config.flightPlanActiveLegStrokeColor);
            this._outlineActive.setAttribute("fill", "none");
            const outlineActiveWidth = fastToFixed((map.config.flightPlanActiveLegStrokeWidth + map.config.flightPlanActiveLegWidth), 0);
            this._outlineActive.setAttribute("stroke-width", outlineActiveWidth);
            this._outlineActive.setAttribute("stroke-linecap", "square");
            container.appendChild(this._outlineActive);
            this._transitionOutlinePath = document.createElementNS(Avionics.SVG.NS, "path");
            this._transitionOutlinePath.setAttribute("stroke", map.config.flightPlanNonActiveLegStrokeColor);
            this._transitionOutlinePath.setAttribute("fill", "none");
            this._transitionOutlinePath.setAttribute("stroke-width", outlinePathWidth);
            this._transitionOutlinePath.setAttribute("stroke-linecap", "square");
            container.appendChild(this._transitionOutlinePath);
        }
        this._colorPath = document.createElementNS(Avionics.SVG.NS, "path");
        this._colorPath.setAttribute("stroke", map.config.flightPlanNonActiveLegColor);
        this._colorPath.setAttribute("fill", "none");
        if (this.flightPlanIndex === 1) {
            this._colorPath.setAttribute("stroke", "yellow");
        }
        const colorPathWidth = fastToFixed(map.config.flightPlanNonActiveLegWidth, 0);
        this._colorPath.setAttribute("stroke-width", colorPathWidth);
        this._colorPath.setAttribute("stroke-linecap", "square");
        container.appendChild(this._colorPath);
        this._colorActive = document.createElementNS(Avionics.SVG.NS, "path");
        this._colorActive.setAttribute("stroke", map.config.flightPlanActiveLegColor);
        this._colorActive.setAttribute("fill", "none");
        if (this.flightPlanIndex === 1) {
            this._colorActive.setAttribute("stroke", "yellow");
        }
        const colorActiveWidth = fastToFixed(map.config.flightPlanActiveLegWidth, 0);
        this._colorActive.setAttribute("stroke-width", colorActiveWidth);
        this._colorActive.setAttribute("stroke-linecap", "square");
        container.appendChild(this._colorActive);
        this._transitionPath = document.createElementNS(Avionics.SVG.NS, "path");
        this._transitionPath.setAttribute("stroke", map.config.flightPlanNonActiveLegColor);
        this._transitionPath.setAttribute("fill", "none");
        if (this.flightPlanIndex === 1) {
            this._transitionPath.setAttribute("stroke", "yellow");
        }
        this._transitionPath.setAttribute("stroke-width", colorPathWidth);
        this._transitionPath.setAttribute("stroke-linecap", "square");
        container.appendChild(this._transitionPath);
        this.setAsDashed(this._isDashed, true);
        return container;
    }

    updateDraw(map) {
        this._highlightedLegIndex = SimVar.GetSimVarValue("L:MAP_FLIGHTPLAN_HIGHLIT_WAYPOINT", "number");
        this.points = [];

        this.source.getContinuousSegments()
            .map(segment => this.makePathFromWaypoints(segment, map))
            .forEach((path, index) => this.makeOrUpdatePathElement(path, index, map));
    }

    /**
     * @param waypoints {WayPoint[]}
     * @param map
     */
    makePathFromWaypoints(waypoints, map) {
        let points = [];
        let pIndex = 0;

        for (const waypoint of waypoints) {
            const pathPoints = [];
            pathPoints.push(waypoint.infos.coordinates.toLatLong());

            for (let j = 0; j < pathPoints.length; j++) {
                this.latLong = pathPoints[j];

                let lastLat = NaN;
                let lastLong = NaN;
                if (this.latLong.lat !== lastLat && this.latLong.long !== lastLong) {
                    const deltaLong = Math.abs(lastLong - this.latLong.long);

                    if (deltaLong > 2) {
                        const lastX = Math.cos(lastLat / 180 * Math.PI) * Math.cos(lastLong / 180 * Math.PI);
                        const lastY = Math.cos(lastLat / 180 * Math.PI) * Math.sin(lastLong / 180 * Math.PI);
                        const lastZ = Math.sin(lastLat / 180 * Math.PI);
                        const X = Math.cos(this.latLong.lat / 180 * Math.PI) * Math.cos(this.latLong.long / 180 * Math.PI);
                        const Y = Math.cos(this.latLong.lat / 180 * Math.PI) * Math.sin(this.latLong.long / 180 * Math.PI);
                        const Z = Math.sin(this.latLong.lat / 180 * Math.PI);
                        const stepCount = Math.floor(deltaLong / 2);
                        for (let k = 0; k < stepCount; k++) {
                            const d = (k + 1) / (stepCount + 1);
                            const x = lastX * (1 - d) + X * d;
                            const y = lastY * (1 - d) + Y * d;
                            const z = lastZ * (1 - d) + Z * d;
                            const long = Math.atan2(y, x) / Math.PI * 180;
                            const hyp = Math.sqrt(x * x + y * y);
                            const lat = Math.atan2(z, hyp) / Math.PI * 180;
                            if (points[pIndex]) {
                                map.coordinatesToXYToRef(new LatLong(lat, long), points[pIndex]);
                            } else {
                                const p = map.coordinatesToXY(new LatLong(lat, long));
                                p.refWP = waypoint;
                                p.refWPIndex = i;
                                points.push(p);
                            }
                            pIndex++;
                        }
                    }

                    lastLat = this.latLong.lat;
                    lastLong = this.latLong.long;

                    if (points[pIndex]) {
                        map.coordinatesToXYToRef(this.latLong, points[pIndex]);
                        if (i === 0) {
                            if (points[0].x === this._lastP0X && points[0].y === this._lastP0Y) {
                                this._forceFullRedraw++;
                                if (this._forceFullRedraw < 60) {
                                    return;
                                }
                                this._forceFullRedraw = 0;
                            }
                            this._lastP0X = points[0].x;
                            this._lastP0Y = points[0].y;
                        }
                    } else {
                        const p = map.coordinatesToXY(this.latLong);
                        p.refWP = waypoint;
                        p.refWPIndex = i;
                        points.push(p);
                    }
                    pIndex++;
                }
            }
        }

        for (let bevels = 0; bevels < 4; bevels++) {
            const bevelAmount = map.NMToPixels(0.5) / (bevels + 1);
            if (points.length > 2) {
                const beveledPoints = [points[0]];
                for (let i = 1; i < points.length - 1; i++) {
                    const pPrev = points[i - 1];
                    const p = points[i];
                    const pNext = points[i + 1];
                    if ((pPrev.x == p.x && pPrev.y == p.y) || (pNext.x == p.x && pNext.y == p.y)) {
                        beveledPoints.push(p);
                        continue;
                    }
                    let xPrev = pPrev.x - p.x;
                    let yPrev = pPrev.y - p.y;
                    const dPrev = Math.sqrt(xPrev * xPrev + yPrev * yPrev);
                    xPrev /= dPrev;
                    yPrev /= dPrev;
                    let xNext = pNext.x - p.x;
                    let yNext = pNext.y - p.y;
                    const dNext = Math.sqrt(xNext * xNext + yNext * yNext);
                    xNext /= dNext;
                    yNext /= dNext;
                    const b = Math.min(dPrev / 3, dNext / 3, bevelAmount);
                    const refWPIndex = p.refWPIndex + (((bevels === 1) && (i % 2 === 0)) ? 1 : 0);
                    const refWP = (((bevels === 1) && (i % 2 === 0)) ? pNext.refWP : p.refWP);
                    beveledPoints.push({
                        x: p.x + xPrev * b,
                        y: p.y + yPrev * b,
                        refWP: refWP,
                        refWPIndex: refWPIndex,
                    }, {
                        x: p.x + xNext * b,
                        y: p.y + yNext * b,
                        refWP: refWP,
                        refWPIndex: refWPIndex,
                    });
                }
                beveledPoints.push(points[points.length - 1]);
                points = beveledPoints;
            }
        }

        if (points.length > 0) {
            let prevRefWPIndex = points[points.length - 1].refWPIndex;
            let prevRefWP = points[points.length - 1].refWP;
            for (let p = points.length - 2; p > 0; p--) {
                const point = points[p];
                if (point.refWPIndex > prevRefWPIndex) {
                    point.refWPIndex = prevRefWPIndex;
                    point.refWP = prevRefWP;
                }
                prevRefWPIndex = point.refWPIndex;
                prevRefWP = point.refWP;
            }
        }

        let pathString = "";

        for (let i = 0; i < points.length - 1; i++) {
            const p1 = points[i];
            let p2 = points[i + 1];
            if (!p2) {
                p2 = points[i + 2];
            }
            if (p1 && p2) {
                const p1x = fastToFixed(p1.x, 0);
                const p1y = fastToFixed(p1.y, 0);
                const p2x = fastToFixed(p2.x, 0);
                const p2y = fastToFixed(p2.y, 0);

                if (p1x !== p2x || p1y !== p2y) {
                    if (i === 0) {
                        pathString += "M" + p1x + " " + p1y + " L" + p2x + " " + p2y + " ";
                    } else {
                        if (p2.refWP.endsInDiscontinuity) {
                            pathString += `M ${p2x} ${p2y} `;
                        } else {
                            pathString += "L" + p2x + " " + p2y + " ";
                        }
                    }
                }
            }
        }

        return pathString;
    }

    /**
     * @param pathString {string}
     * @param index {number}
     * @param map
     */
    makeOrUpdatePathElement(pathString, index, map) {
        const existingElement = document.getElementById(`flight-plan-segment-${index}`);

        if (existingElement) {
            if (existingElement.getAttribute("d") !== pathString) {
                existingElement.setAttribute("d", pathString);
            }
        } else {
            const newElement = document.createElementNS(Avionics.SVG.NS, "path");

            newElement.setAttribute("id", `flight-plan-segment-${index}`);
            newElement.setAttribute("d", pathString);

            if (this.flightPlanIndex === 1) {
                newElement.setAttribute("stroke", "yellow");
            } else {
                newElement.setAttribute("stroke", map.config.flightPlanNonActiveLegColor);
            }

            newElement.setAttribute("fill", "none");

            newElement.setAttribute("stroke-width", fastToFixed(map.config.flightPlanNonActiveLegWidth, 0));
            newElement.setAttribute("stroke-linecap", "square");

            document.getElementById(this.id(map))
                .appendChild(newElement);
        }
    }

    setAsDashed(_val, _force = false) {
        if (_force || (_val != this._isDashed)) {
            this._isDashed = _val;
            if (this._colorActive && this._colorPath) {
                if (this._isDashed) {
                    const length = 14;
                    const spacing = 8;
                    this._colorPath.removeAttribute("stroke-linecap");
                    this._colorPath.setAttribute("stroke-dasharray", length + " " + spacing);
                    this._colorActive.removeAttribute("stroke-linecap");
                    this._colorActive.setAttribute("stroke-dasharray", length + " " + spacing);
                } else {
                    this._colorPath.removeAttribute("stroke-dasharray");
                    this._colorPath.setAttribute("stroke-linecap", "square");
                    this._colorActive.removeAttribute("stroke-dasharray");
                    this._colorActive.setAttribute("stroke-linecap", "square");
                }
            }
        }
    }
}
class SvgBackOnTrackElement extends SvgMapElement {
    constructor(overrideColor = "") {
        super();
        this.overrideColor = overrideColor;
        this._id = "back-on-track-" + SvgBackOnTrackElement.ID;
        SvgBackOnTrackElement.ID++;
    }
    id(map) {
        return this._id + "-map-" + map.index;
        ;
    }
    createDraw(map) {
        const container = document.createElementNS(Avionics.SVG.NS, "svg");
        container.id = this.id(map);
        container.setAttribute("overflow", "visible");
        if (map.config.flightPlanDirectLegStrokeWidth > 0) {
            this._outlineLine = document.createElementNS(Avionics.SVG.NS, "line");
            this._outlineLine.setAttribute("stroke", this.overrideColor != "" ? this.overrideColor : map.config.flightPlanDirectLegStrokeColor);
            const outlineDirectLegWidth = fastToFixed((map.config.flightPlanDirectLegStrokeWidth + map.config.flightPlanDirectLegWidth), 0);
            this._outlineLine.setAttribute("stroke-width", outlineDirectLegWidth);
            this._outlineLine.setAttribute("stroke-linecap", "square");
            container.appendChild(this._outlineLine);
        }
        this._colorLine = document.createElementNS(Avionics.SVG.NS, "line");
        this._colorLine.setAttribute("stroke", this.overrideColor != "" ? this.overrideColor : map.config.flightPlanDirectLegColor);
        const colorDirectLegWidth = fastToFixed(map.config.flightPlanDirectLegWidth, 0);
        this._colorLine.setAttribute("stroke-width", colorDirectLegWidth);
        this._colorLine.setAttribute("stroke-linecap", "square");
        container.appendChild(this._colorLine);
        return container;
    }
    updateDraw(map) {
        const p1 = map.coordinatesToXY(this.llaRequested);
        let p2;
        if (this.targetWaypoint) {
            p2 = map.coordinatesToXY(this.targetWaypoint.infos.coordinates);
        } else if (this.targetLla) {
            p2 = map.coordinatesToXY(this.targetLla);
        }
        let dx = p2.x - p1.x;
        let dy = p2.y - p1.y;
        const d = Math.sqrt(dx * dx + dy * dy);
        dx /= d;
        dy /= d;
        p1.x += dx * 20;
        p1.y += dy * 20;
        p2.x -= dx * 20;
        p2.y -= dy * 20;
        this._colorLine.setAttribute("x1", fastToFixed(p1.x, 0));
        this._colorLine.setAttribute("y1", fastToFixed(p1.y, 0));
        this._colorLine.setAttribute("x2", fastToFixed(p2.x, 0));
        this._colorLine.setAttribute("y2", fastToFixed(p2.y, 0));
        if (this._outlineLine) {
            this._outlineLine.setAttribute("x1", fastToFixed(p1.x, 0));
            this._outlineLine.setAttribute("y1", fastToFixed(p1.y, 0));
            this._outlineLine.setAttribute("x2", fastToFixed(p2.x, 0));
            this._outlineLine.setAttribute("y2", fastToFixed(p2.y, 0));
        }
    }
}
SvgBackOnTrackElement.ID = 0;
class SvgApproachFlightPlanDebugElement extends SvgMapElement {
    constructor() {
        super(...arguments);
        this.flightPlanIndex = NaN;
    }
    id(map) {
        return "flight-plan-" + this.flightPlanIndex + "-map-" + map.index;
        ;
    }
    createDraw(map) {
        const container = document.createElementNS(Avionics.SVG.NS, "svg");
        container.id = this.id(map);
        container.setAttribute("overflow", "visible");
        this._path = document.createElementNS(Avionics.SVG.NS, "path");
        this._path.setAttribute("stroke", "red");
        this._path.setAttribute("stroke-width", "4");
        this._path.setAttribute("fill", "none");
        container.appendChild(this._path);
        return container;
    }
    updateDraw(map) {
        if (this.source && this.source.waypoints) {
            let d = "";
            const waypoints = this.source.waypoints;
            for (let i = 0; i < waypoints.length; i++) {
                const wpPoints = [];
                if (waypoints[i].transitionLLas) {
                    for (let j = 0; j < waypoints[i].transitionLLas.length; j++) {
                        wpPoints.push(waypoints[i].transitionLLas[j]);
                    }
                }
                wpPoints.push(waypoints[i].lla);
                for (let j = 0; j < wpPoints.length; j++) {
                    const lla = wpPoints[j];
                    const xy = map.coordinatesToXY(lla);
                    if (i === 0 && j === 0) {
                        d += "M" + xy.x.toFixed(0) + " " + xy.y.toFixed(0) + " ";
                    } else {
                        d += "L" + xy.x.toFixed(0) + " " + xy.y.toFixed(0) + " ";
                    }
                }
            }
            this._path.setAttribute("d", d);
        }
    }
}
//# sourceMappingURL=SvgFlightPlanElement.js.map
