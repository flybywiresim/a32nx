var Jet_NDCompass_Display;
(function (Jet_NDCompass_Display) {
    Jet_NDCompass_Display[Jet_NDCompass_Display["NONE"] = 0] = "NONE";
    Jet_NDCompass_Display[Jet_NDCompass_Display["ROSE"] = 1] = "ROSE";
    Jet_NDCompass_Display[Jet_NDCompass_Display["ARC"] = 2] = "ARC";
    Jet_NDCompass_Display[Jet_NDCompass_Display["PLAN"] = 3] = "PLAN";
})(Jet_NDCompass_Display || (Jet_NDCompass_Display = {}));
var Jet_NDCompass_Navigation;
(function (Jet_NDCompass_Navigation) {
    Jet_NDCompass_Navigation[Jet_NDCompass_Navigation["NONE"] = 0] = "NONE";
    Jet_NDCompass_Navigation[Jet_NDCompass_Navigation["VOR"] = 1] = "VOR";
    Jet_NDCompass_Navigation[Jet_NDCompass_Navigation["NAV"] = 2] = "NAV";
    Jet_NDCompass_Navigation[Jet_NDCompass_Navigation["ILS"] = 3] = "ILS";
})(Jet_NDCompass_Navigation || (Jet_NDCompass_Navigation = {}));
var Jet_NDCompass_Reference;
(function (Jet_NDCompass_Reference) {
    Jet_NDCompass_Reference[Jet_NDCompass_Reference["NONE"] = 0] = "NONE";
    Jet_NDCompass_Reference[Jet_NDCompass_Reference["HEADING"] = 1] = "HEADING";
    Jet_NDCompass_Reference[Jet_NDCompass_Reference["TRACK"] = 2] = "TRACK";
})(Jet_NDCompass_Reference || (Jet_NDCompass_Reference = {}));
class Jet_NDCompass_Range {
}
class Jet_NDCompass extends HTMLElement {
    constructor() {
        super(...arguments);
        this.lastSelectedHeading = -1;
        this.showSelectedHeadingTimer = 0;
        this.mapRanges = [];
        this.isBearing1Displayed = false;
        this.isBearing2Displayed = false;
        this._showILS = false;
        this._fullscreen = true;
        this.isHud = false;
        this.logic_brg1Source = 0;
        this.logic_brg2Source = 0;
        this._displayMode = Jet_NDCompass_Display.NONE;
        this._navigationMode = Jet_NDCompass_Navigation.NONE;
        this._referenceMode = Jet_NDCompass_Reference.NONE;
        this._aircraft = Aircraft.A320_NEO;
        this._mapRange = 0;
        this._delayedCompass = 0;
    }
    static get dynamicAttributes() {
        return [
            "rotation",
            "tracking_bug_rotation",
            "heading_bug_rotation",
            "selected_heading_bug_rotation",
            "selected_tracking_bug_rotation",
            "ils_bug_rotation",
            "course",
            "course_deviation",
            "show_bearing1",
            "show_bearing2",
            "toggle_bearing1",
            "toggle_bearing2",
            "bearing1_bearing",
            "bearing2_bearing",
            "display_course_deviation",
            "vertical_deviation",
            "display_vertical_deviation"
        ];
    }
    static get observedAttributes() {
        return this.dynamicAttributes.concat([
            "hud"
        ]);
    }
    get displayMode() {
        return this._displayMode;
    }
    get navigationMode() {
        return this._navigationMode;
    }
    get referenceMode() {
        return this._referenceMode;
    }
    setMode(d, n) {
        if (this.displayMode !== d || this.navigationMode != n) {
            this._displayMode = d;
            this._navigationMode = n;
            this.construct();
        }
    }
    get aircraft() {
        return this._aircraft;
    }
    set aircraft(_val) {
        if (this._aircraft != _val) {
            this._aircraft = _val;
            this.construct();
        }
    }
    get mapRange() {
        return this._mapRange;
    }
    set mapRange(_val) {
        this._mapRange = _val;
    }
    showArcRange(_show) {
        if (this.arcRangeGroup) {
            this.arcRangeGroup.setAttribute("visibility", (_show) ? "visible" : "hidden");
        }
    }
    showArcMask(_val) {
        if (this.arcMaskGroup) {
            this.arcMaskGroup.setAttribute("visibility", (_val) ? "visible" : "hidden");
        }
    }
    setFullscreen(_val) {
        if (this._fullscreen != _val) {
            this._fullscreen = _val;
            this.construct();
        }
    }
    connectedCallback() {
        if (this.hasAttribute("display-mode")) {
            const modeValue = this.getAttribute("display-mode").toLocaleLowerCase();
            if (modeValue === "rose") {
                this._displayMode = Jet_NDCompass_Display.ROSE;
            } else if (modeValue === "arc") {
                this._displayMode = Jet_NDCompass_Display.ARC;
            } else if (modeValue === "plan") {
                this._displayMode = Jet_NDCompass_Display.PLAN;
            } else {
                this._displayMode = Jet_NDCompass_Display.NONE;
            }
        }
        if (this.hasAttribute("navigation-mode")) {
            const modeValue = this.getAttribute("navigation-mode").toLocaleLowerCase();
            if (modeValue === "ils") {
                this._navigationMode = Jet_NDCompass_Navigation.ILS;
            } else if (modeValue === "vor") {
                this._navigationMode = Jet_NDCompass_Navigation.VOR;
            } else if (modeValue === "nav") {
                this._navigationMode = Jet_NDCompass_Navigation.NAV;
            } else {
                this._navigationMode = Jet_NDCompass_Navigation.NONE;
            }
        }
        this.construct();
    }
    init() {
    }
    construct() {
        this.destroyLayout();
        switch (this.displayMode) {
            case Jet_NDCompass_Display.ROSE:
            {
                this.constructRose();
                break;
            }
            case Jet_NDCompass_Display.ARC:
            {
                this.constructArc();
                break;
            }
            case Jet_NDCompass_Display.PLAN:
            {
                this.constructPlan();
                break;
            }
        }
    }
    constructArc() {
    }
    constructRose() {
    }
    constructPlan() {
    }
    destroyLayout() {
        if (this.root) {
            this.root.remove();
        }
        for (let i = 0; i < Jet_NDCompass.dynamicAttributes.length; i++) {
            this.removeAttribute(Jet_NDCompass.dynamicAttributes[i]);
        }
        this.rotatingCircle = null;
        this.rotatingCircleTrs = "";
        this.graduations = null;
        this.mapRanges = [];
        this.courseTOLine = null;
        this.courseFROMLine = null;
    }
    update(_deltaTime) {
        this.updateCompass(_deltaTime);
        this.updateNavigationInfo();
        this.updateMapRange();

        // Moved to A32NX_NDCompass.update()
        // if (this.updateFail) this.updateFail();
    }
    updateCompass(_deltaTime) {
        const simHeading = SimVar.GetSimVarValue("PLANE HEADING DEGREES MAGNETIC", "degree");
        const isTRKMode = SimVar.GetSimVarValue("L:A32NX_TRK_FPA_MODE_ACTIVE", "Bool");
        const simSelectedHeading = isTRKMode ?
            SimVar.GetSimVarValue("L:A32NX_AUTOPILOT_TRACK_SELECTED:1", "Degree") :
            SimVar.GetSimVarValue("AUTOPILOT HEADING LOCK DIR", "degree");
        let simTrack = SimVar.GetSimVarValue("GPS GROUND MAGNETIC TRACK", "degree");
        const simSelectedTrack = SimVar.GetSimVarValue("GPS WP DESIRED TRACK", "degree");
        const simGroundSpeed = SimVar.GetSimVarValue("GPS GROUND SPEED", "knots");
        this._referenceMode = Jet_NDCompass_Reference.HEADING;

        let headingChanged = false;
        if (Math.round(simSelectedHeading) != this.lastSelectedHeading) {
            if (this.lastSelectedHeading >= 0) {
                headingChanged = true;
            }
            this.lastSelectedHeading = Math.round(simSelectedHeading);
        }
        let compass = 0;
        if (this.displayMode !== Jet_NDCompass_Display.PLAN) {
            if (this.referenceMode == Jet_NDCompass_Reference.TRACK) {
                compass = simTrack;
                if (this.currentRefMode) {
                    this.currentRefMode.textContent = "TRK";
                }
            } else {
                compass = simHeading;
                if (this.currentRefMode) {
                    this.currentRefMode.textContent = "HDG";
                }
            }

            // This stuff makes the compass do a smooth spin to the actual heading after alignment finishes
            const desiredRotationHeading = SimVar.GetSimVarValue("L:A320_Neo_ADIRS_STATE", "Number") !== 2
                ? 0
                : compass;
            const delta = ((desiredRotationHeading - this._delayedCompass + 540) % 360) - 180;
            if (Math.abs(delta) > 0.01) {
                this._delayedCompass += delta * Math.min(1, 4 * (_deltaTime / 1000));

                // Keep heading values within [0,360] range
                this._delayedCompass %= 360;

                compass = this._delayedCompass;
            }

            const roundedCompass = fastToFixed(compass, 2);
            this.setAttribute("rotation", roundedCompass);
            if (this.currentRefGroup) {
                this.currentRefGroup.classList.toggle('hide', false);
            }
            if (this.currentRefValue) {
                this.currentRefValue.textContent = Utils.leadingZeros(compass % 360, 3, 0);
            }
        } else {
            this.setAttribute("rotation", "0");
            if (this.currentRefGroup) {
                this.currentRefGroup.classList.toggle('hide', true);
            }
        }
        if (this.referenceMode == Jet_NDCompass_Reference.HEADING) {
            if (this.aircraft == Aircraft.A320_NEO) {
                var selectedHeading = simSelectedHeading;
                const showSelectedHeading = SimVar.GetSimVarValue("L:A320_FCU_SHOW_SELECTED_HEADING", "number") === 1;
                const lateralMode = SimVar.GetSimVarValue("L:A32NX_FMA_LATERAL_MODE", "Number");
                const showTrackLine = (lateralMode === 10 || lateralMode === 11);
                if (showSelectedHeading) {
                    selectedHeading = SimVar.GetSimVarValue("L:A32NX_AUTOPILOT_HEADING_SELECTED", "Degrees");
                }
                var roundedSelectedHeading = fastToFixed(selectedHeading, 3);
                this.setAttribute("selected_heading_bug_rotation", roundedSelectedHeading);
                if (this.trackingLine) {
                    this.trackingLine.classList.toggle('hide', !showTrackLine);
                }
                if (this.selectedHeadingGroup) {
                    this.selectedHeadingGroup.classList.toggle('hide', !showSelectedHeading);
                }
                if (this.selectedTrackGroup) {
                    this.selectedTrackGroup.classList.toggle('hide', true);
                }
                if (this.selectedRefGroup) {
                    if (this.selectedRefValue) {
                        this.selectedRefValue.textContent = selectedHeading.toString();
                    }
                    this.selectedRefGroup.classList.toggle('hide', false);
                }
                if (this.headingGroup) {
                    this.headingGroup.classList.toggle('hide', true);
                }
            } else if (this.aircraft == Aircraft.B747_8 || this.aircraft == Aircraft.AS01B) {
                var selectedHeading = simSelectedHeading;
                let showSelectedLine = Simplane.getAutoPilotHeadingLockActive();
                if (!showSelectedLine) {
                    if (headingChanged) {
                        showSelectedLine = true;
                        this.showSelectedHeadingTimer = 5;
                    } else if (this.showSelectedHeadingTimer > 0) {
                        this.showSelectedHeadingTimer -= _deltaTime / 1000;
                        if (this.showSelectedHeadingTimer <= 0) {
                            this.showSelectedHeadingTimer = 0;
                        } else {
                            showSelectedLine = true;
                        }
                    }
                } else {
                    this.showSelectedHeadingTimer = 0;
                }
                var roundedSelectedHeading = fastToFixed(selectedHeading, 3);
                this.setAttribute("selected_heading_bug_rotation", roundedSelectedHeading);
                if (this.navigationMode == Jet_NDCompass_Navigation.NAV) {
                    if (this.selectedHeadingGroup) {
                        this.selectedHeadingGroup.classList.toggle('hide', false);
                    }
                    if (this.selectedHeadingLine) {
                        this.selectedHeadingLine.classList.toggle('hide', !showSelectedLine);
                    }
                } else {
                    if (this.selectedHeadingGroup) {
                        this.selectedHeadingGroup.classList.toggle('hide', true);
                    }
                }
                if (this.selectedTrackGroup) {
                    this.selectedTrackGroup.classList.toggle('hide', true);
                }
                if (this.selectedRefGroup) {
                    if (this.selectedRefValue) {
                        this.selectedRefValue.textContent = selectedHeading.toString();
                    }
                    this.selectedRefGroup.classList.toggle('hide', false);
                }
            } else {
                let showSelectedHeading = Simplane.getAutoPilotHeadingLockActive();
                if (!showSelectedHeading) {
                    if (headingChanged) {
                        showSelectedHeading = true;
                        this.showSelectedHeadingTimer = 5;
                    } else if (this.showSelectedHeadingTimer > 0) {
                        this.showSelectedHeadingTimer -= _deltaTime / 1000;
                        if (this.showSelectedHeadingTimer <= 0) {
                            this.showSelectedHeadingTimer = 0;
                        } else {
                            showSelectedHeading = true;
                        }
                    }
                } else {
                    this.showSelectedHeadingTimer = 0;
                }
                if (showSelectedHeading) {
                    var selectedHeading = simSelectedHeading;
                    var roundedSelectedHeading = fastToFixed(selectedHeading, 3);
                    this.setAttribute("selected_heading_bug_rotation", roundedSelectedHeading);
                    if (this.navigationMode == Jet_NDCompass_Navigation.NAV) {
                        if (this.selectedHeadingGroup) {
                            this.selectedHeadingGroup.classList.toggle('hide', false);
                        }
                        if (this.selectedHeadingLine) {
                            if (this.aircraft == Aircraft.CJ4 && this.displayMode == Jet_NDCompass_Display.ARC) {
                                const CompassAngle = this.degreeToArc(compass);
                                const selectedAngle = this.degreeToArc(simSelectedHeading);
                                const delta = Math.abs(CompassAngle - selectedAngle);
                                this.selectedHeadingLine.classList.toggle('hide', (delta > 65) ? false : true);
                                this.selectedHeadingBug.classList.toggle('hide', (delta > 90) ? true : false);
                            } else {
                                this.selectedHeadingLine.classList.toggle('hide', false);
                            }
                        }
                    } else {
                        if (this.selectedHeadingGroup) {
                            this.selectedHeadingGroup.classList.toggle('hide', true);
                        }
                    }
                    if (this.selectedTrackGroup) {
                        this.selectedTrackGroup.classList.toggle('hide', true);
                    }
                    if (this.selectedRefGroup) {
                        if (this.selectedRefValue) {
                            this.selectedRefValue.textContent = selectedHeading.toString();
                        }
                        this.selectedRefGroup.classList.toggle('hide', false);
                    }
                } else {
                    if (this.selectedHeadingGroup) {
                        this.selectedHeadingGroup.classList.toggle('hide', true);
                    }
                    if (this.selectedTrackGroup) {
                        this.selectedTrackGroup.classList.toggle('hide', true);
                    }
                    if (this.selectedRefGroup) {
                        this.selectedRefGroup.classList.toggle('hide', true);
                    }
                }
            }
        } else {
            const selectedTrack = simSelectedTrack;
            const roundedSelectedTrack = fastToFixed(selectedTrack, 3);
            this.setAttribute("selected_tracking_bug_rotation", roundedSelectedTrack);
            if (this.trackingLine) {
                this.trackingLine.classList.toggle('hide', false);
            }
            if (this.selectedHeadingGroup) {
                this.selectedHeadingGroup.classList.toggle('hide', true);
            }
            if (this.selectedTrackGroup) {
                this.selectedTrackGroup.classList.toggle('hide', false);
            }
            if (this.selectedRefGroup) {
                if (this.selectedRefValue) {
                    this.selectedRefValue.textContent = selectedTrack.toString();
                }
                this.selectedRefGroup.classList.toggle('hide', false);
            }
            if (this.headingGroup) {
                this.headingGroup.classList.toggle('hide', false);
            }
        }
        const heading = simHeading;
        const roundedHeading = fastToFixed(heading, 3);
        this.setAttribute("heading_bug_rotation", roundedHeading);
        if (simGroundSpeed <= 10) {
            simTrack = simHeading;
        }
        const roundedTracking = fastToFixed(simTrack, 3);
        this.setAttribute("tracking_bug_rotation", roundedTracking);
        if (this.ilsGroup) {
            if (this._showILS || this.navigationMode == Jet_NDCompass_Navigation.ILS) {
                const localizer = this.gps.radioNav.getBestILSBeacon();
                if (localizer.id > 0) {
                    const roundedCourse = fastToFixed(localizer.course, 3);
                    this.setAttribute("ils_bug_rotation", roundedCourse);
                    this.ilsGroup.classList.toggle('hide', false);
                } else {
                    this.ilsGroup.classList.toggle('hide', true);
                }
            } else {
                this.ilsGroup.classList.toggle('hide', true);
            }
        }
        this.applyRotation();
    }
    updateMapRange() {
        if (this.mapRanges) {
            for (let i = 0; i < this.mapRanges.length; i++) {
                const range = this.mapRange * this.mapRanges[i].factor;
                if (range < 1.0 && this.mapRanges[i].removeInteger) {
                    const rangeText = (Math.floor(range * 100) / 100).toFixed(2);
                    const radixPos = rangeText.indexOf('.');
                    this.mapRanges[i].text.textContent = rangeText.slice(radixPos);
                } else {
                    this.mapRanges[i].text.textContent = range.toString();
                }
            }
        }
    }
    updateNavigationInfo() {
        if (this.courseGroup) {
            if (this.navigationMode == Jet_NDCompass_Navigation.ILS || this.navigationMode == Jet_NDCompass_Navigation.VOR) {
                this.courseGroup.classList.toggle('hide', false);
                const compass = Number(this.getAttribute('rotation'));
                let displayCourseDeviation = false;
                let displayVerticalDeviation = false;
                if (this.navigationMode == Jet_NDCompass_Navigation.ILS || this.navigationMode === Jet_NDCompass_Navigation.VOR) {
                    let beacon;
                    if (this.navigationMode == Jet_NDCompass_Navigation.ILS) {
                        beacon = this.gps.radioNav.getBestILSBeacon();
                    } else if (this.navigationMode === Jet_NDCompass_Navigation.VOR) {
                        beacon = this.gps.radioNav.getBestVORBeacon();
                    }
                    if (beacon.id > 0) {
                        displayCourseDeviation = true;
                        let deviation = (SimVar.GetSimVarValue("NAV CDI:" + beacon.id, "number") / 127);
                        const backCourse = SimVar.GetSimVarValue("AUTOPILOT BACKCOURSE HOLD", "bool");
                        if (backCourse) {
                            deviation = -deviation;
                        }
                        this.setAttribute("course", beacon.course.toString());
                        this.setAttribute("course_deviation", deviation.toString());
                        if (SimVar.GetSimVarValue("NAV HAS GLIDE SLOPE:" + beacon.id, "Bool")) {
                            displayVerticalDeviation = true;
                            this.setAttribute("vertical_deviation", (SimVar.GetSimVarValue("NAV GSI:" + beacon.id, "number") / 127.0).toString());
                        }
                    } else {
                        this.setAttribute("course", compass.toString());
                        this.setAttribute("course_deviation", "0");
                    }
                }
                this.setAttribute("display_course_deviation", displayCourseDeviation ? "True" : "False");
                this.setAttribute("display_vertical_deviation", displayVerticalDeviation ? "True" : "False");
                switch (this.logic_brg1Source) {
                    case 1:
                    {
                        const hasNav = SimVar.GetSimVarValue("NAV HAS NAV:1", "boolean");
                        if (hasNav) {
                            this.setAttribute("bearing1_bearing", ((180 + SimVar.GetSimVarValue("NAV RADIAL:1", "degree")) % 360).toString());
                        } else {
                            this.setAttribute("bearing1_bearing", "");
                        }
                        break;
                    }
                    case 2:
                    {
                        const hasNav = SimVar.GetSimVarValue("NAV HAS NAV:2", "boolean");
                        if (hasNav) {
                            this.setAttribute("bearing1_bearing", ((180 + SimVar.GetSimVarValue("NAV RADIAL:2", "degree")) % 360).toString());
                        } else {
                            this.setAttribute("bearing1_bearing", "");
                        }
                        break;
                    }
                    case 3:
                    {
                        this.setAttribute("bearing1_bearing", SimVar.GetSimVarValue("GPS WP BEARING", "degree"));
                        break;
                    }
                    case 4:
                    {
                        if (SimVar.GetSimVarValue("ADF SIGNAL:1", "number")) {
                            this.setAttribute("bearing1_bearing", ((SimVar.GetSimVarValue("ADF RADIAL:1", "degree") + compass) % 360).toString());
                        } else {
                            this.setAttribute("bearing1_bearing", "");
                        }
                        break;
                    }
                }
                switch (this.logic_brg2Source) {
                    case 1:
                    {
                        const hasNav = SimVar.GetSimVarValue("NAV HAS NAV:1", "boolean");
                        if (hasNav) {
                            this.setAttribute("bearing2_bearing", ((180 + SimVar.GetSimVarValue("NAV RADIAL:1", "degree")) % 360).toString());
                        } else {
                            this.setAttribute("bearing2_bearing", "");
                        }
                        break;
                    }
                    case 2:
                    {
                        const hasNav = SimVar.GetSimVarValue("NAV HAS NAV:2", "boolean");
                        if (hasNav) {
                            this.setAttribute("bearing2_bearing", ((180 + SimVar.GetSimVarValue("NAV RADIAL:2", "degree")) % 360).toString());
                        } else {
                            this.setAttribute("bearing2_bearing", "");
                        }
                        break;
                    }
                    case 3:
                    {
                        this.setAttribute("bearing2_bearing", SimVar.GetSimVarValue("GPS WP BEARING", "degree"));
                        break;
                    }
                    case 4:
                    {
                        if (SimVar.GetSimVarValue("ADF SIGNAL:2", "number")) {
                            this.setAttribute("bearing2_bearing", ((SimVar.GetSimVarValue("ADF RADIAL:2", "degree") + compass) % 360).toString());
                        } else {
                            this.setAttribute("bearing2_bearing", "");
                        }
                        break;
                    }
                }
            } else {
                this.courseGroup.classList.toggle('hide', true);
            }
        }
    }
    attributeChangedCallback(name, oldValue, newValue) {
        switch (name) {
            case "toggle_bearing1":
                this.isBearing1Displayed = !this.isBearing1Displayed;
                if (this.bearingCircle) {
                    if (this.isBearing1Displayed || this.isBearing2Displayed) {
                        this.bearingCircle.setAttribute("visibility", "visible");
                    } else {
                        this.bearingCircle.setAttribute("visibility", "hidden");
                    }
                }

                if (this.bearing1) {
                    if (this.isBearing1Displayed) {
                        this.bearing1.setAttribute("visibility", "visible");
                    } else {
                        this.bearing1.setAttribute("visibility", "hidden");
                    }
                }
                break;
            case "toggle_bearing2":
                this.isBearing2Displayed = !this.isBearing2Displayed;
                if (this.bearingCircle) {
                    if (this.isBearing1Displayed || this.isBearing2Displayed) {
                        this.bearingCircle.setAttribute("visibility", "visible");
                    } else {
                        this.bearingCircle.setAttribute("visibility", "hidden");
                    }
                }
                if (this.bearing2) {
                    if (this.isBearing2Displayed) {
                        this.bearing2.setAttribute("visibility", "visible");
                    } else {
                        this.bearing2.setAttribute("visibility", "hidden");
                    }
                }
                break;
        }
        if (oldValue == newValue) {
            return;
        }
        const factor = (this.displayMode === Jet_NDCompass_Display.ARC) ? 1 : 10;
        switch (name) {
            case "vertical_deviation":
                if (this.glideSlopeCursor) {
                    const y = (parseFloat(newValue) * 25 * factor) + (50 * factor);
                    this.glideSlopeCursor.setAttribute("transform", "translate(" + (95 * factor) + " " + y.toFixed(0) + ")");
                }
                break;
            case "display_vertical_deviation":
                if (newValue == "True") {
                    if (this.glideSlopeCursor) {
                        this.glideSlopeCursor.setAttribute("visibility", "visible");
                    }
                } else {
                    if (this.glideSlopeCursor) {
                        this.glideSlopeCursor.setAttribute("visibility", "hidden");
                    }
                }
                break;
            case "display_course_deviation":
                if (newValue == "True") {
                    this.courseTO.setAttribute("fill", this.courseColor.toString());
                    this.courseDeviation.setAttribute("visibility", "visible");
                    this.courseFROM.setAttribute("fill", this.courseColor.toString());
                    if (this.courseTOLine) {
                        this.courseTOLine.setAttribute("visibility", "visible");
                    }
                    if (this.courseFROMLine) {
                        this.courseFROMLine.setAttribute("visibility", "visible");
                    }
                } else {
                    this.courseTO.setAttribute("fill", "none");
                    this.courseDeviation.setAttribute("visibility", "hidden");
                    this.courseFROM.setAttribute("fill", "none");
                    if (this.courseTOLine) {
                        this.courseTOLine.setAttribute("visibility", "hidden");
                    }
                    if (this.courseFROMLine) {
                        this.courseFROMLine.setAttribute("visibility", "hidden");
                    }
                }
                break;
            case "course":
                if (this.course) {
                    this.course.setAttribute("transform", "rotate(" + (newValue) + " " + (50 * factor) + " " + (50 * factor) + ")");
                }
                break;
            case "course_deviation":
                if (this.courseDeviation) {
                    if (this.displayMode == Jet_NDCompass_Display.ARC) {
                        this.courseDeviation.setAttribute("transform", "translate(" + (Math.min(Math.max(parseFloat(newValue), -1), 1) * 100 * factor) + ")");
                    } else {
                        this.courseDeviation.setAttribute("transform", "translate(" + (Math.min(Math.max(parseFloat(newValue), -1), 1) * 20 * factor) + ")");
                    }
                }
                break;
            case "show_bearing1":
                this.isBearing1Displayed = newValue == "true";
                if (this.bearingCircle) {
                    if (this.isBearing1Displayed || this.isBearing2Displayed) {
                        this.bearingCircle.setAttribute("visibility", "visible");
                    } else {
                        this.bearingCircle.setAttribute("visibility", "hidden");
                    }
                }
                if (this.bearing1) {
                    if (this.isBearing1Displayed) {
                        this.bearing1.setAttribute("visibility", "visible");
                    } else {
                        this.bearing1.setAttribute("visibility", "hidden");
                    }
                }
                break;
            case "show_bearing2":
                this.isBearing2Displayed = newValue == "true";
                if (this.bearingCircle) {
                    if (this.isBearing1Displayed || this.isBearing2Displayed) {
                        this.bearingCircle.setAttribute("visibility", "visible");
                    } else {
                        this.bearingCircle.setAttribute("visibility", "hidden");
                    }
                }
                if (this.bearing2) {
                    if (this.isBearing2Displayed) {
                        this.bearing2.setAttribute("visibility", "visible");
                    } else {
                        this.bearing2.setAttribute("visibility", "hidden");
                    }
                }
                break;
            case "bearing1_bearing":
                if (this.bearing1) {
                    if (newValue != "") {
                        this.bearing1.setAttribute("transform", "rotate(" + newValue + " " + (50 * factor) + " " + (50 * factor) + ")");
                        this.bearing1.setAttribute("visibility", "visible");
                    } else {
                        this.bearing1.setAttribute("visibility", "hidden");
                    }
                }
                break;
            case "bearing2_bearing":
                if (this.bearing2) {
                    if (newValue != "") {
                        this.bearing2.setAttribute("transform", "rotate(" + newValue + " " + (50 * factor) + " " + (50 * factor) + ")");
                        this.bearing2.setAttribute("visibility", "visible");
                    } else {
                        this.bearing2.setAttribute("visibility", "hidden");
                    }
                }
                break;
            case "hud":
                this.isHud = newValue == "true";
                break;
        }
    }
    applyRotation() {
        const course = Number(this.getAttribute('rotation'));
        let trackingBug = Number(this.getAttribute('tracking_bug_rotation'));
        let headingBug = Number(this.getAttribute('heading_bug_rotation'));
        let selectedHeadingBug = Number(this.getAttribute('selected_heading_bug_rotation'));
        let selectedTrackingBug = Number(this.getAttribute('selected_tracking_bug_rotation'));
        let ilsBug = Number(this.getAttribute('ils_bug_rotation'));
        let factor = 500;
        if (this.displayMode === Jet_NDCompass_Display.ARC) {
            factor = 50;
            trackingBug = this.degreeToArc(trackingBug);
            headingBug = this.degreeToArc(headingBug);
            selectedHeadingBug = this.degreeToArc(selectedHeadingBug);
            selectedTrackingBug = this.degreeToArc(selectedTrackingBug);
            ilsBug = this.degreeToArc(ilsBug);
        }
        if (this.rotatingCircle) {
            this.rotatingCircle.setAttribute("transform", this.rotatingCircleTrs + " rotate(" + (-course) + " " + factor + " " + factor + ")");
        }
        if (this.graduations) {
            this.graduations.setAttribute("transform", "rotate(" + (-course) + " " + factor + " " + factor + ")");
        }
        if (this.trackingGroup) {
            this.trackingGroup.setAttribute("transform", "rotate(" + trackingBug + " " + factor + " " + factor + ")");
        }
        if (this.headingGroup) {
            this.headingGroup.setAttribute("transform", "rotate(" + headingBug + " " + factor + " " + factor + ")");
        }
        if (this.selectedHeadingGroup) {
            this.selectedHeadingGroup.setAttribute("transform", "rotate(" + selectedHeadingBug + " " + factor + " " + factor + ")");
        }
        if (this.selectedTrackGroup) {
            this.selectedTrackGroup.setAttribute("transform", "rotate(" + selectedTrackingBug + " " + factor + " " + factor + ")");
        }
        if (this.ilsGroup) {
            this.ilsGroup.setAttribute("transform", "rotate(" + ilsBug + " " + factor + " " + factor + ")");
        }
    }
    degreeToArc(_wantedAngle) {
        const tracking = _wantedAngle;
        const angle = (tracking + 180) % 360;
        return angle;
    }
    getExternalTextZonePath(radius, beginAngle, endAngle, xEnd, reverse = false) {
        const beginX = 50 - (radius * Math.cos(beginAngle));
        const beginY = 50 - (radius * Math.sin(beginAngle));
        const endX = 50 - (radius * Math.cos(endAngle));
        const endY = 50 - (radius * Math.sin(endAngle));
        let path = "M" + beginX + " " + beginY + "L" + xEnd + " " + beginY + "L" + xEnd + " " + endY + "L" + endX + " " + endY;
        path += "A " + radius + " " + radius + " 0 0 " + (reverse ? 0 : 1) + " " + beginX + " " + beginY;
        return path;
    }
    addMapRange(_parent, _x, _y, _color, _size, _withBg, _rangeFactor, _removeInteger) {
        const range = new Jet_NDCompass_Range();
        {
            range.text = document.createElementNS(Avionics.SVG.NS, "text");
            range.text.textContent = "";
            range.text.setAttribute("x", _x.toString());
            range.text.setAttribute("y", _y.toString());
            range.text.setAttribute("fill", _color);
            range.text.setAttribute("font-size", _size.toString());
            range.text.setAttribute("font-family", "Roboto-Light");
            range.text.setAttribute("text-anchor", "middle");
            range.text.setAttribute("alignment-baseline", "central");
            range.factor = _rangeFactor;
            range.removeInteger = _removeInteger;
            this.mapRanges.push(range);
        }
        if (_withBg) {
            const w = 80;
            const h = 40;
            const bg = document.createElementNS(Avionics.SVG.NS, "rect");
            bg.setAttribute("x", (_x - w * 0.5).toString());
            bg.setAttribute("y", (_y - h * 0.5).toString());
            bg.setAttribute("width", w.toString());
            bg.setAttribute("height", h.toString());
            bg.setAttribute("fill", "black");
            _parent.appendChild(bg);
        }
        _parent.appendChild(range.text);
        return range.text;
    }
    onEvent(_event) {
    }
    onExit() {
    }
    showILS(_val) {
        this._showILS = _val;
    }
}
