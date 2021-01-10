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

class A320_Neo_MFD extends BaseAirliners {
    constructor() {
        super();
        this.initDuration = 11000;
        this.showFlightPlan = true;
    }
    get templateID() {
        return "A320_Neo_MFD";
    }
    get IsGlassCockpit() {
        return true;
    }
    connectedCallback() {
        super.connectedCallback();
        this.pageGroups = [
            new NavSystemPageGroup("Main", this, [
                new A320_Neo_MFD_MainPage()
            ]),
        ];
    }
    disconnectedCallback() {
    }
    onEvent(_event) {
        switch (_event) {
            case "CLR_Long":
                this.currentInteractionState = 0;
                this.popUpElement = null;
                this.overridePage = null;
                this.currentEventLinkedPageGroup = null;
                this.currentPageGroupIndex = 0;
                this.getCurrentPageGroup().pageIndex = 0;
                break;
        }
    }
    onUpdate(_deltaTime) {
        super.onUpdate(_deltaTime);
    }
}
class A320_Neo_MFD_MainElement extends NavSystemElement {
    init(root) {
    }
    onEnter() {
    }
    onUpdate(_deltaTime) {
    }
    onExit() {
    }
    onEvent(_event) {
    }
}
class A320_Neo_MFD_MainPage extends NavSystemPage {
    constructor() {
        super("Main", "Mainframe", new A320_Neo_MFD_MainElement());
        this.wxRadarOn = false;
        this.wxRadarMode = -1;
        this.terrainOn = false;
        this.mapMode = -1;
        this.mapRange = -1;
        this.mapConfigId = 0;
        this.modeChangeTimer = -1;
        this.rangeChangeTimer = -1;
        this.headingSelected = false;
        this.showILS = false;
        this.map = new A320_Neo_MFD_Map();
        this.compass = new A320_Neo_MFD_Compass();
        this.info = new A320_Neo_MFD_NDInfo();
        this.element = new NavSystemElementGroup([
            this.map,
            this.compass,
            this.info
        ]);
        this.chronoAcc = 0;
        this.chronoStart = 0;
    }
    init() {
        super.init();
        this.getDeltaTime = A32NX_Util.createDeltaTimeCalculator(this._lastTime);
        this.modeChangeMask = this.gps.getChildById("ModeChangeMask");
        this.rangeChangeMask = this.gps.getChildById("RangeChangeMask");
        this.map.instrument.showRoads = false;
        this.map.instrument.showObstacles = false;
        this.map.instrument.showVORs = false;
        this.map.instrument.showIntersections = false;
        this.map.instrument.showNDBs = false;
        this.map.instrument.showAirports = false;
        this.map.instrument.showAirspaces = false;
        this.map.instrument.intersectionMaxRange = Infinity;
        this.map.instrument.vorMaxRange = Infinity;
        this.map.instrument.ndbMaxRange = Infinity;
        this.map.instrument.smallAirportMaxRange = Infinity;
        this.map.instrument.medAirportMaxRange = Infinity;
        this.map.instrument.largeAirportMaxRange = Infinity;
        SimVar.SetSimVarValue("L:A320_Neo_MFD_NAV_MODE_1", "number", 3);
        SimVar.SetSimVarValue("L:A320_Neo_MFD_NAV_MODE_2", "number", 3);
        SimVar.SetSimVarValue("L:XMLVAR_A320_WeatherRadar_Sys", "number", 1);
        const url = document.getElementsByTagName("a320-neo-mfd-element")[0].getAttribute("url");
        // 1 is captain, 2 is first officer
        this.screenIndex = parseInt(url.substring(url.length - 1));
        this.potIndex = this.screenIndex == 1 ? 89 : 91;
        this.isCaptainsND = this.screenIndex === 1;
        this.isFirstOfficersND = this.screenIndex === 1;
        this.side = this.isCaptainsND ? 'L' : 'R';
        this.showILS = SimVar.GetSimVarValue(`L:BTN_LS_${this.screenIndex}_FILTER_ACTIVE`, "bool");
        this.compass.showILS(this.showILS);
        this.info.showILS(this.showILS);

        //SELF TEST
        this.selfTestDiv = document.querySelector("#SelfTestDiv");
        this.selfTestTimerStarted = false;
        this.selfTestTimer = -1;

        //ENGINEERING TEST
        this.engTestDiv = document.querySelector("#MfdEngTest");
        this.engMaintDiv = document.querySelector("#MfdMaintMode");

        //CHRONO
        SimVar.SetSimVarValue(`L:AUTOPILOT_CHRONO_STATE_${this.side}`, "number", 0);
        this.chronoDiv = document.querySelector("#div_Chrono");
        this.text_chrono_prefix = document.querySelector("#text_chrono_prefix");
        this.text_chrono_HRSymbol = document.querySelector("#text_chrono_HRSymbol");
        this.text_chrono_suffix = document.querySelector("#text_chrono_suffix");
        this.IsChronoDisplayed = 0;

        this.electricity = this.gps.getChildById("Electricity");

        this.mapUpdateThrottler = new UpdateThrottler(this.screenIndex == 1 ? 100 : 400);
        this.updateThrottler = new UpdateThrottler(this.screenIndex == 1 ? 300 : 500);
    }
    displayChronoTime() {
        const totalSeconds = this.getTotalChronoSeconds();
        if (this.chronoStart || totalSeconds > 0) {
            const hours = Math.floor(totalSeconds / 3600).toString().padStart(2, '0');
            const minutes = Math.floor((totalSeconds % 3600) / 60).toString().padStart(2, '0');
            if (hours === "00") {
                const seconds = Math.floor(totalSeconds % 60).toString().padStart(2, '0');
                this.text_chrono_prefix.innerHTML = minutes + "'";
                this.text_chrono_HRSymbol.innerHTML = "";
                this.text_chrono_suffix.innerHTML = seconds + '"';
            } else {
                this.text_chrono_prefix.innerHTML = hours;
                this.text_chrono_HRSymbol.innerHTML = "H";
                this.text_chrono_suffix.innerHTML = minutes + "'";
            }
        }
        return "";
    }
    getTotalChronoSeconds() {
        let totalSeconds = this.chronoAcc;
        if (this.chronoStart) {
            totalSeconds += SimVar.GetGlobalVarValue("ABSOLUTE TIME", "Seconds") - this.chronoStart;
        }
        return Math.max(totalSeconds, 0);
    }

    _hasMapModeChanged() {
        if (this._previousMapMode !== this.mapMode) {
            this._previousMapMode = this.mapMode;
            return true;
        }
        return false;
    }

    onUpdate() {
        const currentKnobValue = SimVar.GetSimVarValue("LIGHT POTENTIOMETER:" + this.potIndex, "number");
        if (currentKnobValue <= 0.0) {
            this.selfTestLastKnobValue = currentKnobValue;
            return;
        }
        let _deltaTime = this.getDeltaTime();
        super.onUpdate(_deltaTime);
        const mapDeltaTime = this.mapUpdateThrottler.canUpdate(_deltaTime);
        if (mapDeltaTime != -1) {
            this.updateMap(mapDeltaTime);
        }
        _deltaTime = this.updateThrottler.canUpdate(_deltaTime);
        const KnobChanged = (currentKnobValue >= 0.1 && this.selfTestLastKnobValue < 0.1);
        if (_deltaTime === -1 && !KnobChanged) {
            return;
        }
        this.updateNDInfo(_deltaTime);

        //TCAS
        this.map.instrument.TCASManager.update(_deltaTime);

        const ADIRSState = SimVar.GetSimVarValue("L:A320_Neo_ADIRS_STATE", "Enum");

        if (ADIRSState != 2) {
            document.querySelector("#GPSPrimary").setAttribute("visibility", "hidden");
            document.querySelector("#GPSPrimaryLost").setAttribute("visibility", "visible");
            document.querySelector("#rect_GPSPrimary").setAttribute("visibility", "visible");

            //Simvar to find out if the GPS messages have been already acknowledged in the FCU by clicking the CLR button.
            var GPSPrimAck = SimVar.GetSimVarValue("L:GPSPrimaryAcknowledged", "bool");
            //Getting the current GPS value to compare whether if the GPS is lost in the middle (After Aligns)
            //OR The aircraft is started from the very first time.
            var GPSPrimaryCurrVal = SimVar.GetSimVarValue("L:GPSPrimary", "bool");
            if (GPSPrimaryCurrVal && GPSPrimAck) { //Resetting the acknowledged flag, if the GPS flag changed in the middle.
                SimVar.SetSimVarValue("L:GPSPrimaryAcknowledged", "bool", 0);
            }
            SimVar.SetSimVarValue("L:GPSPrimary", "bool", 0);
        } else {
            document.querySelector("#GPSPrimaryLost").setAttribute("visibility", "hidden");
            var GPSPrimAck = SimVar.GetSimVarValue("L:GPSPrimaryAcknowledged", "bool");
            var GPSPrimaryCurrVal = SimVar.GetSimVarValue("L:GPSPrimary", "bool");
            if (!GPSPrimaryCurrVal && GPSPrimAck) { //Resetting the acknowledged flag, if the GPS flag changed in the middle.
                SimVar.SetSimVarValue("L:GPSPrimaryAcknowledged", "bool", 0);
            }

            //Clearing the ND message if the message displayed in the FMC is acknowledged by pressing the CLR button.
            if (!GPSPrimAck) {
                document.querySelector("#GPSPrimary").setAttribute("visibility", "visible");
                document.querySelector("#rect_GPSPrimary").setAttribute("visibility", "visible");
            } else {
                document.querySelector("#rect_GPSPrimary").setAttribute("visibility", "hidden");
                document.querySelector("#GPSPrimary").setAttribute("visibility", "hidden");
            }
            SimVar.SetSimVarValue("L:GPSPrimary", "bool", 1);
        }

        if (ADIRSState != 2 && !this.map.planMode && this.modeChangeTimer == -1) {
            document.querySelector("#MapFail").setAttribute("visibility", "visible");
            document.querySelector("#Map").setAttribute("style", "display:none");
        } else {
            document.querySelector("#MapFail").setAttribute("visibility", "hidden");
            document.querySelector("#Map").setAttribute("style", "");
        }

        if (this.map.instrument.airplaneIconElement._image) {
            this.map.instrument.airplaneIconElement._image.setAttribute("visibility", ADIRSState != 2 ? "hidden" : "visible");
        }

        const ACPowerStateChange = SimVar.GetSimVarValue("L:ACPowerStateChange","Bool");
        const ACPowerAvailable = SimVar.GetSimVarValue("L:ACPowerAvailable","Bool");

        if ((KnobChanged || ACPowerStateChange) && ACPowerAvailable && !this.selfTestTimerStarted) {
            this.selfTestDiv.style.display = "block";
            this.selfTestTimer = parseInt(NXDataStore.get("CONFIG_SELF_TEST_TIME", "15"));
            this.selfTestTimerStarted = true;
        }

        if (this.selfTestTimer >= 0) {
            this.selfTestTimer -= _deltaTime / 1000;
            if (this.selfTestTimer <= 0) {
                this.selfTestDiv.style.display = "none";
                this.selfTestTimerStarted = false;
            }
        }

        // ND Chrono Logic
        // Copied the base logic from the Clock functionality.
        // Chrono button on the FCU panel works a bit different to the Chrono is the clock. For e.g.
        // 1st Push starts the timer.
        // 2nd Push Stops the Chrono.
        // 3rd Push Clears the Chrono.
        // Chrono State simvar is used to handle thes above states.
        const AP_ChronoBtnState = SimVar.GetSimVarValue(`L:PUSH_AUTOPILOT_CHRONO_${this.side}`, "bool");
        const AP_IsChronoState = SimVar.GetSimVarValue(`L:AUTOPILOT_CHRONO_STATE_${this.side}`, "number");
        // AUTOPILOT_CHRONO_STATE_X States
        // 0 Off
        // 1 Running
        // 2 Stopped
        if (ACPowerAvailable) {
            // If Chrono button is pushed on
            if (AP_ChronoBtnState) {
                if (AP_IsChronoState === 2) {
                    // Hide & Clear the chrono if it is already stopped.
                    this.chronoDiv.setAttribute("style", "display:none");
                    SimVar.SetSimVarValue(`L:AUTOPILOT_CHRONO_STATE_${this.side}`, "number", 0);
                    SimVar.SetSimVarValue(`L:PUSH_AUTOPILOT_CHRONO_${this.side}`, "bool", 0);
                    this.chronoStart = 0;
                    this.chronoAcc = 0;
                    this.IsChronoDisplayed = 0;
                } else {
                    // Display and start the timer.
                    this.chronoDiv.setAttribute("style", "display:block");
                    this.IsChronoDisplayed = 1;
                    SimVar.SetSimVarValue(`L:AUTOPILOT_CHRONO_STATE_${this.side}`, "number", 1);
                    if (this.chronoStart === 0) {
                        this.chronoStart = SimVar.GetGlobalVarValue("ABSOLUTE TIME", "Seconds");
                    }
                    this.displayChronoTime();
                }
            } else {
                // Chrono button is pushed OFF
                if (AP_IsChronoState !== 0 && this.IsChronoDisplayed) {
                    // Will stop the timer ONLY if it is visible on screen
                    SimVar.SetSimVarValue(`L:AUTOPILOT_CHRONO_STATE_${this.side}`, "number", 2);
                }
            }
        } else {
            if (AP_ChronoBtnState !== 0 || AP_IsChronoState !== 0) {
                SimVar.SetSimVarValue(`L:PUSH_AUTOPILOT_CHRONO_${this.side}`, "bool", 0);
                SimVar.SetSimVarValue(`L:AUTOPILOT_CHRONO_STATE_${this.side}`, "number", 0);
                this.chronoDiv.setAttribute("style", "display:none");
                this.chronoStart = 0;
                this.chronoAcc = 0;
                this.IsChronoDisplayed = 0;
            }
        }

        if (this.screenIndex == 1) {
            updateDisplayDMC("MFD1", this.engTestDiv, this.engMaintDiv);
        } else {
            updateDisplayDMC("MFD2", this.engTestDiv, this.engMaintDiv);
        }

        this.selfTestLastKnobValue = currentKnobValue;
        this.updateScreenState();
    }

    updateScreenState() {
        if (SimVar.GetSimVarValue("L:ACPowerAvailable","bool")) {
            this.electricity.style.display = "block";
        } else {
            this.electricity.style.display = "none";
        }
    }

    _updateNDFiltersStatuses() {
        SimVar.SetSimVarValue("L:BTN_CSTR_" + this.screenIndex + "_FILTER_ACTIVE", "number", this.map.instrument.showConstraints ? 1 : 0);
        SimVar.SetSimVarValue("L:BTN_VORD_" + this.screenIndex + "_FILTER_ACTIVE", "number", this.map.instrument.showVORs ? 1 : 0);
        SimVar.SetSimVarValue("L:BTN_WPT_" + this.screenIndex + "_FILTER_ACTIVE", "number", this.map.instrument.showIntersections ? 1 : 0);
        SimVar.SetSimVarValue("L:BTN_NDB_" + this.screenIndex + "_FILTER_ACTIVE", "number", this.map.instrument.showNDBs ? 1 : 0);
        SimVar.SetSimVarValue("L:BTN_ARPT_" + this.screenIndex + "_FILTER_ACTIVE", "number", this.map.instrument.showAirports ? 1 : 0);
    }

    updateMap(_deltaTime) {
        if (this.modeChangeMask && this.modeChangeTimer >= 0) {
            this.modeChangeTimer -= _deltaTime / 1000;
            if (this.modeChangeTimer <= 0) {
                this.modeChangeMask.style.display = "none";
                this.modeChangeTimer = -1;
            }
        }
        if (this.rangeChangeMask && this.rangeChangeTimer >= 0) {
            this.rangeChangeTimer -= _deltaTime / 1000;
            if (this.rangeChangeTimer <= 0) {
                this.rangeChangeMask.style.display = "none";
                this.rangeChangeTimer = -1;
            }
        }
        const wxRadarOn = (SimVar.GetSimVarValue("L:XMLVAR_A320_WeatherRadar_Sys", "number") != 1) ? true : false;
        const wxRadarMode = SimVar.GetSimVarValue("L:XMLVAR_A320_WeatherRadar_Mode", "number");
        const terrainOn = SimVar.GetSimVarValue(`L:BTN_TERRONND_${this.screenIndex}_ACTIVE`, "number");
        const mapMode = SimVar.GetSimVarValue("L:A320_Neo_MFD_NAV_MODE_" + this.screenIndex, "number");
        const mapRange = SimVar.GetSimVarValue("L:A320_Neo_MFD_Range_" + this.screenIndex, "number");
        const shouldShowWeather = (this.map && !this.map.planMode) && wxRadarOn && wxRadarMode != 3;
        if (this.wxRadarOn != wxRadarOn || this.terrainOn != terrainOn || this.wxRadarMode != wxRadarMode || this.mapMode != mapMode) {
            this.wxRadarOn = wxRadarOn;
            this.wxRadarMode = wxRadarMode;
            this.terrainOn = terrainOn;
            this.mapMode = mapMode;
            this.setMapMode(this.mapMode);
            if (this.map && this.map.planMode) {
                this.mapConfigId = 0;
            } else if (this.terrainOn) {
                this.mapConfigId = 1;
                this.map.hideWeather();
            } else if (shouldShowWeather) {
                this.showWeather();
            } else {
                this.mapConfigId = 0;
            }
            if (this.compass.svg.displayMode === Jet_NDCompass_Display.ARC) {
                this.map.showWxMask();
                if (terrainOn) {
                    // TODO: remove need for compass mask by editing SvgMap.js
                    this.map.showCompassMask();
                } else {
                    this.map.hideCompassMask();
                }
                this.map.hidePlanMask();
            } else {
                this.map.showPlanMask();
                this.map.hideWxMask();
                this.map.hideCompassMask();
            }
            // This animation should play only when MODE CHANGE nob is used
            if (this.modeChangeMask && this._hasMapModeChanged()) {
                this.modeChangeMask.style.display = "block";
                this.modeChangeTimer = 0.5;
            }
        }
        switch (this.mapConfigId) {
            case 0:
                if (this.map.instrument.mapConfigId != 0) {
                    this.map.instrument.mapConfigId = 0;
                    this.map.instrument.bingMapRef = EBingReference.SEA;
                }
                break;
            case 1:
                const altitude = Simplane.getAltitudeAboveGround();
                if (altitude >= 500 && this.map.instrument.mapConfigId != 1) {
                    this.map.instrument.mapConfigId = 1;
                    this.map.instrument.bingMapRef = EBingReference.PLANE;
                } else if (altitude < 490 && this.map.instrument.mapConfigId != 0) {
                    this.map.instrument.mapConfigId = 0;
                    this.map.instrument.bingMapRef = EBingReference.SEA;
                }
                break;
        }

        // This code shows the BingMap only when either weather or terrain are
        // supposed to be shown.
        // When neither weather nor terrain are supposed to be shown,
        // the BingMap will be hidden.
        const isTerrainVisible = this.map.instrument.mapConfigId == 1;
        const isWeatherVisible = !terrainOn && shouldShowWeather;
        if (isTerrainVisible || isWeatherVisible) {
            this.setShowBingMap(true);
        } else {
            this.setShowBingMap(false);
        }

        if (this.mapRange != mapRange) {
            this.mapRange = mapRange;
            this.map.instrument.setZoom(this.mapRange);
            this.compass.svg.mapRange = this.map.zoomRanges[this.mapRange];

            if (this.rangeChangeMask) {
                this.rangeChangeMask.style.display = "block";
                this.rangeChangeTimer = 0.5;
            }
        }
        const selected = Simplane.getAutoPilotHeadingSelected();
        if (selected != this.headingSelected) {
            this.headingSelected = selected;
            if (selected) {
                this.map.instrument.setFlightPlanAsDashed(true);
            } else {
                this.map.instrument.setFlightPlanAsDashed(false);
            }
        }
        if (SimVar.GetSimVarValue("L:A320_NEO_PREVIEW_DIRECT_TO", "number")) {
            if (!this.map.instrument.tmpDirectToElement) {
                this.map.instrument.tmpDirectToElement = new SvgBackOnTrackElement("yellow");
            }
            this.map.instrument.tmpDirectToElement.llaRequested = new LatLongAlt(SimVar.GetSimVarValue("L:A320_NEO_PREVIEW_DIRECT_TO_LAT_0", "number"), SimVar.GetSimVarValue("L:A320_NEO_PREVIEW_DIRECT_TO_LONG_0", "number"));
            this.map.instrument.tmpDirectToElement.targetLla = new LatLongAlt(SimVar.GetSimVarValue("L:A320_NEO_PREVIEW_DIRECT_TO_LAT_1", "number"), SimVar.GetSimVarValue("L:A320_NEO_PREVIEW_DIRECT_TO_LONG_1", "number"));
            this.map.instrument.tmpDirectToElement.planeHeading = SimVar.GetSimVarValue("PLANE HEADING DEGREES TRUE", "degree");
        } else {
            this.map.instrument.tmpDirectToElement = undefined;
        }
        if (this.showFlightPlan) {
            this.map.updateTopOfDescent();
            this.map.updateTopOfClimb();
        }
    }
    // The BingMap is used by the A320 to render terrain and weather,
    // but it also renders airports, which the real A320 does not.
    setShowBingMap(showBingMap) {
        this.map.instrument.attributeChangedCallback("show-bing-map", null, showBingMap ? "true" : "false");
        if (showBingMap) {
            // Setting the visibility property manually, for some reason the setVisible function called by "attributeChangedCallback:show-bing-map" is not working properly when mixBlendMode is enabled.
            this.map.instrument.bingMap.style.visibility = "visible";
        } else {
            // Setting the visibility property manually, for some reason the setVisible function called by "attributeChangedCallback:show-bing-map" is not working properly when mixBlendMode is enabled.
            this.map.instrument.bingMap.style.visibility = "hidden";
        }
    }
    // Show/Hide the route
    setFlightPlanVisibility(flightPlan) {
        if (flightPlan != this.showFlightPlan) {
            this.showFlightPlan = flightPlan;
            this.map.instrument.attributeChangedCallback("show-flightplan", null, flightPlan ? "true" : "false");
            if (!flightPlan) {
                this.map.instrument.showConstraints = false;
                this.map.removeTopOf();
            } else {
                const active = SimVar.GetSimVarValue("L:BTN_CSTR_" + this.screenIndex + "_FILTER_ACTIVE", "number");
                if (active) {
                    this.map.instrument.showConstraints = true;
                }
            }
        }
    }

    onEvent(_event) {
        switch (_event) {
            case "BTN_CSTR_" + this.screenIndex:
                if (this.showFlightPlan) {
                    this.map.instrument.showConstraints = !this.map.instrument.showConstraints;
                    this.map.instrument.showIntersections = false;
                    this.map.instrument.showNDBs = false;
                    this.map.instrument.showAirports = false;
                    this.map.instrument.showVORs = false;
                }
                this._updateNDFiltersStatuses();
                break;
            case "BTN_VORD_" + this.screenIndex:
                this.map.instrument.showConstraints = false;
                this.map.instrument.showIntersections = false;
                this.map.instrument.showNDBs = false;
                this.map.instrument.showAirports = false;
                this.map.instrument.showVORs = !this.map.instrument.showVORs;
                this._updateNDFiltersStatuses();
                break;
            case "BTN_WPT_" + this.screenIndex:
                this.map.instrument.showConstraints = false;
                this.map.instrument.showVORs = false;
                this.map.instrument.showNDBs = false;
                this.map.instrument.showAirports = false;
                this.map.instrument.showIntersections = !this.map.instrument.showIntersections;
                this._updateNDFiltersStatuses();
                break;
            case "BTN_NDB_" + this.screenIndex:
                this.map.instrument.showConstraints = false;
                this.map.instrument.showVORs = false;
                this.map.instrument.showIntersections = false;
                this.map.instrument.showAirports = false;
                this.map.instrument.showNDBs = !this.map.instrument.showNDBs;
                this._updateNDFiltersStatuses();
                break;
            case "BTN_ARPT_" + this.screenIndex:
                this.map.instrument.showConstraints = false;
                this.map.instrument.showVORs = false;
                this.map.instrument.showIntersections = false;
                this.map.instrument.showNDBs = false;
                this.map.instrument.showAirports = !this.map.instrument.showAirports;
                this._updateNDFiltersStatuses();
                break;
            case "BTN_TERRONND_" + this.screenIndex:
                SimVar.SetSimVarValue(`L:BTN_TERRONND_${this.screenIndex}_ACTIVE`, "number", (this.terrainOn) ? 0 : 1);
                break;
            case "BTN_LS_" + this.screenIndex:
                this.showILS = !this.showILS;
                this.compass.showILS(this.showILS);
                this.info.showILS(this.showILS);
                break;
        }
    }
    setMapMode(_mode) {
        switch (_mode) {
            case 0:
                this.compass.svg.setMode(Jet_NDCompass_Display.ROSE, Jet_NDCompass_Navigation.ILS);
                this.map.setMode(Jet_NDCompass_Display.ROSE);
                this.info.setMode(Jet_NDCompass_Navigation.ILS);
                this.setFlightPlanVisibility(false);
                break;
            case 1:
                this.compass.svg.setMode(Jet_NDCompass_Display.ROSE, Jet_NDCompass_Navigation.VOR);
                this.map.setMode(Jet_NDCompass_Display.ROSE);
                this.info.setMode(Jet_NDCompass_Navigation.VOR);
                this.setFlightPlanVisibility(false);
                break;
            case 2:
                this.compass.svg.setMode(Jet_NDCompass_Display.ROSE, Jet_NDCompass_Navigation.NAV);
                this.map.setMode(Jet_NDCompass_Display.ROSE);
                this.info.setMode(Jet_NDCompass_Navigation.NAV);
                this.setFlightPlanVisibility(true);
                break;
            case 3:
                this.compass.svg.setMode(Jet_NDCompass_Display.ARC, Jet_NDCompass_Navigation.NAV);
                this.map.setMode(Jet_NDCompass_Display.ARC);
                this.info.setMode(Jet_NDCompass_Navigation.NAV);
                this.setFlightPlanVisibility(true);
                break;
            case 4:
                this.compass.svg.setMode(Jet_NDCompass_Display.PLAN, Jet_NDCompass_Navigation.NAV);
                this.map.setMode(Jet_NDCompass_Display.PLAN);
                this.info.setMode(Jet_NDCompass_Navigation.NAV);
                this.setFlightPlanVisibility(true);
                break;
        }
    }
    showWeather() {
        this.setMapMode(this.mapMode);
        this.map.showWeather();
    }
    updateNDInfo(_deltaTime) {
        this.info.showSymbol(A320_Neo_ND_Symbol.TERR, this.terrainOn);
        this.info.showSymbol(A320_Neo_ND_Symbol.WXR, this.wxRadarOn && !this.terrainOn && this.wxRadarMode == 0);
        this.info.showSymbol(A320_Neo_ND_Symbol.WXRTURB, this.wxRadarOn && !this.terrainOn && this.wxRadarMode == 1);
        this.info.showSymbol(A320_Neo_ND_Symbol.TURB, this.wxRadarOn && !this.terrainOn && this.wxRadarMode == 2);
        this.info.showSymbol(A320_Neo_ND_Symbol.MAP, this.wxRadarOn && !this.terrainOn && this.wxRadarMode == 3);
    }
}
class A320_Neo_MFD_Compass extends NavSystemElement {
    init(root) {
        this.svg = this.gps.getChildById("Compass");
        this.svg.aircraft = Aircraft.A320_NEO;
        this.svg.gps = this.gps;
        const url = document.getElementsByTagName("a320-neo-mfd-element")[0].getAttribute("url");
        this.screenIndex = parseInt(url.substring(url.length - 1));
        this.potIndex = this.screenIndex == 1 ? 89 : 91;
        this.updateThrottler = new UpdateThrottler(this.screenIndex == 1 ? 20 : 100);
    }
    onEnter() {
    }
    onUpdate(_deltaTime) {
        _deltaTime = this.updateThrottler.canUpdate(_deltaTime);
        if (_deltaTime === -1) {
            return;
        }
        const currentKnobValue = SimVar.GetSimVarValue("LIGHT POTENTIOMETER:" + this.potIndex, "number");
        if (currentKnobValue <= 0.0) {
            return;
        }
        this.svg.update(_deltaTime);
    }
    onExit() {
    }
    onEvent(_event) {
    }
    showILS(_val) {
        if (this.svg) {
            this.svg.showILS(_val);
        }
    }
}
class A320_Neo_MFD_Map extends MapInstrumentElement {
    constructor() {
        super(...arguments);
        this.zoomRanges = [10, 20, 40, 80, 160, 320];
        const url = document.getElementsByTagName("a320-neo-mfd-element")[0].getAttribute("url");
        const screenIndex = parseInt(url.substring(url.length - 1));
        this.potIndex = screenIndex == 1 ? 89 : 91;
        this.updateThrottler = new UpdateThrottler(screenIndex === 1 ? 300 : 600);
        this.mcduWaypointCheckThrottler = new UpdateThrottler(50);
        this.lastMcduCurrentFplnWaypointIndex = -1;
        this.lastHeading = 0;
        this.headingDiffForceUpdateThreshold = screenIndex === 1 ? 0.3 : 0.6;
    }

    onUpdate(_deltaTime) {
        const currentKnobValue = SimVar.GetSimVarValue("LIGHT POTENTIOMETER:" + this.potIndex, "number");
        if (currentKnobValue <= 0.0) {
            return;
        }

        // If scrolling through flight plan from MCDU, force update immediately.
        var forceUpdate = false;
        if (this.mcduWaypointCheckThrottler.canUpdate(_deltaTime) !== -1) {
            const mcduCurrentFplnWaypointIndex = SimVar.GetSimVarValue("L:AIRLINER_MCDU_CURRENT_FPLN_WAYPOINT", "number");
            if (mcduCurrentFplnWaypointIndex !== this.lastMcduCurrentFplnWaypointIndex) {
                this.lastMcduCurrentFplnWaypointIndex = mcduCurrentFplnWaypointIndex;
                forceUpdate = true;
            }
        }

        // When the plane is turning, update map faster.
        const heading = Simplane.getHeadingMagnetic();
        if (Math.abs(heading - this.lastHeading) > this.headingDiffForceUpdateThreshold) {
            forceUpdate = true;
        }
        _deltaTime = this.updateThrottler.canUpdate(_deltaTime);
        if (!forceUpdate && _deltaTime === -1) {
            return;
        }
        this.lastHeading = heading;
        if (forceUpdate) {
            this.updateThrottler.notifyForceUpdated();
        }
        super.onUpdate(_deltaTime);
    }

    updateTopOfDescent() {
        const showTopOfDescent = SimVar.GetSimVarValue("L:AIRLINER_FMS_SHOW_TOP_DSCNT", "number") === 1;
        if (showTopOfDescent) {
            if (!this.topOfDescentIcon) {
                this.topOfDescentIcon = new SvgTopOfXElement("a320-neo-top-of-descent", "ICON_TOP_DSCNT_WHITE");
            }
            this.topOfDescentIcon.lat = SimVar.GetSimVarValue("L:AIRLINER_FMS_LAT_TOP_DSCNT", "number");
            this.topOfDescentIcon.long = SimVar.GetSimVarValue("L:AIRLINER_FMS_LONG_TOP_DSCNT", "number");
            this.topOfDescentIcon.heading = SimVar.GetSimVarValue("L:AIRLINER_FMS_HEADING_TOP_DSCNT", "number");
            const index = this.instrument.topOfCurveElements.indexOf(this.topOfDescentIcon);
            if (index === -1) {
                this.instrument.topOfCurveElements.push(this.topOfDescentIcon);
            }
        } else {
            const index = this.instrument.topOfCurveElements.indexOf(this.topOfDescentIcon);
            if (index != -1) {
                this.instrument.topOfCurveElements.splice(index, 1);
            }
        }
    }
    updateTopOfClimb() {
        const showTopOfClimb = SimVar.GetSimVarValue("L:AIRLINER_FMS_SHOW_TOP_CLIMB", "number") === 1;
        if (showTopOfClimb) {
            if (!this.topOfClimbIcon) {
                this.topOfClimbIcon = new SvgTopOfXElement("a320-neo-top-of-climb", "ICON_LEVEL_OFF_BLUE");
            }
            this.topOfClimbIcon.lat = SimVar.GetSimVarValue("L:AIRLINER_FMS_LAT_TOP_CLIMB", "number");
            this.topOfClimbIcon.long = SimVar.GetSimVarValue("L:AIRLINER_FMS_LONG_TOP_CLIMB", "number");
            this.topOfClimbIcon.heading = SimVar.GetSimVarValue("L:AIRLINER_FMS_HEADING_TOP_CLIMB", "number");
            const index = this.instrument.topOfCurveElements.indexOf(this.topOfClimbIcon);
            if (index === -1) {
                this.instrument.topOfCurveElements.push(this.topOfClimbIcon);
            }
        } else {
            const index = this.instrument.topOfCurveElements.indexOf(this.topOfClimbIcon);
            if (index != -1) {
                this.instrument.topOfCurveElements.splice(index, 1);
            }
        }
    }

    removeTopOf() {
        this.instrument.topOfCurveElements = [];
    }

    onTemplateLoaded() {
        super.onTemplateLoaded();
        this.compassModeMask = new SvgBottomMaskElement("a320-compass-mask", 0, -30);
        this.wxMask = new SvgPlanMaskElement("a320-wx-mask", 0, 0, "M702 600 850 600 850 457 750 457 702 500z");
        this.planModeMask = new SvgPlanMaskElement("a320-plan-mask", 0, 0, "M0,0 v1000 h1000 V0 H0 z M813.559,739.539 H186.442 v-471 h627.117 V739.539 z");
    }
    getAdaptiveRanges(_factor) {
        const ranges = Array.from(this.zoomRanges);
        for (let i = 0; i < ranges.length; i++) {
            ranges[i] *= _factor;
        }
        return ranges;
    }
    setMode(display) {
        switch (display) {
            case Jet_NDCompass_Display.ROSE:
            {
                this.planMode = false;
                this.instrument.zoomRanges = this.getAdaptiveRanges(4.5);
                this.instrument.style.top = "0%";
                this.instrument.rotateWithPlane(true);
                this.instrument.centerOnActiveWaypoint(false);
                this.instrument.setPlaneScale(2.0);
                break;
            }
            case Jet_NDCompass_Display.ARC:
            {
                this.planMode = false;
                this.instrument.zoomRanges = this.getAdaptiveRanges(2.3);
                this.instrument.style.top = "24%";
                this.instrument.rotateWithPlane(true);
                this.instrument.centerOnActiveWaypoint(false);
                this.instrument.setPlaneScale(3.5);
                break;
            }
            case Jet_NDCompass_Display.PLAN:
            {
                this.planMode = true;
                this.instrument.zoomRanges = this.getAdaptiveRanges(4.5);
                this.instrument.style.top = "0%";
                this.instrument.rotateWithPlane(false);
                this.instrument.centerOnActiveWaypoint(true);
                this.instrument.setPlaneScale(2.0);
                break;
            }
            default:
                this.planMode = false;
                this.instrument.style.top = "0%";
                this.instrument.rotateWithPlane(false);
                this.instrument.centerOnActiveWaypoint(false);
                this.instrument.setPlaneScale(1.0);
                break;
        }
    }
    showWeather() {
        this.instrument.showWeatherWithGPS(EWeatherRadar.HORIZONTAL, Math.PI);
        this.instrument.setBingMapStyle("5.5%", "5.5%", "89%", "89%");
    }
    hideWeather() {
        if (this.instrument.getWeather() != EWeatherRadar.OFF) {
            this.instrument.showWeather(EWeatherRadar.OFF);
        }
    }
    showWxMask() {
        if (this.wxMask) {
            if (this.instrument.maskElements.indexOf(this.wxMask) === -1) {
                this.instrument.maskElements.push(this.wxMask);
            }
        }
    }
    hideWxMask() {
        if (this.wxMask) {
            const maskIndex = this.instrument.maskElements.indexOf(this.wxMask);
            if (maskIndex !== -1) {
                this.instrument.maskElements.splice(maskIndex, 1);
            }
        }
    }

    showCompassMask() {
        if (this.compassModeMask) {
            if (this.instrument.maskElements.indexOf(this.compassModeMask) === -1) {
                this.instrument.maskElements.push(this.compassModeMask);
            }
        }
    }
    hideCompassMask() {
        if (this.compassModeMask) {
            const maskIndex = this.instrument.maskElements.indexOf(this.compassModeMask);
            if (maskIndex !== -1) {
                this.instrument.maskElements.splice(maskIndex, 1);
            }
        }
    }
    showPlanMask() {
        if (this.planModeMask) {
            if (this.instrument.maskElements.indexOf(this.planModeMask) === -1) {
                this.instrument.maskElements.push(this.planModeMask);
            }
        }
    }
    hidePlanMask() {
        if (this.planModeMask) {
            const maskIndex = this.instrument.maskElements.indexOf(this.planModeMask);
            if (maskIndex !== -1) {
                this.instrument.maskElements.splice(maskIndex, 1);
            }
        }
    }
}
var A320_Neo_ND_Symbol;
(function (A320_Neo_ND_Symbol) {
    A320_Neo_ND_Symbol[A320_Neo_ND_Symbol["WXR"] = 0] = "WXR";
    A320_Neo_ND_Symbol[A320_Neo_ND_Symbol["WXRTURB"] = 1] = "WXRTURB";
    A320_Neo_ND_Symbol[A320_Neo_ND_Symbol["TURB"] = 2] = "TURB";
    A320_Neo_ND_Symbol[A320_Neo_ND_Symbol["MAP"] = 3] = "MAP";
    A320_Neo_ND_Symbol[A320_Neo_ND_Symbol["TERR"] = 4] = "TERR";
})(A320_Neo_ND_Symbol || (A320_Neo_ND_Symbol = {}));
class A320_Neo_MFD_NDInfo extends NavSystemElement {
    constructor() {
        super(...arguments);
        this.allSymbols = new Array();
    }
    init(root) {
        this.ndInfo = this.gps.getChildById("NDInfo");
        this.ndInfo.aircraft = Aircraft.A320_NEO;
        this.ndInfo.gps = this.gps;
        this.allSymbols.push(this.ndInfo.querySelector("#WXR"));
        this.allSymbols.push(this.ndInfo.querySelector("#WXRTURB"));
        this.allSymbols.push(this.ndInfo.querySelector("#TURB"));
        this.allSymbols.push(this.ndInfo.querySelector("#MAP"));
        this.allSymbols.push(this.ndInfo.querySelector("#TERR"));
        const url = document.getElementsByTagName("a320-neo-mfd-element")[0].getAttribute("url");
        this.screenIndex = parseInt(url.substring(url.length - 1));
        this.potIndex = this.screenIndex == 1 ? 89 : 91;
        this.updateThrottler = new UpdateThrottler(this.screenIndex == 1 ? 200 : 400);
    }
    onEnter() {
    }
    onUpdate(_deltaTime) {
        _deltaTime = this.updateThrottler.canUpdate(_deltaTime);
        if (_deltaTime === -1) {
            return;
        }
        const currentKnobValue = SimVar.GetSimVarValue("LIGHT POTENTIOMETER:" + this.potIndex, "number");
        if (currentKnobValue <= 0.0) {
            return;
        }
        if (this.ndInfo != null) {
            this.ndInfo.update(_deltaTime);
        }
        const ADIRSState = SimVar.GetSimVarValue("L:A320_Neo_ADIRS_STATE", "Enum");
        const groundSpeed = Math.round(Simplane.getGroundSpeed());
        const gs = this.ndInfo.querySelector("#GS_Value");
        const tas = this.ndInfo.querySelector("#TAS_Value");
        const wd = this.ndInfo.querySelector("#Wind_Direction");
        const ws = this.ndInfo.querySelector("#Wind_Strength");
        const wa = this.ndInfo.querySelector("#Wind_Arrow");
        const wptg = this.ndInfo.querySelector("#Waypoint_Group");
        if (ADIRSState != 2 || groundSpeed <= 100) {
            //Hide TAS, and wind info
            tas.textContent = "---";
            wd.textContent = "---";
            ws.textContent = "---";
            wa.setAttribute("visibility", "hidden");
        } else {
            wa.setAttribute("visibility", "visible");
        }
        if (ADIRSState != 2) {
            gs.textContent = "---";
        } else {
            gs.textContent = groundSpeed.toString().padStart(3);
        }
        //Hide waypoint when ADIRS not aligned
        wptg.setAttribute("visibility", (ADIRSState != 2) ? "hidden" : "visible");
    }
    onExit() {
    }
    onEvent(_event) {
    }
    setMode(display) {
        if (this.ndInfo) {
            this.ndInfo.setMode(display, 0);
        }
    }
    showILS(_val) {
        if (this.ndInfo) {
            this.ndInfo.showILS(_val);
        }
    }
    showSymbol(_symbol, _show) {
        if (this.allSymbols[_symbol]) {
            this.allSymbols[_symbol].setAttribute("visibility", (_show) ? "visible" : "hidden");
        }
    }
}
registerInstrument("a320-neo-mfd-element", A320_Neo_MFD);
