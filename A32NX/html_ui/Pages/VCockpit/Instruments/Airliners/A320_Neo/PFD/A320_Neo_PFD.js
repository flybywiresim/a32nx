class A320_Neo_PFD extends BaseAirliners {
    constructor() {
        super();
        this.initDuration = 7000;
    }
    get templateID() {
        return "A320_Neo_PFD";
    }
    get IsGlassCockpit() {
        return true;
    }
    connectedCallback() {
        super.connectedCallback();
        this.pageGroups = [
            new NavSystemPageGroup("Main", this, [
                new A320_Neo_PFD_MainPage()
            ]),
        ];
        this.maxUpdateBudget = 12;
    }
    disconnectedCallback() {
        window.console.log("A320 Neo PFD - destroyed");
        super.disconnectedCallback();
    }
    onUpdate(_deltaTime) {
        super.onUpdate(_deltaTime);
    }
}
class A320_Neo_PFD_MainElement extends NavSystemElement {
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
class A320_Neo_PFD_MainPage extends NavSystemPage {
    constructor() {
        super("Main", "Mainframe", new A320_Neo_PFD_MainElement());
        this.showILS = false;
        this.attitude = new A320_Neo_PFD_Attitude();
        this.vSpeed = new A320_Neo_PFD_VSpeed();
        this.airspeed = new A320_Neo_PFD_Airspeed();
        this.altimeter = new A320_Neo_PFD_Altimeter();
        this.compass = new A320_Neo_PFD_Compass();
        this.navStatus = new A320_Neo_PFD_NavStatus();
        this.ils = new A320_Neo_PFD_ILS();
        this.element = new NavSystemElementGroup([
            this.attitude,
            this.vSpeed,
            this.airspeed,
            this.altimeter,
            this.compass,
            this.navStatus,
            this.ils
        ]);
        const url = document.getElementsByTagName("a320-neo-pfd-element")[0].getAttribute("url");
        const dispIndex = parseInt(url.substring(url.length - 1));
        this.updateThrottler = new UpdateThrottler(dispIndex == 1 ? 300 : 1000);
        this.yokePositionThrottler = new UpdateThrottler(dispIndex == 1 ? 33 : 66);
    }
    init() {
        super.init();

        const url = document.getElementsByTagName("a320-neo-pfd-element")[0].getAttribute("url");
        this.disp_index = parseInt(url.substring(url.length - 1));
        this.pot_index = this.disp_index == 1 ? 88 : 90;

        this.getDeltaTime = A32NX_Util.createDeltaTimeCalculator(this._lastTime);
        this.showILS = SimVar.GetSimVarValue(`L:BTN_LS_${this.disp_index}_FILTER_ACTIVE`, "bool");
        this.ils.showILS(this.showILS);
        this.compass.showILS(this.showILS);

        this.groundCursor = this.gps.getChildById("GroundCursor");
        this.groundCursorLimits = this.gps.getChildById("GroundCursorLimits");

        //ADIRS
        this.flashTimer = 1;
        this.flashState = false;

        this.miscFail = document.querySelector("#MiscFail");

        this.attitudeFail = document.querySelector("#AttitudeFail");
        this.attFlash = document.querySelector("#att_flash");

        this.headingFail = document.querySelector("#HeadingFail");
        this.hdgFlash = document.querySelector("#hdg_flash");

        this.spdFlash = document.querySelector("#spd_flash");
        this.altFlash = document.querySelector("#alt_flash");
        this.vsFlash = document.querySelector("#vs_flash");

        this.hasInitialized = true;

        //SELF TEST
        this.selfTestDiv = document.querySelector('#SelfTestDiv');
        this.selfTestTimerStarted = false;
        this.selfTestTimer = -1;
        this.selfTestLastKnobValueFO = 1;
        this.selfTestLastKnobValueCAP = 1;

        //ENGINEERING TEST
        this.engTestDiv = document.querySelector('#PfdEngTest');
        this.engMaintDiv = document.querySelector('#PfdMaintMode');

        this.electricity = document.querySelector('#Electricity');
    }
    onUpdate() {
        let _deltaTime = this.getDeltaTime();
        const currentKnobValue = SimVar.GetSimVarValue("LIGHT POTENTIOMETER:" + this.pot_index, "number");
        if (currentKnobValue <= 0.0) {
            this.selfTestLastKnobValue = currentKnobValue;
            return;
        }
        super.onUpdate(_deltaTime);
        if (!this.hasInitialized) {
            return;
        }

        if (this.yokePositionThrottler.canUpdate(_deltaTime) != -1) {
            const YokeXPosition = 40.45 + (18.4 * SimVar.GetSimVarValue("L:A32NX_SIDESTICK_POSITION_X", "Number"));
            const YokeYPosition = 47.95 - (14.45 * SimVar.GetSimVarValue("L:A32NX_SIDESTICK_POSITION_Y", "Number"));

            this.groundCursor.style.left = YokeXPosition.toString() + "%";
            this.groundCursor.style.top = YokeYPosition.toString() + "%";
        }

        const KnobChanged = (currentKnobValue >= 0.1 && this.selfTestLastKnobValue < 0.1);

        _deltaTime = this.updateThrottler.canUpdate(_deltaTime);
        if (_deltaTime === -1 && !KnobChanged) {
            return;
        }
        this.flashTimer -= _deltaTime / 1000;
        if (this.flashTimer <= 0) {
            this.flashTimer = 1;
            this.flashState = !this.flashState;
        }
        if (this.flashState) {
            this.attFlash.setAttribute("visibility", "visible");
            this.hdgFlash.setAttribute("visibility", "visible");
            this.spdFlash.setAttribute("visibility", "visible");
            this.altFlash.setAttribute("visibility", "visible");
            this.vsFlash.setAttribute("visibility", "visible");
        } else if (this.selfTestTimer > -5) {
            this.attFlash.setAttribute("visibility", "hidden");
            this.hdgFlash.setAttribute("visibility", "hidden");
            this.spdFlash.setAttribute("visibility", "hidden");
            this.altFlash.setAttribute("visibility", "hidden");
            this.vsFlash.setAttribute("visibility", "hidden");
        }

        const IsOnGround = SimVar.GetSimVarValue("SIM ON GROUND", "Bool");
        const isAnyEngineSwitchOn = SimVar.GetSimVarValue("FUELSYSTEM VALVE SWITCH:1", "Bool") || SimVar.GetSimVarValue("FUELSYSTEM VALVE SWITCH:2", "Bool");

        if (IsOnGround && isAnyEngineSwitchOn) {
            this.groundCursor.style.display = "block";
            this.groundCursorLimits.style.display = "block";
        } else {
            this.groundCursor.style.display = "none";
            this.groundCursorLimits.style.display = "none";
        }
        const ADIRSState = SimVar.GetSimVarValue("L:A320_Neo_ADIRS_STATE", "Enum");
        const PFDAlignedFirst = SimVar.GetSimVarValue("L:A32NX_ADIRS_PFD_ALIGNED_FIRST", "Bool");
        const PFDAlignedATT = SimVar.GetSimVarValue("L:A32NX_ADIRS_PFD_ALIGNED_ATT", "Bool");

        if (PFDAlignedFirst) {
            this.miscFail.setAttribute("style", "display:none");
        } else {
            this.miscFail.setAttribute("style", "");
        }

        if (PFDAlignedATT) {
            this.attitudeFail.setAttribute("style", "display:none");
        } else {
            this.attitudeFail.setAttribute("style", "");
        }

        if (ADIRSState == 2) {
            this.headingFail.setAttribute("style", "display:none");
        } else {
            this.headingFail.setAttribute("style", "");
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

        if (this.disp_index == 1) {
            updateDisplayDMC("PFD1", this.engTestDiv, this.engMaintDiv);
        } else {
            updateDisplayDMC("PFD2", this.engTestDiv, this.engMaintDiv);
        }

        this.selfTestLastKnobValue = currentKnobValue;
        this.updateScreenState();
    }
    onEvent(_event) {
        switch (_event) {
            case `BTN_LS_${this.disp_index}`:

                this.showILS = !this.showILS;
                SimVar.SetSimVarValue(`L:BTN_LS_${this.disp_index}_FILTER_ACTIVE`, "number", this.showILS ? 1 : 0);
                this.ils.showILS(this.showILS);
                this.compass.showILS(this.showILS);
                break;
        }
    }

    updateScreenState() {
        if (SimVar.GetSimVarValue("L:ACPowerAvailable","bool")) {
            this.electricity.style.display = "block";
        } else {
            this.electricity.style.display = "none";
        }
    }

}
class A320_Neo_PFD_VSpeed extends NavSystemElement {
    init(root) {
        this.vsi = this.gps.getChildById("VSpeed");
        this.vsi.aircraft = Aircraft.A320_NEO;
        this.vsi.gps = this.gps;
        this.getDeltaTime = A32NX_Util.createDeltaTimeCalculator(this._lastTime);
        const url = document.getElementsByTagName("a320-neo-pfd-element")[0].getAttribute("url");
        this.disp_index = parseInt(url.substring(url.length - 1));
        this.pot_index = this.disp_index == 1 ? 88 : 90;
        this.updateThrottler = new UpdateThrottler(this.disp_index == 1 ? 50 : 100);
    }
    onEnter() {
    }
    onUpdate() {
        if (this.updateThrottler.canUpdate(this.getDeltaTime()) === -1) {
            return;
        }
        const currentKnobValue = SimVar.GetSimVarValue("LIGHT POTENTIOMETER:" + this.pot_index, "number");
        if (currentKnobValue <= 0.0) {
            return;
        }
        const vSpeed = Math.round(Simplane.getVerticalSpeed());
        this.vsi.setAttribute("vspeed", vSpeed.toString());
        if (Simplane.getAutoPilotVerticalSpeedHoldActive()) {
            const selVSpeed = Math.round(Simplane.getAutoPilotVerticalSpeedHoldValue());
            this.vsi.setAttribute("selected_vspeed", selVSpeed.toString());
            this.vsi.setAttribute("selected_vspeed_active", "true");
        } else {
            this.vsi.setAttribute("selected_vspeed_active", "false");
        }
    }
    onExit() {
    }
    onEvent(_event) {
    }
}
class A320_Neo_PFD_Airspeed extends NavSystemElement {
    constructor() {
        super();
    }
    init(root) {
        this.airspeed = this.gps.getChildById("Airspeed");
        this.airspeed.aircraft = Aircraft.A320_NEO;
        this.airspeed.gps = this.gps;
        this.getDeltaTime = A32NX_Util.createDeltaTimeCalculator(this._lastTime);
        const url = document.getElementsByTagName("a320-neo-pfd-element")[0].getAttribute("url");
        this.disp_index = parseInt(url.substring(url.length - 1));
        this.pot_index = this.disp_index == 1 ? 88 : 90;
        this.updateThrottler = new UpdateThrottler(this.disp_index == 1 ? 50 : 100);
    }
    onEnter() {
    }
    onUpdate() {
        const _deltaTime = this.updateThrottler.canUpdate(this.getDeltaTime());
        if (_deltaTime === -1) {
            return;
        }
        const currentKnobValue = SimVar.GetSimVarValue("LIGHT POTENTIOMETER:" + this.pot_index, "number");
        if (currentKnobValue <= 0.0) {
            return;
        }
        this.airspeed.update(_deltaTime);
    }
    onExit() {
    }
    onEvent(_event) {
    }
}
class A320_Neo_PFD_Altimeter extends NavSystemElement {
    constructor() {
        super();
    }
    init(root) {
        this.altimeter = this.gps.getChildById("Altimeter");
        this.altimeter.aircraft = Aircraft.A320_NEO;
        this.altimeter.gps = this.gps;
        this.getDeltaTime = A32NX_Util.createDeltaTimeCalculator(this._lastTime);
        const url = document.getElementsByTagName("a320-neo-pfd-element")[0].getAttribute("url");
        this.disp_index = parseInt(url.substring(url.length - 1));
        this.pot_index = this.disp_index == 1 ? 88 : 90;
        this.updateThrottler = new UpdateThrottler(this.disp_index == 1 ? 25 : 100);
    }
    onEnter() {
    }
    onUpdate() {
        const _deltaTime = this.updateThrottler.canUpdate(this.getDeltaTime());
        if (_deltaTime === -1) {
            return;
        }
        const currentKnobValue = SimVar.GetSimVarValue("LIGHT POTENTIOMETER:" + this.pot_index, "number");
        if (currentKnobValue <= 0.0) {
            return;
        }
        this.altimeter.update(_deltaTime);
    }
    onExit() {
    }
    onEvent(_event) {
        switch (_event) {
            case "BARO_INC":
                SimVar.SetSimVarValue("K:KOHLSMAN_INC", "number", 1);
                break;
            case "BARO_DEC":
                SimVar.SetSimVarValue("K:KOHLSMAN_DEC", "number", 1);
                break;
        }
    }
}
class A320_Neo_PFD_Attitude extends NavSystemElement {
    constructor() {
        super(...arguments);
        this.vDir = new Vec2();
    }
    init(root) {
        this.hsi = this.gps.getChildById("Horizon");
        this.hsi.aircraft = Aircraft.A320_NEO;
        this.hsi.gps = this.gps;
        this.getDeltaTime = A32NX_Util.createDeltaTimeCalculator(this._lastTime);
        const url = document.getElementsByTagName("a320-neo-pfd-element")[0].getAttribute("url");
        this.disp_index = parseInt(url.substring(url.length - 1));
        this.pot_index = this.disp_index == 1 ? 88 : 90;
        this.updateThrottler = new UpdateThrottler(this.disp_index == 1 ? 50 : 100);
    }
    onEnter() {
    }
    onUpdate() {
        const _deltaTime = this.updateThrottler.canUpdate(this.getDeltaTime());
        if (_deltaTime === -1) {
            return;
        }
        const currentKnobValue = SimVar.GetSimVarValue("LIGHT POTENTIOMETER:" + this.pot_index, "number");
        if (currentKnobValue <= 0.0) {
            return;
        }
        if (this.hsi) {
            this.hsi.update(_deltaTime);

            const flightDirectorActive = SimVar.GetSimVarValue(`AUTOPILOT FLIGHT DIRECTOR ACTIVE:1`, "bool");
            const apHeadingModeSelected = Simplane.getAutoPilotHeadingSelected();
            const fcuShowHdg = SimVar.GetSimVarValue("L:A320_FCU_SHOW_SELECTED_HEADING", "number");
            const showSelectedHdg = !flightDirectorActive && (fcuShowHdg || apHeadingModeSelected);

            const selectedHeading = Simplane.getAutoPilotSelectedHeadingLockValue(false);
            const compass = SimVar.GetSimVarValue("PLANE HEADING DEGREES MAGNETIC", "degree");
            const xyz = Simplane.getOrientationAxis();

            if (xyz) {
                this.hsi.setAttribute("horizon", (xyz.pitch / Math.PI * 180).toString());
                this.hsi.setAttribute("pitch", (xyz.pitch / Math.PI * 180).toString());
                this.hsi.setAttribute("bank", (xyz.bank / Math.PI * 180).toString());
            }

            this.hsi.setAttribute("slip_skid", Simplane.getInclinometer().toString());
            this.hsi.setAttribute("radio_altitude", Simplane.getAltitudeAboveGround(true).toString());
            this.hsi.setAttribute("radio_decision_height", this.gps.radioNav.getRadioDecisionHeight().toString());
            this.hsi.setAttribute("compass", compass.toString());
            this.hsi.setAttribute("show_selected_hdg", showSelectedHdg.toString());
            this.hsi.setAttribute("ap_hdg", selectedHeading.toString());
        }
    }
    onExit() {
    }
    onEvent(_event) {
    }
}
class A320_Neo_PFD_Compass extends NavSystemElement {
    init(root) {
        this.hsi = this.gps.getChildById("Compass");
        this.hsi.aircraft = Aircraft.A320_NEO;
        this.hsi.gps = this.gps;
        this.getDeltaTime = A32NX_Util.createDeltaTimeCalculator(this._lastTime);
        const url = document.getElementsByTagName("a320-neo-pfd-element")[0].getAttribute("url");
        this.disp_index = parseInt(url.substring(url.length - 1));
        this.pot_index = this.disp_index == 1 ? 88 : 90;
        this.updateThrottler = new UpdateThrottler(this.disp_index == 1 ? 60 : 200);
    }
    onEnter() {
    }
    onUpdate() {
        const _deltaTime = this.updateThrottler.canUpdate(this.getDeltaTime());
        if (_deltaTime === -1) {
            return;
        }
        const currentKnobValue = SimVar.GetSimVarValue("LIGHT POTENTIOMETER:" + this.pot_index, "number");
        if (currentKnobValue <= 0.0) {
            return;
        }
        this.hsi.update(_deltaTime);
    }
    onExit() {
    }
    onEvent(_event) {
    }
    showILS(_val) {
        if (this.hsi) {
            this.hsi.showILS(_val);
        }
    }
}
class A320_Neo_PFD_NavStatus extends NavSystemElement {
    init(root) {
        this.fma = this.gps.getChildById("FMA");
        this.fma.aircraft = Aircraft.A320_NEO;
        this.fma.gps = this.gps;
        this.isInitialized = true;
        this.getDeltaTime = A32NX_Util.createDeltaTimeCalculator(this._lastTime);
        const url = document.getElementsByTagName("a320-neo-pfd-element")[0].getAttribute("url");
        this.disp_index = parseInt(url.substring(url.length - 1));
        this.pot_index = this.disp_index == 1 ? 88 : 90;
        this.updateThrottler = new UpdateThrottler(this.disp_index == 1 ? 200 : 500);
    }
    onEnter() {
    }
    onUpdate() {
        const _deltaTime = this.updateThrottler.canUpdate(this.getDeltaTime());
        if (_deltaTime === -1) {
            return;
        }
        const currentKnobValue = SimVar.GetSimVarValue("LIGHT POTENTIOMETER:" + this.pot_index, "number");
        if (currentKnobValue <= 0.0) {
            return;
        }
        if (this.fma != null) {
            this.fma.update(_deltaTime);
        }
    }
    onExit() {
    }
    onEvent(_event) {
    }
}
class A320_Neo_PFD_ILS extends NavSystemElement {
    init(root) {
        this.ils = this.gps.getChildById("ILS");
        this.ils.aircraft = Aircraft.A320_NEO;
        this.ils.gps = this.gps;
        this.getDeltaTime = A32NX_Util.createDeltaTimeCalculator(this._lastTime);
        const url = document.getElementsByTagName("a320-neo-pfd-element")[0].getAttribute("url");
        this.disp_index = parseInt(url.substring(url.length - 1));
        this.pot_index = this.disp_index == 1 ? 88 : 90;
        this.updateThrottler = new UpdateThrottler(this.disp_index == 1 ? 150 : 300);
    }
    onEnter() {
    }
    onUpdate() {
        const _deltaTime = this.updateThrottler.canUpdate(this.getDeltaTime());
        if (_deltaTime === -1) {
            return;
        }
        const currentKnobValue = SimVar.GetSimVarValue("LIGHT POTENTIOMETER:" + this.pot_index, "number");
        if (currentKnobValue <= 0.0) {
            return;
        }
        if (this.ils) {
            this.ils.update(_deltaTime);
        }
    }
    onExit() {
    }
    onEvent(_event) {
    }
    showILS(_val) {
        if (this.ils) {
            this.ils.showLocalizer(_val);
            this.ils.showGlideslope(_val);
            this.ils.showNavInfo(_val);
        }
    }
}
registerInstrument("a320-neo-pfd-element", A320_Neo_PFD);
