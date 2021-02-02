class A320_Neo_FCU extends BaseAirliners {
    constructor() {
        super();
        this.initDuration = 3000;
    }
    get templateID() {
        return "A320_Neo_FCU";
    }
    connectedCallback() {
        super.connectedCallback();
        RegisterViewListener("JS_LISTENER_KEYEVENT", this.onListenerRegistered.bind(this));
        this.maxUpdateBudget = 12;
    }
    disconnectedCallback() {
        super.disconnectedCallback();
    }
    onListenerRegistered() {
        this.mainPage = new A320_Neo_FCU_MainPage();
        this.pageGroups = [
            new NavSystemPageGroup("Main", this, [
                this.mainPage
            ]),
        ];
    }
    reboot() {
        super.reboot();
        this.mainPage.reboot();
    }
    onUpdate(_deltaTime) {
        super.onUpdate(_deltaTime);
        this.updateMachTransition();
    }
    onEvent(_event) {
    }
    onFlightStart() {
        super.onFlightStart();
        if (this.mainPage) {
            this.mainPage.onFlightStart();
        }
    }
}

class A320_Neo_FCU_MainElement extends NavSystemElement {
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

class A320_Neo_FCU_MainPage extends NavSystemPage {
    constructor() {
        super("Main", "Mainframe", new A320_Neo_FCU_MainElement());
        this.largeScreen = new A320_Neo_FCU_LargeScreen();
        this.smallScreen = new A320_Neo_FCU_SmallScreen();
        this.element = new NavSystemElementGroup([
            this.largeScreen,
            this.smallScreen
        ]);
    }
    init() {
        super.init();
    }
    onEvent(_event) {
        this.largeScreen.onEvent(_event);
    }
    reboot() {
        this.largeScreen.reboot();
        this.smallScreen.reboot();
    }
    onFlightStart() {
        this.largeScreen.onFlightStart();
        this.smallScreen.onFlightStart();
    }
}

class A320_Neo_FCU_Component {
    getDivElement(_name) {
        if (this.divRef != null) {
            return this.divRef.querySelector("#" + _name);
        }
    }
    set textValueContent(_textContent) {
        if (this.textValue != null) {
            this.textValue.textContent = _textContent;
            this.textValue.innerHTML = this.textValue.innerHTML.replace("{sp}", "&nbsp;");
        }
    }
    getElement(_type, _name) {
        if (this.divRef != null) {
            const allText = this.divRef.getElementsByTagName(_type);
            if (allText != null) {
                for (let i = 0; i < allText.length; ++i) {
                    if (allText[i].id == _name) {
                        return allText[i];
                    }
                }
            }
        }
        return null;
    }
    getTextElement(_name) {
        return this.getElement("text", _name);
    }
    setTextElementActive(_text, _active) {
        if (_text != null) {
            _text.setAttribute("class", "Common " + (_active ? "Active" : "Inactive"));
        }
    }
    setElementVisibility(_element, _show) {
        if (_element != null) {
            _element.style.display = _show ? "block" : "none";
        }
    }
    constructor(_gps, _divName) {
        this.gps = _gps;
        this.divRef = _gps.getChildById(_divName);
        this.textValue = this.getTextElement("Value");
        this.init();
        this.update(0);
    }
    reboot() {
        this.init();
    }
    onFlightStart() {
    }
}

class A320_Neo_FCU_Speed extends A320_Neo_FCU_Component {
    constructor() {
        super(...arguments);
        this.isActive = false;
        this.isManaged = false;
        this.showSelectedSpeed = false;
        this.currentValue = 0;
    }
    init() {
        this.textSPD = this.getTextElement("SPD");
        this.textMACH = this.getTextElement("MACH");
        this.illuminator = this.getElement("circle", "Illuminator");
        this.refresh(false, false, false, false, 0, 0, true);
    }
    update(_deltaTime) {
        const showSelectedSpeed = SimVar.GetSimVarValue("L:A320_FCU_SHOW_SELECTED_SPEED", "number") === 1;
        const isManaged = Simplane.getAutoPilotAirspeedManaged();
        const isMachActive = Simplane.getAutoPilotMachModeActive();
        this.refresh(true, isManaged, showSelectedSpeed, isMachActive, (isMachActive) ? Simplane.getAutoPilotSelectedMachHoldValue() * 100 : Simplane.getAutoPilotSelectedAirspeedHoldValue(), SimVar.GetSimVarValue("L:XMLVAR_LTS_Test", "Bool"));
    }
    refresh(_isActive, _isManaged, _showSelectedSpeed, _machActive, _value, _lightsTest, _force = false) {
        if ((_isActive != this.isActive) || (_isManaged != this.isManaged) || (_showSelectedSpeed != this.showSelectedSpeed) || (_value != this.currentValue) || (_lightsTest !== this.lightsTest) || _force) {
            this.isActive = _isActive;
            this.isManaged = _isManaged;
            this.showSelectedSpeed = _showSelectedSpeed;
            this.currentValue = _value;
            this.setTextElementActive(this.textSPD, !_machActive);
            this.setTextElementActive(this.textMACH, _machActive);
            this.lightsTest = _lightsTest;
            if (this.lightsTest) {
                this.setElementVisibility(this.illuminator, true);
                this.textValueContent = ".8.8.8";
                this.setTextElementActive(this.textSPD, true);
                this.setTextElementActive(this.textMACH, true);
                return;
            }
            let value = Math.round(Math.max(this.currentValue, 0)).toString().padStart(3, "0");
            if (!this.isManaged) {
                if (_machActive) {
                    value = `${value.substring(0,1)}.${value.substring(1)}`;
                }
                this.textValueContent = value;
                this.setElementVisibility(this.illuminator, false);
            } else if (this.isManaged) {
                if (this.showSelectedSpeed) {
                    this.textValueContent = value;
                } else {
                    this.textValueContent = "---";
                }
            }
            this.setElementVisibility(this.illuminator, this.isManaged);
        }
    }
}

class A320_Neo_FCU_Autopilot extends A320_Neo_FCU_Component {
    constructor() {
        super(...arguments);
    }

    init() {
    }

    onEvent(_event) {
        if (_event === "AP_1_PUSH") {
            SimVar.SetSimVarValue("K:A32NX.FCU_AP_1_PUSH", "number", 0);
        } else if (_event === "AP_2_PUSH") {
            SimVar.SetSimVarValue("K:A32NX.FCU_AP_2_PUSH", "number", 0);
        } else if (_event === "LOC_PUSH") {
            SimVar.SetSimVarValue("K:A32NX.FCU_LOC_PUSH", "number", 0);
        } else if (_event === "APPR_PUSH") {
            SimVar.SetSimVarValue("K:A32NX.FCU_APPR_PUSH", "number", 0);
        }
    }

    update(_deltaTime) {
    }
}

class A320_Neo_FCU_Heading extends A320_Neo_FCU_Component {
    constructor() {
        super(...arguments);
        this.backToIdleTimeout = 45000;
        this.inSelection = false;

        this._rotaryEncoderCurrentSpeed = 1;
        this._rotaryEncoderMaximumSpeed = 5;
        this._rotaryEncoderTimeout = 350;
        this._rotaryEncoderIncrement = 0.1;
        this._rotaryEncoderPreviousTimestamp = 0;
    }

    init() {
        this.textHDG = this.getTextElement("HDG");
        this.textTRK = this.getTextElement("TRK");
        this.textLAT = this.getTextElement("LAT");
        this.illuminator = this.getElement("circle", "Illuminator");
        this.refresh(false, false, true, false, 0, 0, true);
        this.selectedValue = -1;
        this.isSelectedValueActive = false;
        this.isPreselectionModeActive = false;
    }

    onFlightStart() {
        super.onFlightStart();
    }

    onRotate() {
        const lateralMode = SimVar.GetSimVarValue("L:A32NX_FMA_LATERAL_MODE", "Number");
        const lateralArmed = SimVar.GetSimVarValue("L:A32NX_FMA_LATERAL_ARMED", "Number");
        const isTRKMode = SimVar.GetSimVarValue("L:A32NX_TRK_FPA_MODE_ACTIVE", "Bool");
        const radioHeight = SimVar.GetSimVarValue("RADIO HEIGHT", "feet");

        if (!this.inSelection
            && (this.isManagedModeActive(lateralMode)
                || this.isPreselectionAvailable(radioHeight, lateralMode))) {
            this.inSelection = true;
            if (!this.isSelectedValueActive) {
                if (isTRKMode) {
                    this.selectedValue = this.getCurrentTrack();
                } else {
                    this.selectedValue = this.getCurrentHeading();
                }
            }
        }

        this.isSelectedValueActive = true;

        if (this.inSelection && !this.isPreselectionAvailable(radioHeight, lateralMode)) {
            this.isPreselectionModeActive = false;
            clearTimeout(this._resetSelectionTimeout);
            this._resetSelectionTimeout = setTimeout(() => {
                this.selectedValue = -1;
                this.isSelectedValueActive = false;
                this.inSelection = false;
            }, this.backToIdleTimeout);
        } else {
            this.isPreselectionModeActive = true;
        }
    }

    getCurrentHeading() {
        return ((Math.round(SimVar.GetSimVarValue("PLANE HEADING DEGREES MAGNETIC", "degree")) % 360) + 360) % 360;
    }

    getCurrentTrack() {
        return ((Math.round(SimVar.GetSimVarValue("GPS GROUND MAGNETIC TRACK", "degree")) % 360) + 360) % 360;
    }

    onPush() {
        clearTimeout(this._resetSelectionTimeout);
        this.isPreselectionModeActive = false;
        this.inSelection = false;
        SimVar.SetSimVarValue("K:A32NX.FCU_HDG_PUSH", "number", 0);
    }

    onPull() {
        clearTimeout(this._resetSelectionTimeout);
        const isTRKMode = SimVar.GetSimVarValue("L:A32NX_TRK_FPA_MODE_ACTIVE", "Bool");
        if (!this.isSelectedValueActive) {
            if (isTRKMode) {
                this.selectedValue = this.getCurrentTrack();
            } else {
                this.selectedValue = this.getCurrentHeading();
            }
        }
        this.inSelection = false;
        this.isSelectedValueActive = true;
        this.isPreselectionModeActive = false;
        SimVar.SetSimVarValue("K:A32NX.FCU_HDG_PULL", "number", 0);
    }

    update(_deltaTime) {
        const lateralMode = SimVar.GetSimVarValue("L:A32NX_FMA_LATERAL_MODE", "Number");
        const lateralArmed = SimVar.GetSimVarValue("L:A32NX_FMA_LATERAL_ARMED", "Number");
        const isTRKMode = SimVar.GetSimVarValue("L:A32NX_TRK_FPA_MODE_ACTIVE", "Bool");
        const lightsTest = SimVar.GetSimVarValue("L:XMLVAR_LTS_Test", "Bool");
        const isManagedActive = this.isManagedModeActive(lateralMode);
        const isManagedArmed = this.isManagedModeArmed(lateralArmed);
        const showSelectedValue = (this.isSelectedValueActive || this.inSelection || this.isPreselectionModeActive);

        this.refresh(true, isManagedArmed, isManagedActive, isTRKMode, showSelectedValue, this.selectedValue, lightsTest);
    }

    refresh(_isActive, _isManagedArmed, _isManagedActive, _isTRKMode, _showSelectedHeading, _value, _lightsTest, _force = false) {
        if ((_isActive != this.isActive)
            || (_isManagedArmed != this.isManagedArmed)
            || (_isManagedActive != this.isManagedActive)
            || (_isTRKMode != this.isTRKMode)
            || (_showSelectedHeading != this.showSelectedHeading)
            || (_value != this.currentValue)
            || (_lightsTest !== this.lightsTest)
            || _force) {
            if (_isTRKMode != this.isTRKMode) {
                this.onTRKModeChanged(_isTRKMode);
            }
            if (_isManagedArmed
                && _isManagedArmed !== this.isManagedArmed
                && SimVar.GetSimVarValue("RADIO HEIGHT", "feet") < 30) {
                _value = -1;
                _showSelectedHeading = false;
                this.selectedValue = _value;
                this.isSelectedValueActive = false;
                this.isPreselectionModeActive = false;
            }
            if (_isManagedActive !== this.isManagedActive) {
                if (_isManagedActive) {
                    _value = -1;
                    _showSelectedHeading = false;
                    this.selectedValue = _value;
                    this.isSelectedValueActive = false;
                    this.isPreselectionModeActive = false;
                } else {
                    _showSelectedHeading = true;
                    if (!this.isSelectedValueActive) {
                        this.isSelectedValueActive = true;
                        if (_isTRKMode) {
                            _value = this.getCurrentTrack();
                            this.selectedValue = _value;
                        } else {
                            _value = this.getCurrentHeading();
                            this.selectedValue = _value;
                        }
                    }
                }
            }
            if (_showSelectedHeading !== this.showSelectedHeading) {
                SimVar.SetSimVarValue("L:A320_FCU_SHOW_SELECTED_HEADING", "number", _showSelectedHeading == true ? 1 : 0);
            }
            if (_value !== this.currentValue) {
                SimVar.SetSimVarValue("L:A32NX_AUTOPILOT_HEADING_SELECTED", "Degrees", _value);
                Coherent.call("HEADING_BUG_SET", 1, Math.max(0, _value));
            }
            this.isActive = _isActive;
            this.isManagedActive = _isManagedActive;
            this.isManagedArmed = _isManagedArmed;
            this.isTRKMode = _isTRKMode;
            this.showSelectedHeading = _showSelectedHeading;
            this.currentValue = _value;
            this.setTextElementActive(this.textHDG, !this.isTRKMode);
            this.setTextElementActive(this.textTRK, this.isTRKMode);
            this.lightsTest = _lightsTest;
            if (this.lightsTest) {
                this.setTextElementActive(this.textHDG, true);
                this.setTextElementActive(this.textTRK, true);
                this.setTextElementActive(this.textLAT, true);
                this.textValueContent = ".8.8.8";
                this.setElementVisibility(this.illuminator, true);
                return;
            }
            if ((this.isManagedArmed || this.isManagedActive) && !this.showSelectedHeading) {
                this.textValueContent = "---";
            } else {
                var value = Math.round(Math.max(this.currentValue, 0)) % 360;
                this.textValueContent = value.toString().padStart(3, "0");
            }
            this.setElementVisibility(this.illuminator, this.isManagedArmed || this.isManagedActive);
        }
    }

    isManagedModeActive(_mode) {
        return (_mode !== 0 && _mode !== 10 && _mode !== 11 && _mode !== 40 && _mode !== 41);
    }

    isManagedModeArmed(_armed) {
        return (_armed > 0);
    }

    isPreselectionAvailable(_radioHeight, _mode) {
        return (
            _radioHeight < 30
            || ((_mode >= 30 && _mode <= 34) || _mode === 50)
        );
    }

    onTRKModeChanged(_newValue) {
        if (_newValue) {
            this.selectedValue = this.calculateTrackForHeading(this.selectedValue);
        } else {
            this.selectedValue = this.calculateHeadingForTrack(this.selectedValue);
        }
    }

    /**
     * Calculates the corresponding track for a given heading, assuming it is flown in the current conditions (TAS + wind).
     * @param {number} _heading The heading in degrees.
     * @returns {number} The corresponding track in degrees.
     */
    calculateTrackForHeading(_heading) {
        const trueAirspeed = SimVar.GetSimVarValue("AIRSPEED TRUE", "Knots");
        if (trueAirspeed < 50) {
            return _heading;
        }

        const heading = _heading * Math.PI / 180;
        const windVelocity = SimVar.GetSimVarValue("AMBIENT WIND VELOCITY", "Knots");
        const windDirection = SimVar.GetSimVarValue("AMBIENT WIND DIRECTION", "Degrees") * Math.PI / 180;
        // https://web.archive.org/web/20160302090326/http://williams.best.vwh.net/avform.htm#Wind
        const wca = Math.atan2(windVelocity * Math.sin(heading - windDirection), trueAirspeed - windVelocity * Math.cos(heading - windDirection));
        const track = heading + wca % (2 * Math.PI);
        return (((track * 180 / Math.PI) % 360) + 360) % 360;
    }

    /**
     * Calculates the heading needed to fly a given track in the current conditions (TAS + wind).
     * @param {number} _track The track in degrees.
     * @returns {number} The corresponding heading in degrees.
     */
    calculateHeadingForTrack(_track) {
        const trueAirspeed = SimVar.GetSimVarValue("AIRSPEED TRUE", "Knots");
        if (trueAirspeed < 50) {
            return _track;
        }

        const track = _track * Math.PI / 180;
        const windVelocity = SimVar.GetSimVarValue("AMBIENT WIND VELOCITY", "Knots");
        const windDirection = SimVar.GetSimVarValue("AMBIENT WIND DIRECTION", "Degrees") * Math.PI / 180;
        // https://web.archive.org/web/20160302090326/http://williams.best.vwh.net/avform.htm#Wind
        const swc = (windVelocity / trueAirspeed) * Math.sin(windDirection - track);
        const heading = track + Math.asin(swc) % (2 * Math.PI);
        const _heading = (((heading * 180 / Math.PI) % 360) + 360) % 360;
        return _heading == NaN ? _track : _heading;
    }

    getRotationSpeed() {
        if (this._rotaryEncoderCurrentSpeed < 1
            || (Date.now() - this._rotaryEncoderPreviousTimestamp) > this._rotaryEncoderTimeout) {
            this._rotaryEncoderCurrentSpeed = 1;
        } else {
            this._rotaryEncoderCurrentSpeed += this._rotaryEncoderIncrement;
        }
        this._rotaryEncoderPreviousTimestamp = Date.now();
        return Math.min(this._rotaryEncoderMaximumSpeed, Math.floor(this._rotaryEncoderCurrentSpeed));
    }

    onEvent(_event) {
        if (_event === "HDG_INC_HEADING") {
            this.selectedValue = (((this.selectedValue + this.getRotationSpeed()) % 360) + 360) % 360;
            this.onRotate();
        } else if (_event === "HDG_DEC_HEADING") {
            this.selectedValue = (((this.selectedValue - this.getRotationSpeed()) % 360) + 360) % 360;
            this.onRotate();
        } else if (_event === "HDG_INC_TRACK") {
            this.selectedValue = (((this.selectedValue + this.getRotationSpeed()) % 360) + 360) % 360;
            this.onRotate();
        } else if (_event === "HDG_DEC_TRACK") {
            this.selectedValue = (((this.selectedValue - this.getRotationSpeed()) % 360) + 360) % 360;
            this.onRotate();
        } else if (_event === "HDG_PUSH") {
            this.onPush();
        } else if (_event === "HDG_PULL") {
            this.onPull();
        }
    }
}

class A320_Neo_FCU_Mode extends A320_Neo_FCU_Component {
    init() {
        this.textHDG = this.getTextElement("HDG");
        this.textVS = this.getTextElement("VS");
        this.textTRK = this.getTextElement("TRK");
        this.textFPA = this.getTextElement("FPA");
        this.refresh(false, 0, true);
    }
    update(_deltaTime) {
        if (SimVar.GetSimVarValue("L:A32NX_FCU_MODE_REVERSION_TRK_FPA_ACTIVE", "Bool")) {
            SimVar.SetSimVarValue("L:A32NX_TRK_FPA_MODE_ACTIVE", "Bool", 0);
        }
        const _isTRKFPADisplayMode = SimVar.GetSimVarValue("L:A32NX_TRK_FPA_MODE_ACTIVE", "Bool");
        this.refresh(_isTRKFPADisplayMode, SimVar.GetSimVarValue("L:XMLVAR_LTS_Test", "Bool"));
    }
    refresh(_isTRKFPADisplayMode, _lightsTest, _force = false) {
        if ((_isTRKFPADisplayMode != this.isTRKFPADisplayMode) || (_lightsTest !== this.lightsTest) || _force) {
            this.isTRKFPADisplayMode = _isTRKFPADisplayMode;
            this.lightsTest = _lightsTest;
            if (this.lightsTest) {
                this.setTextElementActive(this.textHDG, true);
                this.setTextElementActive(this.textVS, true);
                this.setTextElementActive(this.textTRK, true);
                this.setTextElementActive(this.textFPA, true);
                return;
            }
            this.setTextElementActive(this.textHDG, !this.isTRKFPADisplayMode);
            this.setTextElementActive(this.textVS, !this.isTRKFPADisplayMode);
            this.setTextElementActive(this.textTRK, this.isTRKFPADisplayMode);
            this.setTextElementActive(this.textFPA, this.isTRKFPADisplayMode);
        }
    }
}

class A320_Neo_FCU_Altitude extends A320_Neo_FCU_Component {
    init() {
        this.illuminator = this.getElement("circle", "Illuminator");
        this.isActive = false;
        this.isManaged = false;
        this.currentValue = 0;
        let initValue = Simplane.getAltitude();
        if (initValue <= 5000) {
            initValue = 5000;
        } else {
            initValue = Math.round(initValue / 100) * 100;
        }
        Coherent.call("AP_ALT_VAR_SET_ENGLISH", 1, initValue, true);
        this.refresh(false, false, initValue, 0, true);
    }
    reboot() {
        this.init();
    }
    isManagedModeActiveOrArmed(_mode, _armed) {
        return (
            (_mode >= 20 && _mode <= 34)
            || (_armed >> 1 & 1
                || _armed >> 2 & 1
                || _armed >> 3 & 1
                || _armed >> 4 & 1
            )
        );
    }
    update(_deltaTime) {
        const verticalMode = SimVar.GetSimVarValue("L:A32NX_FMA_VERTICAL_MODE", "Number");
        const verticalArmed = SimVar.GetSimVarValue("L:A32NX_FMA_VERTICAL_ARMED", "Number");
        const isManaged = this.isManagedModeActiveOrArmed(verticalMode, verticalArmed);

        this.refresh(Simplane.getAutoPilotActive(), isManaged, Simplane.getAutoPilotDisplayedAltitudeLockValue(Simplane.getAutoPilotAltitudeLockUnits()), SimVar.GetSimVarValue("L:XMLVAR_LTS_Test", "Bool"));
    }
    refresh(_isActive, _isManaged, _value, _lightsTest, _force = false) {
        if ((_isActive != this.isActive) || (_isManaged != this.isManaged) || (_value != this.currentValue) || (_lightsTest !== this.lightsTest) || _force) {
            this.isActive = _isActive;
            this.isManaged = _isManaged;
            this.currentValue = _value;
            this.lightsTest = _lightsTest;
            if (this.lightsTest) {
                this.textValueContent = "88888";
                this.setElementVisibility(this.illuminator, true);
                return;
            }
            const value = Math.floor(Math.max(this.currentValue, 100));
            this.textValueContent = value.toString().padStart(5, "0");
            this.setElementVisibility(this.illuminator, this.isManaged);
        }
    }
    onEvent(_event) {
        if (_event === "ALT_PUSH") {
            SimVar.SetSimVarValue("K:A32NX.FCU_ALT_PUSH", "number", 0);
        } else if (_event === "ALT_PULL") {
            SimVar.SetSimVarValue("K:A32NX.FCU_ALT_PULL", "number", 0);
        }
    }
}

var A320_Neo_FCU_VSpeed_State;
(function (A320_Neo_FCU_VSpeed_State) {
    A320_Neo_FCU_VSpeed_State[A320_Neo_FCU_VSpeed_State["Idle"] = 0] = "Idle";
    A320_Neo_FCU_VSpeed_State[A320_Neo_FCU_VSpeed_State["Zeroing"] = 1] = "Zeroing";
    A320_Neo_FCU_VSpeed_State[A320_Neo_FCU_VSpeed_State["Selecting"] = 2] = "Selecting";
    A320_Neo_FCU_VSpeed_State[A320_Neo_FCU_VSpeed_State["Flying"] = 3] = "Flying";
})(A320_Neo_FCU_VSpeed_State || (A320_Neo_FCU_VSpeed_State = {}));
class A320_Neo_FCU_VerticalSpeed extends A320_Neo_FCU_Component {
    constructor() {
        super(...arguments);
        this.forceUpdate = true;
        this.ABS_MINMAX_FPA = 9.9;
        this.ABS_MINMAX_VS = 6000;
        this.backToIdleTimeout = 45000;
        this.previousVerticalMode = 0;
    }
    get currentState() {
        return this._currentState;
    }
    set currentState(v) {
        this._currentState = v;
        SimVar.SetSimVarValue("L:A320_NE0_FCU_STATE", "number", this.currentState);
    }

    init() {
        this.textVS = this.getTextElement("VS");
        this.textFPA = this.getTextElement("FPA");
        this.isActive = false;
        this.isFPAMode = false;
        this._enterIdleState();
        this.selectedVs = 0;
        this.selectedFpa = 0;
        this.refresh(false, false, 0, 0, true);
    }

    onFlightStart() {
        super.onFlightStart();
    }

    onPush() {
        clearTimeout(this._resetSelectionTimeout);
        this.forceUpdate = true;

        this.currentState = A320_Neo_FCU_VSpeed_State.Zeroing;

        this.selectedVs = 0;
        this.selectedFpa = 0;

        SimVar.SetSimVarValue("K:A32NX.FCU_VS_PUSH", "number", 0);
    }

    onRotate() {
        if (this.currentState === A320_Neo_FCU_VSpeed_State.Idle || this.currentState === A320_Neo_FCU_VSpeed_State.Selecting) {
            clearTimeout(this._resetSelectionTimeout);
            this.forceUpdate = true;

            if (this.currentState === A320_Neo_FCU_VSpeed_State.Idle) {
                this.selectedVs = this.getCurrentVerticalSpeed();
                this.selectedFpa = this.getCurrentFlightPathAngle();
            }

            this.currentState = A320_Neo_FCU_VSpeed_State.Selecting;

            this._resetSelectionTimeout = setTimeout(() => {
                this.selectedVs = 0;
                this.selectedFpa = 0;
                this.currentState = A320_Neo_FCU_VSpeed_State.Idle;
                this.forceUpdate = true;
            }, this.backToIdleTimeout);
        } else if (this.currentState === A320_Neo_FCU_VSpeed_State.Zeroing) {
            this.currentState = A320_Neo_FCU_VSpeed_State.Flying;
            this.forceUpdate = true;
        }
    }

    onPull() {
        clearTimeout(this._resetSelectionTimeout);
        this.forceUpdate = true;

        if (this.currentState === A320_Neo_FCU_VSpeed_State.Idle) {
            if (this.isFPAMode) {
                this.selectedFpa = this.getCurrentFlightPathAngle();
            } else {
                this.selectedVs = this.getCurrentVerticalSpeed();
            }
        }

        SimVar.SetSimVarValue("K:A32NX.FCU_VS_PULL", "number", 0);
    }

    getCurrentFlightPathAngle() {
        return this.calculateAngleForVerticalSpeed(Simplane.getVerticalSpeed());
    }

    getCurrentVerticalSpeed() {
        return Utils.Clamp(Math.round(Simplane.getVerticalSpeed() / 100) * 100, -this.ABS_MINMAX_VS, this.ABS_MINMAX_VS);
    }

    _enterIdleState(idleVSpeed) {
        this.selectedVs = 0;
        this.selectedFpa = 0;
        this.currentState = A320_Neo_FCU_VSpeed_State.Idle;
        this.forceUpdate = true;
    }

    update(_deltaTime) {
        const lightsTest = SimVar.GetSimVarValue("L:XMLVAR_LTS_Test", "Bool");
        const isFPAMode = SimVar.GetSimVarValue("L:A32NX_TRK_FPA_MODE_ACTIVE", "Bool");
        const verticalMode = SimVar.GetSimVarValue("L:A32NX_FMA_VERTICAL_MODE", "Number");

        if ((this.previousVerticalMode != verticalMode)
            && (verticalMode !== 14 && verticalMode !== 15)) {
            clearTimeout(this._resetSelectionTimeout);
            this._enterIdleState();
        }

        if (this.currentState !== A320_Neo_FCU_VSpeed_State.Flying
            && this.currentState !== A320_Neo_FCU_VSpeed_State.Zeroing
            && (verticalMode === 14 || verticalMode === 15)) {
            clearTimeout(this._resetSelectionTimeout);
            this.forceUpdate = true;
            const isModeReversion = SimVar.GetSimVarValue("L:A32NX_FCU_MODE_REVERSION_ACTIVE", "Number");
            if (isFPAMode) {
                if (isModeReversion === 1) {
                    this.currentState = A320_Neo_FCU_VSpeed_State.Flying;
                    this.selectedFpa = this.getCurrentFlightPathAngle();
                } else if (this.selectedFpa !== 0) {
                    this.currentState = A320_Neo_FCU_VSpeed_State.Flying;
                } else {
                    this.currentState = A320_Neo_FCU_VSpeed_State.Zeroing;
                }
            } else {
                if (isModeReversion === 1) {
                    this.currentState = A320_Neo_FCU_VSpeed_State.Flying;
                    this.selectedVs = this.getCurrentVerticalSpeed();
                } else if (this.currentVs !== 0) {
                    this.currentState = A320_Neo_FCU_VSpeed_State.Flying;
                } else {
                    this.currentState = A320_Neo_FCU_VSpeed_State.Zeroing;
                }
            }
        }

        if (isFPAMode) {
            this.refresh(true, true, this.selectedFpa, lightsTest, this.forceUpdate);
        } else {
            this.refresh(true, false, this.selectedVs, lightsTest, this.forceUpdate);
        }

        this.forceUpdate = false;
        this.previousVerticalMode = verticalMode;
    }

    refresh(_isActive, _isFPAMode, _value, _lightsTest, _force = false) {
        if ((_isActive != this.isActive) || (_isFPAMode != this.isFPAMode) || (_value != this.currentValue) || (_lightsTest !== this.lightsTest) || _force) {
            if (this.isFPAMode != _isFPAMode) {
                this.onFPAModeChanged(_isFPAMode);
            }
            if (this.currentValue !== _value) {
                if (_isFPAMode) {
                    SimVar.SetSimVarValue("L:A32NX_AUTOPILOT_FPA_SELECTED", "Degree", this.selectedFpa);
                    SimVar.SetSimVarValue("L:A32NX_AUTOPILOT_VS_SELECTED", "feet per minute", this.calculateVerticalSpeedForAngle(this.selectedFpa));

                } else {
                    SimVar.SetSimVarValue("L:A32NX_AUTOPILOT_FPA_SELECTED", "Degree", this.calculateAngleForVerticalSpeed(this.selectedVs));
                    SimVar.SetSimVarValue("L:A32NX_AUTOPILOT_VS_SELECTED", "feet per minute", this.selectedVs);
                }
            }
            this.isActive = _isActive;
            this.isFPAMode = _isFPAMode;
            this.currentValue = _value;
            this.lightsTest = _lightsTest;
            if (this.lightsTest) {
                this.setTextElementActive(this.textVS, true);
                this.setTextElementActive(this.textFPA, true);
                this.textValueContent = "+8.888";
                return;
            }
            this.setTextElementActive(this.textVS, !this.isFPAMode);
            this.setTextElementActive(this.textFPA, this.isFPAMode);
            if (this.isActive && this.currentState != A320_Neo_FCU_VSpeed_State.Idle) {
                const sign = (this.currentValue < 0) ? "~" : "+";
                if (this.isFPAMode) {
                    let value = Math.abs(this.currentValue);
                    value = Math.round(value * 10).toString().padStart(2, "0");
                    value = `${value.substring(0, 1)}.${value.substring(1)}`;
                    this.textValueContent = sign + value;
                } else {
                    if (this.currentState === A320_Neo_FCU_VSpeed_State.Zeroing) {
                        this.textValueContent = ("{sp}00oo");
                    } else {
                        var value = Math.floor(this.currentValue);
                        value = Math.abs(value);
                        this.textValueContent = sign + (Math.floor(value * 0.01).toString().padStart(2, "0")) + "oo";
                    }
                }
            } else {
                this.textValueContent = "~----";
            }
        }
    }

    onEvent(_event) {
        if (_event === "VS_INC_VS") {
            this.selectedVs = Utils.Clamp(Math.round(this.selectedVs + 100), -this.ABS_MINMAX_VS, this.ABS_MINMAX_VS);
            this.onRotate();
        } else if (_event === "VS_DEC_VS") {
            this.selectedVs = Utils.Clamp(Math.round(this.selectedVs - 100), -this.ABS_MINMAX_VS, this.ABS_MINMAX_VS);
            this.onRotate();
        } else if (_event === "VS_INC_FPA") {
            this.selectedFpa = Utils.Clamp(Math.round((this.selectedFpa + 0.1) * 10) / 10, -this.ABS_MINMAX_FPA, this.ABS_MINMAX_FPA);
            this.onRotate();
        } else if (_event === "VS_DEC_FPA") {
            this.selectedFpa = Utils.Clamp(Math.round((this.selectedFpa - 0.1) * 10) / 10, -this.ABS_MINMAX_FPA, this.ABS_MINMAX_FPA);
            this.onRotate();
        } else if (_event === "VS_PUSH") {
            this.onPush();
        } else if (_event === "VS_PULL") {
            this.onPull();
        }
    }
    onFPAModeChanged(_newValue) {
        if (_newValue) {
            this.selectedFpa = this.calculateAngleForVerticalSpeed(this.selectedVs);
        } else {
            this.selectedVs = this.calculateVerticalSpeedForAngle(this.selectedFpa);
        }
    }
    /**
     * Calculates the vertical speed needed to fly a flight path angle at the current ground speed.
     * @param {number} _angle The flight path angle in degrees.
     * @returns {number} The corresponding vertical speed in feet per minute.
     */
    calculateVerticalSpeedForAngle(_angle) {
        if (_angle == 0) {
            return 0;
        }
        const _groundSpeed = SimVar.GetSimVarValue("GPS GROUND SPEED", "Meters per second");
        const groundSpeed = _groundSpeed * 3.28084 * 60; // Now in feet per minute.
        const angle = _angle * Math.PI / 180; // Now in radians.
        const verticalSpeed = Math.tan(angle) * groundSpeed;
        return Utils.Clamp(Math.round(verticalSpeed / 100) * 100, -this.ABS_MINMAX_VS, this.ABS_MINMAX_VS);
    }
    /**
     * Calculates the flight path angle for a given vertical speed, assuming it is flown at the current ground speed.
     * @param {number} verticalSpeed The flight path angle in feet per minute.
     * @returns {number} The corresponding flight path angle in degrees.
     */
    calculateAngleForVerticalSpeed(verticalSpeed) {
        if (Math.abs(verticalSpeed) < 10) {
            return 0;
        }
        const _groundSpeed = SimVar.GetSimVarValue("GPS GROUND SPEED", "Meters per second");
        const groundSpeed = _groundSpeed * 3.28084 * 60; // Now in feet per minute.
        const angle = Math.atan(verticalSpeed / groundSpeed);
        const _angle = angle * 180 / Math.PI;
        return Utils.Clamp(Math.round(_angle * 10) / 10, -this.ABS_MINMAX_FPA, this.ABS_MINMAX_FPA);
    }
}

class A320_Neo_FCU_LargeScreen extends NavSystemElement {
    init(root) {
        if (this.components == null) {
            this.components = new Array();
            this.components.push(new A320_Neo_FCU_Speed(this.gps, "Speed"));
            this.headingDisplay = new A320_Neo_FCU_Heading(this.gps, "Heading");
            this.components.push(this.headingDisplay);
            this.components.push(new A320_Neo_FCU_Mode(this.gps, "Mode"));
            this.altitudeDisplay = new A320_Neo_FCU_Altitude(this.gps, "Altitude");
            this.components.push(this.altitudeDisplay);
            this.verticalSpeedDisplay = new A320_Neo_FCU_VerticalSpeed(this.gps, "VerticalSpeed");
            this.components.push(this.verticalSpeedDisplay);
            this.autopilotInterface = new A320_Neo_FCU_Autopilot(this.gps, "Autopilot");
        }
    }
    onEnter() {
    }
    reboot() {
        if (this.components != null) {
            for (let i = 0; i < this.components.length; ++i) {
                if (this.components[i] != null) {
                    this.components[i].reboot();
                }
            }
        }
    }
    onFlightStart() {
        if (this.components != null) {
            for (let i = 0; i < this.components.length; ++i) {
                if (this.components[i] != null) {
                    this.components[i].onFlightStart();
                }
            }
        }
    }
    onUpdate(_deltaTime) {
        if (this.components != null) {
            for (let i = 0; i < this.components.length; ++i) {
                if (this.components[i] != null) {
                    this.components[i].update(_deltaTime);
                }
            }
        }
    }
    onExit() {
    }
    onEvent(_event) {
        this.autopilotInterface.onEvent(_event);
        this.headingDisplay.onEvent(_event);
        this.altitudeDisplay.onEvent(_event);
        this.verticalSpeedDisplay.onEvent(_event);
    }
}

class A320_Neo_FCU_Pressure extends A320_Neo_FCU_Component {
    init() {
        this.selectedElem = this.getDivElement("Selected");
        this.standardElem = this.getDivElement("Standard");
        this.textQFE = this.getTextElement("QFE");
        this.textQNH = this.getTextElement("QNH");
        this.refresh("QFE", true, 0, 0, true);
    }
    update(_deltaTime) {
        const units = Simplane.getPressureSelectedUnits();
        const mode = Simplane.getPressureSelectedMode(Aircraft.A320_NEO);
        this.refresh(mode, (units != "millibar"), Simplane.getPressureValue(units), SimVar.GetSimVarValue("L:XMLVAR_LTS_Test", "Bool"));
    }
    refresh(_mode, _isHGUnit, _value, _lightsTest, _force = false) {
        if ((_mode != this.currentMode) || (_isHGUnit != this.isHGUnit) || (_value != this.currentValue) || (_lightsTest !== this.lightsTest) || _force) {
            var wasStd = this.currentMode == "STD" && _mode != "STD";
            this.currentMode = _mode;
            this.isHGUnit = _isHGUnit;
            this.currentValue = _value;
            this.lightsTest = _lightsTest;
            if (this.lightsTest) {
                this.standardElem.style.display = "none";
                this.selectedElem.style.display = "block";
                this.setTextElementActive(this.textQFE, true);
                this.setTextElementActive(this.textQNH, true);
                this.textValueContent = "88.88";
                return;
            }
            if (this.currentMode == "STD") {
                this.standardElem.style.display = "block";
                this.selectedElem.style.display = "none";
                SimVar.SetSimVarValue("KOHLSMAN SETTING STD", "Bool", 1);
            } else {
                this.standardElem.style.display = "none";
                this.selectedElem.style.display = "block";
                SimVar.SetSimVarValue("KOHLSMAN SETTING STD", "Bool", 0);
                const isQFE = (this.currentMode == "QFE") ? true : false;
                this.setTextElementActive(this.textQFE, isQFE);
                this.setTextElementActive(this.textQNH, !isQFE);
                let value = Math.round(Math.max(this.isHGUnit ? (this.currentValue * 100) : this.currentValue, 0));
                if (!wasStd) {
                    value = value.toString().padStart(4, "0");
                    if (this.isHGUnit) {
                        value = `${value.substring(0, 2)}.${value.substring(2)}`;
                    }
                    this.textValueContent = value;
                }
            }
        }
    }
}

class A320_Neo_FCU_SmallScreen extends NavSystemElement {
    init(root) {
        if (this.pressure == null) {
            this.pressure = new A320_Neo_FCU_Pressure(this.gps, "SmallScreen");
        }
    }
    onEnter() {
    }
    onUpdate(_deltaTime) {
        if (this.pressure != null) {
            this.pressure.update(_deltaTime);
        }
    }
    onExit() {
    }
    onEvent(_event) {
    }
    reboot() {
        if (this.pressure) {
            this.pressure.reboot();
        }
    }
    onFlightStart() {
    }
}

registerInstrument("a320-neo-fcu-element", A320_Neo_FCU);
