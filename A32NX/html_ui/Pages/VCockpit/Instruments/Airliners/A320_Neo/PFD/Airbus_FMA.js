var Airbus;
(function (Airbus) {
    class FMA extends HTMLElement {
        constructor() {
            super(...arguments);
            this._aircraft = Aircraft.A320_NEO;
        }

        get aircraft() {
            return this._aircraft;
        }

        set aircraft(_val) {
            if (this._aircraft != _val) {
                this._aircraft = _val;
                this.init();
            }
        }

        connectedCallback() {
            this.init();
        }

        init() {
            Utils.RemoveAllChildren(this);
            Airbus_FMA.CurrentPlaneState.init();
            this.columns = new Array();
            this.columns.push(new Airbus_FMA.Column1(this));
            this.columns.push(new Airbus_FMA.Column2(this));
            this.columns.push(new Airbus_FMA.Column3(this));
            this.columns.push(new Airbus_FMA.Column4(this));
            this.columns.push(new Airbus_FMA.Column5(this));
            for (let i = 0; i < this.columns.length; ++i) {
                this.columns[i].init();
            }
            this.vertAndLat = new Airbus_FMA.VerticalAndLateral(this);
            this.vertAndLat.init();
            this.specialMessages = new Airbus_FMA.SpecialMessages(this);
            this.specialMessages.init();
        }

        update(_dTime) {
            Airbus_FMA.CurrentPlaneState.refresh();
            for (let i = 0; i < this.columns.length; ++i) {
                if (this.columns[i] != null) {
                    this.columns[i].update(_dTime);
                }
            }
            if (this.vertAndLat != null) {
                this.vertAndLat.update(_dTime);
            }
            if (this.specialMessages != null) {
                this.specialMessages.update(_dTime);
            }
        }
    }

    Airbus.FMA = FMA;
})(Airbus || (Airbus = {}));
var Airbus_FMA;
(function (Airbus_FMA) {
    class Definitions {
    }

    Definitions.HIGHLIGHT_LENGTH = 10 * 1000;
    Definitions.ALT_ENGAGED_RANGE = 20;
    Airbus_FMA.Definitions = Definitions;
    let MODE_STATE;
    (function (MODE_STATE) {
        MODE_STATE[MODE_STATE["NONE"] = 0] = "NONE";
        MODE_STATE[MODE_STATE["ENGAGED"] = 1] = "ENGAGED";
        MODE_STATE[MODE_STATE["CAPTURED"] = 2] = "CAPTURED";
        MODE_STATE[MODE_STATE["ARMED"] = 3] = "ARMED";
        MODE_STATE[MODE_STATE["CONSTRAINED"] = 4] = "CONSTRAINED";
        MODE_STATE[MODE_STATE["ACTIVE"] = 5] = "ACTIVE";
        MODE_STATE[MODE_STATE["STATUS"] = 6] = "STATUS";
        MODE_STATE[MODE_STATE["WARNING"] = 7] = "WARNING";
        MODE_STATE[MODE_STATE["DANGER"] = 8] = "DANGER";
        MODE_STATE[MODE_STATE["ACTIVE_BLINK"] = 9] = "ACTIVE_BLINK";
    })(MODE_STATE = Airbus_FMA.MODE_STATE || (Airbus_FMA.MODE_STATE = {}));
    let HIGHLIGHT_STYLE;
    (function (HIGHLIGHT_STYLE) {
        HIGHLIGHT_STYLE[HIGHLIGHT_STYLE["NONE"] = 0] = "NONE";
        HIGHLIGHT_STYLE[HIGHLIGHT_STYLE["FULL"] = 1] = "FULL";
        HIGHLIGHT_STYLE[HIGHLIGHT_STYLE["FULL_WARNING"] = 2] = "FULL_WARNING";
        HIGHLIGHT_STYLE[HIGHLIGHT_STYLE["OPEN_TOP"] = 3] = "OPEN_TOP";
        HIGHLIGHT_STYLE[HIGHLIGHT_STYLE["OPEN_TOP_WARNING"] = 4] = "OPEN_TOP_WARNING";
        HIGHLIGHT_STYLE[HIGHLIGHT_STYLE["OPEN_BOTTOM"] = 5] = "OPEN_BOTTOM";
        HIGHLIGHT_STYLE[HIGHLIGHT_STYLE["OPEN_BOTTOM_WARNING"] = 6] = "OPEN_BOTTOM_WARNING";
    })(HIGHLIGHT_STYLE = Airbus_FMA.HIGHLIGHT_STYLE || (Airbus_FMA.HIGHLIGHT_STYLE = {}));

    function TEXT_STYLE_CLASS(_style) {
        switch (_style) {
            case Airbus_FMA.MODE_STATE.ENGAGED:
                return "Engaged";
            case Airbus_FMA.MODE_STATE.CAPTURED:
                return "Captured";
            case Airbus_FMA.MODE_STATE.ARMED:
                return "Armed";
            case Airbus_FMA.MODE_STATE.CONSTRAINED:
                return "Constrained";
            case Airbus_FMA.MODE_STATE.ACTIVE:
                return "Active";
            case Airbus_FMA.MODE_STATE.STATUS:
                return "Status";
            case Airbus_FMA.MODE_STATE.WARNING:
                return "Warning";
            case Airbus_FMA.MODE_STATE.DANGER:
                return "Danger";
            case Airbus_FMA.MODE_STATE.ACTIVE_BLINK:
                return "ActiveBlink";
        }
        return "";
    }

    Airbus_FMA.TEXT_STYLE_CLASS = TEXT_STYLE_CLASS;

    class CurrentPlaneState {
        static init() {
        }

        static refresh() {
            this.autoPilotActive[0] = SimVar.GetSimVarValue("L:A32NX_AUTOPILOT_1_ACTIVE", "number");
            this.autoPilotActive[1] = SimVar.GetSimVarValue("L:A32NX_AUTOPILOT_2_ACTIVE", "number");
            this.anyAutoPilotsActive = (this.autoPilotActive[0] || this.autoPilotActive[1]);
            this.bothAutoPilotsActive = (this.autoPilotActive[0] && this.autoPilotActive[1]);
            this.bothAutoPilotsInactive = (!this.autoPilotActive[0] && !this.autoPilotActive[1]);
            this.autoPilotFlightDirectorActive[0] = Simplane.getAutoPilotFlightDirectorActive(1);
            this.autoPilotFlightDirectorActive[1] = Simplane.getAutoPilotFlightDirectorActive(2);
            const url = document.getElementsByTagName("a320-neo-pfd-element")[0].getAttribute("url");
            const index = parseInt(url.substring(url.length - 1));
            this.thisFlightDirectorActive = (this.autoPilotFlightDirectorActive[index - 1]);
            this.bothFlightDirectorsActive = (this.autoPilotFlightDirectorActive[0] && this.autoPilotFlightDirectorActive[1]);
            this.autoPilotTOGAActive = Simplane.getAutoPilotTOGAActive();
            this.autoPilotThrottleArmed = Simplane.getAutoPilotThrottleArmed();
            this.autoPilotAirspeedHoldActive = Simplane.getAutoPilotAirspeedHoldActive();
            this.autoPilotVerticalSpeedHoldActive = Simplane.getAutoPilotVerticalSpeedHoldActive();
            this.autoPilotVerticalSpeedHoldValue = SimVar.GetSimVarValue("L:A32NX_AUTOPILOT_VS_SELECTED", "feet per minute");
            this.autoPilotFlightPathAngle = SimVar.GetSimVarValue("L:A32NX_AUTOPILOT_FPA_SELECTED", "Degree");
            this.autoPilotAltitudeLockActive = Simplane.getAutoPilotAltitudeLockActive();
            this.autoPilotAltitudeArmed = Simplane.getAutoPilotAltitudeArmed();
            this.autoPilotAltitudeLockValue = Simplane.getAutoPilotAltitudeLockValue();
            this.autoPilotHeadingSelectedMode = Simplane.getAutoPilotHeadingSelected();
            this.autoPilotHeadingManagedMode = Simplane.getAutoPilotHeadingManaged();
            this.autoPilotThrottleActive = Simplane.getAutoPilotThrottleActive();
            this.autoPilotThrottleLocked = Simplane.getAutoPilotThrottleLocked();
            this.autoPilotMachModeActive = Simplane.getAutoPilotMachModeActive();
            this.autoPilotGlideslopeActive = Simplane.getAutoPilotGlideslopeActive();
            this.autoPilotTRKFPAModeActive = SimVar.GetSimVarValue("L:A32NX_TRK_FPA_MODE_ACTIVE", "Bool");
            this.altitude = Simplane.getAltitude();
            this.flapsHandlePercent = Simplane.getFlapsHandlePercent();
            this.flightPhase = Simplane.getCurrentFlightPhase(true);
            const approachType = ApproachType.APPROACH_TYPE_ILS;
            this.isILSApproachActive = ((this.flightPhase == FlightPhase.FLIGHT_PHASE_APPROACH) && (approachType == ApproachType.APPROACH_TYPE_ILS));
            this.isNonILSApproachActive = ((this.flightPhase == FlightPhase.FLIGHT_PHASE_APPROACH) && (approachType != ApproachType.APPROACH_TYPE_ILS));
            this.leftThrottleDetent = Simplane.getEngineThrottleMode(0);
            this.rightThrottleDetent = Simplane.getEngineThrottleMode(1);
            this.bothEnginesActive = Simplane.getEngineActive(0) && Simplane.getEngineActive(1);
            this.highestThrottleDetent = (this.leftThrottleDetent >= this.rightThrottleDetent) ? this.leftThrottleDetent : this.rightThrottleDetent;
            this.managedAirspeed = SimVar.GetSimVarValue("L:AP_MANAGED_AIRSPEED", "number");
            this.minimumDescentAltitude = SimVar.GetSimVarValue("L:AIRLINER_MINIMUM_DESCENT_ALTITUDE", "feet");
            this.decisionHeight = SimVar.GetSimVarValue("L:AIRLINER_DECISION_HEIGHT", "feet");
            this.radioAltitude = Simplane.getAltitudeAboveGround();
            this.flexTemperature = Simplane.getFlexTemperature();
            this.isOnGround = Simplane.getIsGrounded();
            this.autoBreakLevel = SimVar.GetSimVarValue("L:XMLVAR_Autobrakes_Level", "Enum");
            this.autoBreakDecel = this.autoBreakLevel !== 0 && this.isOnGround && (this.autoBreakLevel === 3 ? -6 : -2) > SimVar.GetSimVarValue("A:ACCELERATION BODY Z", "feet per second squared");
            this.autoBreakActivated = SimVar.GetSimVarValue("L:A32NX_AUTOBRAKES_BRAKING", "Bool") === 1;
        }
    }

    CurrentPlaneState.autoPilotActive = [false, false];
    CurrentPlaneState.autoPilotFlightDirectorActive = [false, false];
    Airbus_FMA.CurrentPlaneState = CurrentPlaneState;
    let SRSEnabled = false;

    class Cell {
        constructor(_parent, _className) {
            this.parent = _parent;
            this.divRef = window.document.createElement("div");
            this.divRef.id = _className;
            this.divRef.className = Airbus_FMA.TEXT_STYLE_CLASS(Airbus_FMA.MODE_STATE.NONE);
            if (_parent != null) {
                _parent.appendChild(this.divRef);
            }
        }

        init() {
            this.textStyle = Airbus_FMA.MODE_STATE.NONE;
        }

        setVisibility(_show) {
            if (this.divRef != null) {
                this.divRef.style.display = _show ? "block" : "none";
            }
        }

        setText(_text, _style) {
            if (this.divRef != null) {
                if ((this.divRef.textContent != _text) || (this.textStyle != _style)) {
                    this.divRef.textContent = _text;
                    this.textStyle = _style;
                    this.divRef.className = Airbus_FMA.TEXT_STYLE_CLASS(this.textStyle);
                    return true;
                }
            }
            return false;
        }
    }

    Airbus_FMA.Cell = Cell;

    class Row extends Airbus_FMA.Cell {
        constructor(_parent, _className, _createSubCells) {
            super(_parent, _className);
            this.subLeft = null;
            this.subRight = null;
            if (_createSubCells && (this.parent != null)) {
                this.subLeft = new Airbus_FMA.Cell(this.parent, _className + "SubLeft");
                this.subLeft.setText("", Airbus_FMA.MODE_STATE.NONE);
                this.subRight = new Airbus_FMA.Cell(this.parent, _className + "SubRight");
                this.subRight.setText("", Airbus_FMA.MODE_STATE.NONE);
            }
        }

        init() {
            super.init();
            this.highlightStyle = Airbus_FMA.HIGHLIGHT_STYLE.NONE;
            this.highlightPermanent = false;
            this.hideHighlight();
            if (this.subLeft != null) {
                this.subLeft.init();
            }
            if (this.subRight != null) {
                this.subRight.init();
            }
        }

        update(_deltaTime) {
            if (!this.highlightPermanent && this.highlightTime > 0) {
                this.highlightTime -= _deltaTime;
                if (this.highlightTime <= 0) {
                    this.hideHighlight();
                }
            }
            // Temporary fix for FMA blinking until css animations work again
            if (this.blink) {
                if (Math.sin((Date.now() / 1000) * Math.PI * 2) > 0) {
                    if (this.blinkIsVisible) {
                        this.blinkIsVisible = false;
                        this.divRef.style.visibility = 'hidden';
                    }
                } else {
                    if (!this.blinkIsVisible) {
                        this.blinkIsVisible = true;
                        this.divRef.style.visibility = 'visible';
                    }
                }
            }
        }

        setMainText(_text, _style) {
            const changed = this.setText(_text, _style);
            this.blink = _style === Airbus_FMA.MODE_STATE.ACTIVE_BLINK;
            if (!this.blink) {
                this.divRef.style.visibility = 'visible';
            }
            if (_text.length <= 0) {
                this.hideHighlight();
            } else if (changed) {
                this.showHighlight();
            }
            if (this.subLeft != null) {
                this.subLeft.setText("", Airbus_FMA.MODE_STATE.NONE);
            }
            if (this.subRight != null) {
                this.subRight.setText("", Airbus_FMA.MODE_STATE.NONE);
            }
        }

        setSubTexts(_textLeft, _styleLeft, _textRight, _styleRight) {
            this.setText("", Airbus_FMA.MODE_STATE.NONE);
            if (this.subLeft != null) {
                if (this.subLeft.setText(_textLeft, _styleLeft)) {
                    if (_textLeft.length > 0) {
                        this.showHighlight();
                    } else {
                        this.hideHighlight();
                    }
                }
            }
            if (this.subRight != null) {
                this.subRight.setText(_textRight, _styleRight);
            }
        }

        setHighlightStyle(_highlightStyle, _permanent = false) {
            this.highlightStyle = _highlightStyle;
            this.highlightPermanent = _permanent;
        }

        showHighlight() {
            if (this.divRef != null) {
                if (this.highlightStyle != Airbus_FMA.HIGHLIGHT_STYLE.NONE) {
                    let colour = "white";
                    let showTop = true;
                    let showBottom = true;
                    switch (this.highlightStyle) {
                        case Airbus_FMA.HIGHLIGHT_STYLE.FULL_WARNING: {
                            colour = "orange";
                            break;
                        }
                        case Airbus_FMA.HIGHLIGHT_STYLE.OPEN_BOTTOM: {
                            showBottom = false;
                            break;
                        }
                        case Airbus_FMA.HIGHLIGHT_STYLE.OPEN_BOTTOM_WARNING: {
                            colour = "orange";
                            showBottom = false;
                            break;
                        }
                        case Airbus_FMA.HIGHLIGHT_STYLE.OPEN_TOP: {
                            showTop = false;
                            break;
                        }
                        case Airbus_FMA.HIGHLIGHT_STYLE.OPEN_TOP_WARNING: {
                            colour = "orange";
                            showTop = false;
                            break;
                        }
                    }
                    const styleStr = "solid 2px " + colour;
                    this.divRef.style.borderLeft = styleStr;
                    this.divRef.style.borderRight = styleStr;
                    if (showTop) {
                        this.divRef.style.borderTop = styleStr;
                    } else {
                        this.divRef.style.borderTop = "none";
                    }
                    if (showBottom) {
                        this.divRef.style.borderBottom = styleStr;
                    } else {
                        this.divRef.style.borderBottom = "none";
                    }
                    this.highlightTime = Airbus_FMA.Definitions.HIGHLIGHT_LENGTH;
                }
            }
        }

        hideHighlight() {
            if (this.divRef != null) {
                this.divRef.style.borderLeft = "none";
                this.divRef.style.borderRight = "none";
                this.divRef.style.borderTop = "none";
                this.divRef.style.borderBottom = "none";
            }
            this.highlightTime = 0;
            this.highlightPermanent = false;
        }
    }

    Airbus_FMA.Row = Row;

    class ColumnBase {
        constructor(_parent) {
            this.parent = _parent;
            this.divMain = window.document.createElement("div");
            this.divMain.id = this.getMainDivName();
            _parent.appendChild(this.divMain);
            this.rows = new Array();
            this.rows.push(new Airbus_FMA.Row(this.divMain, "Row1", this.createSubCellsOnRow1()));
            this.rows.push(new Airbus_FMA.Row(this.divMain, "Row2", this.createSubCellsOnRow2()));
            this.rows.push(new Airbus_FMA.Row(this.divMain, "Row3", this.createSubCellsOnRow3()));
        }

        createSubCellsOnRow1() {
            return false;
        }

        createSubCellsOnRow2() {
            return false;
        }

        createSubCellsOnRow3() {
            return false;
        }

        init() {
            if (this.rows != null) {
                for (let i = 0; i < this.rows.length; ++i) {
                    if (this.rows[i] != null) {
                        this.rows[i].init();
                    }
                }
            }
            this.initChild();
        }

        update(_deltaTime) {
            if (this.rows != null) {
                for (let i = 0; i < this.rows.length; ++i) {
                    if (this.rows[i] != null) {
                        this.rows[i].update(_deltaTime);
                    }
                }
            }
            this.updateChild(_deltaTime);
        }

        setRowHighlightStyle(_row, _highlightStyle, _permanent = false) {
            if (this.rows != null) {
                if ((_row >= 0) && (_row < this.rows.length)) {
                    if (this.rows[_row] != null) {
                        this.rows[_row].setHighlightStyle(_highlightStyle, _permanent);
                    }
                }
            }
        }

        setRowText(_row, _text, _style) {
            if (this.rows != null) {
                if ((_row >= 0) && (_row < this.rows.length)) {
                    if (this.rows[_row] != null) {
                        this.rows[_row].setMainText(_text, _style);
                    }
                }
            }
        }

        setRowMultiText(_row, _textLeft, _styleLeft, _textRight, _styleRight) {
            if (this.rows != null) {
                if ((_row >= 0) && (_row < this.rows.length)) {
                    if (this.rows[_row] != null) {
                        this.rows[_row].setSubTexts(_textLeft, _styleLeft, _textRight, _styleRight);
                    }
                }
            }
        }
    }

    Airbus_FMA.ColumnBase = ColumnBase;

    class Column1 extends Airbus_FMA.ColumnBase {
        constructor() {
            super(...arguments);
            this._blink = 0;
        }

        createSubCellsOnRow2() {
            return true;
        }

        getMainDivName() {
            return "Column1";
        }

        initChild() {
            this.currentRow1State = Column1.ROW_1_STATE.NONE;
            this.currentRow1And2State = Column1.ROW_1_2_STATE.NONE;
            this.currentRow2State = Column1.ROW_2_STATE.NONE;
            this.currentRow3State = Column1.ROW_3_STATE.NONE;
        }

        updateChild(_deltaTime) {
            const targetRow1State = this.getTargetRow1State();
            const targetRow1And2State = this.getTargetRow1And2State();
            const targetRow2State = this.getTargetRow2State();
            const targetRow3State = this.getTargetRow3State();
            const lineOne = targetRow1State !== this.currentRow1State;
            const lineOneTwo = this.currentRow1And2State === Column1.ROW_1_2_STATE.MANFLX || this.currentRow1And2State !== targetRow1And2State;
            const lineTwo = lineOneTwo && targetRow1And2State === Airbus_FMA.MODE_STATE.NONE ? true : targetRow2State !== this.currentRow2State;
            const lineThree = targetRow3State !== this.currentRow3State;
            if (lineOne || lineOneTwo || lineTwo || lineThree) {
                this.currentRow1State = targetRow1State;
                this.currentRow1And2State = targetRow1And2State;
                this.currentRow2State = targetRow2State;
                this.currentRow3State = targetRow3State;
                setTimeout(() => {
                    if (lineOne || lineOneTwo) {
                        this.setRowText(0, "", Airbus_FMA.MODE_STATE.NONE);
                    }
                    if (lineTwo || lineOneTwo) {
                        if (this.currentRow1And2State === Airbus_FMA.MODE_STATE.NONE) {
                            this.setRowText(1, "", Airbus_FMA.MODE_STATE.NONE);
                        }
                    }
                    if (lineOne && this.currentRow1State !== Airbus_FMA.MODE_STATE.NONE) {
                        switch (this.currentRow1State) {
                            case Column1.ROW_1_STATE.TOGA: {
                                this.setRowHighlightStyle(0, Airbus_FMA.HIGHLIGHT_STYLE.FULL, true);
                                this.setRowText(0, "TOGA", Airbus_FMA.MODE_STATE.STATUS);
                                break;
                            }
                            case Column1.ROW_1_STATE.THRMCT: {
                                this.setRowHighlightStyle(0, Airbus_FMA.HIGHLIGHT_STYLE.FULL);
                                this.setRowText(0, "THR MCT", Airbus_FMA.MODE_STATE.ENGAGED);
                                break;
                            }
                            case Column1.ROW_1_STATE.THRCLB: {
                                this.setRowHighlightStyle(0, Airbus_FMA.HIGHLIGHT_STYLE.FULL);
                                this.setRowText(0, "THR CLB", Airbus_FMA.MODE_STATE.ENGAGED);
                                break;
                            }
                            case Column1.ROW_1_STATE.THRLVR: {
                                this.setRowHighlightStyle(0, Airbus_FMA.HIGHLIGHT_STYLE.FULL);
                                this.setRowText(0, "THR LVR", Airbus_FMA.MODE_STATE.ENGAGED);
                                break;
                            }
                            case Column1.ROW_1_STATE.THRIDLE: {
                                this.setRowHighlightStyle(0, Airbus_FMA.HIGHLIGHT_STYLE.FULL);
                                this.setRowText(0, "THR IDLE", Airbus_FMA.MODE_STATE.ENGAGED);
                                break;
                            }
                            case Column1.ROW_1_STATE.AFLOOR: {
                                this.setRowHighlightStyle(0, Airbus_FMA.HIGHLIGHT_STYLE.FULL_WARNING, true);
                                this.setRowText(0, "A.FLOOR", Airbus_FMA.MODE_STATE.ENGAGED);
                                break;
                            }
                            case Column1.ROW_1_STATE.TOGALK: {
                                this.setRowHighlightStyle(0, Airbus_FMA.HIGHLIGHT_STYLE.FULL_WARNING, true);
                                this.setRowText(0, "TOGA LK", Airbus_FMA.MODE_STATE.ENGAGED);
                                break;
                            }
                            case Column1.ROW_1_STATE.SPEED: {
                                this.setRowHighlightStyle(0, Airbus_FMA.HIGHLIGHT_STYLE.FULL);
                                this.setRowText(0, "SPEED", Airbus_FMA.MODE_STATE.ENGAGED);
                                break;
                            }
                            case Column1.ROW_1_STATE.MACH: {
                                this.setRowHighlightStyle(0, Airbus_FMA.HIGHLIGHT_STYLE.FULL);
                                this.setRowText(0, "MACH", Airbus_FMA.MODE_STATE.ENGAGED);
                                break;
                            }
                            case Column1.ROW_1_STATE.THRLK: {
                                this.setRowHighlightStyle(0, Airbus_FMA.HIGHLIGHT_STYLE.FULL);
                                this.setRowText(0, "THR LK", Airbus_FMA.MODE_STATE.WARNING);
                                break;
                            }
                            case Column1.ROW_1_STATE.SGA: {
                                this.setRowHighlightStyle(0, Airbus_FMA.HIGHLIGHT_STYLE.FULL, true);
                                this.setRowText(0, "GA SOFT", Airbus_FMA.MODE_STATE.ENGAGED);
                                break;
                            }
                            case Column1.ROW_1_STATE.BRKLO: {
                                this.setRowHighlightStyle(0, Airbus_FMA.HIGHLIGHT_STYLE.FULL);
                                this.setRowText(0, "BRK LO", Airbus_FMA.MODE_STATE.CAPTURED);
                                break;
                            }
                            case Column1.ROW_1_STATE.BRKMED: {
                                this.setRowHighlightStyle(0, Airbus_FMA.HIGHLIGHT_STYLE.FULL);
                                this.setRowText(0, "BRK MED", Airbus_FMA.MODE_STATE.CAPTURED);
                                break;
                            }
                            case Column1.ROW_1_STATE.BRKMAX: {
                                this.setRowHighlightStyle(0, Airbus_FMA.HIGHLIGHT_STYLE.FULL);
                                this.setRowText(0, "BRK MAX", Airbus_FMA.MODE_STATE.CAPTURED);
                                break;
                            }
                        }
                    }
                    if (lineTwo && this.currentRow2State !== Airbus_FMA.MODE_STATE.NONE && this.currentRow1And2State === Airbus_FMA.MODE_STATE.NONE) {
                        switch (this.currentRow2State) {
                            case Column1.ROW_2_STATE.BRKLO: {
                                this.setRowHighlightStyle(1, Airbus_FMA.HIGHLIGHT_STYLE.NONE);
                                this.setRowText(1, "BRK LO", Airbus_FMA.MODE_STATE.ARMED);
                                break;
                            }
                            case Column1.ROW_2_STATE.BRKMED: {
                                this.setRowHighlightStyle(1, Airbus_FMA.HIGHLIGHT_STYLE.NONE);
                                this.setRowText(1, "BRK MED", Airbus_FMA.MODE_STATE.ARMED);
                                break;
                            }
                        }
                    }
                    if (lineOneTwo && this.currentRow1And2State !== Airbus_FMA.MODE_STATE.NONE) {
                        switch (this.currentRow1And2State) {
                            case Column1.ROW_1_2_STATE.MANTOGA: {
                                this.setRowHighlightStyle(0, Airbus_FMA.HIGHLIGHT_STYLE.OPEN_BOTTOM, true);
                                this.setRowHighlightStyle(1, Airbus_FMA.HIGHLIGHT_STYLE.OPEN_TOP, true);
                                this.setRowText(0, "MAN", Airbus_FMA.MODE_STATE.STATUS);
                                this.setRowText(1, "TOGA", Airbus_FMA.MODE_STATE.STATUS);
                                break;
                            }
                            case Column1.ROW_1_2_STATE.MANFLX: {
                                this.setRowHighlightStyle(0, Airbus_FMA.HIGHLIGHT_STYLE.OPEN_BOTTOM, true);
                                this.setRowHighlightStyle(1, Airbus_FMA.HIGHLIGHT_STYLE.OPEN_TOP, true);
                                this.setRowText(0, "MAN", Airbus_FMA.MODE_STATE.STATUS);
                                let temperatureText = Airbus_FMA.CurrentPlaneState.flexTemperature >= 0 ? "+" : "";
                                temperatureText += Airbus_FMA.CurrentPlaneState.flexTemperature.toFixed(0);
                                this.setRowMultiText(1, "FLX", Airbus_FMA.MODE_STATE.STATUS, temperatureText, Airbus_FMA.MODE_STATE.ARMED);
                                break;
                            }
                            case Column1.ROW_1_2_STATE.MANMCT: {
                                this.setRowHighlightStyle(0, Airbus_FMA.HIGHLIGHT_STYLE.OPEN_BOTTOM, true);
                                this.setRowHighlightStyle(1, Airbus_FMA.HIGHLIGHT_STYLE.OPEN_TOP, true);
                                this.setRowText(0, "MAN", Airbus_FMA.MODE_STATE.STATUS);
                                this.setRowText(1, "MCT", Airbus_FMA.MODE_STATE.STATUS);
                                break;
                            }
                            case Column1.ROW_1_2_STATE.MANTHR: {
                                this.setRowHighlightStyle(0, Airbus_FMA.HIGHLIGHT_STYLE.OPEN_BOTTOM_WARNING, true);
                                this.setRowHighlightStyle(1, Airbus_FMA.HIGHLIGHT_STYLE.OPEN_TOP_WARNING, true);
                                this.setRowText(0, "MAN", Airbus_FMA.MODE_STATE.STATUS);
                                this.setRowText(1, "THR", Airbus_FMA.MODE_STATE.STATUS);
                                break;
                            }
                            case Column1.ROW_1_2_STATE.MANSGA: {
                                this.setRowHighlightStyle(0, Airbus_FMA.HIGHLIGHT_STYLE.OPEN_BOTTOM, true);
                                this.setRowHighlightStyle(1, Airbus_FMA.HIGHLIGHT_STYLE.OPEN_TOP, true);
                                this.setRowText(0, "MAN", Airbus_FMA.MODE_STATE.STATUS);
                                this.setRowText(1, "GA SOFT", Airbus_FMA.MODE_STATE.STATUS);
                                break;
                            }
                        }
                    }
                    if (lineThree) {
                        switch (this.currentRow3State) {
                            case Column1.ROW_3_STATE.LVRCLB: {
                                this.setRowText(2, "LVR CLB", Airbus_FMA.MODE_STATE.ACTIVE_BLINK);
                                break;
                            }
                            case Column1.ROW_3_STATE.LVRMCT: {
                                this.setRowText(2, "LVR MCT", Airbus_FMA.MODE_STATE.STATUS);
                                break;
                            }
                            case Column1.ROW_3_STATE.LVRASYM: {
                                this.setRowText(2, "LVR ASYM", Airbus_FMA.MODE_STATE.WARNING);
                                break;
                            }
                            case Column1.ROW_3_STATE.BRKMAX: {
                                this.setRowHighlightStyle(1, Airbus_FMA.HIGHLIGHT_STYLE.NONE);
                                this.setRowText(2, "BRK MAX", Airbus_FMA.MODE_STATE.ARMED);
                                break;
                            }
                            default: {
                                this.setRowText(2, "", Airbus_FMA.MODE_STATE.NONE);
                                break;
                            }
                        }
                    }
                }, 500 + Math.random() * 100);
            }
        }

        getTargetRow1State() {
            if (this.IsActive_AFLOOR()) {
                return Column1.ROW_1_STATE.AFLOOR;
            } else if (this.IsActive_MANTOGA()) {
                return Column1.ROW_1_STATE.NONE;
            } else if (this.IsActive_TOGA()) {
                return Column1.ROW_1_STATE.TOGA;
            } else if (this.IsActive_TOGALK()) {
                return Column1.ROW_1_STATE.TOGALK;
            } else if (this.IsActive_SGA()) {
                return Column1.ROW_1_STATE.SGA;
            } else if (this.IsActive_THRMCT()) {
                return Column1.ROW_1_STATE.THRMCT;
            } else if (this.IsActive_THRCLB()) {
                return Column1.ROW_1_STATE.THRCLB;
            } else if (this.IsActive_THRLVR()) {
                return Column1.ROW_1_STATE.THRLVR;
            } else if (this.IsActive_THRIDLE()) {
                return Column1.ROW_1_STATE.THRIDLE;
            } else if (this.IsActive_SPEED()) {
                return Column1.ROW_1_STATE.SPEED;
            } else if (this.IsActive_MACH()) {
                return Column1.ROW_1_STATE.MACH;
            } else if (this.IsActive_THRLK()) {
                return Column1.ROW_1_STATE.THRLK;
            } else if (this.IsActive_BRKLO()) {
                return Column1.ROW_1_STATE.BRKLO;
            } else if (this.IsActive_BRKMED()) {
                return Column1.ROW_1_STATE.BRKMED;
            } else if (this.IsActive_BRKMAX()) {
                return Column1.ROW_1_STATE.BRKMAX;
            } else {
                return Column1.ROW_1_STATE.NONE;
            }
        }

        getTargetRow1And2State() {
            if (this.IsActive_MANTOGA()) {
                return Column1.ROW_1_2_STATE.MANTOGA;
            } else if (this.IsActive_MANFLX()) {
                return Column1.ROW_1_2_STATE.MANFLX;
            } else if (this.IsActive_MANSGA()) {
                return Column1.ROW_1_2_STATE.MANSGA;
            } else if (this.IsActive_MANMCT()) {
                return Column1.ROW_1_2_STATE.MANMCT;
            } else if (this.IsActive_MANTHR()) {
                return Column1.ROW_1_2_STATE.MANTHR;
            } else {
                return Column1.ROW_1_2_STATE.NONE;
            }
        }

        getTargetRow2State() {
            if (this.IsActive_BRKLOARMED()) {
                return Column1.ROW_2_STATE.BRKLO;
            } else if (this.IsActive_BRKMEDARMED()) {
                return Column1.ROW_2_STATE.BRKMED;
            } else {
                return Column1.ROW_2_STATE.NONE;
            }
        }

        getTargetRow3State() {
            if (this.IsActive_LVRCLB()) {
                return Column1.ROW_3_STATE.LVRCLB;
            } else if (this.IsActive_LVRMCT()) {
                return Column1.ROW_3_STATE.LVRMCT;
            } else if (this.IsActive_LVRASYM()) {
                return Column1.ROW_3_STATE.LVRASYM;
            } else if (this.IsActive_BRKMAXARMED()) {
                return Column1.ROW_3_STATE.BRKMAX;
            } else {
                return Column1.ROW_3_STATE.NONE;
            }
        }

        IsActive_MANTOGA() {
            if (!Airbus_FMA.CurrentPlaneState.autoPilotThrottleActive && Airbus_FMA.CurrentPlaneState.autoPilotThrottleArmed && (Airbus_FMA.CurrentPlaneState.highestThrottleDetent == ThrottleMode.TOGA)) {
                return true;
            }
            return false;
        }

        IsActive_TOGA() {
            if (Airbus_FMA.CurrentPlaneState.autoPilotThrottleActive && (Airbus_FMA.CurrentPlaneState.highestThrottleDetent == ThrottleMode.TOGA)) {
                return true;
            }
            return false;
        }

        IsActive_MANFLX() {
            if (!Airbus_FMA.CurrentPlaneState.autoPilotThrottleActive && Airbus_FMA.CurrentPlaneState.autoPilotThrottleArmed && (Airbus_FMA.CurrentPlaneState.highestThrottleDetent == ThrottleMode.FLEX_MCT)) {
                if ((Airbus_FMA.CurrentPlaneState.flightPhase <= FlightPhase.FLIGHT_PHASE_TAKEOFF) && (Airbus_FMA.CurrentPlaneState.flexTemperature !== 0)) {
                    return true;
                }
            }
            return false;
        }

        IsActive_MANSGA() {
            return Airbus_FMA.CurrentPlaneState.bothEnginesActive
                && !Airbus_FMA.CurrentPlaneState.autoPilotThrottleActive
                && Airbus_FMA.CurrentPlaneState.autoPilotThrottleArmed
                && Airbus_FMA.CurrentPlaneState.highestThrottleDetent === ThrottleMode.FLEX_MCT
                && Airbus_FMA.CurrentPlaneState.flightPhase === FlightPhase.FLIGHT_PHASE_GOAROUND;
        }

        IsActive_SGA() {
            return Airbus_FMA.CurrentPlaneState.bothEnginesActive
                && Airbus_FMA.CurrentPlaneState.autoPilotThrottleActive
                && Airbus_FMA.CurrentPlaneState.autoPilotThrottleArmed
                && Airbus_FMA.CurrentPlaneState.highestThrottleDetent === ThrottleMode.FLEX_MCT
                && Airbus_FMA.CurrentPlaneState.flightPhase === FlightPhase.FLIGHT_PHASE_GOAROUND;
        }

        IsActive_MANMCT() {
            if (!Airbus_FMA.CurrentPlaneState.autoPilotThrottleActive && Airbus_FMA.CurrentPlaneState.autoPilotThrottleArmed && (Airbus_FMA.CurrentPlaneState.highestThrottleDetent == ThrottleMode.FLEX_MCT)) {
                if ((Airbus_FMA.CurrentPlaneState.flightPhase > FlightPhase.FLIGHT_PHASE_TAKEOFF) || (Airbus_FMA.CurrentPlaneState.flexTemperature == 0)) {
                    return true;
                }
            }
            return false;
        }

        IsActive_MANTHR() {
            if (!Airbus_FMA.CurrentPlaneState.autoPilotThrottleActive && Airbus_FMA.CurrentPlaneState.autoPilotThrottleArmed && (Airbus_FMA.CurrentPlaneState.highestThrottleDetent == ThrottleMode.CLIMB)) {
                return true;
            }
            return false;
        }

        IsActive_THRMCT() {
            if (Airbus_FMA.CurrentPlaneState.autoPilotThrottleActive && (Airbus_FMA.CurrentPlaneState.highestThrottleDetent == ThrottleMode.FLEX_MCT)) {
                return true;
            }
            return false;
        }

        IsActive_THRCLB() {
            if (Airbus_FMA.CurrentPlaneState.autoPilotThrottleActive && Airbus_FMA.CurrentPlaneState.highestThrottleDetent == ThrottleMode.CLIMB && (Column2.IsActive_OPCLB() || Column2.GetModeState_CLB() == Airbus_FMA.MODE_STATE.ENGAGED) && Column2.GetModeState_ALT() != MODE_STATE.ENGAGED && Column2.GetModeState_ALT() != MODE_STATE.CAPTURED && Column2.GetModeState_GS() != MODE_STATE.ENGAGED && Column2.GetModeState_GS() != MODE_STATE.CAPTURED) {
                return true;
            }
            return false;
        }

        IsActive_THRLVR() {
            if (Airbus_FMA.CurrentPlaneState.autoPilotThrottleActive &&
                Airbus_FMA.CurrentPlaneState.highestThrottleDetent == ThrottleMode.AUTO && Airbus_FMA.CurrentPlaneState.anyAutoPilotsActive) {
                return true;
            }
            return false;
        }

        IsActive_THRIDLE() {
            if (Airbus_FMA.CurrentPlaneState.autoPilotThrottleActive && Airbus_FMA.CurrentPlaneState.highestThrottleDetent == ThrottleMode.CLIMB && (Column2.IsActive_OPDES() || Column2.IsActive_DES()) && Column2.GetModeState_ALT() != MODE_STATE.ENGAGED && Column2.GetModeState_ALT() != MODE_STATE.CAPTURED && Column2.GetModeState_GS() != MODE_STATE.ENGAGED && Column2.GetModeState_GS() != MODE_STATE.CAPTURED) {
                return true;
            }
            return false;
        }

        IsActive_AFLOOR() {
            return Airbus_FMA.CurrentPlaneState.autoPilotThrottleLocked;
        }

        IsActive_TOGALK() {
            return Airbus_FMA.CurrentPlaneState.highestThrottleDetent == ThrottleMode.TOGA;
        }

        static IsActive_SpeedCommon() {
            if (Airbus_FMA.CurrentPlaneState.autoPilotThrottleActive) {
                if (!Airbus_FMA.CurrentPlaneState.anyAutoPilotsActive && !Airbus_FMA.CurrentPlaneState.thisFlightDirectorActive) {
                    return true;
                } else {
                    if ((Column2.GetModeState_ALT() == MODE_STATE.ENGAGED) || Column2.IsActive_VS() || Column2.IsActive_FPA() || (Column2.GetModeState_GS() == MODE_STATE.ENGAGED) || VerticalAndLateral.IsActive_FinalApp()) {
                        if (Airbus_FMA.CurrentPlaneState.highestThrottleDetent == ThrottleMode.AUTO) {
                            return true;
                        }
                    }
                }
            }
            return false;
        }

        IsActive_SPEED() {
            if (!Airbus_FMA.CurrentPlaneState.autoPilotMachModeActive && Airbus_FMA.CurrentPlaneState.autoPilotThrottleActive && Airbus_FMA.CurrentPlaneState.radioAltitude > 1.5) {
                if (Column2.GetModeState_ALT() == MODE_STATE.ENGAGED || Column2.GetModeState_ALT() == MODE_STATE.CAPTURED) {
                    return true;
                }
                if (!Column2.IsActive_DES() && !Column2.IsActive_OPDES() && !Column2.IsActive_OPCLB() && Column2.GetModeState_CLB() != Airbus_FMA.MODE_STATE.ENGAGED) {
                    return true;
                }
                if (Column2.GetModeState_GS() == MODE_STATE.ENGAGED || Column2.GetModeState_GS() == MODE_STATE.CAPTURED || Column2.IsActive_VS()) {
                    return true;
                }
                if (!Airbus_FMA.CurrentPlaneState.anyAutoPilotsActive) {
                    return true;
                }
                if (!Airbus_FMA.CurrentPlaneState.thisFlightDirectorActive && Airbus_FMA.CurrentPlaneState.highestThrottleDetent > ThrottleMode.IDLE) {
                    return true;
                }
            }
            return false;
        }

        IsActive_MACH() {
            if (Airbus_FMA.CurrentPlaneState.autoPilotMachModeActive && Airbus_FMA.CurrentPlaneState.autoPilotThrottleActive && Airbus_FMA.CurrentPlaneState.radioAltitude > 1.5) {
                if (Column2.GetModeState_ALT() == MODE_STATE.ENGAGED || Column2.GetModeState_ALT() == MODE_STATE.CAPTURED) {
                    return true;
                }
                if (Simplane.getVerticalSpeed() < 150 && Simplane.getVerticalSpeed() > -150) {
                    return true;
                }
                if (!Airbus_FMA.CurrentPlaneState.anyAutoPilotsActive) {
                    return true;
                }
            }
            return false;
        }

        IsActive_LVRASYM() {
            const leftCLOrMCTFLX = ((Airbus_FMA.CurrentPlaneState.leftThrottleDetent == ThrottleMode.CLIMB) || (Airbus_FMA.CurrentPlaneState.leftThrottleDetent == ThrottleMode.FLEX_MCT));
            const rightCLOrMCTFLX = ((Airbus_FMA.CurrentPlaneState.rightThrottleDetent == ThrottleMode.CLIMB) || (Airbus_FMA.CurrentPlaneState.rightThrottleDetent == ThrottleMode.FLEX_MCT));
            if ((leftCLOrMCTFLX || rightCLOrMCTFLX) && (leftCLOrMCTFLX != rightCLOrMCTFLX)) {
                return true;
            }
            return false;
        }

        IsActive_THRLK() {
            return false;
        }

        IsActive_LVRCLB() {
            const thrustReductionAltitude = Simplane.getThrustReductionAltitude();
            const thrustReductionAltitudeGoaround = SimVar.GetSimVarValue("L:AIRLINER_THR_RED_ALT_GOAROUND", "number");
            const inTakeoff = (Airbus_FMA.CurrentPlaneState.flightPhase == FlightPhase.FLIGHT_PHASE_TAKEOFF);
            const inClimb = (Airbus_FMA.CurrentPlaneState.flightPhase == FlightPhase.FLIGHT_PHASE_CLIMB);
            const inGoAround = (Airbus_FMA.CurrentPlaneState.flightPhase == FlightPhase.FLIGHT_PHASE_GOAROUND) || SimVar.GetSimVarValue("L:A32NX_GOAROUND_PASSED", "bool") === 1;

            // If the thrust reduction altitude is not set, we never show this meessage
            if (!thrustReductionAltitude) {
                return false;
            }

            if (Simplane.getAltitude() > thrustReductionAltitude) {

                if ((inTakeoff || inClimb) && !inGoAround) {
                    return Airbus_FMA.CurrentPlaneState.highestThrottleDetent > ThrottleMode.CLIMB;
                }

            }
            if (Simplane.getAltitude() > thrustReductionAltitudeGoaround) {

                if (inGoAround && Airbus_FMA.CurrentPlaneState.highestThrottleDetent == ThrottleMode.TOGA) {
                    return true;
                }

            }
            return false;
        }

        IsActive_LVRMCT() {
            // Disabling the LVR MCT FMA until it can be properly implemented
            // It requires the pre flight phase to be implemented, which it is not yet
            return false;
            const inTakeoff = (Airbus_FMA.CurrentPlaneState.flightPhase == FlightPhase.FLIGHT_PHASE_TAKEOFF);
            const inGoAround = (Airbus_FMA.CurrentPlaneState.flightPhase == FlightPhase.FLIGHT_PHASE_GOAROUND);

            const oneEngineOff = SimVar.GetSimVarValue("ENG COMBUSTION:1", "Bool") == 0 || SimVar.GetSimVarValue("ENG COMBUSTION:2", "Bool") == 0;

            if (oneEngineOff && (inTakeoff || inGoAround)) {
                const greenDotSpeed = inGoAround ? Simplane.getFMCApprGreenDotSpeed() : Simplane.getFMCGreenDotSpeed();

                const aboveGreenDotSpeed = SimVar.GetSimVarValue("AIRSPEED TRUE", "Number") > greenDotSpeed;
                const throttleBelowFlexMctDetent = Airbus_FMA.CurrentPlaneState.highestThrottleDetent < ThrottleMode.FLEX_MCT;

                if (aboveGreenDotSpeed && throttleBelowFlexMctDetent) {
                    return true;
                }
            }

            return false;
        }

        IsActive_BRKLO() {
            return Airbus_FMA.CurrentPlaneState.autoBreakActivated
                && Airbus_FMA.CurrentPlaneState.autoBreakDecel
                && Airbus_FMA.CurrentPlaneState.autoBreakLevel === 1
                && (Airbus_FMA.CurrentPlaneState.flightPhase > FlightPhase.FLIGHT_PHASE_TAKEOFF
                    || Airbus_FMA.CurrentPlaneState.flightPhase === FlightPhase.FLIGHT_PHASE_PREFLIGHT);
        }
        IsActive_BRKLOARMED() {
            return !(Airbus_FMA.CurrentPlaneState.autoBreakDecel && Airbus_FMA.CurrentPlaneState.autoBreakActivated)
                && Airbus_FMA.CurrentPlaneState.autoBreakLevel === 1
                && Airbus_FMA.CurrentPlaneState.flightPhase > FlightPhase.FLIGHT_PHASE_TAKEOFF;
        }

        IsActive_BRKMED() {
            return Airbus_FMA.CurrentPlaneState.autoBreakActivated
                && Airbus_FMA.CurrentPlaneState.autoBreakDecel
                && Airbus_FMA.CurrentPlaneState.autoBreakLevel === 2
                && (Airbus_FMA.CurrentPlaneState.flightPhase > FlightPhase.FLIGHT_PHASE_TAKEOFF
                    || Airbus_FMA.CurrentPlaneState.flightPhase === FlightPhase.FLIGHT_PHASE_PREFLIGHT);
        }
        IsActive_BRKMEDARMED() {
            return !(Airbus_FMA.CurrentPlaneState.autoBreakDecel && Airbus_FMA.CurrentPlaneState.autoBreakActivated)
                && Airbus_FMA.CurrentPlaneState.autoBreakLevel === 2
                && Airbus_FMA.CurrentPlaneState.flightPhase > FlightPhase.FLIGHT_PHASE_TAKEOFF;
        }

        IsActive_BRKMAX() {
            return Airbus_FMA.CurrentPlaneState.autoBreakDecel &&
                Airbus_FMA.CurrentPlaneState.isOnGround && Airbus_FMA.CurrentPlaneState.autoBreakLevel === 3
                && Airbus_FMA.CurrentPlaneState.flightPhase < FlightPhase.FLIGHT_PHASE_CLIMB;
        }
        IsActive_BRKMAXARMED() {
            return (!Airbus_FMA.CurrentPlaneState.autoBreakDecel)
                && Airbus_FMA.CurrentPlaneState.isOnGround
                && Airbus_FMA.CurrentPlaneState.autoBreakLevel === 3
                && Airbus_FMA.CurrentPlaneState.flightPhase < FlightPhase.FLIGHT_PHASE_CLIMB;
        }

    }

    Column1.ROW_1_STATE = {
        NONE: 0,
        THRMCT: 1,
        THRCLB: 2,
        THRLVR: 3,
        THRIDLE: 4,
        AFLOOR: 5,
        TOGALK: 6,
        SPEED: 7,
        MACH: 8,
        THRLK: 9,
        TOGA: 10,
        SGA: 11,
        BRKLO: 12,
        BRKMED: 13,
        BRKMAX: 14
    };
    Column1.ROW_1_2_STATE = {
        NONE: 0,
        MANTOGA: 1,
        MANFLX: 2,
        MANMCT: 3,
        MANTHR: 4,
        MANSGA: 5
    };
    Column1.ROW_2_STATE = {
        NONE: 0,
        BRKLO: 1,
        BRKMED: 2
    };
    Column1.ROW_3_STATE = {
        NONE: 0,
        LVRCLB: 1,
        LVRMCT: 2,
        LVRASYM: 3,
        BRKMAX: 4
    };
    Airbus_FMA.Column1 = Column1;

    class Column2 extends Airbus_FMA.ColumnBase {
        createSubCellsOnRow1() {
            return true;
        }

        createSubCellsOnRow2() {
            return true;
        }

        getMainDivName() {
            return "Column2";
        }

        initChild() {
            this.setRowHighlightStyle(0, Airbus_FMA.HIGHLIGHT_STYLE.FULL);
            this.currentMode = -1;
            this.currentArmed = -1;
        }

        updateChild(_deltaTime) {
            const mode = SimVar.GetSimVarValue("L:A32NX_FMA_VERTICAL_MODE", "number");
            const armed = SimVar.GetSimVarValue("L:A32NX_FMA_VERTICAL_ARMED", "number");

            if (this.currentMode != mode || this.currentAred != armed) {
                this.currentMode = mode;
                this.currentArmed = armed;

                setTimeout(() => {
                    switch (this.currentMode) {
                        case 10:
                            this.setRowText(0, "ALT", Airbus_FMA.MODE_STATE.ENGAGED);
                            break;

                        case 11:
                            this.setRowText(0, "ALT *", Airbus_FMA.MODE_STATE.CAPTURED);
                            break;

                        case 12:
                            this.setRowText(0, "OP CLB", Airbus_FMA.MODE_STATE.ENGAGED);
                            break;

                        case 13:
                            this.setRowText(0, "OP DES", Airbus_FMA.MODE_STATE.ENGAGED);
                            break;

                        case 14:
                            const speed = SimVar.GetSimVarValue("L:A32NX_AUTOPILOT_VS_SELECTED", "feet per minute");
                            var str;
                            if (speed >= 0) {
                                str = "+" + speed.toFixed(0);
                            } else {
                                str = speed.toFixed(0);
                            }
                            this.setRowMultiText(0, "V/S", Airbus_FMA.MODE_STATE.ENGAGED, str, Airbus_FMA.MODE_STATE.ARMED);
                            break;

                        case 15:
                            let value = SimVar.GetSimVarValue("L:A32NX_AUTOPILOT_FPA_SELECTED", "Degree");
                            const sign = (value > 0) ? "-" : "+";
                            value = Math.min(Math.abs(value), 9.9);
                            var str = sign + value.toFixed(1) + String.fromCharCode(176);
                            this.setRowMultiText(0, "FPA", Airbus_FMA.MODE_STATE.ENGAGED, str, Airbus_FMA.MODE_STATE.ARMED);
                            break;

                        case 20:
                            this.setRowText(0, "ALT CST", Airbus_FMA.MODE_STATE.ENGAGED);
                            break;

                        case 21:
                            this.setRowText(0, "ALT CST*", Airbus_FMA.MODE_STATE.CAPTURED);
                            break;

                        case 22:
                            this.setRowText(0, "CLB", Airbus_FMA.MODE_STATE.ENGAGED);
                            break;

                        case 23:
                            this.setRowText(0, "DES", Airbus_FMA.MODE_STATE.ENGAGED);
                            break;

                        case 30:
                            this.setRowText(0, "G/S *", Airbus_FMA.MODE_STATE.CAPTURED);
                            break;

                        case 31:
                            this.setRowText(0, "G/S", Airbus_FMA.MODE_STATE.ENGAGED);
                            break;

                        case 40:
                        case 41:
                            this.setRowText(0, "SRS", Airbus_FMA.MODE_STATE.ENGAGED);
                            break;

                        case 32: // LAND
                        case 33: // FLARE
                        case 34: // ROLLOUT
                        default:
                            this.setRowText(0, "", Airbus_FMA.MODE_STATE.NONE);
                            break;
                    }

                    const ALT_armed = (this.currentArmed >> 0) & 1;
                    const ALT_CST_armed = (this.currentArmed >> 1) & 1;
                    const CLB_armed = (this.currentArmed >> 2) & 1;
                    const DES_armed = (this.currentArmed >> 3) & 1;
                    const GS_armed = (this.currentArmed >> 4) & 1;

                    if (ALT_armed || ALT_CST_armed) {
                        this.setRowMultiText(1, "ALT", ALT_CST_armed ? Airbus_FMA.MODE_STATE.CONSTRAINED : Airbus_FMA.MODE_STATE.ARMED, GS_armed ? "G/S" : "", GS_armed ? Airbus_FMA.MODE_STATE.ARMED : Airbus_FMA.MODE_STATE.ACTIVE);
                    } else if (CLB_armed) {
                        this.setRowMultiText(1, "CLB", Airbus_FMA.MODE_STATE.ARMED, "", Airbus_FMA.MODE_STATE.ACTIVE);
                    } else if (DES_armed) {
                        this.setRowMultiText(1, "DES", Airbus_FMA.MODE_STATE.ARMED, "", Airbus_FMA.MODE_STATE.ACTIVE);
                    } else if (GS_armed && !(ALT_armed || ALT_CST_armed)) {
                        this.setRowText(1, "G/S", Airbus_FMA.MODE_STATE.ARMED);
                    } else {
                        this.setRowText(1, "", Airbus_FMA.MODE_STATE.NONE);
                    }

                }, 400 + Math.random() * 100);
            }
        }

        GetModeState_SRS() {
            if (Airbus_FMA.CurrentPlaneState.flightPhase == FlightPhase.FLIGHT_PHASE_TAKEOFF && Airbus_FMA.CurrentPlaneState.thisFlightDirectorActive) {
                if (Airbus_FMA.CurrentPlaneState.highestThrottleDetent == ThrottleMode.TOGA) {
                    SRSEnabled = true;
                } else if ((Airbus_FMA.CurrentPlaneState.highestThrottleDetent == ThrottleMode.FLEX_MCT) && (Airbus_FMA.CurrentPlaneState.flexTemperature !== 0)) {
                    SRSEnabled = true;
                }
            } else if (Airbus_FMA.CurrentPlaneState.flightPhase === FlightPhase.FLIGHT_PHASE_GOAROUND) {
                if ((Airbus_FMA.CurrentPlaneState.highestThrottleDetent === ThrottleMode.TOGA
                    || (Airbus_FMA.CurrentPlaneState.highestThrottleDetent === ThrottleMode.FLEX_MCT && Airbus_FMA.CurrentPlaneState.bothEnginesActive))
                    && Airbus_FMA.CurrentPlaneState.flapsHandlePercent !== 0) {
                    SRSEnabled = true;
                    return Airbus_FMA.MODE_STATE.ENGAGED;
                }
            }
            if (Airbus_FMA.CurrentPlaneState.flightPhase != FlightPhase.FLIGHT_PHASE_TAKEOFF) {
                SRSEnabled = false;
            }
            if (SRSEnabled) {
                return Airbus_FMA.MODE_STATE.ENGAGED;
            }
            return Airbus_FMA.MODE_STATE.NONE;
        }

        static GetModeState_ALT() {
            if (Airbus_FMA.CurrentPlaneState.anyAutoPilotsActive && Airbus_FMA.CurrentPlaneState.autoPilotAltitudeLockActive && Airbus_FMA.CurrentPlaneState.thisFlightDirectorActive) {
                const diffAlt = Math.abs(Airbus_FMA.CurrentPlaneState.autoPilotAltitudeLockValue - Airbus_FMA.CurrentPlaneState.altitude);

                if (diffAlt <= Airbus_FMA.Definitions.ALT_ENGAGED_RANGE) {
                    this._lastALTMode = Airbus_FMA.MODE_STATE.ENGAGED;
                } else {
                    if (this._lastALTMode === Airbus_FMA.MODE_STATE.ENGAGED) {
                        if ((diffAlt - Airbus_FMA.Definitions.ALT_ENGAGED_RANGE) > 50) {
                            this._lastALTMode = Airbus_FMA.MODE_STATE.CAPTURED;
                        }
                    } else {
                        this._lastALTMode = Airbus_FMA.MODE_STATE.CAPTURED;
                    }
                }
                return this._lastALTMode;
            }
            return Airbus_FMA.MODE_STATE.NONE;
        }

        static IsActive_VS() {
            if (Airbus_FMA.CurrentPlaneState.autoPilotVerticalSpeedHoldActive && !Airbus_FMA.CurrentPlaneState.autoPilotTRKFPAModeActive && Airbus_FMA.CurrentPlaneState.thisFlightDirectorActive) {
                return true;
            } else {
                return false;
            }
        }

        static IsActive_FPA() {
            if (Airbus_FMA.CurrentPlaneState.autoPilotVerticalSpeedHoldActive && Airbus_FMA.CurrentPlaneState.autoPilotTRKFPAModeActive && Airbus_FMA.CurrentPlaneState.thisFlightDirectorActive) {
                return true;
            } else {
                return false;
            }
        }

        IsActive_EXPCLB() {
            return false;
        }

        IsActive_EXPDES() {
            return false;
        }

        static GetModeState_GS() {
            SimVar.SetSimVarValue("L:GLIDE_SLOPE_CAPTURED", "bool", 0);
            if (Airbus_FMA.CurrentPlaneState.flightPhase != FlightPhase.FLIGHT_PHASE_GOAROUND) {
                if (CurrentPlaneState.bothAutoPilotsInactive && Simplane.getAutoPilotApproachType() == 4 && Simplane.getAutoPilotAPPRActive() && SimVar.GetSimVarValue("L:A32NX_AUTOPILOT_LOC_MODE", "bool") != 1) {
                    return Airbus_FMA.MODE_STATE.ENGAGED;
                }

                if (Simplane.getAutoPilotAPPRHold() && Simplane.getAutoPilotGlideslopeHold() && Airbus_FMA.CurrentPlaneState.thisFlightDirectorActive) {
                    // normally use only for GPS APPROACH APPROACH TYPE = 4 = ILS but the APPROACH TYPE is set too late
                    if (Simplane.getAutoPilotApproachType() == 4 || Simplane.getAutoPilotApproachType() == 0) {
                        if (SimVar.GetSimVarValue("AUTOPILOT GLIDESLOPE ARM", "bool") === 1) {
                            return Airbus_FMA.MODE_STATE.ARMED;
                        }
                        // L:A32NX_OFF_GS is identified too late (see ILSIndicator.js)?
                        if (Simplane.getAutoPilotAPPRCaptured() && Simplane.getAutoPilotAPPRActive() && !Simplane.getAutoPilotGlideslopeActive()) {
                            return Airbus_FMA.MODE_STATE.CAPTURED;
                        }
                        if (Simplane.getAutoPilotGlideslopeActive() && Simplane.getAutoPilotAPPRActive()) {
                            SimVar.SetSimVarValue("L:GLIDE_SLOPE_CAPTURED", "bool", 1);
                            return Airbus_FMA.MODE_STATE.ENGAGED;
                        }

                    }

                }
            }
            return Airbus_FMA.MODE_STATE.NONE;
        }

        static IsArmed_OPCLB_MA() {
            if (Airbus_FMA.CurrentPlaneState.flightPhase == FlightPhase.FLIGHT_PHASE_GOAROUND) {
                return true;
            }
        }

        static IsActive_OPCLB() {
            const accelAlt = SimVar.GetSimVarValue("L:AIRLINER_ACC_ALT_GOAROUND", "number");
            const targetAltitude = Simplane.getAutoPilotAltitudeLockValue("feet");
            const altitude = Simplane.getAltitude();

            if (altitude < accelAlt && Airbus_FMA.CurrentPlaneState.highestThrottleDetent == ThrottleMode.TOGA) {
                return false;
            }

            if ((Simplane.getAutoPilotAltitudeSelected() || Simplane.getAutoPilotAltitudeArmed()) && Airbus_FMA.CurrentPlaneState.thisFlightDirectorActive && Airbus_FMA.CurrentPlaneState.anyAutoPilotsActive) {
                if (altitude < targetAltitude - 100) {
                    return true;
                }
            }

        }

        static IsActive_OPDES() {
            if ((Simplane.getAutoPilotAltitudeSelected() || Simplane.getAutoPilotAltitudeArmed()) && Airbus_FMA.CurrentPlaneState.thisFlightDirectorActive && Airbus_FMA.CurrentPlaneState.anyAutoPilotsActive) {
                const targetAltitude = Simplane.getAutoPilotAltitudeLockValue("feet");
                const altitude = Simplane.getAltitude();
                if (altitude > targetAltitude + 100) {
                    return true;
                }
            }
        }

        static GetModeState_CLB() {
            const accelAlt = SimVar.GetSimVarValue("L:AIRLINER_ACC_ALT_GOAROUND", "number");
            const targetAltitude = Simplane.getAutoPilotAltitudeLockValue("feet");
            const altitude = Simplane.getAltitude();

            if ((Airbus_FMA.CurrentPlaneState.autoPilotAltitudeLockActive && Airbus_FMA.CurrentPlaneState.thisFlightDirectorActive || Simplane.getAutoPilotFLCActive() && Airbus_FMA.CurrentPlaneState.thisFlightDirectorActive) && (Airbus_FMA.CurrentPlaneState.altitude < Airbus_FMA.CurrentPlaneState.autoPilotAltitudeLockValue) && Airbus_FMA.CurrentPlaneState.thisFlightDirectorActive) {

                if (altitude < accelAlt && Airbus_FMA.CurrentPlaneState.highestThrottleDetent == ThrottleMode.TOGA) {
                    return Airbus_FMA.MODE_STATE.NONE;
                }

                if (Airbus_FMA.CurrentPlaneState.flightPhase < FlightPhase.FLIGHT_PHASE_CLIMB) {
                    return Airbus_FMA.MODE_STATE.ARMED;
                } else if (Airbus_FMA.CurrentPlaneState.radioAltitude <= 1.5) {
                    return Airbus_FMA.MODE_STATE.NONE;
                }

                if (altitude < targetAltitude - 100 && !Simplane.getAutoPilotAPPRActive()) {
                    if (Airbus_FMA.CurrentPlaneState.radioAltitude > 1.5 && Simplane.getAutoPilotHeadingManaged()) {
                        return Airbus_FMA.MODE_STATE.ENGAGED;
                    } else {
                        return Airbus_FMA.MODE_STATE.NONE;
                    }
                }
            }
            if (Simplane.getAutoPilotFLCActive() && Airbus_FMA.CurrentPlaneState.thisFlightDirectorActive) {

                if (altitude < accelAlt && Airbus_FMA.CurrentPlaneState.highestThrottleDetent == ThrottleMode.TOGA) {
                    return Airbus_FMA.MODE_STATE.NONE;
                }

                if (altitude < targetAltitude - 100) {
                    if (Airbus_FMA.CurrentPlaneState.radioAltitude <= 1.5) {
                        return Airbus_FMA.MODE_STATE.ENGAGED;
                    } else {
                        return Airbus_FMA.MODE_STATE.NONE;
                    }
                }
            }
            return Airbus_FMA.MODE_STATE.NONE;
        }

        static IsActive_DES() {
            if (Airbus_FMA.CurrentPlaneState.autoPilotAltitudeLockActive && (Airbus_FMA.CurrentPlaneState.altitude > Airbus_FMA.CurrentPlaneState.autoPilotAltitudeLockValue) && Airbus_FMA.CurrentPlaneState.thisFlightDirectorActive) {
                if (Airbus_FMA.CurrentPlaneState.flightPhase == FlightPhase.FLIGHT_PHASE_DESCENT && Simplane.getVerticalSpeed() < -10) {
                    return true;
                }
            }
            if (Simplane.getAutoPilotFLCActive() && Airbus_FMA.CurrentPlaneState.thisFlightDirectorActive) {
                const targetAltitude = Simplane.getAutoPilotAltitudeLockValue("feet");
                const altitude = Simplane.getAltitude();
                if (altitude > targetAltitude + 100) {
                    return true;
                }
            }
            return false;
        }

        IsActive_FinalArmed() {
            if (Simplane.getAutoPilotAPPRHold() && Simplane.getAutoPilotGlideslopeHold() && Simplane.getAutoPilotApproachType() == 10) {
                if (!Simplane.getAutoPilotGlideslopeActive() || !Simplane.getAutoPilotAPPRActive()) {
                    return true;
                }
            }
            return false;
        }
    }

    Column2.ROW_1_STATE = {
        NONE: 0,
        SRS_ARMED: 1,
        SRS_ENGAGED: 2,
        ALT_ENGAGED: 3,
        ALT_CAPTURED: 4,
        ALT_CST_ENGAGED: 5,
        ALT_CST_CAPTURED: 6,
        VS: 7,
        FPA: 8,
        EXP_CLB: 9,
        EXP_DES: 10,
        GS_ENGAGED: 11,
        GS_CAPTURED: 12,
        OP_CLB: 13,
        OP_DES: 14,
        CLB_ENGAGED: 15,
        DES: 16,
        ALT_CRZ: 17,
    };
    Column2.ROW_2_STATE = {
        NONE: 0,
        ALT_ARMED: 1,
        ALT_CST_CONSTRAINED: 2,
        GS_ARMED: 3,
        GS_FINAL_ARMED: 4,
        CLB_ARMED: 5,
        ALT_GS_ARMED: 6
    };
    Column2._lastALTMode = undefined;
    Airbus_FMA.Column2 = Column2;

    class Column3 extends Airbus_FMA.ColumnBase {
        constructor() {
            super(...arguments);
        }

        getMainDivName() {
            return "Column3";
        }

        initChild() {
            this.setRowHighlightStyle(0, Airbus_FMA.HIGHLIGHT_STYLE.FULL);
            this.currentMode = -1;
            this.currentArmed = -1;
        }

        updateChild(_deltaTime) {
            const mode = SimVar.GetSimVarValue("L:A32NX_FMA_LATERAL_MODE", "number");
            const armed = SimVar.GetSimVarValue("L:A32NX_FMA_LATERAL_ARMED", "number");

            if (this.currentMode != mode || this.currentArmed != armed) {
                this.currentMode = mode;
                this.currentArmed = armed;
                setTimeout(() => {
                    switch (this.currentMode) {
                        case 10:
                            this.setRowText(0, "HDG", Airbus_FMA.MODE_STATE.ENGAGED);
                            break;
                        case 11:
                            this.setRowText(0, "TRACK", Airbus_FMA.MODE_STATE.ENGAGED);
                            break;
                        case 20:
                            this.setRowText(0, "NAV", Airbus_FMA.MODE_STATE.ENGAGED);
                            break;
                        case 30:
                            this.setRowText(0, "LOC *", Airbus_FMA.MODE_STATE.ENGAGED);
                            break;
                        case 31:
                            this.setRowText(0, "LOC", Airbus_FMA.MODE_STATE.ENGAGED);
                            break;
                        case 40:
                            this.setRowText(0, "RWY", Airbus_FMA.MODE_STATE.ENGAGED);
                            break;
                        case 41:
                            this.setRowText(0, "RWY TRK", Airbus_FMA.MODE_STATE.ENGAGED);
                            break;
                        case 50:
                            this.setRowText(0, "GA TRK", Airbus_FMA.MODE_STATE.ENGAGED);
                            break;
                        default:
                            this.setRowText(0, "", Airbus_FMA.MODE_STATE.NONE);
                            break;
                    }

                    const NAV_armed = (armed >> 0) & 1;
                    const LOC_armed = (armed >> 1) & 1;

                    if (NAV_armed) {
                        this.setRowText(1, "NAV", Airbus_FMA.MODE_STATE.ARMED);
                    } else if (LOC_armed) {
                        this.setRowText(1, "LOC", Airbus_FMA.MODE_STATE.ARMED);
                    } else {
                        this.setRowText(1, "", Airbus_FMA.MODE_STATE.NONE);
                    }

                }, 400 + Math.random() * 100);
            }
        }

        IsActive_RWY() {
            if (Airbus_FMA.CurrentPlaneState.flightPhase <= FlightPhase.FLIGHT_PHASE_CLIMB && Airbus_FMA.CurrentPlaneState.thisFlightDirectorActive) {
                if (Airbus_FMA.CurrentPlaneState.highestThrottleDetent >= ThrottleMode.FLEX_MCT) {
                    if (Airbus_FMA.CurrentPlaneState.radioAltitude < 30) {
                        return true;
                    }
                }
            }
            return false;
        }

        IsArmed_HDG() {
            //until we have a valid missed approach path to follow, during goaround HDG will be the armed and later activated mode
            if (Airbus_FMA.CurrentPlaneState.flightPhase == FlightPhase.FLIGHT_PHASE_GOAROUND && Airbus_FMA.CurrentPlaneState.flapsHandlePercent != 0) {

                if (SimVar.GetSimVarValue("L:A32NX_GOAROUND_NAV_MODE", "bool") === 1) {
                    return false;
                }

                if (SimVar.GetSimVarValue("L:A32NX_GOAROUND_HDG_MODE", "bool") === 1) {
                    return false;
                }
                return true;
            } else {
                return false;
            }
        }

        IsActive_HDG() {
            // when GOAROUND flightphase is available in future, replace logic instead of checking SRS State
            // when on GOAROUND show GA TRK until HDG SELECTED (HDG) is pressed and returned TRUE here
            // Airbus_FMA.CurrentPlaneState.flightPhase != FlightPhase.FLIGHT_PHASE_GOAROUND || Column2.GetModeState_SRS() == false
            if (Airbus_FMA.CurrentPlaneState.autoPilotHeadingSelectedMode && !Airbus_FMA.CurrentPlaneState.autoPilotTRKFPAModeActive && Airbus_FMA.CurrentPlaneState.thisFlightDirectorActive && Airbus_FMA.CurrentPlaneState.radioAltitude >= 1.5) {
                let retVal = false;

                if (!Simplane.getAutoPilotAPPRActive() && Airbus_FMA.CurrentPlaneState.flightPhase != FlightPhase.FLIGHT_PHASE_GOAROUND) {
                    retVal = true;
                }

                if (SimVar.GetSimVarValue("L:A32NX_GOAROUND_HDG_MODE", "bool") === 1 && Airbus_FMA.CurrentPlaneState.flightPhase == FlightPhase.FLIGHT_PHASE_GOAROUND) {
                    retVal = true;
                }
                return retVal;
            }

            return false;

        }

        IsActive_TRACK() {
            if (Airbus_FMA.CurrentPlaneState.autoPilotHeadingSelectedMode && Airbus_FMA.CurrentPlaneState.autoPilotTRKFPAModeActive && Airbus_FMA.CurrentPlaneState.thisFlightDirectorActive) {
                return true;
            } else {
                return false;
            }
        }

        GetModeState_LOC() {
            if (Airbus_FMA.CurrentPlaneState.thisFlightDirectorActive && Simplane.getAutoPilotApproachType() != 10) {
                if (Simplane.getAutoPilotAPPRActive() && SimVar.GetSimVarValue("L:A32NX_OFF_LOC", "bool") == 1 && Airbus_FMA.CurrentPlaneState.flightPhase != FlightPhase.FLIGHT_PHASE_GOAROUND) {
                    return Airbus_FMA.MODE_STATE.CAPTURED;
                }
                if (Simplane.getAutoPilotAPPRActive() && Airbus_FMA.CurrentPlaneState.flightPhase != FlightPhase.FLIGHT_PHASE_GOAROUND) {
                    return Airbus_FMA.MODE_STATE.ENGAGED;
                } else if (Simplane.getAutoPilotAPPRArm()) {
                    return Airbus_FMA.MODE_STATE.ARMED;
                }
            }
            return Airbus_FMA.MODE_STATE.NONE;
        }

        GetModeState_NAV() {
            if (
                (Airbus_FMA.CurrentPlaneState.flightPhase !== FlightPhase.FLIGHT_PHASE_GOAROUND
                || (Airbus_FMA.CurrentPlaneState.flightPhase === FlightPhase.FLIGHT_PHASE_GOAROUND && Airbus_FMA.CurrentPlaneState.highestThrottleDetent === ThrottleMode.FLEX_MCT))
                && Airbus_FMA.CurrentPlaneState.autoPilotHeadingManagedMode && Airbus_FMA.CurrentPlaneState.thisFlightDirectorActive
            ) {
                if (Airbus_FMA.CurrentPlaneState.radioAltitude >= 30) {
                    return Airbus_FMA.MODE_STATE.ENGAGED;
                } else {
                    return Airbus_FMA.MODE_STATE.ARMED;
                }
            }

            // Future: NAV should not arm/engage during GA when there is no missed approach / secondary fp to follow
            // NAV will engage automatically when there is a missed approach flightplan to follow, or waypoint or whatever
            if (Airbus_FMA.CurrentPlaneState.flightPhase == FlightPhase.FLIGHT_PHASE_GOAROUND && SimVar.GetSimVarValue("L:A32NX_GOAROUND_NAV_MODE", "bool") === 1 && SimVar.GetSimVarValue("L:A320_FCU_SHOW_SELECTED_HEADING", "number") === 0) {
                if (Airbus_FMA.CurrentPlaneState.radioAltitude >= 30) {
                    return Airbus_FMA.MODE_STATE.ENGAGED;
                } else {
                    return Airbus_FMA.MODE_STATE.ARMED;
                }
            }

            return Airbus_FMA.MODE_STATE.NONE;
        }

        IsActive_RWYTRK() {
            if (!this.rwytrkDisabled) {
                if (Airbus_FMA.CurrentPlaneState.flightPhase <= FlightPhase.FLIGHT_PHASE_CLIMB && Airbus_FMA.CurrentPlaneState.thisFlightDirectorActive) {
                    if (Airbus_FMA.CurrentPlaneState.highestThrottleDetent >= ThrottleMode.FLEX_MCT) {
                        if (!Airbus_FMA.CurrentPlaneState.autoPilotHeadingManagedMode) {
                            if (Airbus_FMA.CurrentPlaneState.radioAltitude >= 30) {
                                return true;
                            }
                        }
                    }
                }
            }
            return false;
        }

        IsActive_GATRK() {
            //GATRK only shows up when HDG was preset, not on NAV mode
            //if (Simplane.getAutoPilotAPPRActive() && !Simplane.getAutoPilotThrottleActive() && Simplane.getCurrentFlightPhase() == 6 && Airbus_FMA.CurrentPlaneState.highestThrottleDetent == ThrottleMode.TOGA) {
            if (Airbus_FMA.CurrentPlaneState.flightPhase == FlightPhase.FLIGHT_PHASE_GOAROUND && Airbus_FMA.CurrentPlaneState.flapsHandlePercent != 0) {

                if (SimVar.GetSimVarValue("L:A32NX_GOAROUND_NAV_MODE", "bool") === 1) {
                    return false;
                }

                if (SimVar.GetSimVarValue("L:A32NX_GOAROUND_HDG_MODE", "bool") === 1) {
                    return false;
                }
                SimVar.SetSimVarValue("K:AP_WING_LEVELER_ON", "number", 1);
                return true;
            } else {
                return false;
            }
        }

        GetModeState_APPNAV() {
            if (Simplane.getAutoPilotAPPRHold() && Simplane.getAutoPilotGlideslopeHold() && (!Simplane.getAutoPilotApproachLoaded() || Simplane.getAutoPilotApproachType() !== 4)) {
                if (Simplane.getAutoPilotAPPRActive()) {
                    return Airbus_FMA.MODE_STATE.ENGAGED;
                } else if (Simplane.getAutoPilotAPPRArm()) {
                    return Airbus_FMA.MODE_STATE.ARMED;
                }
            }
            return Airbus_FMA.MODE_STATE.NONE;
        }
    }

    Column3.ROW_1_STATE = {
        NONE: 0,
        RWY: 1,
        HDG: 2,
        TRACK: 3,
        LOC_ENGAGED: 4,
        LOC_CAPTURED: 5,
        NAV_ENGAGED: 6,
        NAV_CAPTURED: 7,
        RWY_TRK: 8,
        GA_TRK: 9,
        APPNAV_ENGAGED: 10
    };
    Column3.ROW_2_STATE = {
        NONE: 0,
        LOC_ARMED: 1,
        NAV_ARMED: 2,
        APPNAV_ARMED: 3
    };
    Airbus_FMA.Column3 = Column3;

    class Column4 extends Airbus_FMA.ColumnBase {
        createSubCellsOnRow3() {
            return true;
        }

        getMainDivName() {
            return "Column4";
        }

        initChild() {
            this.setRowHighlightStyle(0, Airbus_FMA.HIGHLIGHT_STYLE.OPEN_BOTTOM);
            this.setRowHighlightStyle(1, Airbus_FMA.HIGHLIGHT_STYLE.OPEN_TOP);
            this.currentRow1State = Column4.ROW_1_STATE.NONE;
            this.currentRow2State = Column4.ROW_2_STATE.NONE;
            this.currentRow3State = Column4.ROW_3_STATE.NONE;
        }

        updateChild(_deltaTime) {
            this.updateRow1(_deltaTime);
            this.updateRow2(_deltaTime);
            this.updateRow3(_deltaTime);
        }

        updateRow1(_deltaTime) {
            let targetState = Column4.ROW_1_STATE.NONE;

            const isAnyAutopilotActive = SimVar.GetSimVarValue("L:A32NX_AUTOPILOT_ACTIVE", "number");
            const isApproachModeActive = SimVar.GetSimVarValue("L:A32NX_FCU_APPR_MODE_ACTIVE", "number");
            const isAutothrottleActive = Airbus_FMA.CurrentPlaneState.autoPilotThrottleActive;

            if (isApproachModeActive) {
                if (isAnyAutopilotActive
                    && isAutothrottleActive
                    && Airbus_FMA.CurrentPlaneState.radioAltitude < 5000) {
                    targetState = Column4.ROW_1_STATE.CAT_3;
                } else if (isAnyAutopilotActive) {
                    targetState = Column4.ROW_1_STATE.CAT_2;
                } else {
                    targetState = Column4.ROW_1_STATE.CAT_1;
                }
            }

            if (targetState != this.currentRow1State) {
                this.currentRow1State = targetState;
                setTimeout(() => {
                    switch (targetState) {
                        case Column4.ROW_1_STATE.CAT_1: {
                            this.setRowText(0, "CAT 1", Airbus_FMA.MODE_STATE.STATUS);
                            break;
                        }
                        case Column4.ROW_1_STATE.CAT_2: {
                            this.setRowText(0, "CAT 2", Airbus_FMA.MODE_STATE.STATUS);
                            break;
                        }
                        case Column4.ROW_1_STATE.CAT_3: {
                            this.setRowText(0, "CAT 3", Airbus_FMA.MODE_STATE.STATUS);
                            break;
                        }
                        default: {
                            this.setRowText(0, "", Airbus_FMA.MODE_STATE.NONE);
                            break;
                        }
                    }
                }, 500 + Math.random() * 100);
            }
        }

        updateRow2(_deltaTime) {
            let targetState = Column4.ROW_2_STATE.NONE;

            const isAnyAutopilotActive = SimVar.GetSimVarValue("L:A32NX_AUTOPILOT_ACTIVE", "number");
            const isAutopilotActive_1 = SimVar.GetSimVarValue("L:A32NX_AUTOPILOT_1_ACTIVE", "number");
            const isAutopilotActive_2 = SimVar.GetSimVarValue("L:A32NX_AUTOPILOT_2_ACTIVE", "number");
            const isApproachModeActive = SimVar.GetSimVarValue("L:A32NX_FCU_APPR_MODE_ACTIVE", "number");
            const isAutothrottleActive = Airbus_FMA.CurrentPlaneState.autoPilotThrottleActive;

            if (isApproachModeActive
                && isAnyAutopilotActive
                && isAutothrottleActive
                && Airbus_FMA.CurrentPlaneState.radioAltitude < 5000) {
                if (isAutopilotActive_1 && isAutopilotActive_2) {
                    targetState = Column4.ROW_2_STATE.DUAL;
                } else {
                    targetState = Column4.ROW_2_STATE.SINGLE;
                }
            }

            if (targetState != this.currentRow2State) {
                this.currentRow2State = targetState;
                setTimeout(() => {
                    switch (targetState) {
                        case Column4.ROW_2_STATE.SINGLE: {
                            this.setRowText(1, "SINGLE", Airbus_FMA.MODE_STATE.STATUS);
                            break;
                        }
                        case Column4.ROW_2_STATE.DUAL: {
                            this.setRowText(1, "DUAL", Airbus_FMA.MODE_STATE.STATUS);
                            break;
                        }
                        default: {
                            this.setRowText(1, "", Airbus_FMA.MODE_STATE.NONE);
                            break;
                        }
                    }
                }, 500 + Math.random() * 100);
            }
        }

        updateRow3(_deltaTime) {
            let targetState = Column4.ROW_3_STATE.NONE;
            if ((Airbus_FMA.CurrentPlaneState.flightPhase == FlightPhase.FLIGHT_PHASE_DESCENT) || (Airbus_FMA.CurrentPlaneState.flightPhase == FlightPhase.FLIGHT_PHASE_APPROACH)) {
                if (Airbus_FMA.CurrentPlaneState.minimumDescentAltitude > 0) {
                    targetState = Column4.ROW_3_STATE.MDA;
                } else if (Airbus_FMA.CurrentPlaneState.decisionHeight >= 0) {
                    targetState = Column4.ROW_3_STATE.DH;
                } else if (Airbus_FMA.CurrentPlaneState.decisionHeight == -2) {
                    targetState = Column4.ROW_3_STATE.NO_DH;
                } else {
                    targetState = Column4.ROW_3_STATE.NONE;
                }
            }
            if ((targetState != this.currentRow3State) || (targetState == Column4.ROW_3_STATE.DH) || (targetState == Column4.ROW_3_STATE.MDA)) {
                this.currentRow3State = targetState;
                setTimeout(() => {
                    switch (targetState) {
                        case Column4.ROW_3_STATE.DH: {
                            var value = Airbus_FMA.CurrentPlaneState.decisionHeight;
                            this.setRowMultiText(2, "DH", Airbus_FMA.MODE_STATE.STATUS, value.toFixed(0), Airbus_FMA.MODE_STATE.ARMED);
                            break;
                        }
                        case Column4.ROW_3_STATE.NO_DH: {
                            this.setRowText(2, "NO DH", Airbus_FMA.MODE_STATE.STATUS);
                            break;
                        }
                        case Column4.ROW_3_STATE.MDA: {
                            var value = Airbus_FMA.CurrentPlaneState.minimumDescentAltitude;
                            this.setRowMultiText(2, "MDA", Airbus_FMA.MODE_STATE.STATUS, value.toFixed(0), Airbus_FMA.MODE_STATE.ARMED);
                            break;
                        }
                        default: {
                            this.setRowText(2, "", Airbus_FMA.MODE_STATE.NONE);
                            break;
                        }
                    }
                }, 500 + Math.random() * 100);
            }
        }
    }

    Column4.ROW_1_STATE = {
        NONE: 0,
        CAT_1: 1,
        CAT_2: 2,
        CAT_3: 3
    };
    Column4.ROW_2_STATE = {
        NONE: 0,
        SINGLE: 1,
        DUAL: 2
    };
    Column4.ROW_3_STATE = {
        NONE: 0,
        DH: 1,
        NO_DH: 2,
        MDA: 3
    };
    Airbus_FMA.Column4 = Column4;

    class Column5 extends Airbus_FMA.ColumnBase {
        getMainDivName() {
            return "Column5";
        }

        initChild() {
            this.setRowHighlightStyle(0, Airbus_FMA.HIGHLIGHT_STYLE.FULL);
            this.setRowHighlightStyle(2, Airbus_FMA.HIGHLIGHT_STYLE.FULL);
            this.currentRow1State = Column5.ROW_1_STATE.NONE;
            this.currentRow2State = Column5.ROW_2_STATE.NONE;
            this.currentRow3State = Column5.ROW_3_STATE.NONE;
        }

        updateChild(_deltaTime) {
            this.refreshAutoPilotDisplay();
            this.refreshFlightDirectorDisplay();
            this.refreshAutoThrottleDisplay();
        }

        refreshAutoPilotDisplay() {
            let targetState = Column5.ROW_1_STATE.NONE;
            if (Airbus_FMA.CurrentPlaneState.bothAutoPilotsActive) {
                targetState = Column5.ROW_1_STATE.AP_BOTH;
            } else if (Airbus_FMA.CurrentPlaneState.autoPilotActive[0]) {
                targetState = Column5.ROW_1_STATE.AP_1;
            } else if (Airbus_FMA.CurrentPlaneState.autoPilotActive[1]) {
                targetState = Column5.ROW_1_STATE.AP_2;
            }
            if (targetState != this.currentRow1State) {
                this.currentRow1State = targetState;
                setTimeout(() => {
                    switch (targetState) {
                        case Column5.ROW_1_STATE.AP_1: {
                            this.setRowText(0, "AP1", Airbus_FMA.MODE_STATE.ACTIVE);
                            break;
                        }
                        case Column5.ROW_1_STATE.AP_2: {
                            this.setRowText(0, "AP2", Airbus_FMA.MODE_STATE.ACTIVE);
                            break;
                        }
                        case Column5.ROW_1_STATE.AP_BOTH: {
                            this.setRowText(0, "AP1+2", Airbus_FMA.MODE_STATE.ACTIVE);
                            break;
                        }
                        default: {
                            this.setRowText(0, "", Airbus_FMA.MODE_STATE.NONE);
                            break;
                        }
                    }
                }, 600 + Math.random() * 100);
            }
        }

        refreshFlightDirectorDisplay() {
            let targetState = Column5.ROW_2_STATE.NONE;
            if (Airbus_FMA.CurrentPlaneState.autoPilotFlightDirectorActive[0]) {
                targetState = Column5.ROW_2_STATE.FD_1;
            }
            if (Airbus_FMA.CurrentPlaneState.autoPilotFlightDirectorActive[1]) {
                targetState = Column5.ROW_2_STATE.FD_2;
            }
            if (Airbus_FMA.CurrentPlaneState.bothFlightDirectorsActive) {
                targetState = Column5.ROW_2_STATE.FD_BOTH;
            }
            if (targetState != this.currentRow2State) {
                this.currentRow2State = targetState;
                setTimeout(() => {
                    switch (targetState) {
                        case Column5.ROW_2_STATE.FD_1: {
                            this.setRowText(1, "1 FD -", Airbus_FMA.MODE_STATE.ACTIVE);
                            break;
                        }
                        case Column5.ROW_2_STATE.FD_2: {
                            this.setRowText(1, "- FD 2", Airbus_FMA.MODE_STATE.ACTIVE);
                            break;
                        }
                        case Column5.ROW_2_STATE.FD_BOTH: {
                            this.setRowText(1, "1 FD 2", Airbus_FMA.MODE_STATE.ACTIVE);
                            break;
                        }
                        default: {
                            this.setRowText(1, "", Airbus_FMA.MODE_STATE.NONE);
                            break;
                        }
                    }
                }, 600 + Math.random() * 100);
            }
        }

        refreshAutoThrottleDisplay() {
            let targetState = Column5.ROW_3_STATE.NONE;
            if (Airbus_FMA.CurrentPlaneState.autoPilotThrottleActive && Airbus_FMA.CurrentPlaneState.radioAltitude > 1.5) {
                targetState = Column5.ROW_3_STATE.ATHR_ACTIVE;
            } else if (Airbus_FMA.CurrentPlaneState.autoPilotThrottleArmed) {
                targetState = Column5.ROW_3_STATE.ATHR_ARMED;
            }
            if (targetState != this.currentRow3State) {
                this.currentRow3State = targetState;
                setTimeout(() => {
                    switch (targetState) {
                        case Column5.ROW_3_STATE.ATHR_ARMED: {
                            this.setRowText(2, "A/THR", Airbus_FMA.MODE_STATE.ARMED);
                            break;
                        }
                        case Column5.ROW_3_STATE.ATHR_ACTIVE: {
                            this.setRowText(2, "A/THR", Airbus_FMA.MODE_STATE.ACTIVE);
                            break;
                        }
                        default: {
                            this.setRowText(2, "", Airbus_FMA.MODE_STATE.NONE);
                            break;
                        }
                    }
                }, 600 + Math.random() * 100);
            }
        }
    }

    Column5.ROW_1_STATE = {
        NONE: 0,
        AP_1: 1,
        AP_2: 2,
        AP_BOTH: 3
    };
    Column5.ROW_2_STATE = {
        NONE: 0,
        FD_1: 1,
        FD_2: 2,
        FD_BOTH: 3
    };
    Column5.ROW_3_STATE = {
        NONE: 0,
        ATHR_ARMED: 1,
        ATHR_ACTIVE: 2
    };
    Airbus_FMA.Column5 = Column5;
    let SPECIAL_MESSAGE;
    (function (SPECIAL_MESSAGE) {
        SPECIAL_MESSAGE[SPECIAL_MESSAGE["NONE"] = 0] = "NONE";
        SPECIAL_MESSAGE[SPECIAL_MESSAGE["MACH_SEL"] = 1] = "MACH_SEL";
        SPECIAL_MESSAGE[SPECIAL_MESSAGE["SPEED_SEL"] = 2] = "SPEED_SEL";
    })(SPECIAL_MESSAGE = Airbus_FMA.SPECIAL_MESSAGE || (Airbus_FMA.SPECIAL_MESSAGE = {}));

    class VerticalAndLateral {
        constructor(_parent) {
            this.rolloutDelay = 0;
            this.parent = _parent;
            this.divMain = window.document.createElement("div");
            this.divMain.id = "VerticalAndLateral";
            _parent.appendChild(this.divMain);
            this.msg = new Airbus_FMA.Cell(this.divMain, "Msg");
        }

        init() {
            if (this.msg != null) {
                this.msg.init();
                this.msg.setText("", Airbus_FMA.MODE_STATE.NONE);
                this.msg.setVisibility(false);
                Airbus_FMA.VerticalAndLateral.isActive = false;
            }
            this.currentState = VerticalAndLateral.STATE.NONE;
        }

        update(_deltaTime) {
            let targetState = VerticalAndLateral.STATE.NONE;
            if (this.msg != null) {
                const lateralMode = SimVar.GetSimVarValue("L:A32NX_FMA_LATERAL_MODE", "number");
                const verticalMode = SimVar.GetSimVarValue("L:A32NX_FMA_VERTICAL_MODE", "number");
                targetState = VerticalAndLateral.STATE.NONE;
                if (lateralMode == 32 && verticalMode == 32) {
                    targetState = VerticalAndLateral.STATE.LAND;
                } else if (lateralMode == 33 && verticalMode == 33) {
                    targetState = VerticalAndLateral.STATE.FLARE;
                } else if (lateralMode == 34 && verticalMode == 34) {
                    targetState = VerticalAndLateral.STATE.ROLLOUT;
                } else if (!Airbus_FMA.CurrentPlaneState.thisFlightDirectorActive || Simplane.getGroundSpeed() < 40) {
                    targetState = VerticalAndLateral.STATE.NONE;
                }
                if (targetState != this.currentState) {
                    if (this.currentState == VerticalAndLateral.STATE.ROLLOUT) {
                        this.rolloutDelay = 0;
                    }
                    this.currentState = targetState;
                    let str = "";
                    switch (targetState) {
                        case VerticalAndLateral.STATE.FINAL_APP: {
                            str = "FINAL APP";
                            break;
                        }
                        case VerticalAndLateral.STATE.LAND: {
                            str = "LAND";
                            break;
                        }
                        case VerticalAndLateral.STATE.FLARE: {
                            str = "FLARE";
                            break;
                        }
                        case VerticalAndLateral.STATE.ROLLOUT: {
                            str = "ROLLOUT";
                            break;
                        }
                    }
                    if (str.length > 0) {
                        this.msg.setText(str, Airbus_FMA.MODE_STATE.ENGAGED);
                        this.msg.setVisibility(true);
                        Airbus_FMA.VerticalAndLateral.isActive = true;
                    } else {
                        this.msg.setText("", Airbus_FMA.MODE_STATE.NONE);
                        this.msg.setVisibility(false);
                        Airbus_FMA.VerticalAndLateral.isActive = false;
                    }
                }
            }
            this._previousState = this.currentState;
        }

        static IsActive_FinalApp() {
            return Simplane.getAutoPilotGlideslopeActive() && Simplane.getAutoPilotAPPRActive() && Simplane.getAutoPilotApproachType() == 10 && Airbus_FMA.CurrentPlaneState.thisFlightDirectorActive;
        }

        static IsActive_Any() {
            return Airbus_FMA.VerticalAndLateral.isActive;
        }
    }

    VerticalAndLateral.STATE = {
        NONE: 0,
        FINAL_APP: 1,
        LAND: 2,
        FLARE: 3,
        ROLLOUT: 4
    };
    Airbus_FMA.VerticalAndLateral = VerticalAndLateral;

    class SpecialMessages {
        constructor(_parent) {
            this.parent = _parent;
            this.divMain = window.document.createElement("div");
            this.divMain.id = "SpecialMessages";
            _parent.appendChild(this.divMain);
            this.msg1 = new Airbus_FMA.Cell(this.divMain, "Msg1");
            this.msg2 = new Airbus_FMA.Cell(this.divMain, "Msg2");
            this.currentMessage = Airbus_FMA.SPECIAL_MESSAGE.NONE;
        }

        init() {
            if (this.msg1 != null) {
                this.msg1.init();
                this.msg1.setText("", Airbus_FMA.MODE_STATE.NONE);
                this.msg1.setVisibility(false);
            }
            if (this.msg2 != null) {
                this.msg2.init();
                this.msg2.setText("", Airbus_FMA.MODE_STATE.NONE);
                this.msg2.setVisibility(false);
            }
        }

        update(_deltaTime) {
            if ((this.msg1 != null) && (this.msg2 != null)) {
                const requiredMessage = this.getCurrentRequiredMessage();
                if ((requiredMessage != this.currentMessage) || (requiredMessage == Airbus_FMA.SPECIAL_MESSAGE.MACH_SEL) || (requiredMessage == Airbus_FMA.SPECIAL_MESSAGE.SPEED_SEL)) {
                    this.currentMessage = requiredMessage;
                    let useMsgID = 0;
                    switch (this.currentMessage) {
                        case Airbus_FMA.SPECIAL_MESSAGE.MACH_SEL: {
                            useMsgID = 1;
                            var value = Airbus_FMA.CurrentPlaneState.managedAirspeed * 0.0015130718954118;
                            this.msg1.setText("MACH SEL: " + value.toFixed(2), Airbus_FMA.MODE_STATE.ARMED);
                            break;
                        }
                        case Airbus_FMA.SPECIAL_MESSAGE.SPEED_SEL: {
                            useMsgID = 1;
                            var value = Airbus_FMA.CurrentPlaneState.managedAirspeed;
                            this.msg1.setText("SPEED SEL: " + value.toFixed(0), Airbus_FMA.MODE_STATE.ARMED);
                            break;
                        }
                    }
                    if (useMsgID == 1) {
                        this.msg1.setVisibility(true);
                    } else {
                        this.msg1.setVisibility(false);
                        this.msg1.setText("", Airbus_FMA.MODE_STATE.NONE);
                    }
                    if (useMsgID == 2) {
                        this.msg2.setVisibility(true);
                    } else {
                        this.msg2.setVisibility(false);
                        this.msg2.setText("", Airbus_FMA.MODE_STATE.NONE);
                    }
                }
            }
        }

        getCurrentRequiredMessage() {
            if (!Column1.IsActive_SpeedCommon() && (Airbus_FMA.CurrentPlaneState.managedAirspeed > 0) && ((Airbus_FMA.CurrentPlaneState.flightPhase == FlightPhase.FLIGHT_PHASE_TAKEOFF) || (Airbus_FMA.CurrentPlaneState.flightPhase == FlightPhase.FLIGHT_PHASE_CLIMB))) {
                if (Airbus_FMA.CurrentPlaneState.autoPilotMachModeActive) {
                    return Airbus_FMA.SPECIAL_MESSAGE.MACH_SEL;
                } else {
                    return Airbus_FMA.SPECIAL_MESSAGE.SPEED_SEL;
                }
            }
            return Airbus_FMA.SPECIAL_MESSAGE.NONE;
        }
    }

    Airbus_FMA.SpecialMessages = SpecialMessages;
})(Airbus_FMA || (Airbus_FMA = {}));
customElements.define("airbus-fma", Airbus.FMA);
