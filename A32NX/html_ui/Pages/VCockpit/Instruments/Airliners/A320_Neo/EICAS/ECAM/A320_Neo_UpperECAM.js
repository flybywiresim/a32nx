var A320_Neo_UpperECAM;
(function (A320_Neo_UpperECAM) {
    class Definitions {
    }
    Definitions.MIN_GAUGE_EGT = 0;
    Definitions.MAX_GAUGE_EGT = 1200;
    Definitions.MIN_GAUGE_EGT_RED = 1060;
    Definitions.MAX_GAUGE_EGT_RED = 1200;
    Definitions.MIN_GAUGE_N1 = 19.5;
    Definitions.MAX_GAUGE_N1 = 110;
    Definitions.THROTTLE_TO_N1_GAUGE = 100 / Definitions.MAX_GAUGE_N1;
    Definitions.MIN_GAUGE_N1_RED = 100;
    Definitions.MAX_GAUGE_N1_RED = 110;
    Definitions.MAX_FF = 9900;
    A320_Neo_UpperECAM.Definitions = Definitions;
    function createDiv(_id, _class = "", _text = "") {
        const div = document.createElement("div");
        if (_id.length > 0) {
            div.id = _id;
        }
        if (_class.length > 0) {
            div.className = _class;
        }
        if (_text.length > 0) {
            div.textContent = _text;
        }
        return div;
    }
    A320_Neo_UpperECAM.createDiv = createDiv;
    function createSVGText(_text, _class, _x, _y, _alignmentBaseline = "center", _dx = "0%") {
        const textElement = document.createElementNS(Avionics.SVG.NS, "text");
        textElement.textContent = _text;
        textElement.setAttribute("class", _class);
        textElement.setAttribute("x", _x);
        textElement.setAttribute("y", _y);
        textElement.setAttribute("dx", _dx);
        textElement.setAttribute("alignment-baseline", _alignmentBaseline);
        return textElement;
    }
    A320_Neo_UpperECAM.createSVGText = createSVGText;
    function createSVGCircle(_class, _radius, _x, _y) {
        const circleElement = document.createElementNS(Avionics.SVG.NS, "circle");
        circleElement.setAttribute("class", _class);
        circleElement.setAttribute("r", _radius);
        circleElement.setAttribute("cx", _x);
        circleElement.setAttribute("cy", _y);
        return circleElement;
    }
    A320_Neo_UpperECAM.createSVGCircle = createSVGCircle;
    function createRectangle(_class, _x, _y, _width, _height) {
        const rectElement = document.createElementNS(Avionics.SVG.NS, "rect");
        rectElement.setAttribute("x", _x);
        rectElement.setAttribute("y", _y);
        rectElement.setAttribute("width", _width);
        rectElement.setAttribute("height", _height);
        rectElement.setAttribute("class", _class);
        return rectElement;
    }
    A320_Neo_UpperECAM.createRectangle = createRectangle;
    function createSlatParallelogram(_class, _cx, _cy) {
        const parElement = document.createElementNS(Avionics.SVG.NS, "polygon");
        const _xOffset = 3.2;
        const _yOffset = 1;
        const _w = 3;
        const _h = 4;
        const _ox = 1.5;
        const _oy = 1.5;
        const _x = _cx + _xOffset;
        const _y = _cy + _yOffset;
        const _ppoints = String(_x - _w) + "," + String(_y - _oy) + " " + String(_x - _w - _ox) + "," + String(_y + _h) + " " + String(_x + _w) + "," + String(_y + _oy) + " " + String(_x + _w + _ox) + "," + String(_y - _h);
        parElement.setAttribute("class", _class);
        parElement.setAttribute("points", _ppoints);
        return parElement;
    }
    A320_Neo_UpperECAM.createSlatParallelogram = createSlatParallelogram;
    function createFlapParallelogram(_class, _cx, _cy) {
        const parElement = document.createElementNS(Avionics.SVG.NS, "polygon");
        const _xOffset = -1;
        const _yOffset = -0.5;
        const _w = 3.5;
        const _h = 3.5;
        const _ox = 2;
        const _oy = -2;
        const _x = _cx + _xOffset;
        const _y = _cy + _yOffset;
        const _ppoints = String(_x - _w) + "," + String(_y - _oy) + " " + String(_x - _w - _ox) + "," + String(_y - _h) + " " + String(_x + _w) + "," + String(_y + _oy) + " " + String(_x + _w) + "," + String(_y + _h);
        parElement.setAttribute("class", _class);
        parElement.setAttribute("points", _ppoints);
        return parElement;
    }
    A320_Neo_UpperECAM.createFlapParallelogram = createFlapParallelogram;
    class Display extends Airliners.EICASTemplateElement {
        constructor() {
            super();
            this.isInitialised = false;
            this.allPanels = [];
            this.simVarCache = {};
            this.frameCount = 0;
            this._aircraft = Aircraft.A320_NEO;
            this.toInhibitTimer = new NXLogic_ConfirmNode(3);
            this.ldgInhibitTimer = new NXLogic_ConfirmNode(3);
            this.iceSevereDetectedTimer = new NXLogic_ConfirmNode(40, false);
            this.iceDetectedTimer1 = new NXLogic_ConfirmNode(40, false);
            this.iceDetectedTimer2 = new NXLogic_ConfirmNode(5);
            this.iceNotDetTimer1 = new NXLogic_ConfirmNode(60);
            this.iceNotDetTimer2 = new NXLogic_ConfirmNode(130);
            this.predWsMemo = new NXLogic_MemoryNode(true);
        }
        get templateID() {
            return "UpperECAMTemplate";
        }
        connectedCallback() {
            super.connectedCallback();
            TemplateElement.call(this, this.init.bind(this));
        }
        getCachedSimVar(_simvar, _unit) {
            const key = `${_simvar}:${_unit}`;
            if (this.simVarCache[key] != null) {
                return this.simVarCache[key];
            }
            const value = SimVar.GetSimVarValue(_simvar, _unit);
            this.simVarCache[key] = value;
            return value;
        }
        getADIRSMins() {
            const secs = this.getCachedSimVar("L:A320_Neo_ADIRS_TIME", "seconds");
            const mins = Math.ceil(secs / 60);
            if (secs > 0 && this.getCachedSimVar("L:A320_Neo_ADIRS_IN_ALIGN", "Bool")) {
                return mins;
            } else {
                return -1;
            }
        }
        isInFlightPhase(...flightPhases) {
            return flightPhases.indexOf(this.fwcFlightPhase) !== -1;
        }
        engineFailed(_engine) {
            return (this.getCachedSimVar("ENG FAILED:" + _engine, "Bool") == 1) && !this.getCachedSimVar("ENG ON FIRE:" + _engine) && !Simplane.getIsGrounded();
        }
        engineShutdown(_engine) {
            return (this.getCachedSimVar("TURB ENG N1:" + _engine, "Percent") < 15 || this.getCachedSimVar("FUELSYSTEM VALVE SWITCH:" + (_engine), "Bool") == 0) && !Simplane.getIsGrounded();
        }
        isEngineRunning(_engine) {
            return this.getCachedSimVar(`ENG N1 RPM:${_engine}`, "Percent") >= 15;
        }
        getEngineFailActions(_engine) {
            return [
                {
                    style: "action",
                    message: "ENG MODE SEL",
                    action: "IGN",
                    isCompleted: () => {
                        return this.getCachedSimVar("L:XMLVAR_ENG_MODE_SEL", "Enum") == 2;
                    }
                },
                {
                    style: "action",
                    message: "THR LVR " + _engine,
                    action: "IDLE",
                    isCompleted: () => {
                        return this.getCachedSimVar("GENERAL ENG THROTTLE LEVER POSITION:" + _engine, "Enum") == 0;
                    }
                },
                {
                    style: "remark",
                    message: "IF NO RELIGHT AFTER 30S",
                    isCompleted: () => {
                        return this.getCachedSimVar("FUELSYSTEM VALVE SWITCH:" + (_engine), "Bool") == 0;
                    }
                },
                {
                    style: "action",
                    message: "ENG MASTER " + _engine,
                    action: "OFF",
                    isCompleted: () => {
                        return this.getCachedSimVar("FUELSYSTEM VALVE SWITCH:" + (_engine), "Bool") == 0;
                    }
                },
                {
                    style: "remark-indent",
                    message: "IF DAMAGE",
                    isCompleted: () => {
                        return this.getCachedSimVar("L:A32NX_FIRE_BUTTON_ENG" + _engine, "Bool") == 1;
                    }
                },
                {
                    style: "action",
                    message: `ENG ${_engine} FIRE P/B`,
                    action: "PUSH",
                    isCompleted: () => {
                        return this.getCachedSimVar("L:A32NX_FIRE_BUTTON_ENG" + _engine, "Bool") == 1;
                    }
                },
                {
                    style: "remark-indent",
                    message: "IF NO DAMAGE",
                },
                {
                    style: "action",
                    message: `ENG ${_engine} RELIGHT`,
                    action: "CONSIDER",
                }
            ];
        }
        getEngineFailSecondaryFailures() {
            return ["BLEED", "ELEC"];
        }
        getEngineShutdownActions() {
            return [
                {
                    style: "remark-indent",
                    message: "IF NO FUEL LEAK"
                },
                {
                    style: "action",
                    message: "IMBALANCE",
                    action: "MONITOR"
                },
                {
                    style: "action",
                    message: "TCAS MODE SEL",
                    action: "TA"
                },
                {
                    style: "cyan",
                    message: "AVOID ICING CONDITIONS"
                }
            ];
        }
        getEngineShutdownSecondaryFailures() {
            return ["BLEED", "ELEC", "HYD", "STS"];
        }
        getEngineFireActions(_engine) {
            return [
                {
                    style: "action",
                    message: "THR LEVER " + _engine,
                    action: "IDLE",
                    isCompleted: () => {
                        return this.getCachedSimVar("GENERAL ENG THROTTLE LEVER POSITION:" + _engine, "Enum") == 0;
                    }
                },
                {
                    style: "action",
                    message: "ENG MASTER " + _engine,
                    action: "OFF",
                    isCompleted: () => {
                        return this.getCachedSimVar("FUELSYSTEM VALVE SWITCH:" + (_engine), "Bool") == 0;
                    }
                },
                {
                    style: `action`,
                    message: `ENG ${_engine} FIRE P/B`,
                    action: "PUSH",
                    isCompleted: () => {
                        return this.getCachedSimVar("L:A32NX_FIRE_BUTTON_ENG" + _engine, "Bool") == 1;
                    }
                },
                {
                    style: "action-timer",
                    id: `eng${_engine}_agent1`,
                    message: "AGENT 1",
                    action: "DISCH",
                    timerMessage: "AGENT1",
                    duration: 10,
                    isCompleted:() => {
                        return this.getCachedSimVar(`L:A32NX_Fire_ENG${_engine}_Agent1_Discharge`, "Bool") == 1;
                    }
                },
                {
                    style: "action-timer",
                    id: `eng${_engine}_agent2`,
                    message: "AGENT 2",
                    action: "DISCH",
                    timerMessage: "AGENT2",
                    duration: 10,
                    isCompleted:() => {
                        return this.getCachedSimVar(`L:A32NX_Fire_ENG${_engine}_Agent2_Discharge`, "Bool") == 1;
                    }
                },
                {
                    style: "action",
                    message: "ATC",
                    action: "NOTIFY"
                },
            ];
        }
        getEngineFireGroundActions(_engine) {
            return [
                {
                    style: "remark",
                    message: "WHEN A/C IS STOPPED"
                },
                {
                    style: "action",
                    message: "PARKING BRK",
                    action: "ON",
                    isCompleted: () => {
                        return this.getCachedSimVar("BRAKE PARKING INDICATOR", "Bool") == 1;
                    }
                },
                {
                    style: "action",
                    message: "ATC",
                    action: "NOTIFY",
                },
                {
                    style: "action",
                    message: "CABIN CREW",
                    action: "ALERT"
                },
                {
                    style: "action",
                    message: `ENG ${_engine} FIRE P/B`,
                    action: "PUSH",
                    isCompleted: () => {
                        return this.getCachedSimVar("L:A32NX_FIRE_BUTTON_ENG" + _engine, "Bool") == 1;
                    }
                },
                {
                    style: "action",
                    message: "AGENT 1",
                    action: "DISCH",
                    isCompleted:() => {
                        return this.getCachedSimVar(`L:A32NX_Fire_ENG${_engine}_Agent1_Discharge`, "Bool") == 1;
                    }
                },
                {
                    style: "action",
                    message: "AGENT 2",
                    action: "DISCH",
                    isCompleted:() => {
                        return this.getCachedSimVar(`L:A32NX_Fire_ENG${_engine}_Agent2_Discharge`, "Bool") == 1;
                    }
                }
            ];
        }
        secondaryFailureActive(_failure) {
            const secondaryFailures = this.leftEcamMessagePanel.secondaryFailures;
            //if (secondaryFailures.length < 2) return false;
            for (let i = this.clearedSecondaryFailures.length; i < secondaryFailures.length; i++) {
                if (secondaryFailures[i] == _failure) {
                    return true;
                }
            }
            return false;
        }
        getActiveSecondaryFailure() {
            for (const secondaryFailure of this.leftEcamMessagePanel.secondaryFailures) {
                if (!this.clearedSecondaryFailures.includes(secondaryFailure)) {
                    return secondaryFailure;
                }
            }
        }
        init() {
            this.enginePanel = new A320_Neo_UpperECAM.EnginePanel(this, "EnginesPanel");
            this.infoTopPanel = new A320_Neo_UpperECAM.InfoTopPanel(this, "InfoTopPanel");
            this.flapsPanel = new A320_Neo_UpperECAM.FlapsPanel(this, "FlapsPanel");
            this.overflowArrow = this.querySelector("#overflow-arrow");
            this.statusReminder = this.querySelector("#sts-reminder");
            this.activeTakeoffConfigWarnings = [];
            this.clearedSecondaryFailures = [];
            this.ecamMessages = {
                failures: [
                    {
                        name: "ENG",
                        messages: [
                            {
                                message: "DUAL FAILURE",
                                level: 3,
                                landASAP: true,
                                isActive: () => {
                                    return ((this.engineShutdown(1) || this.engineFailed(1)) && (this.engineShutdown(2) || this.engineFailed(2)) && this.getCachedSimVar("AIRSPEED INDICATED", "knots") > 80);
                                },
                                inopSystems: [
                                    "G_HYD",
                                    "Y_HYD",
                                    "R_AIL",
                                    "IR_2",
                                    "IR_3",
                                    "ELAC_2",
                                    "YAW_DAMPER",
                                    "ATHR",
                                    "NW_STEER",
                                    "LG_RETRACT",
                                    "FCTL_PROT",
                                    "REVERSER_1",
                                    "REVERSER_2",
                                    "RA_1",
                                    "RA_2",
                                    "SEC_2",
                                    "SEC_3",
                                    "ACALL_OUT",
                                    "FUEL PUMPS",
                                    "AUTO_BRK",
                                    "CAB_PR_1",
                                    "CAB_PR_2",
                                    "STABILIZER",
                                    "ADR_2",
                                    "ADR_3",
                                    "SPLR_1245",
                                    "FLAPS",
                                    "AP_1",
                                    "AP_2",
                                    "ANTI_SKID",
                                    "CAT_2",
                                    "PACK_1",
                                    "PACK_2"
                                ],
                                actions: [
                                    {
                                        style: "action",
                                        message: "EMER ELEC PWR",
                                        action: "MAN ON"
                                    },
                                    {
                                        style: "action",
                                        message: "OPT RELIGHT SPD",
                                        action: "280KT"
                                    },
                                    {
                                        style: "action",
                                        message: "APU",
                                        action: "START",
                                        isCompleted: () => {
                                            return this.getCachedSimVar("L:A32NX_OVHD_APU_START_PB_IS_AVAILABLE", "Bool") === 1;
                                        }
                                    },
                                    {
                                        style: "action",
                                        message: "THR LEVERS",
                                        action: "IDLE",
                                        isCompleted: () => {
                                            return this.getCachedSimVar("GENERAL ENG THROTTLE LEVER POSITION:1", "Enum") == 0 && this.getCachedSimVar("GENERAL ENG THROTTLE LEVER POSITION:2", "Enum") == 0;
                                        }
                                    },
                                    {
                                        style: "cyan",
                                        message: "GLDG DIST: 2NM/1000FT"
                                    },
                                    {
                                        style: "action",
                                        message: "DIVERSION",
                                        action: "INITIATE"
                                    }
                                ]
                            }
                        ]
                    },
                    //Airborne
                    {
                        name: "OVERSPEED",
                        messages: [
                            {
                                message: "",
                                level: 3,
                                flightPhasesInhib: [2, 3, 4, 8, 9, 10],
                                isActive: () => Simplane.getIndicatedSpeed() > (this.getCachedSimVar("L:A32NX_SPEEDS_VMAX", "number") + 4),
                            },
                        ]
                    },
                    {
                        name: "ENG 1 FIRE",
                        messages: [
                            {
                                message: "",
                                level: 3,
                                landASAP: true,
                                page: "ENG",
                                isActive: () => (
                                    this.isInFlightPhase(5, 6, 7) &&
                                    (this.getCachedSimVar("L:A32NX_FIRE_TEST_ENG1", "Bool") || this.getCachedSimVar("ENG ON FIRE:1", "Bool"))
                                ),
                                actions: this.getEngineFireActions(1)
                            },
                        ]
                    },
                    {
                        name: "ENG 2 FIRE",
                        messages: [
                            {
                                message: "",
                                level: 3,
                                landASAP: true,
                                page: "ENG",
                                isActive: () => (
                                    this.isInFlightPhase(5, 6, 7) &&
                                    (this.getCachedSimVar("L:A32NX_FIRE_TEST_ENG2", "Bool") || this.getCachedSimVar("ENG ON FIRE:2", "Bool"))
                                ),
                                actions: this.getEngineFireActions(2)
                            },
                        ]
                    },
                    //Ground
                    {
                        name: "ENG 1 FIRE",
                        messages: [
                            {
                                message: "",
                                level: 3,
                                isActive: () => (
                                    !this.isInFlightPhase(5, 6, 7) &&
                                    (this.getCachedSimVar("L:A32NX_FIRE_TEST_ENG1", "Bool") || this.getCachedSimVar("ENG ON FIRE:1", "Bool"))
                                ),
                                actions: this.getEngineFireGroundActions(1)
                            },
                        ]
                    },
                    {
                        name: "ENG 2 FIRE",
                        messages: [
                            {
                                message: "",
                                level: 3,
                                isActive: () => (
                                    !this.isInFlightPhase(5, 6, 7) &&
                                    (this.getCachedSimVar("L:A32NX_FIRE_TEST_ENG2", "Bool") || this.getCachedSimVar("ENG ON FIRE:2", "Bool"))
                                ),
                                actions: this.getEngineFireGroundActions(2)
                            },
                        ]
                    },
                    {
                        name: "APU FIRE",
                        messages: [
                            {
                                message: "",
                                level: 3,
                                isActive: () => {
                                    return this.getCachedSimVar("L:A32NX_FIRE_TEST_APU", "Bool");
                                },
                                actions: [
                                    {
                                        style: "remark",
                                        message: "WHEN A/C IS STOPPED"
                                    },
                                    {
                                        style: "action",
                                        message: "PARKING BRK",
                                        action: "ON",
                                        isCompleted: () => {
                                            return this.getCachedSimVar("BRAKE PARKING INDICATOR", "Bool") == 1;
                                        }
                                    },
                                    {
                                        style: "action",
                                        message: "ATC",
                                        action: "NOTIFY",
                                    },
                                    {
                                        style: "action",
                                        message: "CABIN CREW",
                                        action: "ALERT"
                                    },
                                    {
                                        style: "action",
                                        message: "APU FIRE P/B",
                                        action: "PUSH"
                                    },
                                    {
                                        style: "action",
                                        message: "AGENT",
                                        action: "DISCH"
                                    },
                                ]
                            },
                        ]
                    },
                    {
                        name: "CARGO SMOKE",
                        messages: [
                            {
                                message: "",
                                level: 3,
                                flightPhasesInhib: [4, 5, 7, 8],
                                isActive: () => {
                                    return this.getCachedSimVar("L:A32NX_FIRE_TEST_CARGO", "Bool");
                                },
                                actions: [
                                    {
                                        style: "remark",
                                        message: "WHEN A/C IS STOPPED"
                                    },
                                    {
                                        style: "action",
                                        message: "PARKING BRK",
                                        action: "ON",
                                        isCompleted: () => {
                                            return this.getCachedSimVar("BRAKE PARKING INDICATOR", "Bool") == 1;
                                        }
                                    },
                                    {
                                        style: "action",
                                        message: "ATC",
                                        action: "NOTIFY",
                                    },
                                    {
                                        style: "action",
                                        message: "CABIN CREW",
                                        action: "ALERT"
                                    },
                                    {
                                        style: "action",
                                        message: "AGENT 1",
                                        action: "DISCH"
                                    },
                                    {
                                        style: "action",
                                        message: "AGENT 2",
                                        action: "DISCH"
                                    },
                                ]
                            },
                        ]
                    },
                    {
                        name: "CONFIG",
                        messages: [
                            {
                                id: "config_slats",
                                message: "",
                                level: 3,
                                flightPhasesInhib: [5, 6, 7, 8], // TODO
                                isActive: () => this.activeTakeoffConfigWarnings.includes("slats"),
                                alwaysShowCategory: true,
                                actions: [
                                    {
                                        style: "fail-3",
                                        message: "SLATS NOT IN T.O CONFIG"
                                    }
                                ]
                            },
                            {
                                id: "config_flaps",
                                message: "",
                                level: 3,
                                flightPhasesInhib: [5, 6, 7, 8], // TODO
                                isActive: () => this.activeTakeoffConfigWarnings.includes("flaps"),
                                alwaysShowCategory: true,
                                actions: [
                                    {
                                        style: "fail-3",
                                        message: "FLAPS NOT IN T.O CONFIG"
                                    }
                                ]
                            },
                            {
                                id: "config_spd_brk",
                                message: "",
                                level: 3,
                                flightPhasesInhib: [5, 6, 7, 8], // TODO
                                isActive: () => this.activeTakeoffConfigWarnings.includes("spd_brk"),
                                alwaysShowCategory: true,
                                actions: [
                                    {
                                        style: "fail-3",
                                        message: "SPD BRK NOT RETRACTED"
                                    }
                                ]
                            },
                            {
                                id: "config_park_brake",
                                message: "",
                                level: 3,
                                flightPhasesInhib: [1, 4, 5, 6, 7, 8, 9, 10], // TODO
                                isActive: () => {
                                    return this.activeTakeoffConfigWarnings.includes("park_brake") && Simplane.getIsGrounded();
                                },
                                alwaysShowCategory: true,
                                actions: [
                                    {
                                        style: "fail-3",
                                        message: "PARK BRAKE ON"
                                    }
                                ]
                            },
                        ]
                    },
                    {
                        name: "CAB PR",
                        messages: [
                            {
                                message: "EXCESS CAB ALT",
                                level: 3,
                                flightPhasesInhib: [1, 2, 3, 4, 5, 7, 8, 9, 10],
                                isActive: () => (
                                    !Simplane.getIsGrounded() &&
                                    this.getCachedSimVar("PRESSURIZATION CABIN ALTITUDE", "feet") > 10000
                                ),
                                actions: [
                                    {
                                        style: "action",
                                        message: "CREW OXY MASK",
                                        action: "ON"
                                    },
                                    {
                                        style: "action",
                                        message: "DESCENT",
                                        action: "INITIATE",
                                        isCompleted: () => {
                                            return this.getCachedSimVar("VERTICAL SPEED", "feet per minute") < -100;
                                        }
                                    },
                                    {
                                        style: "remark",
                                        message: "IF RAPID DECOMPRESSION"
                                    },
                                    {
                                        style: "action-underline",
                                        message: "EMER DESCENT FL 100/MEA",
                                    },
                                    {
                                        style: "action",
                                        message: "SPEED BRK",
                                        action: "FULL",
                                        isCompleted: () => {
                                            return this.getCachedSimVar("SPOILERS HANDLE POSITION", "Position") > 0.95;
                                        }
                                    },
                                    {
                                        style: "action",
                                        message: "SPD",
                                        action: "MAX/APPROPRIATE"
                                    },
                                    {
                                        style: "remark",
                                        message: "IF CAB ALT>14000FT"
                                    },
                                    {
                                        style: "action",
                                        message: "PAX OXY MASKS",
                                        action: "MAN ON"
                                    },
                                ]
                            }
                        ]
                    },
                    {
                        name: "ENG 1",
                        messages: [
                            {
                                message: "<span class='boxed'>FAIL</span>",
                                level: 2,
                                landASAP: true,
                                inopSystems: [],
                                secondaryFailures: this.getEngineFailSecondaryFailures(),
                                page: "ENG",
                                isActive: () => (
                                    this.fwcFlightPhase !== 1 &&
                                    this.fwcFlightPhase !== 10 &&
                                    this.engineFailed(1) &&
                                    !this.engineFailed(2)
                                ),
                                actions: this.getEngineFailActions(1)
                            },
                            {
                                message: "<span class='boxed'>SHUT DOWN</span>",
                                level: 2,
                                landASAP: true,
                                inopSystems: [
                                    "ENG_1",
                                    "WING_A_ICE",
                                    "CAT_3_DUAL",
                                    "ENG_1_BLEED",
                                    "PACK_1",
                                    "MAIN_GALLEY",
                                    "GEN_1",
                                    "G_ENG_1_PUMP"
                                ],
                                secondaryFailures: this.getEngineShutdownSecondaryFailures(),
                                page: "ENG",
                                isActive: () => {
                                    return this.engineShutdown(1);
                                },
                                actions: this.getEngineShutdownActions()
                            }
                        ]
                    },
                    {
                        name: "ENG 2",
                        messages: [
                            {
                                message: "<span class='boxed'>FAIL</span>",
                                level: 2,
                                landASAP: true,
                                inopSystems: [],
                                secondaryFailures: this.getEngineFailSecondaryFailures(),
                                page: "ENG",
                                isActive: () => (
                                    this.fwcFlightPhase !== 1 &&
                                    this.fwcFlightPhase !== 10 &&
                                    !this.engineFailed(1) &&
                                    this.engineFailed(2)
                                ),
                                actions: () => {
                                    return this.getEngineFailActions(2);
                                }
                            },
                            {
                                message: "<span class='boxed'>SHUT DOWN</span>",
                                level: 2,
                                landASAP: true,
                                inopSystems: [
                                    "ENG_2",
                                    "WING_A_ICE",
                                    "CAT_3_DUAL",
                                    "ENG_2_BLEED",
                                    "PACK_2",
                                    "MAIN_GALLEY",
                                    "GEN_2",
                                    "G_ENG_1_PUMP"
                                ],
                                secondaryFailures: this.getEngineShutdownSecondaryFailures(),
                                page: "ENG",
                                isActive: () => {
                                    return this.engineShutdown(2);
                                },
                                actions: this.getEngineShutdownActions()
                            }
                        ]
                    },
                    {
                        name: "BRAKES",
                        messages : [
                            {
                                message: "HOT",
                                id: "brakes_hot",
                                level: 2,
                                flightPhasesInhib: [4, 8, 9, 10],
                                page: "WHEEL",
                                isActive: () => SimVar.GetSimVarValue("L:A32NX_BRAKES_HOT", "Bool"),
                                actions: [
                                    {
                                        style: "remark",
                                        message: "IF PERF PERMITS",
                                        isCompleted: () => {
                                            return Simplane.getIsGrounded();
                                        }
                                    },
                                    {
                                        style: "action",
                                        message: "PARK BRK",
                                        action: "PREFER CHOCKS",
                                        isCompleted: () => {
                                            return !this.isInFlightPhase(1, 10);
                                        }
                                    },
                                    {
                                        style: "action",
                                        message: "MAX SPEED",
                                        action: "250/.60",
                                        isCompleted: () => {
                                            return Simplane.getIsGrounded();
                                        }
                                    },
                                    {
                                        style: "action",
                                        message: "BRK FAN",
                                        action: "ON",
                                        isCompleted: () => {
                                            return !(this.isInFlightPhase(1, 2) && !SimVar.GetSimVarValue("L:A32NX_BRAKE_FAN", "Bool"));
                                        },
                                    },
                                    {
                                        style: "cyan",
                                        message: "&nbsp;-DELAY T.O FOR COOL",
                                        isCompleted: () => {
                                            return !Simplane.getIsGrounded();
                                        }
                                    },
                                    {
                                        style: "action",
                                        message: "L/G",
                                        action: "DN FOR COOL",
                                        isCompleted: () => {
                                            return Simplane.getIsGrounded();
                                        }
                                    },
                                    {
                                        style: "remark",
                                        message: "FOR L/G RETRACTION",
                                        isCompleted: () => {
                                            return Simplane.getIsGrounded();
                                        }
                                    },
                                    {
                                        style: "action",
                                        message: "MAX SPEED",
                                        action: "220/.54",
                                        isCompleted: () => {
                                            return Simplane.getIsGrounded();
                                        }
                                    }
                                ]
                            },
                            {
                                message: "PARK BRK ON",
                                id: "brakes_park_brk_on",
                                level: 2,
                                flightPhasesInhib: [1, 2, 3, 4, 5, 8, 9, 10],
                                isActive: () => (
                                    this.isInFlightPhase(8, 6, 7) && SimVar.GetSimVarValue("BRAKE PARKING INDICATOR", "Bool")
                                ),
                                actions: [
                                    {
                                        style: "action",
                                        message: "PARK BRK",
                                        action: "OFF",
                                        isCompleted: () => !SimVar.GetSimVarValue("BRAKE PARKING INDICATOR", "Bool"),
                                    },
                                ]
                            },
                            {
                                message: "A/SKID N/WS OFF",
                                id: "brakes_askid_nws_off",
                                level: 2,
                                flightPhasesInhib: [4, 5],
                                page: "WHEEL",
                                inopSystems: [
                                    "CAT_3_DUAL",
                                    "ANTI_SKID",
                                    "NS_STEER",
                                    "NORM_BRK",
                                    "AUTO_BRK",
                                    "ASKID_NWS" // only as trigger for STS messages
                                ],
                                isActive: () => (
                                    !SimVar.GetSimVarValue("ANTISKID BRAKES ACTIVE", "Bool")
                                ),
                                actions: [
                                    {
                                        style: "cyan",
                                        message: "&nbsp;MAX BRK PR......1000PSI"
                                    }
                                ],
                            }
                        ]
                    },
                    {
                        name: "SEVERE ICE",
                        messages: [
                            {
                                message: "DETECTED",
                                level: 2,
                                actions: [
                                    {
                                        style: "action",
                                        message: "WING ANTI ICE",
                                        action: "ON",
                                        isCompleted: () => this.getCachedSimVar("STRUCTURAL DEICE SWITCH", "Bool"),
                                    },
                                    {
                                        style: "action",
                                        message: "ENG MODE SEL",
                                        action: "IGN",
                                        isCompleted: () => this.getCachedSimVar("L:XMLVAR_ENG_MODE_SEL", "Enum") == 2,
                                    }
                                ],
                                flightPhasesInhib: [3, 4, 5, 7, 8],
                                isActive: () => this.iceSevereDetectedTimer.read(),
                            },
                        ]
                    },
                    {
                        name: "ANTI ICE",
                        messages: [
                            {
                                message: "ICE DETECTED",
                                level: 2,
                                actions: [
                                    {
                                        style: "action",
                                        message: "ENG 1 ANTI ICE",
                                        action: "ON",
                                        isCompleted: () => this.getCachedSimVar("ENG ANTI ICE:1", "Bool"),
                                    },
                                    {
                                        style: "action",
                                        message: "ENG 2 ANTI ICE",
                                        action: "ON",
                                        isCompleted: () => this.getCachedSimVar("ENG ANTI ICE:2", "Bool"),
                                    }
                                ],
                                flightPhasesInhib: [3, 4, 5, 7, 8],
                                isActive: () => this.iceDetectedTimer2.read(),
                            },
                        ]
                    },
                    {
                        name: "NAV",
                        messages: [
                            {
                                message: "TCAS FAULT",
                                level: 2,
                                inopSystems: [
                                    "TCAS"
                                ],
                                flightPhasesInhib: [3, 4, 5, 7, 8],
                                isActive: () => !this.isInFlightPhase(1, 10) && this.getCachedSimVar("L:A320_Neo_ADIRS_STATE", "Enum") !== 2,
                            },
                            {
                                message: "TCAS STBY",
                                level: 2,
                                flightPhasesInhib: [1, 2, 3, 4, 5, 7, 8, 9, 10],
                                isActive: () => (
                                    this.fwcFlightPhase === 6 &&
                                    this.getCachedSimVar("L:A32NX_SWITCH_TCAS_Position", "Enum") === 0 &&
                                    this.getCachedSimVar("L:A320_Neo_ADIRS_STATE", "Enum") === 2
                                ),
                            }
                        ]
                    },
                    {
                        name: "F/CTL",
                        messages: [
                            {
                                id: "to_flaps_disagree",
                                message: "FLAP/MCDU DISAGREE",
                                level: 2,
                                flightPhasesInhib: [1, 4, 5, 6, 7, 8, 9, 10],
                                isActive: () => this.getCachedSimVar("L:A32NX_TO_CONFIG_FLAPS_ENTERED", "bool") &&
                                    (this.activeTakeoffConfigWarnings.includes("flaps_disagree") ||
                                        (
                                            (this.fwcFlightPhase === 2 && this.getCachedSimVar("L:A32NX_TO_CONFIG_NORMAL", "Bool") || this.fwcFlightPhase === 3) &&
                                            this.getCachedSimVar("L:A32NX_TO_CONFIG_FLAPS", "number") !== 0 &&
                                            this.getCachedSimVar("FLAPS HANDLE INDEX", "Enum") !== this.getCachedSimVar("L:A32NX_TO_CONFIG_FLAPS", "number")
                                        )
                                    )
                            }
                        ]
                    },
                    {
                        name: "T.O",
                        messages: [
                            {
                                id: "to_speeds_too_low",
                                message: "SPEEDS TOO LOW",
                                level: 2,
                                alwaysShowCategory: true,
                                isActive: () => (
                                    (this.fwcFlightPhase === 2 || this.fwcFlightPhase === 3) &&
                                    this.activeTakeoffConfigWarnings.includes("to_speeds_too_low")
                                ),
                                actions: [
                                    {
                                        style: "cyan",
                                        message: "&nbsp;-TOW AND T.O DATA.CHECK"
                                    }
                                ]
                            },
                            {
                                id: "to_speeds_disagree",
                                message: "V1/VR/V2 DISAGREE",
                                level: 2,
                                alwaysShowCategory: true,
                                isActive: () => (
                                    (this.fwcFlightPhase === 2 || this.fwcFlightPhase === 3) &&
                                    this.activeTakeoffConfigWarnings.includes("to_speeds_disagree")
                                ),
                            },
                            {
                                id: "to_no_speeds",
                                message: "SPEEDS NOT INSERTED",
                                level: 2,
                                alwaysShowCategory: true,
                                isActive: () => (
                                    (this.fwcFlightPhase === 2 || this.fwcFlightPhase === 3) &&
                                    this.activeTakeoffConfigWarnings.includes("to_no_speeds")
                                ),
                            },
                        ]
                    },
                    {
                        name: "APU",
                        messages: [
                            {
                                message: "AUTO SHUT DOWN",
                                level: 2,
                                flightPhasesInhib: [3, 4, 5, 7, 8],
                                isActive: () => this.getCachedSimVar("L:A32NX_APU_IS_AUTO_SHUTDOWN", "Bool"),
                                actions: [
                                    {
                                        style: "action",
                                        message: "MASTER SW",
                                        action: "OFF",
                                        isCompleted: () => !this.getCachedSimVar("L:A32NX_OVHD_APU_MASTER_SW_PB_IS_ON", "Bool"),
                                    },
                                ]
                            },
                            {
                                message: "EMER SHUT DOWN",
                                level: 2,
                                flightPhasesInhib: [3, 4, 5, 7, 8],
                                isActive: () => this.getCachedSimVar("L:A32NX_APU_IS_EMERGENCY_SHUTDOWN", "Bool"),
                                actions: [
                                    {
                                        style: "action",
                                        message: "MASTER SW",
                                        action: "OFF",
                                        isCompleted: () => !this.getCachedSimVar("L:A32NX_OVHD_APU_MASTER_SW_PB_IS_ON", "Bool"),
                                    },
                                ]
                            }
                        ]

                    }
                ],
                normal: [
                    {
                        message: "REFUELG",
                        isActive: () => this.getCachedSimVar("INTERACTIVE POINT OPEN:9", "percent") == 100
                    },
                    {
                        message: "IR IN ALIGN > 7 MN",
                        style: () => this.isEngineRunning(1) || this.isEngineRunning(2) ? "InfoCaution" : "InfoIndication",
                        isActive: () => this.isInFlightPhase(1, 2) && this.getADIRSMins() >= 7
                    },
                    {
                        message: "IR IN ALIGN 6 MN",
                        style: () => this.isEngineRunning(1) || this.isEngineRunning(2) ? "InfoCaution" : "InfoIndication",
                        isActive: () => this.isInFlightPhase(1, 2) && this.getADIRSMins() === 6
                    },
                    {
                        message: "IR IN ALIGN 5 MN",
                        style: () => this.isEngineRunning(1) || this.isEngineRunning(2) ? "InfoCaution" : "InfoIndication",
                        isActive: () => this.isInFlightPhase(1, 2) && this.getADIRSMins() === 5
                    },
                    {
                        message: "IR IN ALIGN 4 MN",
                        style: () => this.isEngineRunning(1) || this.isEngineRunning(2) ? "InfoCaution" : "InfoIndication",
                        isActive: () => this.isInFlightPhase(1, 2) && this.getADIRSMins() === 4
                    },
                    {
                        message: "IR IN ALIGN 3 MN",
                        style: () => this.isEngineRunning(1) || this.isEngineRunning(2) ? "InfoCaution" : "InfoIndication",
                        isActive: () => this.isInFlightPhase(1, 2) && this.getADIRSMins() === 3
                    },
                    {
                        message: "IR IN ALIGN 2 MN",
                        style: () => this.isEngineRunning(1) || this.isEngineRunning(2) ? "InfoCaution" : "InfoIndication",
                        isActive: () => this.isInFlightPhase(1, 2) && this.getADIRSMins() === 2
                    },
                    {
                        message: "IR IN ALIGN 1 MN",
                        style: () => this.isEngineRunning(1) || this.isEngineRunning(2) ? "InfoCaution" : "InfoIndication",
                        isActive: () => this.isInFlightPhase(1, 2) && (this.getADIRSMins() === 0 || this.getADIRSMins() === 1)
                    },
                    {
                        message: "GND SPLRS ARMED",
                        isActive: () => this.getCachedSimVar("SPOILERS ARMED", "Bool")
                    },
                    {
                        message: "SEAT BELTS",
                        isActive: () => this.getCachedSimVar("L:XMLVAR_SWITCH_OVHD_INTLT_SEATBELT_Position", "Bool")
                    },
                    {
                        message: "NO SMOKING",
                        isActive: () => this.getCachedSimVar("L:A32NX_NO_SMOKING_MEMO", "Bool") && !parseInt(NXDataStore.get("CONFIG_USING_PORTABLE_DEVICES", "0"))
                    },
                    {
                        message: "NO PORTABLE DEVICES",
                        isActive: () => this.getCachedSimVar("L:A32NX_NO_SMOKING_MEMO", "Bool") && parseInt(NXDataStore.get("CONFIG_USING_PORTABLE_DEVICES", "0"))
                    },
                    {
                        message: "STROBE LT OFF",
                        isActive: () => (
                            this.isInFlightPhase(6, 7, 8) && !this.getCachedSimVar("LIGHT STROBE ON", "Bool")
                        ),
                    },
                    {
                        message: "OUTR TK FUEL XFRD",
                        isActive: () => {
                            return (
                                this.getCachedSimVar("A:FUELSYSTEM VALVE SWITCH:4", "Bool") ||
                                this.getCachedSimVar("A:FUELSYSTEM VALVE SWITCH:5", "Bool") ||
                                this.getCachedSimVar("A:FUELSYSTEM VALVE SWITCH:6", "Bool") ||
                                this.getCachedSimVar("A:FUELSYSTEM VALVE SWITCH:7", "Bool")
                            );
                        }
                    },
                ]
            };
            this.secondaryEcamMessage = {
                failures: [],
                normal: [
                    {
                        message: "LAND ASAP",
                        style: "fail-3",
                        important: true,
                        isActive: () => this.leftEcamMessagePanel.landASAP === 3 && !Simplane.getIsGrounded()
                    },
                    {
                        message: "LAND ASAP",
                        style: "fail-2",
                        important: true,
                        isActive: () => this.leftEcamMessagePanel.landASAP === 2 && !Simplane.getIsGrounded()
                    },
                    {
                        message: "T.O. INHIBIT",
                        style: "InfoSpecial",
                        important: true,
                        isActive: () => this.showTakeoffInhibit,
                    },
                    {
                        message: "LDG INHIBIT",
                        style: "InfoSpecial",
                        important: true,
                        isActive: () => this.showLandingInhibit,
                    },

                    //Secondary failures
                    {
                        message: "*AIR BLEED",
                        style: "InfoCaution",
                        important: true,
                        isActive: () => {
                            return this.secondaryFailureActive("BLEED");
                        }
                    },
                    {
                        message: "*ELEC",
                        style: "InfoCaution",
                        important: true,
                        isActive: () => {
                            return this.secondaryFailureActive("ELEC");
                        }
                    },
                    {
                        message: "*HYD",
                        style: "InfoCaution",
                        important: true,
                        isActive: () => {
                            return this.secondaryFailureActive("HYD");
                        }
                    },
                    {
                        message: "SPEED BRK",
                        style: () => {
                            if (this.isInFlightPhase(2, 3, 4, 5)) {
                                return "InfoCaution";
                            }
                            // todo
                            return "InfoIndication";
                        },
                        isActive: () => (
                            !(
                                this.fwcFlightPhase === 1 ||
                                this.fwcFlightPhase === 8 ||
                                this.fwcFlightPhase === 9 ||
                                this.fwcFlightPhase === 10
                            ) && SimVar.GetSimVarValue("SPOILERS HANDLE POSITION", "position") > 0
                        ),
                    },
                    {
                        message: "PARK BRK",
                        isActive: () => (
                            this.isInFlightPhase(1, 2, 9, 10) && this.getCachedSimVar("BRAKE PARKING INDICATOR", "Bool")
                        ),
                    },
                    {
                        message: "HYD PTU",
                        isActive: () => {
                            // Rough approximation until hydraulic system fully implemented
                            // Needs the last 2 conditions because PTU_ON is (incorrectly) permanently set to true during first engine start
                            return (this.getCachedSimVar("L:XMLVAR_PTU_ON", "Bool") == 1) && (SimVar.GetSimVarValue("ENG N1 RPM:1", "Percent") < 1 || SimVar.GetSimVarValue("ENG N1 RPM:2", "Percent") < 1);
                        }
                    },
                    {
                        message: "NW STRG DISC",
                        style: () => this.isEngineRunning(1) || this.isEngineRunning(2) ? "InfoCaution" : "InfoIndication",
                        isActive: () => (
                            this.getCachedSimVar("PUSHBACK STATE", "Enum") !== 3
                        )
                    },
                    {
                        message: "CABIN READY",
                        isActive: () => {
                            return (
                                (this.getCachedSimVar("L:A32NX_CABIN_READY", "Bool")) &&
                                ((this.isInFlightPhase(2)) ||
                                (this.isInFlightPhase(6, 7) && this.getCachedSimVar("GEAR CENTER POSITION", "Percent") > 80))
                            );
                        }
                    },
                    {
                        message: "PRED W/S OFF",
                        style: () => (
                            this.isInFlightPhase(3, 4, 5, 7, 8, 9) || this.predWsMemo.read()
                        ) ? "InfoCaution" : "InfoIndication",
                        isActive: () => {
                            return (
                                !this.isInFlightPhase(1, 10) &&
                                !this.getCachedSimVar("L:A32NX_SWITCH_RADAR_PWS_Position", "Bool")
                            );
                        }
                    },
                    {
                        message: "TCAS STBY",
                        style: () => (
                            this.isInFlightPhase(6)
                        ) ? "InfoCaution" : "InfoIndication",
                        isActive: () => SimVar.GetSimVarValue("L:A32NX_SWITCH_TCAS_Position", "Enum") === 0,
                    },
                    {
                        message: "COMPANY MSG",
                        isActive: () => {
                            return this.isInFlightPhase(1, 2, 6, 9, 10) && (
                                this.getCachedSimVar("L:A32NX_COMPANY_MSG_COUNT", "Number") > 0
                            );
                        }
                    },
                    {
                        message: "ENG A.ICE",
                        isActive: () => {
                            return this.getCachedSimVar("ENG ANTI ICE:1", "Bool") || this.getCachedSimVar("ENG ANTI ICE:2", "Bool");
                        }
                    },
                    {
                        message: "WING A.ICE",
                        isActive: () => {
                            return this.getCachedSimVar("STRUCTURAL DEICE SWITCH", "Bool");
                        }
                    },
                    {
                        message: "ICE NOT DET",
                        isActive: () => this.iceNotDetTimer2.read() && !Simplane.getIsGrounded(),
                    },
                    {
                        message: "APU AVAIL",
                        isActive: () => (
                            !SimVar.GetSimVarValue("L:A32NX_APU_BLEED_AIR_VALVE_OPEN", "Bool") &&
                                this.getCachedSimVar("L:A32NX_OVHD_APU_START_PB_IS_AVAILABLE", "Bool")
                        )
                    },
                    {
                        message: "APU BLEED",
                        isActive: () => (
                            SimVar.GetSimVarValue("L:A32NX_APU_BLEED_AIR_VALVE_OPEN", "Bool") &&
                                this.getCachedSimVar("L:A32NX_OVHD_APU_START_PB_IS_AVAILABLE", "Bool")
                        )
                    },
                    {
                        message: "LDG LT",
                        isActive: () => (
                            !SimVar.GetSimVarValue("L:LANDING_1_Retracted", "Bool") ||
                            !SimVar.GetSimVarValue("L:LANDING_2_Retracted", "Bool")
                        ),
                    },
                    {
                        message: "BRK FAN",
                        isActive: () => {
                            return this.getCachedSimVar("L:A32NX_BRAKE_FAN", "Bool");
                        }
                    },
                    {
                        message: "SWITCHG PNL",
                        isActive: () => {
                            return (
                                (SimVar.GetSimVarValue("L:A32NX_KNOB_SWITCHING_3_Position", "Enum") != 1) ||
                                (SimVar.GetSimVarValue("L:A32NX_KNOB_SWITCHING_4_Position", "Enum") != 1)
                            );
                        }
                    },
                    {
                        message: "GPWS FLAP 3",
                        isActive: () => this.getCachedSimVar("L:A32NX_GPWS_FLAPS3", "Bool")
                    },
                    {
                        message: "AUTO BRK LO",
                        isActive: () => (
                            (this.fwcFlightPhase === 7 || this.fwcFlightPhase === 8) &&
                            SimVar.GetSimVarValue("L:XMLVAR_Autobrakes_Level", "Enum") === 1
                        )
                    },
                    {
                        message: "AUTO BRK MED",
                        isActive: () => {
                            return (
                                (this.fwcFlightPhase === 7 || this.fwcFlightPhase === 8) &&
                                SimVar.GetSimVarValue("L:XMLVAR_Autobrakes_Level", "Enum") === 2
                            );
                        }
                    },
                    {
                        message: "AUTO BRK MAX",
                        isActive: () => {
                            return (
                                (this.fwcFlightPhase === 7 || this.fwcFlightPhase === 8) &&
                                SimVar.GetSimVarValue("L:XMLVAR_Autobrakes_Level", "Enum") === 3
                            );
                        }
                    },
                    {
                        message: "FUEL X FEED",
                        style: () => (
                            this.isInFlightPhase(3, 4, 5)
                        ) ? "InfoCaution" : "InfoIndication",
                        isActive: () => {
                            return (
                                this.getCachedSimVar("A:FUELSYSTEM VALVE SWITCH:3", "Bool")
                            );
                        }
                    },
                    {
                        message: "ADIRS SWTG",
                        isActive: () => {
                            return (
                                (SimVar.GetSimVarValue("L:A32NX_KNOB_SWITCHING_1_Position", "Enum") != 1) ||
                                (SimVar.GetSimVarValue("L:A32NX_KNOB_SWITCHING_2_Position", "Enum") != 1)
                            );
                        }
                    },
                ]
            };
            this.leftEcamMessagePanel = new A320_Neo_UpperECAM.EcamMessagePanel(this, "left-msg", 7, this.ecamMessages);
            this.rightEcamMessagePanel = new A320_Neo_UpperECAM.EcamMessagePanel(this, "right-msg", 7, this.secondaryEcamMessage);
            this.takeoffMemo = new A320_Neo_UpperECAM.Memo(this, "to-memo", "T.O", [
                new A320_Neo_UpperECAM.MemoItem(
                    "to-memo-autobrk",
                    "AUTO BRK",
                    "MAX",
                    "MAX",
                    () => SimVar.GetSimVarValue("L:XMLVAR_Autobrakes_Level", "Enum") === 3
                ),
                new A320_Neo_UpperECAM.MemoItem(
                    "to-memo-signs",
                    "SIGNS",
                    "ON",
                    "ON",
                    () => (
                        SimVar.GetSimVarValue("L:XMLVAR_SWITCH_OVHD_INTLT_SEATBELT_Position", "Bool") &&
                        SimVar.GetSimVarValue("L:XMLVAR_SWITCH_OVHD_INTLT_NOSMOKING_Position", "Enum") !== 2
                    )
                ),
                new A320_Neo_UpperECAM.MemoItem(
                    "to-memo-cabin",
                    "CABIN",
                    "CHECK",
                    "READY",
                    () => SimVar.GetSimVarValue("L:A32NX_CABIN_READY", "Bool")
                ),
                new A320_Neo_UpperECAM.MemoItem(
                    "to-memo-splrs",
                    "SPLRS",
                    "ARM",
                    "ARM",
                    () => SimVar.GetSimVarValue("SPOILERS ARMED", "Bool")
                ),
                new A320_Neo_UpperECAM.MemoItem(
                    "to-memo-flaps",
                    "FLAPS",
                    "T.O",
                    "T.O",
                    () => this.getCachedSimVar("FLAPS HANDLE INDEX", "Enum") >= 1 && this.getCachedSimVar("FLAPS HANDLE INDEX", "Enum") <= 3
                ),
                new A320_Neo_UpperECAM.MemoItem(
                    "to-memo-config",
                    "T.O CONFIG",
                    "TEST",
                    "NORMAL",
                    () => SimVar.GetSimVarValue("L:A32NX_TO_CONFIG_NORMAL", "Bool")
                ),
            ]);

            this.landingMemo = new A320_Neo_UpperECAM.Memo(this, "ldg-memo", "LDG", [
                new A320_Neo_UpperECAM.MemoItem(
                    "ldg-memo-gear",
                    "LDG GEAR",
                    "DN",
                    "DN",
                    () => SimVar.GetSimVarValue("GEAR HANDLE POSITION", "Bool")
                ),
                new A320_Neo_UpperECAM.MemoItem(
                    "ldg-memo-signs",
                    "SIGNS",
                    "ON",
                    "ON",
                    () => (
                        SimVar.GetSimVarValue("L:XMLVAR_SWITCH_OVHD_INTLT_SEATBELT_Position", "Bool") &&
                        SimVar.GetSimVarValue("L:XMLVAR_SWITCH_OVHD_INTLT_NOSMOKING_Position", "Enum") !== 2
                    )
                ),
                new A320_Neo_UpperECAM.MemoItem(
                    "ldg-memo-cabin",
                    "CABIN",
                    "CHECK",
                    "READY",
                    () => SimVar.GetSimVarValue("L:A32NX_CABIN_READY", "Bool")
                ),
                new A320_Neo_UpperECAM.MemoItem(
                    "ldg-memo-splrs",
                    "SPLRS",
                    "ARM",
                    "ARM",
                    () => SimVar.GetSimVarValue("SPOILERS ARMED", "Bool")
                ),
                new A320_Neo_UpperECAM.MemoItem(
                    "ldg-memo-flaps",
                    "FLAPS",
                    () => this.getCachedSimVar("L:A32NX_GPWS_FLAPS3", "Bool") ? "CONF 3" : "FULL",
                    () => this.getCachedSimVar("L:A32NX_GPWS_FLAPS3", "Bool") ? "CONF 3" : "FULL",
                    () => (
                        this.getCachedSimVar("L:A32NX_GPWS_FLAPS3", "Bool") ?
                            this.getCachedSimVar("FLAPS HANDLE INDEX", "Enum") === 3 :
                            this.getCachedSimVar("FLAPS HANDLE INDEX", "Enum") === 4
                    )

                ),
            ]);
            this.allPanels.push(this.enginePanel);
            this.allPanels.push(this.infoTopPanel);
            this.allPanels.push(this.flapsPanel);
            this.allPanels.push(this.takeoffMemo);
            this.allPanels.push(this.landingMemo);
            this.allPanels.push(this.leftEcamMessagePanel);
            this.allPanels.push(this.rightEcamMessagePanel);
            for (let i = 0; i < this.allPanels.length; ++i) {
                if (this.allPanels[i] != null) {
                    this.allPanels[i].init();
                }
            }
            if (this.infoPanelsManager != null) {
                this.infoPanelsManager.init(this.infoBottomLeftPanel, this.infoBottomRightPanel);
            }
            this.updateThrottler = new UpdateThrottler(150);
            this.engUpdateThrottler = new UpdateThrottler(50);
            this.isInitialised = true;
        }
        update(_deltaTime) {
            if (!this.isInitialised || !A320_Neo_EICAS.isOnTopScreen()) {
                return;
            }

            const newDeltaTime = this.updateThrottler.canUpdate(_deltaTime);
            const deltaTimeEng = this.engUpdateThrottler.canUpdate(_deltaTime);

            for (let i = 0; i < this.allPanels.length; ++i) {
                if (this.allPanels[i] != null) {
                    const isEng = this.allPanels[i] instanceof A320_Neo_UpperECAM.EnginePanel;
                    const panelDeltaTime = isEng ? deltaTimeEng : newDeltaTime;
                    if (panelDeltaTime != -1) {
                        this.allPanels[i].update(panelDeltaTime);
                    }
                }
            }

            this.frameCount++;
            if (this.frameCount % 16 == 0) {
                this.simVarCache = {};
            }

            if (newDeltaTime == -1) {
                return;
            }
            _deltaTime = newDeltaTime;

            // Packs indicator
            this.packsText = this.querySelector("#packsIndicator");
            this.isTogaFlexMct1 = this.getCachedSimVar("GENERAL ENG THROTTLE MANAGED MODE:1", "number") > 4;
            this.isTogaFlexMct2 = this.getCachedSimVar("GENERAL ENG THROTTLE MANAGED MODE:2", "number") > 4;
            this.isGrounded = this.getCachedSimVar("SIM ON GROUND", "bool");
            this.flightPhaseBeforeClb = this.getCachedSimVar("L:A32NX_FMGC_FLIGHT_PHASE", "number") < FmgcFlightPhases.CLIMB;

            if (this.isGrounded || this.flightPhaseBeforeClb || (this.isTogaFlexMct1 && this.isTogaFlexMct2)) {
                const ignStateActive = this.getCachedSimVar("L:XMLVAR_ENG_MODE_SEL", "Enum") == 2;
                const eng1active = this.getCachedSimVar("ENG COMBUSTION:1", "Bool");
                const eng2active = this.getCachedSimVar("ENG COMBUSTION:2", "Bool");
                const xBleedPos = this.getCachedSimVar("L:A32NX_KNOB_OVHD_AIRCOND_XBLEED_Position", "number");
                const engBleedAndPackActive = xBleedPos === 2 || (xBleedPos === 1 && this.getCachedSimVar("L:A32NX_OVHD_APU_START_PB_IS_AVAILABLE", "Bool") === 1 && SimVar.GetSimVarValue("L:A32NX_APU_BLEED_AIR_VALVE_OPEN", "Bool")) ?
                    (this.getCachedSimVar("BLEED AIR ENGINE:1", "Bool") || this.getCachedSimVar("BLEED AIR ENGINE:2", "Bool"))
                    && ((this.getCachedSimVar("L:A32NX_AIRCOND_PACK1_TOGGLE", "bool") && eng1active) || (this.getCachedSimVar("L:A32NX_AIRCOND_PACK2_TOGGLE", "bool") && eng2active))
                    :
                    (eng1active && this.getCachedSimVar("BLEED AIR ENGINE:1", "Bool") && this.getCachedSimVar("L:A32NX_AIRCOND_PACK1_TOGGLE", "bool"))
                    || (eng2active && this.getCachedSimVar("BLEED AIR ENGINE:2", "Bool") && this.getCachedSimVar("L:A32NX_AIRCOND_PACK2_TOGGLE", "bool"));
                const eng1NAIactive = this.getCachedSimVar("ENG ANTI ICE:1", "Bool");
                const eng2NAIactive = this.getCachedSimVar("ENG ANTI ICE:2", "Bool");
                const WAIactive = this.getCachedSimVar("STRUCTURAL DEICE SWITCH", "Bool");

                const textList = [];
                if ((!ignStateActive || (ignStateActive && !this.isGrounded)) && engBleedAndPackActive) {
                    textList.push("PACKS");
                }
                if ((eng1active && eng1NAIactive) || (eng2active && eng2NAIactive)) {
                    textList.push("NAI");
                }
                if ((eng1active || eng2active) && WAIactive) {
                    textList.push("WAI");
                }
                this.packsText.textContent = textList.join("/");
            } else {
                this.packsText.textContent = "";
            }

            this.fwcFlightPhase = SimVar.GetSimVarValue("L:A32NX_FWC_FLIGHT_PHASE", "Enum");

            this.overflowArrow.setAttribute("opacity", (this.leftEcamMessagePanel.overflow || this.rightEcamMessagePanel.overflow) ? "1" : "0");

            this.updateInhibitMessages(_deltaTime);
            this.updateIcing(_deltaTime);

            const memosInhibited = this.leftEcamMessagePanel.hasWarnings || this.leftEcamMessagePanel.hasCautions;
            const showTOMemo = SimVar.GetSimVarValue("L:A32NX_FWC_TOMEMO", "Bool") && !memosInhibited;
            if (this.takeoffMemo != null) {
                this.takeoffMemo.divMain.style.display = showTOMemo ? "block" : "none";
            }

            const showLdgMemo = SimVar.GetSimVarValue("L:A32NX_FWC_LDGMEMO", "Bool") && !memosInhibited;
            if (this.landingMemo != null) {
                this.landingMemo.divMain.style.display = showLdgMemo ? "block" : "none";
            }

            //Hide left message panel when memo is diplayed
            if (this.leftEcamMessagePanel != null) {
                this.leftEcamMessagePanel.divMain.style.display = (showTOMemo || showLdgMemo) ? "none" : "block";
            }

            let toConfigMomentary = false;
            if (SimVar.GetSimVarValue("L:A32NX_FWC_TOCONFIG", "Bool")) {
                SimVar.SetSimVarValue("L:A32NX_FWC_TOCONFIG", "Bool", 0);
                // FWC ESLD 1.0.180
                if (this.isInFlightPhase(2, 9)) {
                    this.updateTakeoffConfigWarnings(true);
                }
                toConfigMomentary = true;
            }

            // FWC ESLD 2.0.540
            this.predWsMemo.write(this.fwcFlightPhase === 2 && toConfigMomentary, this.fwcFlightPhase !== 2);

            if (SimVar.GetSimVarValue("L:A32NX_BTN_CLR", "Bool") == 1 || SimVar.GetSimVarValue("L:A32NX_BTN_CLR2", "Bool") == 1) {
                SimVar.SetSimVarValue("L:A32NX_BTN_CLR", "Bool", 0);
                SimVar.SetSimVarValue("L:A32NX_BTN_CLR2", "Bool", 0);
                const secondaryFailures = this.leftEcamMessagePanel.secondaryFailures;
                if (this.leftEcamMessagePanel.hasActiveFailures) {
                    //Clear failure
                    this.leftEcamMessagePanel.clearHighestCategory();
                } else if (this.clearedSecondaryFailures.length < secondaryFailures.length) {
                    //Clear secondary failure
                    this.clearedSecondaryFailures.push(this.getActiveSecondaryFailure());
                }

            }

            if (SimVar.GetSimVarValue("L:A32NX_FWC_RECALL", "Bool")) {
                SimVar.SetSimVarValue("L:A32NX_FWC_RECALL", "Bool", 0);
                this.leftEcamMessagePanel.recall();
                this.clearedSecondaryFailures = [];
            }

            const ECAMPageIndices = {
                "ENG": 0,
                "BLEED": 1,
                "PRESS": 2,
                "ELEC": 3,
                "HYD": 4,
                "FUEL": 5,
                "APU": 6,
                "COND": 7,
                "DOOR": 8,
                "WHEEL": 9,
                "FTCL": 10,
                "STS": 11
            };

            const activeSecondaryFailure = this.getActiveSecondaryFailure();
            if (this.leftEcamMessagePanel.activePage != null) {
                SimVar.SetSimVarValue("L:A32NX_ECAM_SFAIL", "Enum", ECAMPageIndices[this.leftEcamMessagePanel.activePage]);
            } else if (activeSecondaryFailure != null) {
                SimVar.SetSimVarValue("L:A32NX_ECAM_SFAIL", "Enum", ECAMPageIndices[activeSecondaryFailure]);
            } else {
                SimVar.SetSimVarValue("L:A32NX_ECAM_SFAIL", "Enum", -1);
            }

            this.rightEcamMessagePanel.failMode = ((this.leftEcamMessagePanel.secondaryFailures.length - this.clearedSecondaryFailures.length) > 0 && (this.leftEcamMessagePanel.secondaryFailures[this.clearedSecondaryFailures.length] != "STS"));

            this.statusReminder.setAttribute("visibility", (this.leftEcamMessagePanel.secondaryFailures.length > 0 && this.leftEcamMessagePanel.secondaryFailures.length == this.clearedSecondaryFailures.length) ? "visible" : "hidden");

            if (this.fwcFlightPhase === 3) {
                this.updateTakeoffConfigWarnings(false);
            }

            if (this.fwcFlightPhase === 5) {
                this.activeTakeoffConfigWarnings = [];
            }

            if (!(this.leftEcamMessagePanel.hasCautions)) {
                SimVar.SetSimVarValue("L:A32NX_MASTER_CAUTION", "Bool", 0);
            }
            if (!(this.leftEcamMessagePanel.hasWarnings)) {
                SimVar.SetSimVarValue("L:A32NX_MASTER_WARNING", "Bool", 0);
            }

            if ((SimVar.GetSimVarValue("L:PUSH_OVHD_CALLS_ALL", "Bool") == 1) || (SimVar.GetSimVarValue("L:PUSH_OVHD_CALLS_FWD", "Bool") == 1) || (SimVar.GetSimVarValue("L:PUSH_OVHD_CALLS_AFT", "Bool") == 1)) {
                SimVar.SetSimVarValue("L:A32NX_CABIN_READY", "Bool", 1);
            }

            if (SimVar.GetSimVarValue("L:A32NX_Fire_ENG1_Agent1_Discharge", "Bool") == 1) {
                SimVar.SetSimVarValue("ENG ON FIRE:1", "Bool", 0);
            }
            if (SimVar.GetSimVarValue("L:A32NX_Fire_ENG2_Agent1_Discharge", "Bool") == 1) {
                SimVar.SetSimVarValue("ENG ON FIRE:2", "Bool", 0);
            }
        }
        updateTakeoffConfigWarnings(_test) {
            const slatsLeft = SimVar.GetSimVarValue("LEADING EDGE FLAPS LEFT ANGLE", "degrees");
            const slatsRight = SimVar.GetSimVarValue("LEADING EDGE FLAPS RIGHT ANGLE", "degrees");
            const flapsLeft = SimVar.GetSimVarValue("TRAILING EDGE FLAPS LEFT ANGLE", "degrees");
            const flapsRight = SimVar.GetSimVarValue("TRAILING EDGE FLAPS RIGHT ANGLE", "degrees");
            const flapsHandle = SimVar.GetSimVarValue("FLAPS HANDLE INDEX", "Enum");
            const flapsMcdu = SimVar.GetSimVarValue("L:A32NX_TO_CONFIG_FLAPS", "number");
            const flapsMcduEntered = SimVar.GetSimVarValue("L:A32NX_TO_CONFIG_FLAPS_ENTERED", "bool");
            const speedBrake = SimVar.GetSimVarValue("SPOILERS HANDLE POSITION", "Position");
            const parkBrake = SimVar.GetSimVarValue("BRAKE PARKING INDICATOR", "Bool");
            const brakesHot = SimVar.GetSimVarValue("L:A32NX_BRAKES_HOT", "Bool");
            const v1Speed = SimVar.GetSimVarValue("L:AIRLINER_V1_SPEED", "Knots");
            const vrSpeed = SimVar.GetSimVarValue("L:AIRLINER_VR_SPEED", "Knots");
            const v2Speed = SimVar.GetSimVarValue("L:AIRLINER_V2_SPEED", "Knots");
            this.activeTakeoffConfigWarnings = [];
            if (slatsLeft > 25 || slatsLeft < 17 || slatsRight > 25 || slatsRight < 17) {
                this.activeTakeoffConfigWarnings.push("slats");
            }
            if (flapsLeft > 24 || flapsLeft < 2 || flapsRight > 24 || flapsRight < 2) {
                this.activeTakeoffConfigWarnings.push("flaps");
            }
            if (speedBrake > 0) {
                this.activeTakeoffConfigWarnings.push("spd_brk");
            }
            if (parkBrake && !_test) {
                this.activeTakeoffConfigWarnings.push("park_brake");
            }
            if (brakesHot) {
                this.activeTakeoffConfigWarnings.push("brakes_hot");
            }
            if (flapsMcduEntered && flapsHandle !== flapsMcdu) {
                this.activeTakeoffConfigWarnings.push("flaps_disagree");
            }
            if (!(v1Speed <= vrSpeed && vrSpeed <= v2Speed)) {
                this.activeTakeoffConfigWarnings.push("to_speeds_disagree");
            }

            if (_test && this.activeTakeoffConfigWarnings.length === 0) {
                SimVar.SetSimVarValue("L:A32NX_TO_CONFIG_NORMAL", "Bool", 1);
            } else {
                SimVar.SetSimVarValue("L:A32NX_TO_CONFIG_NORMAL", "Bool", 0);
                this.leftEcamMessagePanel.recall("config_slats");
                this.leftEcamMessagePanel.recall("config_flaps");
                this.leftEcamMessagePanel.recall("config_spd_brk");
                this.leftEcamMessagePanel.recall("config_park_brake");
                this.leftEcamMessagePanel.recall("brakes_hot");
                this.leftEcamMessagePanel.recall("to_flaps_disagree");
                this.leftEcamMessagePanel.recall("to_speeds_disagree");
            }
        }

        updateInhibitMessages(_deltaTime) {
            this.inhibitOverride = this.getCachedSimVar("L:A32NX_FWC_INHIBOVRD", "Bool");
            this.showTakeoffInhibit = this.toInhibitTimer.write(this.isInFlightPhase(3, 4, 5) && !this.inhibOverride, _deltaTime);
            this.showLandingInhibit = this.ldgInhibitTimer.write(this.isInFlightPhase(7, 8) && !this.inhibOverride, _deltaTime);
        }

        updateIcing(_deltaTime) {
            const ground = Simplane.getIsGrounded();
            const tatInf10 = this.getCachedSimVar("TOTAL AIR TEMPERATURE", "Celsius") < 10;
            const eng1AntiIceOn = this.getCachedSimVar("ENG ANTI ICE:1", "Bool");
            const eng2AntiIceOn = this.getCachedSimVar("ENG ANTI ICE:2", "Bool");
            const wingAntiIceOn = this.getCachedSimVar("STRUCTURAL DEICE SWITCH", "Bool");

            // SEVERE ICING
            const isSevereIceDetected = this.getCachedSimVar("STRUCTURAL ICE PCT", "Percent") >= 50;
            this.iceSevereDetectedTimer.write(
                isSevereIceDetected && tatInf10 && !ground,
                _deltaTime
            );

            // ICE DETECTED
            const isIceDetected = this.getCachedSimVar("STRUCTURAL ICE PCT", "Percent") >= 10;
            const det1 = this.iceDetectedTimer1.write(
                isIceDetected && !ground && tatInf10,
                _deltaTime
            );
            this.iceDetectedTimer2.write(
                det1 && !(eng1AntiIceOn && eng2AntiIceOn),
                _deltaTime
            );

            // ICE NOT DET
            const isActivelyIcing = (
                isIceDetected || (
                    tatInf10 &&
                    this.getCachedSimVar("AMBIENT IN CLOUD", "boolean")
                )
            );
            const isAnyAntiIceOn = eng1AntiIceOn || eng2AntiIceOn || wingAntiIceOn;
            const notDet1 = this.iceNotDetTimer1.write(isAnyAntiIceOn, _deltaTime);
            this.iceNotDetTimer2.write(!isActivelyIcing && notDet1, _deltaTime);

        }

        getInfoPanelManager() {
            return this.infoPanelsManager;
        };
    }
    A320_Neo_UpperECAM.Display = Display;
    class PanelBase {
        constructor(_parent, _divID) {
            this.parent = _parent;
            this.divID = _divID;
        }
        init() {
            this.create();
        }
        update(_deltaTime) {
        }
        create() {
            if (this.parent != null) {
                this.divMain = A320_Neo_UpperECAM.createDiv(this.divID);
                this.parent.appendChild(this.divMain);
            }
        }
    }
    A320_Neo_UpperECAM.PanelBase = PanelBase;
    class EnginePanel extends A320_Neo_UpperECAM.PanelBase {
        constructor() {
            super(...arguments);
            this.engines = [];
            this.linesStyleInfo = [];
        }
        update(_deltaTime) {
            super.update(_deltaTime);
            for (var i = 0; i < this.engines.length; ++i) {
                this.engines[i].update(_deltaTime);
            }
            for (var i = 0; i < this.linesStyleInfo.length; ++i) {
                this.linesStyleInfo[i].update(_deltaTime);
            }
        }
        create() {
            super.create();
            for (let i = 0; i < 2; i++) {
                this.engines.push(new A320_Neo_UpperECAM.Engine(this.divMain, i));
                this.engines[i].init();
            }
            const gaugeInfoDiv = A320_Neo_UpperECAM.createDiv("GaugeInfo");
            this.statusDiv = A320_Neo_UpperECAM.createDiv("", "STATUS", "");
            gaugeInfoDiv.appendChild(this.statusDiv);
            gaugeInfoDiv.appendChild(A320_Neo_UpperECAM.createDiv("", "SLOT1_TITLE", "N1"));
            gaugeInfoDiv.appendChild(A320_Neo_UpperECAM.createDiv("", "SLOT1_UNIT", "%"));
            gaugeInfoDiv.appendChild(A320_Neo_UpperECAM.createDiv("", "SLOT2_TITLE", "EGT"));
            gaugeInfoDiv.appendChild(A320_Neo_UpperECAM.createDiv("", "SLOT2_UNIT", String.fromCharCode(176) + "C"));
            this.linesStyleInfo.push(new A320_Neo_UpperECAM.LinesStyleInfo_N2(this.divMain, "20%"));
            this.linesStyleInfo.push(new A320_Neo_UpperECAM.LinesStyleInfo_FF(this.divMain, "0%"));
            this.divMain.appendChild(gaugeInfoDiv);
        }
    }
    A320_Neo_UpperECAM.EnginePanel = EnginePanel;
    class Engine {
        constructor(_parent, _index) {
            this.allGauges = [];
            this.parent = _parent;
            this.index = _index;
        }
        init() {
            this.divMain = A320_Neo_UpperECAM.createDiv("Engine" + this.index, "engine");
            this.createN1Gauge();
            this.createEGTGauge();
            this.parent.appendChild(this.divMain);
            this.timerTOGA = -1;
            this.throttleMode = Math.max(Simplane.getEngineThrottleMode(0), Simplane.getEngineThrottleMode(1));
            this.timerAvail = -1;
            this.timerAvailFlag = -1;
            this.deltaThrottlePosition = this.getThrottlePosition(this.index);
        }
        update(_deltaTime) {
            if (this.allGauges != null) {
                const active = SimVar.GetSimVarValue("L:A32NX_FADEC_POWERED_ENG" + (this.index + 1), "Bool") == 1;
                for (let i = 0; i < this.allGauges.length; ++i) {
                    if (this.allGauges[i] != null) {
                        this.allGauges[i].active = active;
                        this.allGauges[i].update(_deltaTime);
                    }
                }
            }
            const currentThrottleState = Math.max(Simplane.getEngineThrottleMode(0), Simplane.getEngineThrottleMode(1));
            if (this.throttleMode != currentThrottleState) {
                this.throttleMode = currentThrottleState;
            }
            if (this.throttleMode == ThrottleMode.TOGA) {
                if (this.timerTOGA == -1) {
                    this.timerTOGA = 300;
                }
                if (this.timerTOGA >= 0) {
                    this.timerTOGA -= _deltaTime / 1000;
                }
            } else {
                this.timerTOGA = -1;
            }
            this.checkIgnitionPhaseForAVAIL(_deltaTime);
        }
        checkIgnitionPhaseForAVAIL(_deltaTime) {
            if (this.getN1GaugeValue() < 1) {
                this.timerAvailFlag = 1;
            }
            if (this.getEngineStartStatus() && this.getIgnitionStatus()) {
                if (this.getN1GaugeValue() > 18.3 && this.timerAvailFlag == 1) {
                    if (this.timerAvail == -1) {
                        this.timerAvail = 10;
                    } else if (this.timerAvail >= 0) {
                        this.timerAvail -= _deltaTime / 1000;
                    } else {
                        this.timerAvail = -1;
                        this.timerAvailFlag = -1;
                    }
                }
            } else {
                this.timerAvail = -1;
            }
        }
        createEGTGauge() {
            const gaugeDef = new A320_Neo_ECAM_Common.GaugeDefinition();
            gaugeDef.startAngle = -180;
            gaugeDef.arcSize = 180;
            gaugeDef.minValue = A320_Neo_UpperECAM.Definitions.MIN_GAUGE_EGT;
            gaugeDef.maxValue = A320_Neo_UpperECAM.Definitions.MAX_GAUGE_EGT;
            gaugeDef.minRedValue = A320_Neo_UpperECAM.Definitions.MIN_GAUGE_EGT_RED;
            gaugeDef.maxRedValue = A320_Neo_UpperECAM.Definitions.MAX_GAUGE_EGT_RED;
            gaugeDef.cursorLength = 0.4;
            gaugeDef.currentValuePos.x = 0.75;
            gaugeDef.currentValuePos.y = 0.5;
            gaugeDef.cursorMultiplier = 1.1;
            gaugeDef.currentValueBorderWidth = 0.55;
            gaugeDef.dangerRange[0] = gaugeDef.minRedValue;
            gaugeDef.dangerRange[1] = gaugeDef.maxRedValue;
            gaugeDef.dangerMinDynamicFunction = this.getModeEGTMax.bind(this);
            gaugeDef.currentValueFunction = this.getEGTGaugeValue.bind(this);
            gaugeDef.currentValuePrecision = 0;
            gaugeDef.upperecam = true;
            this.gaugeEGT = window.document.createElement("a320-neo-ecam-gauge");
            this.gaugeEGT.id = "EGT_Gauge";
            this.gaugeEGT.init(gaugeDef);
            this.gaugeEGT.addGraduation(0, true);
            this.gaugeEGT.addGraduation(600, true);
            this.gaugeEGT.addGraduation(gaugeDef.maxRedValue, true, "", false, false, "", "danger");
            this.divMain.appendChild(this.gaugeEGT);
            this.allGauges.push(this.gaugeEGT);
        }
        createN1Gauge() {
            const gaugeDef = new A320_Neo_ECAM_Common.GaugeDefinition();
            gaugeDef.minValue = A320_Neo_UpperECAM.Definitions.MIN_GAUGE_N1;
            gaugeDef.maxValue = A320_Neo_UpperECAM.Definitions.MAX_GAUGE_N1;
            gaugeDef.arcSize = 190;
            gaugeDef.cursorOffset = 5;
            gaugeDef.minRedValue = A320_Neo_UpperECAM.Definitions.MIN_GAUGE_N1_RED;
            gaugeDef.currentValuePos.x = 1.0;
            gaugeDef.currentValuePos.y = 0.75;
            gaugeDef.cursorMultiplier = 1.1;
            gaugeDef.currentValueBorderWidth = 0.68;
            gaugeDef.outerIndicatorFunction = this.getThrottlePosition.bind(this);
            gaugeDef.outerDynamicArcFunction = this.getN1GaugeAutopilotThrottleValues.bind(this);
            gaugeDef.extraMessageFunction = this.getN1GaugeExtraMessage.bind(this);
            gaugeDef.maxRedValue = A320_Neo_UpperECAM.Definitions.MAX_GAUGE_N1_RED;
            gaugeDef.dangerRange[0] = gaugeDef.minRedValue;
            gaugeDef.dangerRange[1] = gaugeDef.maxRedValue;
            gaugeDef.dangerMinDynamicFunction = this.getModeN1Max.bind(this);
            gaugeDef.currentValueFunction = this.getN1GaugeValue.bind(this);
            gaugeDef.currentValuePrecision = 1;
            this.gaugeN1 = window.document.createElement("a320-neo-ecam-gauge");
            this.gaugeN1.id = "N1_Gauge";
            this.gaugeN1.init(gaugeDef);
            //this.gaugeN1.addGraduation(gaugeDef.minValue, false);
            this.gaugeN1.addGraduation(50, true, "5");
            this.gaugeN1.addGraduation(60, true);
            this.gaugeN1.addGraduation(70, true);
            this.gaugeN1.addGraduation(80, true);
            this.gaugeN1.addGraduation(83, true, "", false, false, "", "warning");
            this.gaugeN1.addGraduation(84, false, "", true);
            this.gaugeN1.addGraduation(90, true);
            this.gaugeN1.addGraduation(100, true, "10");
            this.gaugeN1.addGraduation(gaugeDef.maxRedValue, true, "", false, false, "", "danger");
            this.divMain.appendChild(this.gaugeN1);
            this.allGauges.push(this.gaugeN1);
        }
        getModeEGTMax() {
            if (this.throttleMode == ThrottleMode.TOGA && this.timerTOGA > 0) {
                return 1060;
            } else if (this.throttleMode == ThrottleMode.TOGA && this.timerTOGA < 0) {
                return 1025;
            }
            if (this.throttleMode == ThrottleMode.FLEX_MCT) {
                return 1025;
            }
            if (this.throttleMode == ThrottleMode.CLIMB) {
                return 875;
            }
            if (this.throttleMode == ThrottleMode.AUTO) {
                return 850;
            } else {
                return 750;
            }
        }
        getModeN1Max() {
            switch (this.throttleMode) {
                case ThrottleMode.TOGA: return ((this.timerTOGA > 0) ? 101.5 : 100);
                case ThrottleMode.REVERSE: return 66.7;
                default: return 100;
            }
        }
        getEGTGaugeValue() {
            const engineId = this.index + 1;
            const imbalance = SimVar.GetSimVarValue("L:A32NX_ENGINE_IMBALANCE", "number");
            if (parseInt(imbalance.toString().substr(0, 1)) == engineId) {
                var egt_i = parseInt(imbalance.toString().substr(1, 2));
            } else {
                var egt_i = parseInt(0);
            }
            const value = SimVar.GetSimVarValue("L:A32NX_ENGINE_EGT:" + engineId, "celsius") - egt_i;
            return value;
        }
        getN1GaugeValue() {
            const engineId = (this.index + 1);
            const value = SimVar.GetSimVarValue("ENG N1 RPM:" + engineId, "percent");
            return value;
        }
        getN1GaugeThrottleValue() {
            const throttle = Math.abs(SimVar.GetSimVarValue("L:A32NX_AUTOTHRUST_TLA_N1:" + (this.index + 1), "number"));
            const value = throttle * A320_Neo_UpperECAM.Definitions.THROTTLE_TO_N1_GAUGE;
            return value;
        }
        getThrottlePosition() {
            return Math.abs(SimVar.GetSimVarValue("L:A32NX_AUTOTHRUST_TLA_N1:" + (this.index + 1), "number"));
        }
        getN1GaugeAutopilotThrottleValues(_values) {
            if (SimVar.GetSimVarValue("L:A32NX_AUTOTHRUST_STATUS", "number") == 2) {
                // seems still broken on gauge side -> but this will be how to get the values.
                // _values[0] = SimVar.GetSimVarValue("L:A32NX_AUTOTHRUST_N1_COMMANDED:1", "number");
                // _values[1] = SimVar.GetSimVarValue("L:A32NX_AUTOTHRUST_N1_COMMANDED:2", "number");
                _values[0] = 0;
                _values[1] = 0;
            } else {
                _values[0] = 0;
                _values[1] = 0;
            }
        }
        getN1GaugeExtraMessage() {
            if (Simplane.getEngineThrottle(this.index) < 0) {
                return "REV";
            } else if (this.timerAvail >= 0) {
                return "AVAIL";
            } else {
                return "";
            }
        }
        getEngineStartStatus() {
            const engineId = this.index + 1;
            const value = (SimVar.GetSimVarValue("GENERAL ENG STARTER:" + engineId, "bool"));
            return value;
        }
        getIgnitionStatus() {
            const engineId = this.index + 1;
            const value = (SimVar.GetSimVarValue("TURB ENG IS IGNITING:" + engineId, "bool"));
            return value;
        }
    }
    A320_Neo_UpperECAM.Engine = Engine;
    class LinesStyleComponent_Base {
        constructor(_svgRoot) {
            if (_svgRoot != null) {
                const line = document.createElementNS(Avionics.SVG.NS, "line");
                line.setAttribute("x1", this.getLineX1());
                line.setAttribute("x2", this.getLineX2());
                line.setAttribute("y1", "72%");
                line.setAttribute("y2", "58%");
                line.setAttribute("stroke-linecap", "round");
                line.style.stroke = "rgb(255, 255, 255)";
                line.style.strokeWidth = "4";
                _svgRoot.appendChild(line);
                this.valueText = A320_Neo_UpperECAM.createSVGText("--.-", "Value", this.getValueTextX(), "88%", "bottom");
                this.valueText2 = A320_Neo_UpperECAM.createSVGText("", "decimal", this.getValueTextX2(), "88%", "bottom");
                this.valueTextpoint = A320_Neo_UpperECAM.createSVGText("", "decimalpoint", this.getValueTextXpoint(), "88%", "bottom");
                //createRectangle(_class, _x, _y, _width, _height) {
                this.N2Box = A320_Neo_UpperECAM.createRectangle("activeEngine", this.getBoxX(), "70", "150", "40");
                _svgRoot.appendChild(this.N2Box);
                _svgRoot.appendChild(this.valueText);
                _svgRoot.appendChild(this.valueText2);
                _svgRoot.appendChild(this.valueTextpoint);
            }
            this.refresh(false, 0, 0, true);
        }
        refresh(_active, _value, _valueDisplayPrecision, _force = false, _title = "", _displayEngine = "inactiveEngine") {
            if ((this.isActive != _active) || (this.currentValue != _value) || _force) {
                this.N2Box.setAttribute("class", _displayEngine);
                this.isActive = _active;
                this.currentValue = _value;
                if (this.valueText != null) {
                    const valueClass = this.isActive ? "Value" : "Inactive";
                    if (this.isActive) {
                        if (_valueDisplayPrecision > 0) {
                            const dx = 0;
                            if (this.currentValue.toFixed(_valueDisplayPrecision) >= 10) {
                                this.valueText2.setAttribute("dx", "2%");
                                this.valueTextpoint.setAttribute("dx", "2%");
                            } else {
                                this.valueText2.setAttribute("dx", "0%");
                                this.valueTextpoint.setAttribute("dx", "0%");
                            }
                            const strArray = this.currentValue.toFixed(_valueDisplayPrecision).split(".");
                            const wholeNumber = strArray[0];
                            this.valueText.textContent = wholeNumber;
                            this.valueText.setAttribute("x", this.getValueTextX());
                            this.valueText.setAttribute("class", valueClass);
                            const decimal = strArray[1];
                            this.valueText2.textContent = decimal;
                            this.valueText2.setAttribute("class", valueClass + " decimal");
                            this.valueText2.setAttribute("y", "88%");
                            this.valueTextpoint.textContent = ".";
                            this.valueTextpoint.setAttribute("class", valueClass + " decimalpoint");

                        } else {
                            if (_title == "FF") {
                                this.valueText.setAttribute("x", this.getValueTextXFF());
                            }
                            this.valueText.setAttribute("class", valueClass);
                            this.valueText.textContent = this.currentValue.toFixed(_valueDisplayPrecision);
                        }

                    } else {
                        this.valueText.textContent = "XX";
                        this.valueTextpoint.textContent = "";
                        this.valueText2.textContent = "";
                        this.valueText.setAttribute("class", valueClass);
                        this.valueText2.setAttribute("class", valueClass);
                        this.valueText.setAttribute("x", this.getValueTextX2());
                    }
                }
            }
        }
    }
    A320_Neo_UpperECAM.LinesStyleComponent_Base = LinesStyleComponent_Base;
    class LinesStyleComponent_Left extends LinesStyleComponent_Base {
        getLineX1() {
            return "34%";
        }
        getLineX2() {
            return "42%";
        }
        getValueTextX() {
            return "14%";
        }
        getBoxX() {
            return "15";
        }
        getValueTextX2() {
            return "20%";
        }
        getValueTextXpoint() {
            return "17%";
        }
        getValueTextXFF() {
            return "17%";
        }
    }
    A320_Neo_UpperECAM.LinesStyleComponent_Left = LinesStyleComponent_Left;
    class LinesStyleComponent_Right extends LinesStyleComponent_Base {
        getLineX1() {
            return "66%";
        }
        getLineX2() {
            return "58%";
        }
        getValueTextX() {
            return "79%";
        }
        getBoxX() {
            return "538";
        }
        getValueTextX2() {
            return "85%";
        }
        getValueTextXpoint() {
            return "82%";
        }
        getValueTextXFF() {
            return "82%";
        }
    }
    A320_Neo_UpperECAM.LinesStyleComponent_Right = LinesStyleComponent_Right;
    class LinesStyleInfo {
        constructor(_divMain, _bottomValue) {
            this.conversionWeight = parseFloat(NXDataStore.get("CONFIG_USING_METRIC_UNIT", "1"));
            const svgRoot = document.createElementNS(Avionics.SVG.NS, "svg");
            svgRoot.appendChild(A320_Neo_UpperECAM.createSVGText(this.getTitle(), "Title", "50%", "65%", "bottom"));
            svgRoot.appendChild(A320_Neo_UpperECAM.createSVGText(this.getUnit(this.conversionWeight === 1), "Unit", "50%", "100%", "bottom"));
            this.leftComponent = new LinesStyleComponent_Left(svgRoot);
            this.rightComponent = new LinesStyleComponent_Right(svgRoot);
            const div = A320_Neo_UpperECAM.createDiv("LineStyleInfos");
            div.style.bottom = _bottomValue;
            div.appendChild(svgRoot);
            _divMain.appendChild(div);
        }
        update(_deltaTime) {
            if (this.leftComponent != null) {
                this.leftComponent.refresh((SimVar.GetSimVarValue("L:A32NX_FADEC_POWERED_ENG1", "Bool") == 1), this.getValue(1), this.getValueStringPrecision(), false, this.getTitle(), this.getDisplayActiveEngine(1));
            }
            if (this.rightComponent != null) {
                this.rightComponent.refresh((SimVar.GetSimVarValue("L:A32NX_FADEC_POWERED_ENG2", "Bool") == 1), this.getValue(2), this.getValueStringPrecision(), false, this.getTitle(), this.getDisplayActiveEngine(2));
            }
        }
        getValueStringPrecision() {
            return 0;
        }
        getEngineStartStatus(_engine) {
            const value = (SimVar.GetSimVarValue("GENERAL ENG STARTER:" + _engine, "bool"));
            return value;
        }
        getIgnitionStatus(_engine) {
            const value = (SimVar.GetSimVarValue("TURB ENG IS IGNITING:" + _engine, "bool"));
            return value;
        }
    }
    A320_Neo_UpperECAM.LinesStyleInfo = LinesStyleInfo;
    class LinesStyleInfo_N2 extends LinesStyleInfo {
        getTitle() {
            return "N2";
        }
        getUnit() {
            return "%";
        }
        getValue(_engine) {
            const imbalance = SimVar.GetSimVarValue("L:A32NX_ENGINE_IMBALANCE", "number");
            if (parseInt(imbalance.toString().substr(0, 1)) == _engine) {
                var n2_i = parseFloat(imbalance.toString().substr(5, 2)) / 100;
            } else {
                var n2_i = parseFloat(0);
            }
            const name = "ENG N2 RPM:" + _engine;
            const percent = SimVar.GetSimVarValue(name, "percent") - n2_i;
            return percent;
        }
        getDisplayActiveEngine(_engine) {
            if (this.getValue(_engine) < 57.8 && this.getEngineStartStatus(_engine) && this.getIgnitionStatus(_engine)) {
                return "activeEngine";
            } else {
                return "inactiveEngine";
            }
        }
        getValueStringPrecision() {
            return 1;
        }
    }
    A320_Neo_UpperECAM.LinesStyleInfo_N2 = LinesStyleInfo_N2;
    class LinesStyleInfo_FF extends LinesStyleInfo {
        constructor(_divMain, _bottomValue) {
            super(_divMain, _bottomValue);
            this.gallonToKG = SimVar.GetSimVarValue("FUEL WEIGHT PER GALLON", "kilogram");
            this.conversionWeight = parseFloat(NXDataStore.get("CONFIG_USING_METRIC_UNIT", "1"));
        }
        getTitle() {
            return "FF";
        }
        getUnit(_isMetric = true) {
            return _isMetric ? "KG/H" : "LBS/H";
        }
        getValue(_engine, _conversion) {
            const imbalance = SimVar.GetSimVarValue("L:A32NX_ENGINE_IMBALANCE", "number");
            if (parseInt(imbalance.toString().substr(0, 1)) == _engine) {
                var ff_i = parseInt(imbalance.toString().substr(3, 2));
            } else {
                var ff_i = parseInt(0);
            }
            let ff = SimVar.GetSimVarValue("L:A32NX_ENGINE_FF:" + _engine, "number") - ff_i;
            ff -= ff % 20;
            if (ff < 0) {
                return 0;
            }
            return ff;
        }
        getDisplayActiveEngine(_engine) {
            return "inactiveEngine";
        }
    }
    A320_Neo_UpperECAM.LinesStyleInfo_FF = LinesStyleInfo_FF;
    class InfoTopPanel extends A320_Neo_UpperECAM.PanelBase {
        create() {
            super.create();
            if (this.divMain != null) {
                const statusDiv = A320_Neo_UpperECAM.createDiv("Status");
                this.throttleState = A320_Neo_UpperECAM.createDiv("ThrottleState");
                statusDiv.appendChild(this.throttleState);
                this.throttleValue = A320_Neo_UpperECAM.createDiv("ThrottleValue");
                statusDiv.appendChild(this.throttleValue);
                this.flexTemperature = A320_Neo_UpperECAM.createDiv("FlexTemperature");
                statusDiv.appendChild(this.flexTemperature);
                this.divMain.appendChild(statusDiv);
                const fuelOnBoardDiv = A320_Neo_UpperECAM.createDiv("FuelOnBoard");
                fuelOnBoardDiv.appendChild(A320_Neo_UpperECAM.createDiv("Title", "", "FOB :"));
                this.fobValue = A320_Neo_UpperECAM.createDiv("Value");
                fuelOnBoardDiv.appendChild(this.fobValue);
                this.conversionWeight = parseFloat(NXDataStore.get("CONFIG_USING_METRIC_UNIT", "1"));
                this.fobUnit = A320_Neo_UpperECAM.createDiv("Unit", "", this.conversionWeight === 1 ? "KG" : "LBS");
                fuelOnBoardDiv.appendChild(this.fobUnit);
                this.divMain.appendChild(fuelOnBoardDiv);
            }

            this.setThrottle(false, 0, ThrottleMode.UNKNOWN, true, SimVar.GetSimVarValue("L:A32NX_FMGC_FLIGHT_PHASE", "number"));
            this.setFlexTemperature(false, 0, true);
            this.setFuelOnBoard(0, true);
        }

        update(_deltaTime) {
            super.update(_deltaTime);

            if ((SimVar.GetSimVarValue("L:A32NX_FADEC_POWERED_ENG1", "Bool") == 1) || (SimVar.GetSimVarValue("L:A32NX_FADEC_POWERED_ENG2", "Bool") == 1)) {
                const onGround = Simplane.getIsGrounded();
                const thrustLimitType = SimVar.GetSimVarValue("L:A32NX_AUTOTHRUST_THRUST_LIMIT_TYPE", "number");
                const thrustLimit = SimVar.GetSimVarValue("L:A32NX_AUTOTHRUST_THRUST_LIMIT", "number");
                let throttleMode = ThrottleMode.UNKNOWN;
                switch (thrustLimitType) {
                    case 1:
                        throttleMode = ThrottleMode.CLIMB;
                        this.setFlexTemperature(false);
                        break;
                    case 2:
                        throttleMode = ThrottleMode.FLEX_MCT;
                        this.setFlexTemperature(false);
                        break;
                    case 3:
                        throttleMode = ThrottleMode.FLEX_MCT;
                        const flexTemp = Simplane.getFlexTemperature();
                        this.setFlexTemperature(flexTemp !== 0, flexTemp);
                        break;
                    case 4:
                        throttleMode = ThrottleMode.TOGA;
                        this.setFlexTemperature(false);
                        break;
                    case 5:
                        throttleMode = ThrottleMode.REVERSE;
                        this.setFlexTemperature(false);
                        break;
                    default:
                        throttleMode = ThrottleMode.UNKNOWN;
                        this.setFlexTemperature(false);
                        break;
                }
                this.setThrottle(true, thrustLimit, throttleMode, onGround, Simplane.getCurrentFlightPhase());
            } else {
                this.setThrottle(false);
                this.setFlexTemperature(false);
            }

            this.setFuelOnBoard(SimVar.GetSimVarValue("FUEL TOTAL QUANTITY WEIGHT", "kg") * this.conversionWeight);
        }

        /**
         * @param _grounded {boolen}
         * @param _min {string}
         * @param _med {string}
         * @param _max {string}
         * @returns string
         */
        getThrustRatingMode(_grounded, _min = "CLB", _med = "FLX", _max = "TOGA") {
            if (_grounded || (this.currentPhase < FmgcFlightPhases.CLIMB)) {
                return Simplane.getFlexTemperature() > 0 ? _med : _max;
            }
            return _min;
        }

        /**
         * @param _active {boolean}
         * @param _value {number}
         * @param _mode {ThrottleMode}
         * @param _grounded {boolean}
         * @param _phase {FlightPhase}
         */
        setThrottle(_active, _value = 0, _mode = ThrottleMode.UNKNOWN, _grounded = true, _phase = SimVar.GetSimVarValue("L:A32NX_FMGC_FLIGHT_PHASE", "number")) {
            if (_active !== this.currentThrottleIsActive || _value !== this.currentThrottleValue || _mode !== this.currentThrottleMode || _grounded !== this.currentGrounded || this.currentStart || _phase != this.currentPhase) {
                this.currentThrottleIsActive = _active;
                this.currentThrottleValue = _value;
                this.currentThrottleMode = _mode;
                this.currentGrounded = _grounded;
                this.currentStart = false;
                this.currentPhase = _phase;

                if (this.throttleState != null) {
                    if (_active && (this.currentThrottleMode !== ThrottleMode.UNKNOWN)) {
                        this.throttleState.className = "active";
                        switch (this.currentThrottleMode) {
                            case ThrottleMode.TOGA:
                            {
                                this.throttleState.textContent = "TOGA";
                                break;
                            }
                            case ThrottleMode.FLEX_MCT:
                            {
                                if (this.flexTemperatureIsActive) {
                                    this.throttleState.textContent = "FLX";
                                } else {
                                    this.throttleState.textContent = "MCT";
                                }
                                break;
                            }
                            case ThrottleMode.CLIMB:
                            {
                                this.throttleState.textContent = "CLB";
                                break;
                            }
                            case ThrottleMode.REVERSE:
                            {
                                this.throttleState.textContent = "MREV";
                                break;
                            }
                            default: {
                                this.throttleState.className = "inactive";
                                this.throttleState.textContent = "XX";
                                break;
                            };
                        }
                    } else {
                        this.throttleState.className = "inactive";
                        this.throttleState.textContent = "XX";
                    }
                }
                if (this.throttleValue && this.throttleState) {
                    this.throttleValue.className = _active ? "active" : "inactive";
                    if (_active) {
                        if (_value >= 0) {
                            this.throttleState.style.visibility = "visible";
                            this.throttleValue.style.visibility = "visible";
                            const strArray = _value.toFixed(1).split(".");
                            const wholeNumber = strArray[0];
                            const decimal = strArray[1];
                            const throttleVal = wholeNumber + ".<span>" + decimal + "</span>";
                            this.throttleValue.innerHTML = throttleVal;
                        } else {
                            this.throttleState.style.visibility = "hidden";
                            this.throttleValue.style.visibility = "hidden";
                            this.throttleValue.textContent = "";
                        }
                    } else {
                        this.throttleValue.textContent = "XX";
                    }
                }
            }
        }

        setFlexTemperature(_active, _value = 0, _force = false) {
            if ((_active != this.flexTemperatureIsActive) || (_value != this.currentFlexTemperature) || _force) {
                this.currentFlexTemperature = _value;
                this.flexTemperatureIsActive = _active;
                if (this.flexTemperature != null) {
                    this.flexTemperature.textContent = fastToFixed(this.currentFlexTemperature, 0);
                    this.flexTemperature.className = this.flexTemperatureIsActive ? "active" : "inactive";
                }
            }
        }
        setFuelOnBoard(_value, _force = false) {
            if ((this.currentFOBValue !== _value) || _force) {
                this.currentFOBValue = _value - (_value % 20);
                if (this.fobValue != null) {
                    this.fobValue.textContent = fastToFixed(this.currentFOBValue < 0 ? 0 : this.currentFOBValue, 0);
                }
            }
        }
    }
    A320_Neo_UpperECAM.InfoTopPanel = InfoTopPanel;
    class FlapsPanel extends A320_Neo_UpperECAM.PanelBase {
        constructor() {
            super(...arguments);
            this.viewBoxSize = new Vec2(500, 125);
            this.dotSize = 5;
            this.slatArrowPathD = "m20,-12 l-27,8 l-6,18 l27,-8 l6,-18";
            this.slatDotPositions = [
                new Vec2(160, 37),
                new Vec2(110, 52),
                new Vec2(68, 65),
                new Vec2(26, 78)
            ];
            this.flapArrowPathD = "m-20,-12 l31,6 l0,19, l-21,-5 l-10,-20";
            this.flapDotPositions = [
                new Vec2(220, 37),
                new Vec2(280, 50),
                new Vec2(327, 61),
                new Vec2(375, 72),
                new Vec2(423, 83)
            ];
            this.sTextPos = new Vec2(66, 32);
            this.fTextPos = new Vec2(330, 32);
            this.currentStateTextPos = new Vec2(190, 87);
            this.mainShapeCorners = [
                new Vec2(180, 25),
                new Vec2(200, 25),
                new Vec2(210, 45),
                new Vec2(174, 43)
            ];
        }
        create() {
            super.create();
            const rootSVG = document.createElementNS(Avionics.SVG.NS, "svg");
            rootSVG.setAttribute("id", "DiagramSVG");
            rootSVG.setAttribute("viewBox", "0 0 " + this.viewBoxSize.x + " " + this.viewBoxSize.y);
            this.divMain.appendChild(rootSVG);
            this.hideOnInactiveGroup = document.createElementNS(Avionics.SVG.NS, "g");
            this.hideOnInactiveGroup.setAttribute("id", "HideOnInactive");
            rootSVG.appendChild(this.hideOnInactiveGroup);
            const shape = document.createElementNS(Avionics.SVG.NS, "path");
            shape.setAttribute("class", "shape");
            {
                const d = [
                    "M", this.mainShapeCorners[0].x, ",", this.mainShapeCorners[0].y,
                    " L", this.mainShapeCorners[1].x, ",", this.mainShapeCorners[1].y,
                    " L", this.mainShapeCorners[2].x, ",", this.mainShapeCorners[2].y,
                    " L", this.mainShapeCorners[3].x, ",", this.mainShapeCorners[3].y,
                    " Z"
                ].join("");
                shape.setAttribute("d", d);
            }
            rootSVG.appendChild(shape);
            this.currentSlatsArrow = document.createElementNS(Avionics.SVG.NS, "path");
            this.currentSlatsArrow.setAttribute("class", "currentArrow");
            rootSVG.appendChild(this.currentSlatsArrow);
            this.currentFlapsArrow = document.createElementNS(Avionics.SVG.NS, "path");
            this.currentFlapsArrow.setAttribute("class", "currentArrow");
            rootSVG.appendChild(this.currentFlapsArrow);
            this.targetSlatsArrow = document.createElementNS(Avionics.SVG.NS, "polygon");
            this.targetSlatsArrow.setAttribute("class", "targetDot");
            rootSVG.appendChild(this.targetSlatsArrow);
            this.targetFlapsArrow = document.createElementNS(Avionics.SVG.NS, "polygon");
            this.targetFlapsArrow.setAttribute("class", "targetDot");
            rootSVG.appendChild(this.targetFlapsArrow);
            this.targetSlatsArrowsStrings = new Array();
            this.targetSlatsArrowsStrings.push(this.generateArrowPathD(this.slatArrowPathD, null, this.slatDotPositions[0], this.slatDotPositions[1], 0));
            this.targetSlatsArrowsStrings.push(this.generateArrowPathD(this.slatArrowPathD, null, this.slatDotPositions[0], this.slatDotPositions[1], 1));
            this.targetSlatsArrowsStrings.push(this.generateArrowPathD(this.slatArrowPathD, null, this.slatDotPositions[1], this.slatDotPositions[2], 1));
            this.targetSlatsArrowsStrings.push(this.generateArrowPathD(this.slatArrowPathD, null, this.slatDotPositions[1], this.slatDotPositions[2], 1));
            this.targetSlatsArrowsStrings.push(this.generateArrowPathD(this.slatArrowPathD, null, this.slatDotPositions[2], this.slatDotPositions[3], 1));
            this.targetFlapsArrowsStrings = new Array();
            this.targetFlapsArrowsStrings.push(this.generateArrowPathD(this.flapArrowPathD, null, this.flapDotPositions[0], this.flapDotPositions[1], 0));
            this.targetFlapsArrowsStrings.push(this.generateArrowPathD(this.flapArrowPathD, null, this.flapDotPositions[0], this.flapDotPositions[1], 1));
            this.targetFlapsArrowsStrings.push(this.generateArrowPathD(this.flapArrowPathD, null, this.flapDotPositions[1], this.flapDotPositions[2], 1));
            this.targetFlapsArrowsStrings.push(this.generateArrowPathD(this.flapArrowPathD, null, this.flapDotPositions[2], this.flapDotPositions[3], 1));
            this.targetFlapsArrowsStrings.push(this.generateArrowPathD(this.flapArrowPathD, null, this.flapDotPositions[3], this.flapDotPositions[4], 1));
            this.targetSlatsDots = new Array();
            this.targetSlatsDots.push(A320_Neo_UpperECAM.createSlatParallelogram("targetDot", this.slatDotPositions[1].x, this.slatDotPositions[1].y + 20).getAttribute("points"));
            this.targetSlatsDots.push(A320_Neo_UpperECAM.createSlatParallelogram("targetDot", this.slatDotPositions[2].x, this.slatDotPositions[2].y + 20).getAttribute("points"));
            this.targetSlatsDots.push(A320_Neo_UpperECAM.createSlatParallelogram("targetDot", this.slatDotPositions[2].x, this.slatDotPositions[2].y + 20).getAttribute("points"));
            this.targetSlatsDots.push(A320_Neo_UpperECAM.createSlatParallelogram("targetDot", this.slatDotPositions[3].x, this.slatDotPositions[3].y + 20).getAttribute("points"));
            this.targetFlapsDots = new Array();
            this.targetFlapsDots.push(A320_Neo_UpperECAM.createFlapParallelogram("targetDot", this.flapDotPositions[1].x, this.flapDotPositions[1].y + 20).getAttribute("points"));
            this.targetFlapsDots.push(A320_Neo_UpperECAM.createFlapParallelogram("targetDot", this.flapDotPositions[2].x, this.flapDotPositions[2].y + 20).getAttribute("points"));
            this.targetFlapsDots.push(A320_Neo_UpperECAM.createFlapParallelogram("targetDot", this.flapDotPositions[3].x, this.flapDotPositions[3].y + 20).getAttribute("points"));
            this.targetFlapsDots.push(A320_Neo_UpperECAM.createFlapParallelogram("targetDot", this.flapDotPositions[4].x, this.flapDotPositions[4].y + 20).getAttribute("points"));
            this.hideOnInactiveGroup.appendChild(A320_Neo_UpperECAM.createSVGText("S", "sfText", this.sTextPos.x.toString(), this.sTextPos.y.toString()));
            this.hideOnInactiveGroup.appendChild(A320_Neo_UpperECAM.createSVGText("F", "sfText", this.fTextPos.x.toString(), this.fTextPos.y.toString()));
            this.currentStateText = A320_Neo_UpperECAM.createSVGText("", "state", this.currentStateTextPos.x.toString(), this.currentStateTextPos.y.toString());
            this.hideOnInactiveGroup.appendChild(this.currentStateText);
            for (var i = 1; i < this.slatDotPositions.length; ++i) {
                this.hideOnInactiveGroup.appendChild(A320_Neo_UpperECAM.createSlatParallelogram("dot", this.slatDotPositions[i].x, this.slatDotPositions[i].y));
            }
            for (var i = 1; i < this.flapDotPositions.length; ++i) {
                this.hideOnInactiveGroup.appendChild(A320_Neo_UpperECAM.createFlapParallelogram("dot", this.flapDotPositions[i].x, this.flapDotPositions[i].y));
            }
            this.deactivate();
            this.cockpitSettings = SimVar.GetGameVarValue("", "GlassCockpitSettings");
        }
        activate() {
            this.isActive = true;
            if (this.hideOnInactiveGroup != null) {
                this.hideOnInactiveGroup.style.display = "block";
            }
        }
        deactivate() {
            this.isActive = false;
            this.currentSlatsAngle = 0;
            this.currentFlapsAngle = 0;
            if (this.hideOnInactiveGroup != null) {
                this.hideOnInactiveGroup.style.display = "none";
            }
            if (this.currentSlatsArrow != null) {
                this.currentSlatsArrow.setAttribute("d", this.targetSlatsArrowsStrings[0]);
            }
            if (this.currentFlapsArrow != null) {
                this.currentFlapsArrow.setAttribute("d", this.targetFlapsArrowsStrings[0]);
            }
            if (this.targetSlatsArrow != null) {
                this.targetSlatsArrow.style.display = "none";
            }
            if (this.targetFlapsArrow != null) {
                this.targetFlapsArrow.style.display = "none";
            }
        }
        update(_deltaTime) {
            super.update(_deltaTime);
            const slatsAngle = (SimVar.GetSimVarValue("LEADING EDGE FLAPS LEFT ANGLE", "degrees") + SimVar.GetSimVarValue("LEADING EDGE FLAPS RIGHT ANGLE", "degrees")) * 0.5;
            const flapsAngle = Math.max(0, (SimVar.GetSimVarValue("TRAILING EDGE FLAPS LEFT ANGLE", "degrees") + SimVar.GetSimVarValue("TRAILING EDGE FLAPS RIGHT ANGLE", "degrees")) * 0.5);
            const handleIndex = Simplane.getFlapsHandleIndex(true);
            const slatsTargetIndex = handleIndex;
            let flapsTargetIndex = handleIndex;
            const slatsAngleChanged = (this.currentSlatsAngle != slatsAngle);
            const flapsAngleChanged = (this.currentFlapsAngle != flapsAngle);
            if ((slatsAngleChanged || flapsAngleChanged) && ((this.cockpitSettings != null) && (this.cockpitSettings.FlapsLevels != null) && this.cockpitSettings.FlapsLevels.initialised)) {
                if (slatsAngleChanged) {
                    this.currentSlatsAngle = slatsAngle;
                    let dSlatsArrow = "";
                    if (this.currentSlatsAngle <= this.cockpitSettings.FlapsLevels.slatsAngle[0]) {
                        dSlatsArrow = this.targetSlatsArrowsStrings[0];
                    } else if (this.currentSlatsAngle <= this.cockpitSettings.FlapsLevels.slatsAngle[1]) {
                        var lerp = Utils.Clamp(this.currentSlatsAngle / 18, 0, 1);
                        dSlatsArrow = this.generateArrowPathD(this.slatArrowPathD, this.mainShapeCorners[0], this.slatDotPositions[0], this.slatDotPositions[1], lerp);
                    } else if (this.currentSlatsAngle <= this.cockpitSettings.FlapsLevels.slatsAngle[3]) {
                        var lerp = Utils.Clamp((this.currentSlatsAngle - 18) / 4, 0, 1);
                        dSlatsArrow = this.generateArrowPathD(this.slatArrowPathD, this.mainShapeCorners[0], this.slatDotPositions[1], this.slatDotPositions[2], lerp);
                    } else if (this.currentSlatsAngle <= this.cockpitSettings.FlapsLevels.slatsAngle[4]) {
                        var lerp = Utils.Clamp((this.currentSlatsAngle - 22) / 5, 0, 1);
                        dSlatsArrow = this.generateArrowPathD(this.slatArrowPathD, this.mainShapeCorners[0], this.slatDotPositions[2], this.slatDotPositions[3], lerp);
                    }
                    if (this.currentSlatsArrow != null) {
                        this.currentSlatsArrow.setAttribute("d", dSlatsArrow);
                    }
                }
                if (flapsAngleChanged) {
                    this.currentFlapsAngle = flapsAngle;
                    let dFlapsArrow = "";
                    if (this.currentFlapsAngle <= this.cockpitSettings.FlapsLevels.flapsAngle[0]) {
                        dFlapsArrow = this.targetFlapsArrowsStrings[0];
                    } else if (this.currentFlapsAngle <= this.cockpitSettings.FlapsLevels.flapsAngle[1]) {
                        var lerp = Utils.Clamp(this.currentFlapsAngle / 10, 0, 1);
                        dFlapsArrow = this.generateArrowPathD(this.flapArrowPathD, this.mainShapeCorners[1], this.flapDotPositions[0], this.flapDotPositions[1], lerp);
                    } else if (this.currentFlapsAngle <= this.cockpitSettings.FlapsLevels.flapsAngle[2]) {
                        var lerp = Utils.Clamp((this.currentFlapsAngle - 10) / 5, 0, 1);
                        dFlapsArrow = this.generateArrowPathD(this.flapArrowPathD, this.mainShapeCorners[1], this.flapDotPositions[1], this.flapDotPositions[2], lerp);
                    } else if (this.currentFlapsAngle <= this.cockpitSettings.FlapsLevels.flapsAngle[3]) {
                        var lerp = Utils.Clamp((this.currentFlapsAngle - 15) / 5, 0, 1);
                        dFlapsArrow = this.generateArrowPathD(this.flapArrowPathD, this.mainShapeCorners[1], this.flapDotPositions[2], this.flapDotPositions[3], lerp);
                    } else if (this.currentFlapsAngle <= this.cockpitSettings.FlapsLevels.flapsAngle[4] + 5) {
                        var lerp = Utils.Clamp((this.currentFlapsAngle - 20) / 20, 0, 1);
                        dFlapsArrow = this.generateArrowPathD(this.flapArrowPathD, this.mainShapeCorners[1], this.flapDotPositions[3], this.flapDotPositions[4], lerp);
                    }
                    if (this.currentFlapsArrow != null) {
                        this.currentFlapsArrow.setAttribute("d", dFlapsArrow);
                    }
                }
                if (this.currentStateText != null) {
                    switch (handleIndex) {
                        case 1:
                        {
                            if (flapsAngle > 9.99) {
                                this.currentStateText.textContent = "1+F";
                            } else {
                                this.currentStateText.textContent = "1";
                                flapsTargetIndex = 0;
                            }
                            break;
                        }
                        case 2:
                        {
                            this.currentStateText.textContent = "2";
                            break;
                        }
                        case 3:
                        {
                            this.currentStateText.textContent = "3";
                            break;
                        }
                        case 4:
                        {
                            this.currentStateText.textContent = "FULL";
                            break;
                        }
                        default:
                        {
                            this.currentStateText.textContent = "";
                            break;
                        }
                    }
                }
                const active = ((this.currentSlatsAngle > 0) || (this.currentFlapsAngle > 0));
                if (this.isActive != active) {
                    if (active) {
                        this.activate();
                    } else {
                        this.deactivate();
                    }
                }
            }
            if (this.targetSlatsArrow != null) {
                if (slatsAngleChanged && (this.targetSlatsArrowsStrings != null) && (slatsTargetIndex >= 0) && (slatsTargetIndex < this.targetSlatsArrowsStrings.length)) {
                    this.targetSlatsArrow.setAttribute("points", this.targetSlatsDots[slatsTargetIndex - 1]);
                    this.targetSlatsArrow.style.display = "block";
                    this.currentStateText.setAttribute("class", "stateInTransit");
                } else {
                    this.targetSlatsArrow.style.display = "none";
                    if (flapsAngleChanged && (this.targetFlapsArrowsStrings != null) && (flapsTargetIndex >= 0) && (flapsTargetIndex < this.targetFlapsArrowsStrings.length)) {
                        this.currentStateText.setAttribute("class", "state");
                    }
                }
            }
            if (this.targetFlapsArrow != null) {
                if (flapsAngleChanged && (this.targetFlapsArrowsStrings != null) && (flapsTargetIndex >= 0) && (flapsTargetIndex < this.targetFlapsArrowsStrings.length)) {
                    this.targetFlapsArrow.setAttribute("points", this.targetFlapsDots[flapsTargetIndex - 1]);
                    this.targetFlapsArrow.style.display = "block";
                    this.currentStateText.setAttribute("class", "stateInTransit");
                } else {
                    this.targetFlapsArrow.style.display = "none";
                    if (!(slatsAngleChanged && (this.targetSlatsArrowsStrings != null) && (slatsTargetIndex >= 0) && (slatsTargetIndex < this.targetSlatsArrowsStrings.length))) {
                        this.currentStateText.setAttribute("class", "state");
                    }
                }
            }
        }
        generateArrowPathD(_baseString, _origin, _start, _end, _currentLineProgress) {
            const dir = new Vec2(_end.x - _start.x, _end.y - _start.y);
            const finalPos = new Vec2();
            if (_currentLineProgress >= 1) {
                finalPos.x = _end.x;
                finalPos.y = _end.y;
            } else if (_currentLineProgress > 0) {
                finalPos.x = (_start.x + (dir.x * _currentLineProgress));
                finalPos.y = (_start.y + (dir.y * _currentLineProgress));
            } else {
                finalPos.x = _start.x;
                finalPos.y = _start.y;
            }
            let d = "M" + finalPos.x + "," + finalPos.y;
            d += _baseString;
            if (_origin != null) {
                d += " L" + _origin.x + "," + _origin.y;
            }
            return d;
        }
    }
    A320_Neo_UpperECAM.FlapsPanel = FlapsPanel;

    /**
     * Represents an ECAM message panel that can display a list of messages and failures
     */
    class EcamMessagePanel extends A320_Neo_UpperECAM.PanelBase {
        /**
         * @param {*} _parent
         * @param {string} _id
         * @param {number} _max The maximum number of lines this panel can display
         * @param {*} _messages Object containing all posible messages this panel can display
         */
        constructor(_parent, _id, _max, _messages) {
            super(_parent, _id);
            this.allDivs = [];
            this.maxLines = _max;
            this.messages = _messages;
            this.currentLine = 0;
            this.activeCategories = [];
            this.hasActiveFailures = false;
            this.lastWarningsCount = {
                2: 0,
                3: 0
            };
            this.knownFailures = [];
            this.clearedMessages = [];
            this.inopSystems = {};
            this.landASAP = 0;
            this.overflow = false;
            this.failMode = false;
            this.activePage = null;
            this.timers = {};
        }
        init() {
            super.init();
            for (let i = 0; i < this.maxLines; i++) {
                this.addDiv();
            }
        }
        update() {
            this.activeCategories = [];
            this.currentLine = 0;
            this.overflow = false;
            const warningsCount = {
                2: 0,
                3: 0
            };
            for (const div of this.allDivs) {
                div.innerHTML = "";
            }
            const activeFailures = this.getActiveFailures();
            for (const i in activeFailures) {
                const category = activeFailures[i];
                for (const failure of category.messages) {
                    this.addLine("fail-" + failure.level, category.name, failure.message, failure.action, failure.alwaysShowCategory);
                    warningsCount[failure.level] = warningsCount[failure.level] + 1;
                    if (failure.actions != null) {
                        let firstAction = true;
                        for (const action of failure.actions) {
                            if (action.isCompleted == null || !action.isCompleted()) {
                                if ((action.style != "action-timer" || firstAction)) {
                                    this.addLine(action.style, null, action.message, action.action, false, action.id, action.duration, action.timerMessage);
                                }
                                firstAction = false;
                            }
                        }
                    }
                }
            }
            if (!this.hasActiveFailures) {
                const activeMessages = this.getActiveMessages();
                for (const message of activeMessages) {
                    if (!this.failMode || message.important) {
                        let style = message.style;
                        if (typeof style === "function") {
                            style = style();
                        }
                        this.addLine(style || "InfoIndication", null, message.message, null);
                    }
                }
            }

            if (warningsCount[3] > this.lastWarningsCount[3]) {
                // console.warn(warningsCount[3]);
                SimVar.SetSimVarValue("L:A32NX_MASTER_WARNING", "Bool", 1);
            }
            if (warningsCount[2] > this.lastWarningsCount[2]) {
                SimVar.SetSimVarValue("L:A32NX_MASTER_CAUTION", "Bool", 1);
            }
            this.lastWarningsCount[2] = parseInt(warningsCount[2]);
            this.lastWarningsCount[3] = parseInt(warningsCount[3]);

            for (const system in this.inopSystems) {
                if (this.inopSystems[system] == true) {
                    SimVar.SetSimVarValue("L:A32NX_ECAM_INOP_SYS_" + system, "Bool", 1);
                } else {
                    SimVar.SetSimVarValue("L:A32NX_ECAM_INOP_SYS_" + system, "Bool", 0);
                    delete this.inopSystems[system];
                }
            }
        }
        addLine(_style, _category, _message, _action, _alwaysShowCategory = false, _id, _duration, _timerMessage) {
            if (this.currentLine < this.maxLines) {
                const div = this.allDivs[this.currentLine];
                div.innerHTML = "";
                div.className = _style;
                if (div != null) {

                    //Category
                    if (_category != null && (!this.activeCategories.includes(_category) || _alwaysShowCategory)) {
                        const category = document.createElement("span");
                        category.className = "Underline";
                        category.textContent = _category;
                        div.appendChild(category);
                    }

                    //Message
                    const message = document.createElement("span");
                    switch (_style) {
                        case "action-timer":
                            const now = Date.now();
                            div.className = "action";
                            if (this.timers[_id] == null) {
                                this.timers[_id] = _duration;
                                this.lastTimerUpdate = now;
                            }
                            if (this.timers[_id] > 0) {
                                div.className = "action-timer";
                                const seconds = Math.floor(this.timers[_id]) + 1;
                                _message = `${_timerMessage} AFTER ${seconds < 10 ? "&nbsp;" : ""}${seconds}S`;
                                this.timers[_id] += (this.lastTimerUpdate - now) / 1000;
                                this.lastTimerUpdate = now;
                            }
                        case "action":
                            var msgOutput = "&nbsp;-" + _message;
                            for (var i = 0; i < (22 - _message.replace("&nbsp;", " ").length - _action.length); i++) {
                                msgOutput = msgOutput + ".";
                            }
                            msgOutput += _action;
                            break;
                        case "remark":
                            var msgOutput = "." + (_message + ":").substring(0,23);
                            break;
                        case "remark-indent":
                            var msgOutput = `&nbsp;&nbsp;&nbsp;&nbsp;.${_message}:`;
                            break;
                        default:
                            var msgOutput = " " + _message;
                            if (!_category) {
                                break;
                            }
                            if (this.activeCategories.includes(_category)) {
                                for (var i = 0; i < _category.length; i++) {
                                    msgOutput = "&nbsp;" + msgOutput;
                                }
                            }
                            break;
                    }
                    message.innerHTML = msgOutput;
                    div.appendChild(message);

                    if (!this.activeCategories.includes(_category)) {
                        this.activeCategories.push(_category);
                    }
                }
            } else {
                this.overflow = true;
            }
            this.currentLine++;
        }

        getActiveFailures() {
            const output = {};
            this.hasActiveFailures = false;
            this.hasWarnings = false;
            this.hasCautions = false;
            this.landASAP = 0;
            this.secondaryFailures = [];
            this.activePage = null;
            for (const system in this.inopSystems) {
                this.inopSystems[system] = false;
            }
            for (let i = 0; i < this.messages.failures.length; i++) {
                const messages = this.messages.failures[i].messages;
                for (let n = 0; n < messages.length; n++) {
                    const message = messages[n];
                    if (message.id == null) {
                        message.id = `${i} ${n}`;
                    }

                    if (!this.knownFailures.includes(message.id)) {
                        // only check for inhibition if the failure has not occured yet

                        if (!this.parent.fwcFlightPhase) {
                            // inhibition requires a valid flight phase
                            continue;
                        }

                        const inhibitedPhases = Array.isArray(message.flightPhasesInhib) ? message.flightPhasesInhib : [];
                        if (
                            !this.parent.inhibitOverride &&
                            inhibitedPhases.length &&
                            this.parent.isInFlightPhase(...inhibitedPhases) &&
                            !this.knownFailures.includes(message.id)
                        ) {
                            continue;
                        }
                    }

                    const isActive = message.isActive();
                    if (isActive) {
                        if (!this.knownFailures.includes(message.id) && this.parent.fwcFlightPhase) {
                            this.knownFailures.push(message.id);
                        }
                        if (!this.clearedMessages.includes(message.id)) {
                            this.hasActiveFailures = true;
                            if (message.level == 3) {
                                this.hasWarnings = true;
                            }
                            if (message.level == 2) {
                                this.hasCautions = true;
                            }
                            if (!output[i]) {
                                // Load the failure into output if it was not seen before, but take special care not to
                                // just copy the original messages.failures object, as you will mutate it.
                                output[i] = Object.assign({}, this.messages.failures[i], {messages: []});
                            }
                            output[i].messages.push(message);
                            if (this.activePage == null && message.page != null) {
                                this.activePage = message.page;
                            }
                        }
                        if (message.inopSystems != null) {
                            for (const system of message.inopSystems) {
                                this.inopSystems[system] = true;
                            }
                        }
                        if (message.landASAP == true) {
                            if (this.landASAP < message.level) {
                                this.landASAP = message.level;
                            }
                        }
                        if (message.secondaryFailures != null) {
                            for (const secondaryFailure of message.secondaryFailures) {
                                if (!this.secondaryFailures.includes(secondaryFailure)) {
                                    this.secondaryFailures.push(secondaryFailure);
                                }
                            }
                        }
                    } else {
                        if (this.knownFailures.includes(message.id)) {
                            this.knownFailures = this.knownFailures.filter(id => id !== message.id);
                        }
                        if (this.clearedMessages.includes(message.id)) {
                            // A message may reappear if it's conditions are no longer met, and then met again.
                            // We do this by only keeping messages that are active in the list of cleared messages.
                            this.clearedMessages = this.clearedMessages.filter(id => id !== message.id);
                        }
                    }
                }
            }

            return output;
        }

        clearHighestCategory() {
            const activeFailures = this.getActiveFailures();
            for (const category in activeFailures) {
                for (const failure of activeFailures[category].messages) {
                    this.clearedMessages.push(failure.id);
                }
                return;
            }
        }

        recall(_failure) {
            if (_failure != null) {
                const index = this.clearedMessages.indexOf(_failure);
                if (index > -1) {
                    this.clearedMessages.splice(index, 1);
                }
                // By adding the message to the list of known failures it will bypass any flight phase inhibition.
                // If it is not active it will be removed again from the list immediately by getActiveFailures().
                if (!this.knownFailures.includes(_failure)) {
                    this.knownFailures.push(_failure);
                }
            } else {
                this.clearedMessages = [];
            }
        }

        getActiveMessages() {
            const output = [];
            for (const message of this.messages.normal) {
                if (message.isActive()) {
                    output.push(message);
                }
            }
            return output;
        }

        addDiv() {
            if (this.divMain != null) {
                const newDiv = document.createElement("div");
                this.allDivs.push(newDiv);
                this.divMain.appendChild(newDiv);
                return newDiv;
            }
            return null;
        }
    }
    A320_Neo_UpperECAM.EcamMessagePanel = EcamMessagePanel;

    /**
     * Represents an ECAM Memo Item
     */
    class MemoItem {
        constructor(_id, _name, _action, _completed, _condition) {
            /** The unique ID of the item */
            this.id = _id;

            /** The name of the item */
            this.name = _name;

            /** The action text that is displayed after the dots */
            this.action = _action;

            /** The text that displays after the action is completed */
            this.completed = _completed;

            /** Function that returns true if the item is completed */
            this.isCompleted = _condition;
        }
    }
    A320_Neo_UpperECAM.MemoItem = MemoItem;

    /**
     * Represents an ECAM Memo
     */
    class Memo extends A320_Neo_UpperECAM.PanelBase {
        constructor(_parent, _id, _title, _items) {
            super(_parent, _id);
            this.allDivs = [];
            this.title = _title;
            this.items = _items;
        }
        init() {
            super.init();
            for (let i = 0; i < this.items.length; i++) {
                this.addItem(this.items[i], i === 0);
            }
        }
        update() {
            for (const item of this.items) {
                this.setCompleted(item, item.isCompleted());
            }
        }
        getNextAvailableDiv() {
            for (const div of this.allDivs) {
                if (div.textContent.length == 0) {
                    return div;
                }
            }
            if (this.divMain != null) {
                const newDiv = document.createElement("div");
                this.allDivs.push(newDiv);
                this.divMain.appendChild(newDiv);
                return newDiv;
            }
            return null;
        }

        /**
         * Adds an item to the memo
         * @param {MemoItem} _item The item to add to the memo
         * @param {boolean} _first Whether this is the first item
         */
        addItem(_item, _first) {
            const div = this.getNextAvailableDiv();
            if (div != null) {
                if (_first) {
                    const title = document.createElement("span");
                    title.className = "Underline";
                    title.textContent = this.title;
                    div.appendChild(title);
                }

                //Item name
                const itemName = document.createElement("span");
                itemName.innerHTML = _first ? " " + _item.name : "&nbsp;&nbsp;&nbsp;&nbsp;" + _item.name;
                div.appendChild(itemName);

                //Action
                const action = document.createElement("span");
                action.className = "Action";
                let actionText;
                if (typeof _item.action === 'function') {
                    actionText = _item.action();
                } else {
                    actionText = _item.action;
                }
                action.textContent = this.leftPad(actionText, ".", 19 - _item.name.length);
                div.appendChild(action);

                //Completed
                const completed = document.createElement("span");
                completed.className = "Completed";
                if (typeof _item.completed === 'function') {
                    completed.textContent = " " + _item.completed();
                } else {
                    completed.textContent = " " + _item.completed;
                }
                div.appendChild(completed);

                div.className = "InfoIndication";
                div.setAttribute("id", _item.id);
            }
        }

        leftPad(_text, _pad, _length) {
            for (let i = 0; i < (_length - _text.length); i++) {
                _text = _pad + _text;
            }
            return _text;
        }

        /**
         * @param {MemoItem} _item
         * @param {boolean} _completed
         */
        setCompleted(_item, _completed) {
            for (const div of this.allDivs) {
                if (div.id == _item.id) {
                    for (const child of div.children) {
                        if (child.className == "Action") {
                            child.style.display = _completed ? "none" : "inline";
                            if (typeof _item.action === 'function') {
                                const actionText = _item.action();
                                child.textContent = this.leftPad(actionText, ".", 19 - _item.name.length);
                            }
                        }
                        if (child.className == "Completed") {
                            child.style.display = _completed ? "inline" : "none";
                            if (typeof _item.completed === 'function') {
                                child.textContent = " " + _item.completed();
                            }
                        }
                    }
                    return;
                }
            }
        }

        update() {
            for (const item of this.items) {
                this.setCompleted(item, item.isCompleted());
            }
        }

    }
    A320_Neo_UpperECAM.Memo = Memo;

})(A320_Neo_UpperECAM || (A320_Neo_UpperECAM = {}));
customElements.define("a320-neo-upper-ecam", A320_Neo_UpperECAM.Display);
