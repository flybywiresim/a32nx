let A320_Neo_LowerECAM_FTCL;
(function (A320_Neo_LowerECAM_FTCL) {
    class Definitions {
    }
    A320_Neo_LowerECAM_FTCL.Definitions = Definitions;
    class Page extends Airliners.EICASTemplateElement {

        constructor() {
            super();
            this.isInitialised = false;
        }

        get templateID() {
            return "LowerECAMFTCLTemplate";
        }

        connectedCallback() {
            super.connectedCallback();
            TemplateElement.call(this, this.init.bind(this));
        }

        init() {
            // ELAC
            this.elac1Shape = this.querySelector("#elac1");
            this.elac2Shape = this.querySelector("#elac2");
            this.elac1Num = this.querySelector("#elacText_1");
            this.elac2Num = this.querySelector("#elacText_2");

            // SEC
            this.sec1Shape = this.querySelector("#sec1");
            this.sec2Shape = this.querySelector("#sec2");
            this.sec3Shape = this.querySelector("#sec3");
            this.sec1Num = this.querySelector("#secText_1");
            this.sec2Num = this.querySelector("#secText_2");
            this.sec3Num = this.querySelector("#secText_3");

            // PitchTrim
            this.pitchTrimLeadingDecimal = this.querySelector("#pitchTrimLeadingDecimal");
            this.pitchTrimTrailingDecimal = this.querySelector("#pitchTrimTrailingDecimal");
            this.pitchTrimDirection = this.querySelector("#pitchTrimDirection");

            // Ailerons
            this.leftAileronCursor = this.querySelector("#leftAileronCursor");
            this.rightAileronCursor = this.querySelector("#rightAileronCursor");

            // Elevators
            this.leftElevatorCursor = this.querySelector("#leftElevatorCursor");
            this.rightElevatorCursor = this.querySelector("#rightElevatorCursor");

            // Rudder
            this.rudderCursor = this.querySelector("#rudderCursor");
            this.rudderLeftMaxAngle = this.querySelector("#rudderLeftMaxAngle");
            this.rudderRightMaxAngle = this.querySelector("#rudderRightMaxAngle");

            // Spoiler arrows
            this.spoiler_arrow5_left = this.querySelector("#arrow5_left");
            this.spoiler_arrow4_left = this.querySelector("#arrow4_left");
            this.spoiler_arrow3_left = this.querySelector("#arrow3_left");
            this.spoiler_arrow2_left = this.querySelector("#arrow2_left");
            this.spoiler_arrow1_left = this.querySelector("#arrow1_left");
            this.spoiler_arrow5_right = this.querySelector("#arrow5_right");
            this.spoiler_arrow4_right = this.querySelector("#arrow4_right");
            this.spoiler_arrow3_right = this.querySelector("#arrow3_right");
            this.spoiler_arrow2_right = this.querySelector("#arrow2_right");
            this.spoiler_arrow1_right = this.querySelector("#arrow1_right");

            // Spoiler text
            this.spoilertext = this.querySelector("#speedbrake_text");

            this.hydraulicsAvailable = true;

            this.updateThrottler = new UpdateThrottler(50);

            this.isInitialised = true;
        }

        update(_deltaTime) {
            if (!this.isInitialised || !A320_Neo_EICAS.isOnBottomScreen()) {
                return;
            }

            if (this.updateThrottler.canUpdate(_deltaTime) === -1) {
                return;
            }

            // Update hydraulics available state
            const hydraulicsShouldBeAvailable = SimVar.GetSimVarValue("ENG COMBUSTION:1", "Bool") === 0 && SimVar.GetSimVarValue("ENG COMBUSTION:2", "Bool") === 0;

            if (hydraulicsShouldBeAvailable !== this.hydraulicsAvailable) {
                this.hydraulicsAvailable = hydraulicsShouldBeAvailable;

                this.setControlsHydraulicsAvailable(this.hydraulicsAvailable);
            }

            // Update pitch trim
            let rawPitchTrim = SimVar.GetSimVarValue("ELEVATOR TRIM INDICATOR", "Position 16k") / 1213.6296;

            // Cap pitch trim at 13.5 up, 4 down
            if (rawPitchTrim > 16384.0) {
                rawPitchTrim = 16384.0;
            } else if (rawPitchTrim < -4854) {
                rawPitchTrim = -4854;
            }

            const pitchTrimPctLeading = fastToFixed(Math.floor(Math.abs(rawPitchTrim)), 0);
            const pitchTrimPctTrailing = fastToFixed(Math.floor((Math.abs(rawPitchTrim) * 10) % 10), 0);
            const pitchTrimSign = Math.sign(rawPitchTrim);

            this.pitchTrimLeadingDecimal.textContent = pitchTrimPctLeading;
            this.pitchTrimTrailingDecimal.textContent = pitchTrimPctTrailing;

            if (pitchTrimSign === -1) {
                this.pitchTrimDirection.textContent = "DN";
            } else {
                this.pitchTrimDirection.textContent = "UP";
            }

            // Update left aileron
            const leftAileronDeflectPct = SimVar.GetSimVarValue("AILERON LEFT DEFLECTION PCT", "percent over 100");
            const leftAileronDeflectPctNormalized = leftAileronDeflectPct * 54;
            const laCursorPath = "M73," + (204 + leftAileronDeflectPctNormalized) + " l15,-7 l0,14Z";
            this.leftAileronCursor.setAttribute("d", laCursorPath);

            // Update right aileron
            const rightAileronDeflectPct = SimVar.GetSimVarValue("AILERON RIGHT DEFLECTION PCT", "percent over 100");
            const rightAileronDeflectPctNormalized = rightAileronDeflectPct * 54;
            const raCursorPath = "M527," + (204 - rightAileronDeflectPctNormalized) + " l-15,-7 l0,14Z";
            this.rightAileronCursor.setAttribute("d", raCursorPath);

            // Update left/right elevators
            const elevatorDeflectPct = SimVar.GetSimVarValue("ELEVATOR DEFLECTION PCT", "percent over 100");
            const elevatorDeflectPctNormalized = elevatorDeflectPct * (elevatorDeflectPct > 0 ? 70 : 52);
            const leCursorPath = "M169," + (398 - elevatorDeflectPctNormalized) + " l15,-7 l0,14Z";
            const reCursorPath = "M431," + (398 - elevatorDeflectPctNormalized) + " l-15,-7 l0,14Z";
            this.leftElevatorCursor.setAttribute("d", leCursorPath);
            this.rightElevatorCursor.setAttribute("d", reCursorPath);

            // Update rudder
            const rudderDeflectPct = SimVar.GetSimVarValue("RUDDER DEFLECTION PCT", "percent over 100");
            const rudderAngle = -rudderDeflectPct * 25;
            this.rudderCursor.setAttribute("transform", `rotate(${rudderAngle} 300 380)`);

            // Update rudder limits
            const IndicatedAirspeed = SimVar.GetSimVarValue("AIRSPEED INDICATED", "knots");
            let MaxAngleNorm = 1;
            if (IndicatedAirspeed > 380) {
                MaxAngleNorm = 3.4 / 25;
            } else if (IndicatedAirspeed > 160) {
                MaxAngleNorm = (69.2667 - 0.351818 * IndicatedAirspeed + 0.00047 * IndicatedAirspeed ** 2) / 25;
            }

            this.rudderLeftMaxAngle.setAttribute("transform", `rotate(${-26.4 * (1 - MaxAngleNorm)} 300 385)`);
            this.rudderRightMaxAngle.setAttribute("transform", `rotate(${26.4 * (1 - MaxAngleNorm)} 300 385)`);

            // Update ELAC's and SEC's
            const elac1_On = SimVar.GetSimVarValue("L:A32NX_FBW_ELAC_SWITCH:1", "boolean");
            const elac2_On = SimVar.GetSimVarValue("L:A32NX_FBW_ELAC_SWITCH:2", "boolean");
            const elac1_Failed = SimVar.GetSimVarValue("L:A32NX_FBW_ELAC_FAILED:1", "boolean");
            const elac2_Failed = SimVar.GetSimVarValue("L:A32NX_FBW_ELAC_FAILED:2", "boolean");
            const sec1_On = SimVar.GetSimVarValue("L:A32NX_FBW_SEC_SWITCH:1", "boolean");
            const sec2_On = SimVar.GetSimVarValue("L:A32NX_FBW_SEC_SWITCH:2", "boolean");
            const sec3_On = SimVar.GetSimVarValue("L:A32NX_FBW_SEC_SWITCH:3", "boolean");
            const sec1_Failed = SimVar.GetSimVarValue("L:A32NX_FBW_SEC_FAILED:1", "boolean");
            const sec2_Failed = SimVar.GetSimVarValue("L:A32NX_FBW_SEC_FAILED:2", "boolean");
            const sec3_Failed = SimVar.GetSimVarValue("L:A32NX_FBW_SEC_FAILED:3", "boolean");

            if (elac1_On && !elac1_Failed) {
                this.elac1Shape.setAttribute("class", "MainShape");
                this.elac1Num.setAttribute("class", "Value15");
            } else {
                this.elac1Shape.setAttribute("class", "MainShapeWarning");
                this.elac1Num.setAttribute("class", "Value15Warning");
            }

            if (elac2_On && !elac2_Failed) {
                this.elac2Shape.setAttribute("class", "MainShape");
                this.elac2Num.setAttribute("class", "Value15");
            } else {
                this.elac2Shape.setAttribute("class", "MainShapeWarning");
                this.elac2Num.setAttribute("class", "Value15Warning");
            }

            if (sec1_On && !sec1_Failed) {
                this.sec1Shape.setAttribute("class", "MainShape");
                this.sec1Num.setAttribute("class", "Value15");
            } else {
                this.sec1Shape.setAttribute("class", "MainShapeWarning");
                this.sec1Num.setAttribute("class", "Value15Warning");
            }

            if (sec2_On && !sec2_Failed) {
                this.sec2Shape.setAttribute("class", "MainShape");
                this.sec2Num.setAttribute("class", "Value15");
            } else {
                this.sec2Shape.setAttribute("class", "MainShapeWarning");
                this.sec2Num.setAttribute("class", "Value15Warning");
            }

            if (sec3_On && !sec3_Failed) {
                this.sec3Shape.setAttribute("class", "MainShape");
                this.sec3Num.setAttribute("class", "Value15");
            } else {
                this.sec3Shape.setAttribute("class", "MainShapeWarning");
                this.sec3Num.setAttribute("class", "Value15Warning");
            }

            if (this.hydraulicsAvailable) {
                // Show hydraulic numbers and hide everything else
                this.spoilertext.setAttribute("visibility", "visible");
                this.spoiler_arrow5_left.setAttribute("visibility", "hidden");
                this.spoiler_arrow4_left.setAttribute("visibility", "hidden");
                this.spoiler_arrow3_left.setAttribute("visibility", "hidden");
                this.spoiler_arrow2_left.setAttribute("visibility", "hidden");
                this.spoiler_arrow1_left.setAttribute("visibility", "hidden");
                this.spoiler_arrow5_right.setAttribute("visibility", "hidden");
                this.spoiler_arrow4_right.setAttribute("visibility", "hidden");
                this.spoiler_arrow3_right.setAttribute("visibility", "hidden");
                this.spoiler_arrow2_right.setAttribute("visibility", "hidden");
                this.spoiler_arrow1_right.setAttribute("visibility", "hidden");
            } else {
                // Update spoiler variables
                const spoilersArmed = SimVar.GetSimVarValue("SPOILERS ARMED", "boolean");
                const leftSpoilerDeflectPct = SimVar.GetSimVarValue("SPOILERS LEFT POSITION", "percent over 100");
                const rightSpoilerDeflectPct = SimVar.GetSimVarValue("SPOILERS RIGHT POSITION", "percent over 100");
                const planeOnGround = SimVar.GetSimVarValue("SIM ON GROUND", "boolean");
                const eng1_mode = SimVar.GetSimVarValue("GENERAL ENG THROTTLE MANAGED MODE:1", "number");
                const eng2_mode = SimVar.GetSimVarValue("GENERAL ENG THROTTLE MANAGED MODE:2", "number");
                const gspeed = SimVar.GetSimVarValue("SURFACE RELATIVE GROUND SPEED", "feet_per_second");

                // Disable hydraulic numbers
                this.spoilertext.setAttribute("visibility", "hidden");

                // Update left speedbrakes and aileron
                if (leftSpoilerDeflectPct > 0.07) {
                    if (leftAileronDeflectPct < -0.05) {
                        this.spoiler_arrow5_left.setAttribute("visibility", "visible");
                    } else {
                        this.spoiler_arrow5_left.setAttribute("visibility", "hidden");
                    }
                    this.spoiler_arrow4_left.setAttribute("visibility", "visible");
                    this.spoiler_arrow3_left.setAttribute("visibility", "visible");
                    this.spoiler_arrow2_left.setAttribute("visibility", "visible");
                } else {
                    this.spoiler_arrow5_left.setAttribute("visibility", "hidden");
                    this.spoiler_arrow4_left.setAttribute("visibility", "hidden");
                    this.spoiler_arrow3_left.setAttribute("visibility", "hidden");
                    this.spoiler_arrow2_left.setAttribute("visibility", "hidden");
                }

                // Update right speedbrakes and aileron
                if (rightSpoilerDeflectPct > 0.07) {
                    if (rightAileronDeflectPct > 0.05) {
                        this.spoiler_arrow5_right.setAttribute("visibility", "visible");
                    } else {
                        this.spoiler_arrow5_right.setAttribute("visibility", "hidden");
                    }
                    this.spoiler_arrow4_right.setAttribute("visibility", "visible");
                    this.spoiler_arrow3_right.setAttribute("visibility", "visible");
                    this.spoiler_arrow2_right.setAttribute("visibility", "visible");
                } else {
                    this.spoiler_arrow5_right.setAttribute("visibility", "hidden");
                    this.spoiler_arrow4_right.setAttribute("visibility", "hidden");
                    this.spoiler_arrow3_right.setAttribute("visibility", "hidden");
                    this.spoiler_arrow2_right.setAttribute("visibility", "hidden");
                }

                // Ground spoilers deployed
                if (spoilersArmed && planeOnGround && gspeed > 45 && eng1_mode <= 2 && eng2_mode <= 2) {
                    if (leftSpoilerDeflectPct > 0.0625) {
                        this.spoiler_arrow1_left.setAttribute("visibility", "visible");
                        this.spoiler_arrow5_left.setAttribute("visibility", "visible");
                    } else {
                        this.spoiler_arrow1_left.setAttribute("visibility", "hidden");
                    }
                    if (rightSpoilerDeflectPct > 0.0625) {
                        this.spoiler_arrow1_right.setAttribute("visibility", "visible");
                        this.spoiler_arrow5_right.setAttribute("visibility", "visible");
                    } else {
                        this.spoiler_arrow1_right.setAttribute("visibility", "hidden");
                    }
                } else {
                    this.spoiler_arrow1_left.setAttribute("visibility", "hidden");
                    this.spoiler_arrow1_right.setAttribute("visibility", "hidden");
                }
            }
        }

        /**
         * Sets the availability of control surfaces.
         *
         * This temporarily affects all of them as we do not fully model the hydraulic system yet.
         *
         * @param state {boolean}
         * @return {void}
         */
        setControlsHydraulicsAvailable(state) {
            const shapeElements = [
                "speedbrakes",
                "leftAileronPointer",
                "rightAileronPointer",
                "leftElevatorPointer",
                "rightElevatorPointer",
                "rudderCursor"
            ];

            for (const elementName of shapeElements) {
                const element = this.querySelector("#" + elementName);

                for (const node of element.children) {
                    node.setAttribute("class", state ? "WarningShape" : "GreenShape");
                }
            }

            const textElements = [
                "speedbrakeHyd1", "speedbrakeHyd2", "speedbrakeHyd3",
                "leftAileronHyd1", "leftAileronHyd2",
                "rightAileronHyd1", "rightAileronHyd2",
                "leftElevatorHyd1", "leftElevatorHyd2",
                "rightElevatorHyd1", "rightElevatorHyd2",
                "pitchTrimLeadingDecimal", "pitchTrimDecimalPoint", "pitchTrimTrailingDecimal", "pitchTrimDirection",
                "pitchTrimHyd1", "pitchTrimHyd2",
                "stabHyd1", "stabHyd2", "stabHyd3"
            ];

            for (const elementName of textElements) {
                const element = this.querySelector("#" + elementName);

                element.setAttribute("class", state ? "Warning" : "Value15");
            }
        }

    }
    A320_Neo_LowerECAM_FTCL.Page = Page;
})(A320_Neo_LowerECAM_FTCL || (A320_Neo_LowerECAM_FTCL = {}));
customElements.define("a320-neo-lower-ecam-ftcl", A320_Neo_LowerECAM_FTCL.Page);
