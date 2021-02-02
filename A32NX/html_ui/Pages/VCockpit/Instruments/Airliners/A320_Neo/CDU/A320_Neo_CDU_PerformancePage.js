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

class CDUPerformancePage {
    static ShowPage(mcdu) {
        mcdu.activeSystem = 'FMGC';
        if (mcdu.currentFlightPhase <= FmgcFlightPhases.TAKEOFF) {
            CDUPerformancePage.ShowTAKEOFFPage(mcdu);
        } else if (mcdu.currentFlightPhase === FmgcFlightPhases.CLIMB) {
            CDUPerformancePage.ShowCLBPage(mcdu);
        } else if (mcdu.currentFlightPhase === FmgcFlightPhases.CRUISE) {
            CDUPerformancePage.ShowCRZPage(mcdu);
        } else if (mcdu.currentFlightPhase === FmgcFlightPhases.DESCENT) {
            CDUPerformancePage.ShowDESPage(mcdu);
        } else if (mcdu.currentFlightPhase === FmgcFlightPhases.APPROACH) {
            CDUPerformancePage.ShowAPPRPage(mcdu);
        } else if (mcdu.currentFlightPhase === FmgcFlightPhases.GOAROUND) {
            CDUPerformancePage.ShowGOAROUNDPage(mcdu);
        }
    }
    static ShowTAKEOFFPage(mcdu) {
        mcdu.clearDisplay();
        mcdu.page.Current = mcdu.page.PerformancePageTakeoff;
        CDUPerformancePage._timer = 0;
        CDUPerformancePage._lastPhase = mcdu.currentFlightPhase;
        mcdu.pageUpdate = () => {
            CDUPerformancePage._timer++;
            if (CDUPerformancePage._timer >= 15) {
                if (mcdu.currentFlightPhase === CDUPerformancePage._lastPhase) {
                    CDUPerformancePage.ShowTAKEOFFPage(mcdu);
                } else {
                    CDUPerformancePage.ShowPage(mcdu);
                }
            }
        };

        let titleColor = "white";
        if (mcdu.currentFlightPhase === FmgcFlightPhases.TAKEOFF) {
            titleColor = "green";
        }

        // check if we even have an airport
        const hasOrigin = !!mcdu.flightPlanManager.getOrigin();

        // runway
        let runway = "";
        let hasRunway = false;
        if (hasOrigin) {
            const runwayObj = mcdu.flightPlanManager.getDepartureRunway();
            if (runwayObj) {
                runway = Avionics.Utils.formatRunway(runwayObj.designation);
                hasRunway = true;
            }
        }

        // v speeds
        let v1 = "---";
        let vR = "---";
        let v2 = "---";
        let v1Check = "{small}\xa0\xa0\xa0{end}";
        let vRCheck = "{small}\xa0\xa0\xa0{end}";
        let v2Check = "{small}\xa0\xa0\xa0{end}";
        if (mcdu.currentFlightPhase < FmgcFlightPhases.TAKEOFF) {
            v1 = "{amber}___{end}";
            if (mcdu.v1Speed) {
                if (mcdu._v1Checked) {
                    v1 = `{cyan}${("" + mcdu.v1Speed).padEnd(3)}{end}`;
                } else {
                    v1Check = `{small}{cyan}${("" + mcdu.v1Speed).padEnd(3)}{end}{end}`;
                }
            }
            mcdu.onLeftInput[0] = (value) => {
                if (value === "") {
                    if (mcdu._v1Checked) {
                        // not real: v-speed helper
                        mcdu.sendDataToScratchpad(mcdu._getV1Speed().toString());
                    } else {
                        mcdu._v1Checked = true;
                        mcdu.tryRemoveMessage(NXSystemMessages.checkToData.text);
                        mcdu.vSpeedDisagreeCheck();
                    }
                    CDUPerformancePage.ShowTAKEOFFPage(mcdu);
                } else {
                    if (mcdu.trySetV1Speed(value)) {
                        CDUPerformancePage.ShowTAKEOFFPage(mcdu);
                    }
                }
            };
            vR = "{amber}___{end}";
            if (mcdu.vRSpeed) {
                if (mcdu._vRChecked) {
                    vR = `{cyan}${("" + mcdu.vRSpeed).padEnd(3)}{end}`;
                } else {
                    vRCheck = `{small}{cyan}${("" + mcdu.vRSpeed).padEnd(3)}{end}{end}`;
                }
            }
            mcdu.onLeftInput[1] = (value) => {
                if (value === "") {
                    if (mcdu._vRChecked) {
                        mcdu.sendDataToScratchpad(mcdu._getVRSpeed().toString());
                    } else {
                        mcdu._vRChecked = true;
                        mcdu.tryRemoveMessage(NXSystemMessages.checkToData.text);
                        mcdu.vSpeedDisagreeCheck();
                    }
                    CDUPerformancePage.ShowTAKEOFFPage(mcdu);
                } else {
                    if (mcdu.trySetVRSpeed(value)) {
                        CDUPerformancePage.ShowTAKEOFFPage(mcdu);
                    }
                }
            };
            v2 = "{amber}___{end}";
            if (mcdu.v2Speed) {
                if (mcdu._v2Checked) {
                    v2 = `{cyan}${("" + mcdu.v2Speed).padEnd(3)}{end}`;
                } else {
                    v2Check = `{small}{cyan}${("" + mcdu.v2Speed).padEnd(3)}{end}{end}`;
                }
            }
            mcdu.onLeftInput[2] = (value) => {
                if (value === "") {
                    if (mcdu._v2Checked) {
                        mcdu.sendDataToScratchpad(mcdu._getV2Speed().toString());
                    } else {
                        mcdu._v2Checked = true;
                        mcdu.tryRemoveMessage(NXSystemMessages.checkToData.text);
                        mcdu.vSpeedDisagreeCheck();
                    }
                    CDUPerformancePage.ShowTAKEOFFPage(mcdu);
                } else {
                    if (mcdu.trySetV2Speed(value)) {
                        CDUPerformancePage.ShowTAKEOFFPage(mcdu);
                    }
                }
            };
        } else {
            v1 = "\xa0\xa0\xa0";
            vR = "\xa0\xa0\xa0";
            v2 = "\xa0\xa0\xa0";
            if (mcdu.v1Speed) {
                v1 = `{green}${("" + mcdu.v1Speed).padEnd(3)}{end}`;
            }
            if (mcdu.vRSpeed) {
                vR = `{green}${("" + mcdu.vRSpeed).padEnd(3)}{end}`;
            }
            if (mcdu.v2Speed) {
                v2 = `{green}${("" + mcdu.v2Speed).padEnd(3)}{end}`;
            }
            mcdu.onLeftInput[0] = (value) => {
                if (value !== "") {
                    mcdu.addNewMessage(NXSystemMessages.notAllowed);
                }
            };
            mcdu.onLeftInput[1] = (value) => {
                if (value !== "") {
                    mcdu.addNewMessage(NXSystemMessages.notAllowed);
                }
            };
            mcdu.onLeftInput[2] = (value) => {
                if (value !== "") {
                    mcdu.addNewMessage(NXSystemMessages.notAllowed);
                }
            };
        }

        // transition altitude - remains editable during take off
        let transAltCell = "";
        if (hasOrigin) {
            const transAltitude = mcdu.getTransitionAltitude();
            if (isFinite(transAltitude)) {
                transAltCell = `{cyan}${transAltitude}{end}`;
                if (!mcdu.transitionAltitudeIsPilotEntered) {
                    transAltCell += "[s-text]";
                }
            } else {
                transAltCell = "{cyan}[]{end}";
            }
            mcdu.onLeftInput[3] = (value) => {
                if (mcdu.trySetTakeOffTransAltitude(value)) {
                    CDUPerformancePage.ShowTAKEOFFPage(mcdu);
                }
            };
        }

        // thrust reduction / acceleration altitude
        let thrRedAcc = "-----/-----";
        if (hasOrigin) {
            if (mcdu.currentFlightPhase < FmgcFlightPhases.TAKEOFF) {
                let thrRed = "[\xa0]";
                if (isFinite(mcdu.thrustReductionAltitude)) {
                    thrRed = ("" + mcdu.thrustReductionAltitude.toFixed(0)).padStart(5, "\xa0");
                }
                if (!mcdu.thrustReductionAltitudeIsPilotEntered) {
                    thrRed = `{small}${thrRed}{end}`;
                }
                let acc = "[\xa0]";
                if (isFinite(mcdu.accelerationAltitude)) {
                    acc = mcdu.accelerationAltitude.toFixed(0);
                }
                acc = "/" + acc;
                if (!mcdu.accelerationAltitudeIsPilotEntered) {
                    acc = `{small}${acc}{end}`;
                }
                thrRedAcc = `${thrRed}${acc}[color]cyan`;
                mcdu.onLeftInput[4] = (value) => {
                    if (mcdu.trySetThrustReductionAccelerationAltitude(value)) {
                        CDUPerformancePage.ShowTAKEOFFPage(mcdu);
                    }
                };
            } else {
                let thrRed = "-----";
                if (isFinite(mcdu.thrustReductionAltitude)) {
                    thrRed = ("" + mcdu.thrustReductionAltitude.toFixed(0)).padStart(5, "\xa0");
                }
                if (!mcdu.thrustReductionAltitudeIsPilotEntered) {
                    thrRed = `{small}${thrRed}{end}`;
                }
                let acc = "-----";
                if (isFinite(mcdu.accelerationAltitude)) {
                    acc = mcdu.accelerationAltitude.toFixed(0);
                }
                acc = "/" + acc;
                if (!mcdu.accelerationAltitudeIsPilotEntered) {
                    acc = `{small}${acc}{end}`;
                }
                thrRedAcc = `${thrRed}${acc}[color]green`;
            }
        } else if (mcdu.currentFlightPhase >= FmgcFlightPhases.TAKEOFF) {
            thrRedAcc = "";
        }

        // center column
        let flpRetrCell = "---";
        let sltRetrCell = "---";
        let cleanCell = "---";
        const flapSpeed = mcdu.computedVfs;
        if (flapSpeed !== 0) {
            flpRetrCell = `{green}${flapSpeed.toFixed(0)}{end}`;
        }
        const slatSpeed = mcdu.computedVss;
        if (slatSpeed !== 0) {
            sltRetrCell = `{green}${slatSpeed.toFixed(0)}{end}`;
        }
        const cleanSpeed = mcdu.computedVgd;
        if (cleanSpeed !== 0) {
            cleanCell = `{green}${cleanSpeed.toFixed(0)}{end}`;
        }

        // takeoff shift
        let toShiftCell = "{inop}----{end}\xa0";
        if (hasOrigin && hasRunway) {
            toShiftCell = "{inop}{small}[M]{end}[\xa0\xa0]*{end}";
            // TODO store and show TO SHIFT
        }

        // flaps / trim horizontal stabilizer
        let flapsThs = "[]/[\xa0\xa0\xa0][color]cyan";
        // The following line uses a special Javascript concept that is signed
        // zeroes. In Javascript -0 is strictly equal to 0, so for most cases we
        // don't care about that difference. But here, we use that fact to show
        // the pilot the precise value they entered: DN0.0 or UP0.0. The only
        // way to figure that difference out is using Object.is, as
        // Object.is(+0, -0) returns false. Alternatively we could use a helper
        // variable (yuck) or encode it using a very small, but negative value
        // such as -0.001.
        const formattedThs = !isNaN(mcdu.ths)
            ? (mcdu.ths >= 0 && !Object.is(mcdu.ths, -0) ? `UP${Math.abs(mcdu.ths).toFixed(1)}` : `DN${Math.abs(mcdu.ths).toFixed(1)}`)
            : '';
        if (mcdu.currentFlightPhase < FmgcFlightPhases.TAKEOFF) {
            const flaps = !isNaN(mcdu.flaps) ? mcdu.flaps : "[]";
            const ths = formattedThs ? formattedThs : "[\xa0\xa0\xa0]";
            flapsThs = `${flaps}/${ths}[color]cyan`;
            mcdu.onRightInput[2] = (value) => {
                if (mcdu.trySetFlapsTHS(value)) {
                    CDUPerformancePage.ShowTAKEOFFPage(mcdu);
                }
            };
        } else {
            const flaps = !isNaN(mcdu.flaps) ? mcdu.flaps : "";
            const ths = formattedThs ? formattedThs : "\xa0\xa0\xa0\xa0\xa0";
            flapsThs = `${flaps}/${ths}[color]green`;
        }

        // flex takeoff temperature
        let flexTakeOffTempCell = "[\xa0\xa0]°[color]cyan";
        if (mcdu.currentFlightPhase < FmgcFlightPhases.TAKEOFF) {
            if (isFinite(mcdu.perfTOTemp)) {
                if (mcdu._toFlexChecked) {
                    flexTakeOffTempCell = `${mcdu.perfTOTemp.toFixed(0)}°[color]cyan`;
                } else {
                    flexTakeOffTempCell = `{small}${mcdu.perfTOTemp.toFixed(0)}{end}${flexTakeOffTempCell}[color]cyan`;
                }
            }
            mcdu.onRightInput[3] = (value) => {
                if (mcdu._toFlexChecked) {
                    if (mcdu.setPerfTOFlexTemp(value)) {
                        CDUPerformancePage.ShowTAKEOFFPage(mcdu);
                    }
                } else {
                    if (value === "" || mcdu.setPerfTOFlexTemp(value)) {
                        mcdu._toFlexChecked = true;
                        CDUPerformancePage.ShowTAKEOFFPage(mcdu);
                    }
                }
            };
        } else {
            if (isFinite(mcdu.perfTOTemp)) {
                flexTakeOffTempCell = `${mcdu.perfTOTemp.toFixed(0)}°[color]green`;
            } else {
                flexTakeOffTempCell = "";
            }
        }

        // eng out acceleration altitude
        let engOutAcc = "-----";
        if (hasOrigin) {
            if (mcdu.currentFlightPhase < FmgcFlightPhases.TAKEOFF) {
                if (isFinite(mcdu.engineOutAccelerationAltitude)) {
                    engOutAcc = mcdu.engineOutAccelerationAltitude.toFixed(0);
                    if (mcdu.engineOutAccelerationAltitudeIsPilotEntered) {
                        engOutAcc = `${engOutAcc}[color]cyan`;
                    } else {
                        engOutAcc = `${engOutAcc}[s-text][color]cyan`;
                    }
                } else {
                    engOutAcc = "[][color]cyan";
                }
                mcdu.onRightInput[4] = (value) => {
                    if (mcdu.trySetEngineOutAcceleration(value)) {
                        CDUPerformancePage.ShowTAKEOFFPage(mcdu);
                    }
                };
            } else if (isFinite(mcdu.engineOutAccelerationAltitude)) {
                if (mcdu.engineOutAccelerationAltitudeIsPilotEntered) {
                    engOutAcc = `${mcdu.engineOutAccelerationAltitude}[color]green`;
                } else {
                    engOutAcc = `${mcdu.engineOutAccelerationAltitude}[s-text][color]green`;
                }
            }
        } else if (mcdu.currentFlightPhase >= FmgcFlightPhases.TAKEOFF) {
            engOutAcc = "";
        }

        let next = "NEXT\xa0";
        let nextPhase = "PHASE>";
        if (!(mcdu._v1Checked && mcdu._vRChecked && mcdu._v2Checked && mcdu._toFlexChecked) && mcdu.currentFlightPhase < FmgcFlightPhases.TAKEOFF) {
            next = "CONFIRM\xa0";
            nextPhase = "TO DATA*";
            mcdu.onRightInput[5] = (value) => {
                mcdu._v1Checked = true;
                mcdu._vRChecked = true;
                mcdu._v2Checked = true;
                mcdu._toFlexChecked = true;
                mcdu.vSpeedDisagreeCheck();
                CDUPerformancePage.ShowTAKEOFFPage(mcdu);
            };
        } else {
            mcdu.rightInputDelay[5] = () => {
                return mcdu.getDelaySwitchPage();
            };
            mcdu.onRightInput[5] = (value) => {
                CDUPerformancePage.ShowCLBPage(mcdu);
            };
        }

        mcdu.setTemplate([
            ["TAKE OFF RWY\xa0{green}" + runway.padStart(3, "\xa0") + "{end}[color]" + titleColor],
            ["\xa0V1\xa0\xa0FLP RETR", ""],
            [v1 + v1Check + "\xa0F=" + flpRetrCell, ""],
            ["\xa0VR\xa0\xa0SLT RETR", "TO SHIFT\xa0"],
            [vR + vRCheck + "\xa0S=" + sltRetrCell, toShiftCell],
            ["\xa0V2\xa0\xa0\xa0\xa0\xa0CLEAN", "FLAPS/THS"],
            [v2 + v2Check + "\xa0O=" + cleanCell, flapsThs],
            ["TRANS ALT", "FLEX TO TEMP"],
            [transAltCell, flexTakeOffTempCell],
            ["THR RED/ACC", "ENG OUT ACC"],
            [thrRedAcc, engOutAcc],
            ["\xa0UPLINK[color]inop", next],
            ["<TO DATA[color]inop", nextPhase]
        ]);
    }
    static ShowCLBPage(mcdu, confirmAppr = false) {
        mcdu.clearDisplay();
        mcdu.page.Current = mcdu.page.PerformancePageClb;
        CDUPerformancePage._timer = 0;
        CDUPerformancePage._lastPhase = mcdu.currentFlightPhase;
        mcdu.pageUpdate = () => {
            CDUPerformancePage._timer++;
            if (CDUPerformancePage._timer >= 100) {
                if (mcdu.currentFlightPhase === CDUPerformancePage._lastPhase) {
                    CDUPerformancePage.ShowCLBPage(mcdu);
                } else {
                    CDUPerformancePage.ShowPage(mcdu);
                }
            }
        };
        let titleColor = "white";
        if (mcdu.currentFlightPhase === FmgcFlightPhases.CLIMB) {
            titleColor = "green";
        }
        const isFlying = mcdu.getIsFlying();
        let actModeCell = "SELECTED";
        if (mcdu.isAirspeedManaged()) {
            actModeCell = "MANAGED";
        }
        let costIndexCell = "[][color]cyan";
        if (isFinite(mcdu.costIndex)) {
            costIndexCell = mcdu.costIndex.toFixed(0) + "[color]cyan";
        }
        let managedSpeedCell = "";
        let managedSpeed;
        if (SimVar.GetSimVarValue("L:A32NX_GOAROUND_PASSED", "bool") === 1) {
            managedSpeed = mcdu.computedVgd;
        } else {
            managedSpeed = mcdu.getClbManagedSpeed();
        }
        if (isFinite(managedSpeed)) {
            managedSpeedCell = managedSpeed.toFixed(0);
        }
        let selectedSpeedCell = "";
        if (isFinite(mcdu.preSelectedClbSpeed)) {
            selectedSpeedCell = mcdu.preSelectedClbSpeed.toFixed(0);
        } else {
            selectedSpeedCell = "[]";
        }
        mcdu.onLeftInput[1] = (value) => {
            if (mcdu.tryUpdateCostIndex(value)) {
                CDUPerformancePage.ShowCLBPage(mcdu);
            }
        };
        mcdu.onLeftInput[3] = (value) => {
            if (mcdu.trySetPreSelectedClimbSpeed(value)) {
                CDUPerformancePage.ShowCLBPage(mcdu);
            }
        };
        let timeLabel = "TIME";
        if (isFlying) {
            timeLabel = "UTC";
        }
        const bottomRowLabels = ["\xa0PREV", "NEXT\xa0"];
        const bottomRowCells = ["<PHASE", "PHASE>"];
        mcdu.leftInputDelay[5] = () => {
            return mcdu.getDelaySwitchPage();
        };
        if (mcdu.currentFlightPhase === FmgcFlightPhases.CLIMB) {
            if (confirmAppr) {
                bottomRowLabels[0] = "\xa0CONFIRM[color]amber";
                bottomRowCells[0] = "*APPR PHASE[color]amber";
                mcdu.onLeftInput[5] = async () => {
                    if (mcdu.tryGoInApproachPhase()) {
                        CDUPerformancePage.ShowAPPRPage(mcdu);
                    }
                };
            } else {
                bottomRowLabels[0] = "\xa0ACTIVATE[color]cyan";
                bottomRowCells[0] = "{APPR PHASE[color]cyan";
                mcdu.onLeftInput[5] = () => {
                    CDUPerformancePage.ShowCLBPage(mcdu, true);
                };
            }
        } else {
            mcdu.onLeftInput[5] = () => {
                CDUPerformancePage.ShowTAKEOFFPage(mcdu);
            };
        }
        mcdu.rightInputDelay[5] = () => {
            return mcdu.getDelaySwitchPage();
        };
        mcdu.onRightInput[5] = () => {
            CDUPerformancePage.ShowCRZPage(mcdu);
        };
        mcdu.setTemplate([
            ["CLB[color]" + titleColor],
            ["ACT MODE", "EFOB", timeLabel],
            [actModeCell + "[color]green", "6.0[color]green", "----[color]green"],
            ["CI"],
            [costIndexCell + "[color]cyan"],
            ["MANAGED"],
            ["*" + managedSpeedCell + "[color]green"],
            ["SELECTED"],
            [selectedSpeedCell + "[color]green"],
            [""],
            [""],
            bottomRowLabels,
            bottomRowCells
        ]);
    }
    static ShowCRZPage(mcdu, confirmAppr = false) {
        mcdu.clearDisplay();
        mcdu.page.Current = mcdu.page.PerformancePageCrz;
        CDUPerformancePage._timer = 0;
        CDUPerformancePage._lastPhase = mcdu.currentFlightPhase;
        mcdu.pageUpdate = () => {
            CDUPerformancePage._timer++;
            if (CDUPerformancePage._timer >= 100) {
                if (mcdu.currentFlightPhase === CDUPerformancePage._lastPhase) {
                    CDUPerformancePage.ShowCRZPage(mcdu);
                } else {
                    CDUPerformancePage.ShowPage(mcdu);
                }
            }
        };
        let titleColor = "white";
        if (mcdu.currentFlightPhase === FmgcFlightPhases.CRUISE) {
            titleColor = "green";
        }
        const isFlying = false;
        let actModeCell = "SELECTED";
        if (mcdu.isAirspeedManaged()) {
            actModeCell = "MANAGED";
        }
        let costIndexCell = "[][color]cyan";
        if (isFinite(mcdu.costIndex)) {
            costIndexCell = mcdu.costIndex.toFixed(0) + "[color]cyan";
        }
        let managedSpeedCell = "";
        const managedSpeed = mcdu.getCrzManagedSpeed();
        if (isFinite(managedSpeed)) {
            managedSpeedCell = managedSpeed.toFixed(0);
        }
        let selectedSpeedCell = "";
        if (isFinite(mcdu.preSelectedCrzSpeed)) {
            selectedSpeedCell = mcdu.preSelectedCrzSpeed.toFixed(0);
        } else {
            selectedSpeedCell = "[]";
        }
        mcdu.onLeftInput[1] = (value) => {
            if (mcdu.tryUpdateCostIndex(value)) {
                CDUPerformancePage.ShowCRZPage(mcdu);
            }
        };
        mcdu.onLeftInput[3] = (value) => {
            if (mcdu.trySetPreSelectedCruiseSpeed(value)) {
                CDUPerformancePage.ShowCRZPage(mcdu);
            }
        };
        let timeLabel = "TIME";
        if (isFlying) {
            timeLabel = "UTC";
        }
        const bottomRowLabels = ["\xa0PREV", "NEXT\xa0"];
        const bottomRowCells = ["<PHASE", "PHASE>"];
        mcdu.leftInputDelay[5] = () => {
            return mcdu.getDelaySwitchPage();
        };
        if (mcdu.currentFlightPhase === FmgcFlightPhases.CRUISE) {
            if (confirmAppr) {
                bottomRowLabels[0] = "\xa0CONFIRM[color]amber";
                bottomRowCells[0] = "*APPR PHASE[color]amber";
                mcdu.onLeftInput[5] = async () => {
                    if (mcdu.tryGoInApproachPhase()) {
                        CDUPerformancePage.ShowAPPRPage(mcdu);
                    }
                };
            } else {
                bottomRowLabels[0] = "\xa0ACTIVATE[color]cyan";
                bottomRowCells[0] = "{APPR PHASE[color]cyan";
                mcdu.onLeftInput[5] = () => {
                    CDUPerformancePage.ShowCRZPage(mcdu, true);
                };
            }
        } else {
            mcdu.onLeftInput[5] = () => {
                CDUPerformancePage.ShowCLBPage(mcdu);
            };
        }
        mcdu.rightInputDelay[5] = () => {
            return mcdu.getDelaySwitchPage();
        };
        mcdu.onRightInput[5] = () => {
            CDUPerformancePage.ShowDESPage(mcdu);
        };
        mcdu.setTemplate([
            ["CRZ[color]" + titleColor],
            ["ACT MODE", "EFOB", timeLabel],
            [actModeCell + "[color]green", "6.0[color]green", "----[color]green"],
            ["CI"],
            [costIndexCell + "[color]cyan"],
            ["MANAGED"],
            ["*" + managedSpeedCell + "[color]green"],
            ["SELECTED"],
            [selectedSpeedCell + "[color]cyan"],
            ["", "DES CABIN RATE>"],
            ["", "-350FT/MIN[color]green"],
            bottomRowLabels,
            bottomRowCells
        ]);
    }
    static ShowDESPage(mcdu, confirmAppr = false) {
        mcdu.clearDisplay();
        mcdu.page.Current = mcdu.page.PerformancePageDes;
        CDUPerformancePage._timer = 0;
        CDUPerformancePage._lastPhase = mcdu.currentFlightPhase;
        mcdu.pageUpdate = () => {
            CDUPerformancePage._timer++;
            if (CDUPerformancePage._timer >= 100) {
                if (mcdu.currentFlightPhase === CDUPerformancePage._lastPhase) {
                    CDUPerformancePage.ShowDESPage(mcdu);
                } else {
                    CDUPerformancePage.ShowPage(mcdu);
                }
            }
        };
        let titleColor = "white";
        if (mcdu.currentFlightPhase === FmgcFlightPhases.DESCENT) {
            titleColor = "green";
        }
        const isFlying = false;
        let actModeCell = "SELECTED";
        if (mcdu.isAirspeedManaged()) {
            actModeCell = "MANAGED";
        }
        let costIndexCell = "[][color]cyan";
        if (isFinite(mcdu.costIndex)) {
            costIndexCell = mcdu.costIndex.toFixed(0) + "[color]cyan";
        }
        let managedSpeedCell = "";
        const managedSpeed = mcdu.getDesManagedSpeed();
        if (isFinite(managedSpeed)) {
            managedSpeedCell = managedSpeed.toFixed(0);
        }
        let selectedSpeedCell = "";
        if (isFinite(mcdu.preSelectedDesSpeed)) {
            selectedSpeedCell = mcdu.preSelectedDesSpeed.toFixed(0);
        } else {
            selectedSpeedCell = "[]";
        }
        mcdu.onLeftInput[3] = (value) => {
            if (mcdu.trySetPreSelectedDescentSpeed(value)) {
                CDUPerformancePage.ShowDESPage(mcdu);
            }
        };
        let timeLabel = "TIME";
        if (isFlying) {
            timeLabel = "UTC";
        }
        const bottomRowLabels = ["\xa0PREV", "NEXT\xa0"];
        const bottomRowCells = ["<PHASE", "PHASE>"];
        mcdu.leftInputDelay[5] = () => {
            return mcdu.getDelaySwitchPage();
        };
        if (mcdu.currentFlightPhase === FmgcFlightPhases.DESCENT) {
            if (confirmAppr) {
                bottomRowLabels[0] = "\xa0CONFIRM[color]amber";
                bottomRowCells[0] = "*APPR PHASE[color]amber";
                mcdu.onLeftInput[5] = async () => {
                    if (mcdu.tryGoInApproachPhase()) {
                        CDUPerformancePage.ShowAPPRPage(mcdu);
                    }
                };
            } else {
                bottomRowLabels[0] = "\xa0ACTIVATE[color]cyan";
                bottomRowCells[0] = "{APPR PHASE[color]cyan";
                mcdu.onLeftInput[5] = () => {
                    CDUPerformancePage.ShowDESPage(mcdu, true);
                };
            }
        } else {
            mcdu.onLeftInput[5] = () => {
                CDUPerformancePage.ShowCRZPage(mcdu);
            };
        }
        mcdu.onLeftInput[1] = (value) => {
            if (mcdu.tryUpdateCostIndex(value)) {
                CDUPerformancePage.ShowDESPage(mcdu);
            }
        };
        mcdu.rightInputDelay[5] = () => {
            return mcdu.getDelaySwitchPage();
        };
        mcdu.onRightInput[5] = () => {
            CDUPerformancePage.ShowAPPRPage(mcdu);
        };
        mcdu.setTemplate([
            ["DES[color]" + titleColor],
            ["ACT MODE", "EFOB", timeLabel],
            [actModeCell + "[color]green", "6.0[color]green", "----[color]green"],
            ["CI"],
            [costIndexCell + "[color]cyan"],
            ["MANAGED"],
            ["*" + managedSpeedCell + "[color]green"],
            ["SELECTED"],
            [selectedSpeedCell + "[color]cyan"],
            [""],
            [""],
            bottomRowLabels,
            bottomRowCells
        ]);
    }
    static ShowAPPRPage(mcdu) {
        mcdu.clearDisplay();
        mcdu.page.Current = mcdu.page.PerformancePageAppr;
        CDUPerformancePage._timer = 0;
        CDUPerformancePage._lastPhase = mcdu.currentFlightPhase;
        mcdu.pageUpdate = () => {
            CDUPerformancePage._timer++;
            if (CDUPerformancePage._timer >= 100) {
                if (mcdu.currentFlightPhase === CDUPerformancePage._lastPhase) {
                    CDUPerformancePage.ShowAPPRPage(mcdu);
                }
            }
        };
        let titleColor = "white";
        if (mcdu.currentFlightPhase === FmgcFlightPhases.APPROACH) {
            titleColor = "green";
        }
        let qnhCell = "[ ]";
        const QNH_REGEX = /[0-9]{2}.[0-9]{2}/;
        if (isFinite(mcdu.perfApprQNH) || QNH_REGEX.test(mcdu.perfApprQNH)) {
            qnhCell = mcdu.perfApprQNH;
        }
        mcdu.onLeftInput[0] = (value) => {
            if (mcdu.setPerfApprQNH(value)) {
                CDUPerformancePage.ShowAPPRPage(mcdu);
            }
        };
        let tempCell = "[ ]";
        if (isFinite(mcdu.perfApprTemp)) {
            tempCell = mcdu.perfApprTemp.toFixed(0);
        }
        mcdu.onLeftInput[1] = (value) => {
            if (mcdu.setPerfApprTemp(value)) {
                CDUPerformancePage.ShowAPPRPage(mcdu);
            }
        };
        let magWindHeadingCell = "[ ]";
        if (isFinite(mcdu.perfApprWindHeading)) {
            magWindHeadingCell = mcdu.perfApprWindHeading.toFixed(0);
        }
        let magWindSpeedCell = "[ ]";
        if (isFinite(mcdu.perfApprWindSpeed)) {
            magWindSpeedCell = mcdu.perfApprWindSpeed.toFixed(0);
        }
        mcdu.onLeftInput[2] = (value) => {
            if (mcdu.setPerfApprWind(value)) {
                mcdu.updateTowerHeadwind();
                mcdu.updatePerfSpeeds();
                CDUPerformancePage.ShowAPPRPage(mcdu);
            }
        };
        let transAltCell = "---";
        if (isFinite(mcdu.perfApprTransAlt)) {
            transAltCell = mcdu.perfApprTransAlt.toFixed(0);
        }
        mcdu.onLeftInput[3] = (value) => {
            if (mcdu.setPerfApprTransAlt(value)) {
                CDUPerformancePage.ShowAPPRPage(mcdu);
            }
        };

        let vappCell = "---";
        let vlsCell = "---";
        let flpRetrCell = "---";
        let sltRetrCell = "---";
        let cleanCell = "---";
        if (mcdu.approachSpeeds && mcdu.approachSpeeds.valid) {
            vappCell = mcdu.approachSpeeds.vapp.toFixed(0) + "[s-text]";
            vlsCell = mcdu.approachSpeeds.vls.toFixed(0);
            flpRetrCell = mcdu.approachSpeeds.f.toFixed(0) + "[color]green";
            sltRetrCell = mcdu.approachSpeeds.s.toFixed(0) + "[color]green";
            cleanCell = mcdu.approachSpeeds.gd.toFixed(0) + "[color]green";
        }
        if (isFinite(mcdu.vApp)) { // pilot override
            vappCell = mcdu.vApp.toFixed(0);
        }
        mcdu.onLeftInput[4] = (value) => {
            if (mcdu.setPerfApprVApp(value)) {
                CDUPerformancePage.ShowAPPRPage(mcdu);
            }
        };
        mcdu.onRightInput[3] = () => {
            mcdu.setPerfApprFlaps3(true);
            mcdu.updatePerfSpeeds();
            CDUPerformancePage.ShowAPPRPage(mcdu);
        };
        mcdu.onRightInput[4] = () => {
            mcdu.setPerfApprFlaps3(false);
            mcdu.updatePerfSpeeds();
            CDUPerformancePage.ShowAPPRPage(mcdu);
        };
        let finalCell = "-----";
        const approach = mcdu.flightPlanManager.getApproach();
        if (approach && approach.name) {
            finalCell = Avionics.Utils.formatRunway(approach.name);
        }
        let mdaCell = "[ ]";
        if (isFinite(mcdu.perfApprMDA)) {
            mdaCell = mcdu.perfApprMDA.toFixed(0);
        }
        mcdu.onRightInput[1] = (value) => {
            if (mcdu.setPerfApprMDA(value) && mcdu.setPerfApprDH(FMCMainDisplay.clrValue)) {
                CDUPerformancePage.ShowAPPRPage(mcdu);
            }
        };
        let dhCell = "[ ]";
        if (isFinite(mcdu.perfApprDH)) {
            dhCell = mcdu.perfApprDH.toFixed(0);
        } else if (mcdu.perfApprDH === "NO DH") {
            dhCell = "NO DH";
        }
        mcdu.onRightInput[2] = (value) => {
            if (mcdu.setPerfApprDH(value) && mcdu.setPerfApprMDA(FMCMainDisplay.clrValue)) {
                CDUPerformancePage.ShowAPPRPage(mcdu);
            }
        };

        const bottomRowLabels = ["\xa0PREV", "NEXT\xa0"];
        const bottomRowCells = ["<PHASE", "PHASE>"];
        if (mcdu.currentFlightPhase === FmgcFlightPhases.APPROACH) {
            bottomRowLabels[0] = "";
            bottomRowCells[0] = "";
        } else {
            if (mcdu.currentFlightPhase === FmgcFlightPhases.GOAROUND) {
                mcdu.leftInputDelay[5] = () => {
                    return mcdu.getDelaySwitchPage();
                };
                mcdu.onLeftInput[5] = (value) => {
                    CDUPerformancePage.ShowGOAROUNDPage(mcdu);
                };
            } else {
                mcdu.leftInputDelay[5] = () => {
                    return mcdu.getDelaySwitchPage();
                };
                mcdu.onLeftInput[5] = (value) => {
                    CDUPerformancePage.ShowDESPage(mcdu);
                };
            }
        }
        if (mcdu.currentFlightPhase === FmgcFlightPhases.GOAROUND) {
            bottomRowLabels[1] = "";
            bottomRowCells[1] = "";
        } else {
            mcdu.rightInputDelay[5] = () => {
                return mcdu.getDelaySwitchPage();
            };
            mcdu.onRightInput[5] = (value) => {
                CDUPerformancePage.ShowGOAROUNDPage(mcdu);
            };
        }

        mcdu.setTemplate([
            ["APPR[color]" + titleColor],
            ["QNH", "FINAL", "FLP RETR{sp}"],
            [qnhCell + "[color]cyan", finalCell + "[color]green", "F=" + flpRetrCell + "[color]green"],
            ["TEMP", "MDA", "SLT RETR{sp}"],
            [tempCell + "°[color]cyan", mdaCell + "[color]cyan", "S=" + sltRetrCell + "[color]green"],
            ["MAG WIND", "DH", "{sp}{sp}CLEAN"],
            [magWindHeadingCell + "°/" + magWindSpeedCell + "[color]cyan", dhCell + "[color]cyan", "O=" + cleanCell + "[color]green"],
            ["TRANS ALT", "LDG CONF"],
            [transAltCell + "[color]cyan", mcdu.perfApprFlaps3 ? "CONF3[color]cyan" : "[s-text]CONF3*[color]cyan"],
            ["VAPP", "", "VLS"],
            [vappCell + "[color]cyan", mcdu.perfApprFlaps3 ? "[s-text]FULL*[color]cyan" : "FULL[color]cyan", vlsCell + "[color]green"],
            bottomRowLabels,
            bottomRowCells,
        ]);
    }
    static ShowGOAROUNDPage(mcdu, confirmAppr = false) {
        mcdu.clearDisplay();
        mcdu.page.Current = mcdu.page.PerformancePageGoAround;
        CDUPerformancePage._timer = 0;
        CDUPerformancePage._lastPhase = mcdu.currentFlightPhase;
        mcdu.pageUpdate = () => {
            CDUPerformancePage._timer++;
            if (CDUPerformancePage._timer >= 100) {
                if (mcdu.currentFlightPhase === CDUPerformancePage._lastPhase) {
                    CDUPerformancePage.ShowGOAROUNDPage(mcdu);
                } else {
                    CDUPerformancePage.ShowPage(mcdu);
                }
            }
        };

        let titleColor = "white";
        if (mcdu.currentFlightPhase === FmgcFlightPhases.GOAROUND) {
            titleColor = "green";
        }
        let thrRedAcc = "---";
        if (isFinite(mcdu.thrustReductionAltitudeGoaround) && mcdu.thrustReductionAltitudeGoaround != 0) {
            thrRedAcc = mcdu.thrustReductionAltitudeGoaround.toFixed(0);
        }

        thrRedAcc += "/";

        if (isFinite(mcdu.accelerationAltitudeGoaround && mcdu.accelerationAltitudeGoaround != 0)) {
            thrRedAcc += mcdu.accelerationAltitudeGoaround.toFixed(0);
        } else {
            thrRedAcc += "---";
        }
        thrRedAcc += "[color]cyan";

        mcdu.onLeftInput[4] = (value) => {
            if (mcdu.trySetThrustReductionAccelerationAltitudeGoaround(value)) {
                CDUPerformancePage.ShowGOAROUNDPage(mcdu);
            }
        };

        let engOut = "---";
        if (isFinite(mcdu.engineOutAccelerationAltitudeGoaround) && mcdu.engineOutAccelerationAltitudeGoaround != 0) {
            engOut = mcdu.engineOutAccelerationAltitudeGoaround.toFixed(0);
        } else if (isFinite(mcdu.thrustReductionAltitudeGoaround) && mcdu.thrustReductionAltitudeGoaround != 0) {
            engOut = mcdu.thrustReductionAltitudeGoaround.toFixed(0);
        }
        engOut += "[color]cyan";

        mcdu.onRightInput[4] = (value) => {
            if (mcdu.trySetengineOutAccelerationAltitudeGoaround(value)) {
                CDUPerformancePage.ShowGOAROUNDPage(mcdu);
            }
        };
        let flpRetrCell = "---";
        const flapSpeed = mcdu.computedVfs;
        if (isFinite(flapSpeed)) {
            flpRetrCell = flapSpeed.toFixed(0) + "[color]green";
        }
        let sltRetrCell = "---";
        const slatSpeed = mcdu.computedVss;
        if (isFinite(slatSpeed)) {
            sltRetrCell = slatSpeed.toFixed(0) + "[color]green";
        }
        let cleanCell = "---";
        const cleanSpeed = mcdu.computedVgd;
        if (isFinite(cleanSpeed)) {
            cleanCell = cleanSpeed.toFixed(0) + "[color]green";
        }

        const bottomRowLabels = ["", ""];
        const bottomRowCells = ["", ""];
        if (mcdu.currentFlightPhase === FmgcFlightPhases.GOAROUND) {
            if (confirmAppr) {
                bottomRowLabels[0] = "\xa0CONFIRM[color]amber";
                bottomRowCells[0] = "*APPR PHASE[color]amber";
                mcdu.leftInputDelay[5] = () => {
                    return mcdu.getDelaySwitchPage();
                };
                mcdu.onLeftInput[5] = async () => {
                    if (mcdu.tryGoInApproachPhase()) {
                        CDUPerformancePage.ShowAPPRPage(mcdu);
                    }
                };
            } else {
                bottomRowLabels[0] = "\xa0ACTIVATE[color]cyan";
                bottomRowCells[0] = "{APPR PHASE[color]cyan";
                mcdu.leftInputDelay[5] = () => {
                    return mcdu.getDelaySwitchPage();
                };
                mcdu.onLeftInput[5] = () => {
                    CDUPerformancePage.ShowGOAROUNDPage(mcdu, true);
                };
            }
            bottomRowLabels[1] = "NEXT\xa0";
            bottomRowCells[1] = "PHASE>";
            mcdu.rightInputDelay[5] = () => {
                return mcdu.getDelaySwitchPage();
            };
            mcdu.onRightInput[5] = () => {
                CDUPerformancePage.ShowAPPRPage(mcdu);
            };
        } else {
            bottomRowLabels[0] = "\xa0PREV";
            bottomRowCells[0] = "<PHASE";
            mcdu.leftInputDelay[5] = () => {
                return mcdu.getDelaySwitchPage();
            };
            mcdu.onLeftInput[5] = () => {
                CDUPerformancePage.ShowAPPRPage(mcdu);
            };
        }
        mcdu.setTemplate([
            ["GO AROUND[color]" + titleColor],
            ["", "", "FLP RETR{sp}"],
            ["", "", "F=" + flpRetrCell + "[color]green"],
            ["", "", "SLT RETR{sp}"],
            ["", "", "S=" + sltRetrCell + "[color]green"],
            ["", "", "{sp}{sp}CLEAN"],
            ["", "", "O=" + cleanCell + "[color]green"],
            [""],
            [""],
            ["THR RED/ACC", "ENG OUT ACC"],
            [thrRedAcc + "[color]cyan", engOut + "[color]cyan]"],
            bottomRowLabels,
            bottomRowCells,
        ]);
    }
    static UpdateThrRedAccFromOrigin(mcdu, updateThrRedAlt = true, updateAccAlt = true) {
        const origin = mcdu.flightPlanManager.getOrigin();
        const elevation = origin ? origin.altitudeinFP : SimVar.GetSimVarValue("GROUND ALTITUDE", "feet");

        if (updateThrRedAlt && !mcdu.thrustReductionAltitudeIsPilotEntered) {
            const thrRedOffset = +NXDataStore.get("CONFIG_THR_RED_ALT", "1500");
            const thrRedAltitude = Math.round((elevation + thrRedOffset) / 10) * 10;

            mcdu.thrustReductionAltitude = thrRedAltitude;
            mcdu.thrustReductionAltitudeIsPilotEntered = false;
            SimVar.SetSimVarValue("L:AIRLINER_THR_RED_ALT", "Number", thrRedAltitude);
        }

        if (updateAccAlt && !mcdu.accelerationAltitudeIsPilotEntered) {
            const accOffset = +NXDataStore.get("CONFIG_ACCEL_ALT", "1500");
            const accAlt = Math.round((elevation + accOffset) / 10) * 10;

            mcdu.accelerationAltitude = accAlt;
            mcdu.accelerationAltitudeIsPilotEntered = false;
            SimVar.SetSimVarValue("L:AIRLINER_ACC_ALT", "Number", accAlt);
        }
    }
    static UpdateEngOutAccFromOrigin(mcdu) {
        if (mcdu.engineOutAccelerationAltitudeIsPilotEntered) {
            return;
        }
        const origin = mcdu.flightPlanManager.getOrigin();
        const elevation = origin ? origin.altitudeinFP : SimVar.GetSimVarValue("GROUND ALTITUDE", "feet");

        const offset = +NXDataStore.get("CONFIG_ENG_OUT_ACCEL_ALT", "1500");
        const alt = Math.round((elevation + offset) / 10) * 10;

        mcdu.engineOutAccelerationAltitude = alt;
        mcdu.engineOutAccelerationAltitudeIsPilotEntered = false;
        SimVar.SetSimVarValue("L:A32NX_ENG_OUT_ACC_ALT", "feet", alt);
    }
    static UpdateThrRedAccFromDestination(mcdu) {
        const destination = mcdu.flightPlanManager.getDestination();
        const elevation = destination ? destination.altitudeinFP : SimVar.GetSimVarValue("GROUND ALTITUDE", "feet");

        const offset = +NXDataStore.get("CONFIG_ENG_OUT_ACCEL_ALT", "1500");
        const alt = Math.round((elevation + offset) / 10) * 10;

        mcdu.thrustReductionAltitudeGoaround = alt;
        mcdu.accelerationAltitudeGoaround = alt;
        mcdu.engineOutAccelerationAltitudeGoaround = alt;

        SimVar.SetSimVarValue("L:AIRLINER_THR_RED_ALT_GOAROUND", "Number", alt);
        SimVar.SetSimVarValue("L:AIRLINER_ACC_ALT_GOAROUND", "Number", alt);
        SimVar.SetSimVarValue("L:AIRLINER_ENG_OUT_ACC_ALT_GOAROUND", "Number", alt);
    }
}
CDUPerformancePage._timer = 0;
