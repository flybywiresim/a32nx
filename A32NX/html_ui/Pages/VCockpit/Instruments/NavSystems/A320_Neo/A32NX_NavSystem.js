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

class NavSystem extends BaseInstrument {
    constructor() {
        super(...arguments);
        this.soundSourceNode = "AS1000_MFD";
        this.eventAliases = [];
        this.IndependentsElements = [];
        this.pageGroups = [];
        this.eventLinkedPageGroups = [];
        this.currentEventLinkedPageGroup = null;
        this.overridePage = null;
        this.eventLinkedPopUpElements = [];
        this.popUpElement = null;
        this.popUpCloseCallback = null;
        this.currentPageGroupIndex = 0;
        this.currentInteractionState = 0;
        this.cursorIndex = 0;
        this.currentSelectableArray = [];
        this.currentContextualMenu = null;
        this.currentSearchFieldWaypoint = null;
        this.contextualMenuDisplayBeginIndex = 0;
        this.menuMaxElems = 6;
        this.useUpdateBudget = true;
        this.maxUpdateBudget = 6;
        this.budgetedItemId = 0;
        this.aspectRatioElement = null;
        this.forcedAspectRatioSet = false;
        this.forcedAspectRatio = 1;
        this.forcedScreenRatio = 1;
        this.initDuration = 0;
        this.hasBeenOff = false;
        this.needValidationAfterInit = false;
        this.isStarted = false;
        this.initAcknowledged = true;
        this.reversionaryMode = false;
        this.alwaysUpdateList = new Array();
        this.accumulatedDeltaTime = 0;
    }
    get flightPlanManager() {
        return this.currFlightPlanManager;
    }
    get instrumentAlias() {
        return null;
    }
    connectedCallback() {
        super.connectedCallback();
        this.contextualMenu = this.getChildById("ContextualMenu");
        this.contextualMenuTitle = this.getChildById("ContextualMenuTitle");
        this.contextualMenuElements = this.getChildById("ContextualMenuElements");
        this.menuSlider = this.getChildById("SliderMenu");
        this.menuSliderCursor = this.getChildById("SliderMenuCursor");
        this.currFlightPlanManager = new fpm.FlightPlanManager(this);
        this.currFlightPlan = new fpm.ManagedFlightPlan();
    }
    disconnectedCallback() {
        super.disconnectedCallback();
    }
    parseXMLConfig() {
        super.parseXMLConfig();
        const soundSourceNodeElem = this.xmlConfig.getElementsByTagName("SoundSourceNode");
        if (soundSourceNodeElem.length > 0) {
            this.soundSourceNode = soundSourceNodeElem[0].textContent;
        }
        if (this.instrumentXmlConfig) {
            const skipValidationAfterInitElem = this.instrumentXmlConfig.getElementsByTagName("SkipValidationAfterInit");
            if (skipValidationAfterInitElem.length > 0 && this.needValidationAfterInit) {
                this.needValidationAfterInit = skipValidationAfterInitElem[0].textContent != "True";
            }
            const styleNode = this.instrumentXmlConfig.getElementsByTagName("Style");
            if (styleNode.length > 0) {
                this.electricity.setAttribute("displaystyle", styleNode[0].textContent);
            }
        }
    }
    computeEvent(_event) {
        if (this.isBootProcedureComplete()) {
            for (let i = 0; i < this.eventLinkedPageGroups.length; i++) {
                if (_event == this.eventLinkedPageGroups[i].popUpEvent) {
                    this.lastRelevantICAO = null;
                    this.lastRelevantICAOType = null;
                    if (this.overridePage) {
                        this.closeOverridePage();
                    }
                    if (this.eventLinkedPageGroups[i] == this.currentEventLinkedPageGroup) {
                        this.exitEventLinkedPageGroup();
                    } else {
                        const currentGroup = this.getCurrentPageGroup();
                        if (currentGroup) {
                            currentGroup.onExit();
                        }
                        this.currentEventLinkedPageGroup = this.eventLinkedPageGroups[i];
                        this.currentEventLinkedPageGroup.pageGroup.onEnter();
                    }
                    this.SwitchToInteractionState(0);
                }
            }
            for (let i = 0; i < this.eventLinkedPopUpElements.length; i++) {
                if (_event == this.eventLinkedPopUpElements[i].popUpEvent) {
                    if (this.popUpElement == this.eventLinkedPopUpElements[i]) {
                        this.popUpElement.onExit();
                        this.popUpElement = null;
                    } else {
                        this.switchToPopUpPage(this.eventLinkedPopUpElements[i]);
                    }
                }
            }
            for (let i = 0; i < this.IndependentsElements.length; i++) {
                this.IndependentsElements[i].onEvent(_event);
            }
            if (this.popUpElement) {
                this.popUpElement.onEvent(_event);
            }
            const currentPage = this.getCurrentPage();
            if (currentPage) {
                currentPage.onEvent(_event);
            }
            switch (this.currentInteractionState) {
                case 1:
                    if (this.currentSelectableArray[this.cursorIndex].SendEvent(_event)) {
                        break;
                    }
                    if (_event == "NavigationPush") {
                        this.SwitchToInteractionState(0);
                    }
                    if (_event == "NavigationLargeInc") {
                        this.cursorIndex = (this.cursorIndex + 1) % this.currentSelectableArray.length;
                        while (!this.currentSelectableArray[this.cursorIndex].onSelection(_event)) {
                            this.cursorIndex = (this.cursorIndex + 1) % this.currentSelectableArray.length;
                        }
                    }
                    if (_event == "NavigationLargeDec") {
                        this.cursorIndex = (this.cursorIndex - 1) < 0 ? (this.currentSelectableArray.length - 1) : (this.cursorIndex - 1);
                        while (!this.currentSelectableArray[this.cursorIndex].onSelection(_event)) {
                            this.cursorIndex = (this.cursorIndex - 1) < 0 ? (this.currentSelectableArray.length - 1) : (this.cursorIndex - 1);
                        }
                    }
                    if (_event == "MENU_Push") {
                        var defaultMenu;
                        if (this.popUpElement) {
                            defaultMenu = this.popUpElement.getDefaultMenu();
                        }
                        if (!defaultMenu) {
                            var defaultMenu = this.getCurrentPage().defaultMenu;
                        }
                        if (defaultMenu != null) {
                            this.ShowContextualMenu(defaultMenu);
                        }
                    }
                    break;
                case 2:
                    if (_event == "NavigationSmallInc") {
                        let count = 0;
                        do {
                            this.cursorIndex = (this.cursorIndex + 1) % this.currentContextualMenu.elements.length;
                            if (this.cursorIndex > (this.contextualMenuDisplayBeginIndex + 5)) {
                                this.contextualMenuDisplayBeginIndex++;
                            }
                            if (this.cursorIndex < (this.contextualMenuDisplayBeginIndex)) {
                                this.contextualMenuDisplayBeginIndex = 0;
                            }
                            count++;
                        } while (this.currentContextualMenu.elements[this.cursorIndex].isInactive() == true && count < this.currentContextualMenu.elements.length);
                    }
                    if (_event == "NavigationSmallDec") {
                        const count = 0;
                        do {
                            this.cursorIndex = (this.cursorIndex - 1) < 0 ? (this.currentContextualMenu.elements.length - 1) : (this.cursorIndex - 1);
                            if (this.cursorIndex < (this.contextualMenuDisplayBeginIndex)) {
                                this.contextualMenuDisplayBeginIndex--;
                            }
                            if (this.cursorIndex > (this.contextualMenuDisplayBeginIndex + 5)) {
                                this.contextualMenuDisplayBeginIndex = this.currentContextualMenu.elements.length - 5;
                            }
                        } while (this.currentContextualMenu.elements[this.cursorIndex].isInactive() == true && count < this.currentContextualMenu.elements.length);
                    }
                    if (_event == "MENU_Push") {
                        this.SwitchToInteractionState(0);
                    }
                    if (_event == "ENT_Push") {
                        this.currentContextualMenu.elements[this.cursorIndex].SendEvent();
                    }
                    break;
                case 3:
                    this.currentSearchFieldWaypoint.onInteractionEvent([_event]);
                    break;
                case 0:
                    if (_event == "MENU_Push") {
                        var defaultMenu;
                        if (this.popUpElement) {
                            defaultMenu = this.popUpElement.getDefaultMenu();
                        }
                        if (!defaultMenu) {
                            var defaultMenu = this.getCurrentPage().defaultMenu;
                        }
                        if (defaultMenu != null) {
                            this.ShowContextualMenu(defaultMenu);
                        }
                    }
                    if (_event == "NavigationSmallInc") {
                        this.lastRelevantICAO = null;
                        this.lastRelevantICAOType = null;
                        this.getCurrentPageGroup().nextPage();
                    }
                    if (_event == "NavigationSmallDec") {
                        this.lastRelevantICAO = null;
                        this.lastRelevantICAOType = null;
                        this.getCurrentPageGroup().prevPage();
                    }
                    if (_event == "NavigationLargeInc") {
                        this.lastRelevantICAO = null;
                        this.lastRelevantICAOType = null;
                        if (this.pageGroups.length > 1 && !this.currentEventLinkedPageGroup) {
                            this.pageGroups[this.currentPageGroupIndex].onExit();
                            this.currentPageGroupIndex = (this.currentPageGroupIndex + 1) % this.pageGroups.length;
                            this.pageGroups[this.currentPageGroupIndex].onEnter();
                        }
                    }
                    if (_event == "NavigationLargeDec") {
                        this.lastRelevantICAO = null;
                        this.lastRelevantICAOType = null;
                        if (this.pageGroups.length > 1 && !this.currentEventLinkedPageGroup) {
                            this.pageGroups[this.currentPageGroupIndex].onExit();
                            this.currentPageGroupIndex = (this.currentPageGroupIndex + this.pageGroups.length - 1) % this.pageGroups.length;
                            this.pageGroups[this.currentPageGroupIndex].onEnter();
                        }
                    }
                    if (_event == "NavigationPush") {
                        let defaultSelectableArray = this.getCurrentPage().element.getDefaultSelectables();
                        if (this.popUpElement) {
                            defaultSelectableArray = this.popUpElement.element.getDefaultSelectables();
                        }
                        if (defaultSelectableArray != null && defaultSelectableArray.length > 0) {
                            this.ActiveSelection(defaultSelectableArray);
                        }
                    }
                    break;
            }
            switch (_event) {
                case "ActiveFPL_Modified":
                    this.currFlightPlan.FillWithCurrentFP();
            }
        }
        this.onEvent(_event);
    }
    exitEventLinkedPageGroup() {
        this.currentEventLinkedPageGroup.pageGroup.onExit();
        this.currentEventLinkedPageGroup = null;
        const currentGroup = this.getCurrentPageGroup();
        if (currentGroup) {
            currentGroup.onEnter();
        }
    }
    DecomposeEventFromPrefix(_args) {
        let search = this.instrumentIdentifier + "_";
        if (_args[0].startsWith(search)) {
            return _args[0].slice(search.length);
        }
        search = this.instrumentAlias;
        if (search != null && search != "") {
            if (this.urlConfig.index) {
                search += "_" + this.urlConfig.index;
            }
            search += "_";
            if (_args[0].startsWith(search)) {
                return _args[0].slice(search.length);
            }
        }
        search = this.templateID + "_";
        if (_args[0].startsWith(search)) {
            const evt = _args[0].slice(search.length);
            const separator = evt.search("_");
            if (separator >= 0) {
                if (!isFinite(parseInt(evt.substring(0, separator)))) {
                    return evt;
                }
            } else if (!isFinite(parseInt(evt))) {
                return evt;
            }
        }
        search = "Generic_";
        if (_args[0].startsWith(search)) {
            return _args[0].slice(search.length);
        }
        return null;
    }
    onInteractionEvent(_args) {
        if (this.isElectricityAvailable() || SimVar.GetSimVarValue("L:DCPowerAvailable", "Bool") == 1) {
            let event = this.DecomposeEventFromPrefix(_args);
            if (event) {
                if (event == "ElementSetAttribute" && _args.length >= 4) {
                    const element = this.getChildById(_args[1]);
                    if (element) {
                        Avionics.Utils.diffAndSetAttribute(element, _args[2], _args[3]);
                    }
                } else {
                    this.computeEvent(event);
                    for (let i = 0; i < this.eventAliases.length; i++) {
                        if (this.eventAliases[i].source == event) {
                            this.computeEvent(this.eventAliases[i].output);
                        }
                    }
                }
            } else if (_args[0].startsWith("NavSystem_")) {
                event = _args[0].slice("NavSystem_".length);
                this.computeEvent(event);
                for (let i = 0; i < this.eventAliases.length; i++) {
                    if (this.eventAliases[i].source == event) {
                        this.computeEvent(this.eventAliases[i].output);
                    }
                }
            }
        } else {
            console.log("Electricity Is NOT Available");
        }
    }
    reboot() {
        super.reboot();
        this.startTime = Date.now();
        this.hasBeenOff = false;
        this.isStarted = false;
        this.initAcknowledged = true;
        this.budgetedItemId = 0;
    }
    Update() {
        super.Update();
        this.accumulatedDeltaTime += this.deltaTime;
        if (this.CanUpdate()) {
            if (NavSystem._iterations < 10000) {
                NavSystem._iterations += 1;
            }
            const t0 = performance.now();
            if (this.isElectricityAvailable()) {
                if (!this.isStarted) {
                    this.onPowerOn();
                }
                if (this.isBootProcedureComplete()) {
                    if (this.reversionaryMode) {
                        if (this.electricity) {
                            this.electricity.setAttribute("state", "Backup");
                            SimVar.SetSimVarValue("L:" + this.instrumentIdentifier + "_ScreenLuminosity", "number", 1);
                            SimVar.SetSimVarValue("L:" + this.instrumentIdentifier + "_State", "number", 3);
                        }
                    } else {
                        if (this.electricity) {
                            this.electricity.setAttribute("state", "on");
                            SimVar.SetSimVarValue("L:" + this.instrumentIdentifier + "_ScreenLuminosity", "number", 1);
                            SimVar.SetSimVarValue("L:" + this.instrumentIdentifier + "_State", "number", 2);
                        }
                    }
                } else if (Date.now() - this.startTime > this.initDuration) {
                    if (this.electricity) {
                        this.electricity.setAttribute("state", "initWaitingValidation");
                        SimVar.SetSimVarValue("L:" + this.instrumentIdentifier + "_ScreenLuminosity", "number", 0.2);
                        SimVar.SetSimVarValue("L:" + this.instrumentIdentifier + "_State", "number", 1);
                    }
                } else {
                    if (this.electricity) {
                        this.electricity.setAttribute("state", "init");
                        SimVar.SetSimVarValue("L:" + this.instrumentIdentifier + "_ScreenLuminosity", "number", 0.2);
                        SimVar.SetSimVarValue("L:" + this.instrumentIdentifier + "_State", "number", 1);
                    }
                }
            } else {
                if (this.isStarted) {
                    this.onShutDown();
                }
                if (this.electricity) {
                    this.electricity.setAttribute("state", "off");
                    SimVar.SetSimVarValue("L:" + this.instrumentIdentifier + "_ScreenLuminosity", "number", 0);
                    SimVar.SetSimVarValue("L:" + this.instrumentIdentifier + "_State", "number", 0);
                }
            }
            this.updateAspectRatio();
            if (this.popUpElement) {
                this.popUpElement.onUpdate(this.accumulatedDeltaTime);
            }
            if (this.pagesContainer) {
                this.pagesContainer.setAttribute("state", this.getCurrentPage().htmlElemId);
            }
            if (this.useUpdateBudget) {
                this.updateGroupsWithBudget();
            } else {
                this.updateGroups();
            }
            switch (this.currentInteractionState) {
                case 0:
                    for (var i = 0; i < this.currentSelectableArray.length; i++) {
                        this.currentSelectableArray[i].updateSelection(false);
                    }
                    break;
                case 1:
                    for (var i = 0; i < this.currentSelectableArray.length; i++) {
                        if (i == this.cursorIndex) {
                            this.currentSelectableArray[i].updateSelection(this.blinkGetState(400, 200));
                        } else {
                            this.currentSelectableArray[i].updateSelection(false);
                        }
                    }
                    break;
                case 2:
                    this.currentContextualMenu.Update(this, this.menuMaxElems);
                    break;
            }
            this.onUpdate(this.accumulatedDeltaTime);
            const t = performance.now() - t0;
            NavSystem.maxTimeUpdateAllTime = Math.max(t, NavSystem.maxTimeUpdateAllTime);
            NavSystem.maxTimeUpdate = Math.max(t, NavSystem.maxTimeUpdate);
            const factor = 1 / NavSystem._iterations;
            NavSystem.mediumTimeUpdate *= (1 - factor);
            NavSystem.mediumTimeUpdate += factor * t;
            this.accumulatedDeltaTime = 0;
        } else {
            for (var i = 0; i < this.alwaysUpdateList.length; i++) {
                this.alwaysUpdateList[i].onUpdate(this.deltaTime);
            }
        }
    }
    updateGroups() {
        for (let i = 0; i < this.IndependentsElements.length; i++) {
            this.IndependentsElements[i].onUpdate(this.accumulatedDeltaTime);
        }
        if (!this.overridePage) {
            const currentGroup = this.getCurrentPageGroup();
            if (currentGroup) {
                currentGroup.onUpdate(this.accumulatedDeltaTime);
            }
        } else {
            this.overridePage.onUpdate(this.accumulatedDeltaTime);
        }
    }
    updateGroupsWithBudget() {
        const target = this.budgetedItemId + this.maxUpdateBudget;
        while (this.budgetedItemId < target) {
            if (this.budgetedItemId < this.IndependentsElements.length) {
                this.IndependentsElements[this.budgetedItemId].onUpdate(this.accumulatedDeltaTime);
                this.budgetedItemId++;
                continue;
            }
            if (!this.overridePage) {
                const currentGroup = this.getCurrentPageGroup();
                if (currentGroup) {
                    const itemId = this.budgetedItemId - this.IndependentsElements.length;
                    if (currentGroup.onUpdateSpecificItem(this.accumulatedDeltaTime, itemId)) {
                        this.budgetedItemId++;
                        continue;
                    }
                }
            } else {
                this.overridePage.onUpdate(this.accumulatedDeltaTime);
            }
            this.budgetedItemId = 0;
            break;
        }
    }
    onEvent(_event) {
    }
    onUpdate(_deltaTime) {
    }
    GetComActiveFreq() {
        return this.frequencyFormat(SimVar.GetSimVarValue("COM ACTIVE FREQUENCY:1", "MHz"), 3);
    }
    GetComStandbyFreq() {
        return this.frequencyFormat(SimVar.GetSimVarValue("COM STANDBY FREQUENCY:1", "MHz"), 3);
    }
    GetNavActiveFreq() {
        return this.frequencyFormat(SimVar.GetSimVarValue("NAV ACTIVE FREQUENCY:1", "MHz"), 2);
    }
    GetNavStandbyFreq() {
        return this.frequencyFormat(SimVar.GetSimVarValue("NAV STANDBY FREQUENCY:1", "MHz"), 2);
    }
    UpdateSlider(_slider, _cursor, _index, _nbElem, _maxElems) {
        if (_nbElem > _maxElems) {
            _slider.setAttribute("state", "Active");
            _cursor.setAttribute("style", "height:" + (_maxElems * 100 / _nbElem) +
                "%;top:" + (_index * 100 / _nbElem) + "%");
        } else {
            _slider.setAttribute("state", "Inactive");
        }
    }
    frequencyFormat(_frequency, _nbDigits) {
        const IntPart = Math.floor(_frequency);
        const Digits = Math.round((_frequency - IntPart) * Math.pow(10, _nbDigits));
        return fastToFixed(IntPart, 0) + '.<span class="FrecDecimals">' + ('000' + (Digits)).slice(-_nbDigits) + '</span>';
    }
    frequencyListFormat(_airport, _baseId, _maxElem = -1, _firstElemIndex = 0) {
        if (_airport && _airport.frequencies) {
            let htmlFreq = "";
            let endIndex = _airport.frequencies.length;
            if (_maxElem != -1) {
                endIndex = Math.min(_airport.frequencies.length, _firstElemIndex + _maxElem);
            }
            for (let i = 0; i < Math.min(_airport.frequencies.length, _maxElem); i++) {
                htmlFreq += '<div><div class="Align LeftDisplay" id="' + _baseId + "Name_" + i + '">' + _airport.frequencies[i + _firstElemIndex].name.replace(" ", "&nbsp;").slice(0, 15) + '</div> <div class="Align RightValue SelectableElement" id="' + _baseId + i + '">' + this.frequencyFormat(_airport.frequencies[i + _firstElemIndex].mhValue, 3) + '</div></div>';
            }
            return htmlFreq;
        } else {
            return "";
        }
    }
    airportPrivateTypeStrFromEnum(_enum) {
        switch (_enum) {
            case 0:
                return "Unknown";
            case 1:
                return "Public";
            case 2:
                return "Military";
            case 3:
                return "Private";
        }
    }
    longitudeFormat(_longitude) {
        let format = "";
        if (_longitude < 0) {
            format += "W";
            _longitude = Math.abs(_longitude);
        } else {
            format += "E";
        }
        const degrees = Math.floor(_longitude);
        const minutes = ((_longitude - degrees) * 60);
        format += fastToFixed(degrees, 0);
        format += "°";
        format += fastToFixed(minutes, 2);
        format += "'";
        return format;
    }
    latitudeFormat(_latitude) {
        let format = "";
        if (_latitude < 0) {
            format += "S";
            _latitude = Math.abs(_latitude);
        } else {
            format += "N";
        }
        const degrees = Math.floor(_latitude);
        const minutes = ((_latitude - degrees) * 60);
        format += fastToFixed(degrees, 0);
        format += "°";
        format += fastToFixed(minutes, 2);
        format += "'";
        return format;
    }
    InteractionStateOut() {
        switch (this.currentInteractionState) {
            case 0:
                break;
            case 1:
                for (let i = 0; i < this.currentSelectableArray.length; i++) {
                    this.currentSelectableArray[i].updateSelection(false);
                }
                break;
            case 2:
                this.contextualMenu.setAttribute("state", "Inactive");
                break;
        }
    }
    InteractionStateIn() {
        switch (this.currentInteractionState) {
            case 0:
                break;
            case 1:
                this.cursorIndex = 0;
                break;
            case 2:
                this.contextualMenu.setAttribute("state", "Active");
                this.contextualMenuDisplayBeginIndex = 0;
                this.cursorIndex = 0;
                if (this.currentContextualMenu.elements[0].isInactive()) {
                    this.computeEvent("NavigationSmallInc");
                }
                break;
        }
    }
    SwitchToInteractionState(_newState) {
        this.InteractionStateOut();
        this.currentInteractionState = _newState;
        this.InteractionStateIn();
    }
    ShowContextualMenu(_menu) {
        this.currentContextualMenu = _menu;
        this.SwitchToInteractionState(2);
        this.currentContextualMenu.Update(this, this.menuMaxElems);
    }
    ActiveSelection(_selectables) {
        this.SwitchToInteractionState(1);
        this.currentSelectableArray = _selectables;
        const begin = this.cursorIndex;
        while (!this.currentSelectableArray[this.cursorIndex].isActive) {
            this.cursorIndex = (this.cursorIndex + 1) % this.currentSelectableArray.length;
            if (this.cursorIndex == begin) {
                this.SwitchToInteractionState(0);
                return;
            }
        }
    }
    setOverridePage(_page) {
        if (this.overridePage) {
            this.overridePage.onExit();
        }
        if (this.currentContextualMenu) {
            this.SwitchToInteractionState(0);
        }
        this.overridePage = _page;
        this.overridePage.onEnter();
    }
    closeOverridePage() {
        if (this.overridePage) {
            this.overridePage.onExit();
        }
        if (this.currentContextualMenu) {
            this.SwitchToInteractionState(0);
        }
        this.overridePage = null;
    }
    SwitchToPageName(_menu, _page) {
        this.lastRelevantICAO = null;
        this.lastRelevantICAOType = null;
        if (this.overridePage) {
            this.closeOverridePage();
        }
        if (this.currentEventLinkedPageGroup) {
            this.currentEventLinkedPageGroup.pageGroup.onExit();
            this.currentEventLinkedPageGroup = null;
        }
        this.pageGroups[this.currentPageGroupIndex].onExit();
        if (this.currentContextualMenu) {
            this.SwitchToInteractionState(0);
        }
        for (let i = 0; i < this.pageGroups.length; i++) {
            if (this.pageGroups[i].name == _menu) {
                this.currentPageGroupIndex = i;
            }
        }
        this.pageGroups[this.currentPageGroupIndex].goToPage(_page, true);
    }
    SwitchToMenuName(_name) {
        this.lastRelevantICAO = null;
        this.lastRelevantICAOType = null;
        this.pageGroups[this.currentPageGroupIndex].onExit();
        if (this.currentContextualMenu) {
            this.SwitchToInteractionState(0);
        }
        for (let i = 0; i < this.pageGroups.length; i++) {
            if (this.pageGroups[i].name == _name) {
                this.currentPageGroupIndex = i;
            }
        }
        this.pageGroups[this.currentPageGroupIndex].onEnter();
    }
    GetInteractionState() {
        return this.currentInteractionState;
    }
    blinkGetState(_blinkPeriod, _duration) {
        return Math.round((new Date().getTime()) / _duration) % (_blinkPeriod / _duration) == 0;
    }
    IsEditingSearchField() {
        return this.GetInteractionState() == 3;
    }
    OnSearchFieldEndEditing() {
        this.SwitchToInteractionState(0);
    }
    addEventLinkedPageGroup(_event, _pageGroup) {
        this.eventLinkedPageGroups.push(new NavSystemEventLinkedPageGroup(_pageGroup, _event));
    }
    addEventLinkedPopupWindow(_popUp) {
        this.eventLinkedPopUpElements.push(_popUp);
        _popUp.gps = this;
    }
    addIndependentElementContainer(_container) {
        _container.setGPS(this);
        this.IndependentsElements.push(_container);
    }
    getCurrentPageGroup() {
        if (this.currentEventLinkedPageGroup) {
            return this.currentEventLinkedPageGroup.pageGroup;
        } else {
            return this.pageGroups[this.currentPageGroupIndex];
        }
    }
    getCurrentPage() {
        if (!this.overridePage) {
            const currentGroup = this.getCurrentPageGroup();
            if (currentGroup) {
                return currentGroup.getCurrentPage();
            }
            return undefined;
        } else {
            return this.overridePage;
        }
    }
    leaveEventPage() {
        this.lastRelevantICAO = null;
        this.lastRelevantICAOType = null;
        this.currentEventLinkedPageGroup.pageGroup.onExit();
        this.currentEventLinkedPageGroup = null;
        this.getCurrentPageGroup().onEnter();
    }
    addEventAlias(_source, _output) {
        this.eventAliases.push(new NavSystemEventAlias(_source, _output));
    }
    closePopUpElement() {
        let callback = null;
        if (this.popUpElement) {
            callback = this.popUpCloseCallback;
            this.popUpElement.onExit();
        }
        this.popUpElement = null;
        this.popUpCloseCallback = null;
        if (this.currentContextualMenu) {
            this.SwitchToInteractionState(0);
        }
        if (callback) {
            callback();
        }
        ;
    }
    getElementOfType(c) {
        for (let i = 0; i < this.IndependentsElements.length; i++) {
            const elem = this.IndependentsElements[i].getElementOfType(c);
            if (elem) {
                return elem;
            }
        }
        const curr = this.getCurrentPage().element.getElementOfType(c);
        if (curr) {
            return curr;
        } else {
            for (let i = 0; i < this.pageGroups.length; i++) {
                for (let j = 0; j < this.pageGroups[i].pages.length; j++) {
                    const elem = this.pageGroups[i].pages[j].getElementOfType(c);
                    if (elem) {
                        return elem;
                    }
                }
            }
        }
        return null;
    }
    switchToPopUpPage(_pageContainer, _PopUpCloseCallback = null) {
        if (this.popUpElement) {
            this.popUpElement.onExit();
            if (this.popUpCloseCallback) {
                this.popUpCloseCallback();
            }
            ;
        }
        if (this.currentContextualMenu) {
            this.SwitchToInteractionState(0);
        }
        this.popUpCloseCallback = _PopUpCloseCallback;
        this.popUpElement = _pageContainer;
        this.popUpElement.onEnter();
    }
    preserveAspectRatio(_HtmlElementId) {
        this.aspectRatioElement = _HtmlElementId;
        this.forcedAspectRatioSet = false;
        this.updateAspectRatio();
    }
    updateAspectRatio() {
        if (this.forcedAspectRatioSet) {
            return;
        }
        if (this.aspectRatioElement == null) {
            return;
        }
        const frame = this.getChildById(this.aspectRatioElement);
        if (!frame) {
            return;
        }
        const vpRect = this.getBoundingClientRect();
        const vpWidth = vpRect.width;
        const vpHeight = vpRect.height;
        if (vpWidth <= 0 || vpHeight <= 0) {
            return;
        }
        const frameStyle = window.getComputedStyle(frame);
        if (!frameStyle) {
            return;
        }
        const refWidth = parseInt(frameStyle.getPropertyValue('--refWidth'));
        const refHeight = parseInt(frameStyle.getPropertyValue('--refHeight'));
        const curWidth = parseInt(frameStyle.width);
        const curHeight = parseInt(frameStyle.height);
        const curRatio = curHeight / curWidth;
        let refRatio = curRatio;
        if (refWidth > 0 && refHeight > 0) {
            console.log("Forcing aspectratio to " + refWidth + "*" + refHeight);
            refRatio = refHeight / refWidth;
            let newLeft = parseInt(frameStyle.left);
            let newWidth = curWidth;
            let newHeight = curWidth * refRatio;
            if (newHeight > vpHeight) {
                newWidth = vpHeight / refRatio;
                newHeight = vpHeight;
                newLeft += (curWidth - newWidth) * 0.5;
            }
            newLeft = Math.round(newLeft);
            newWidth = Math.round(newWidth);
            newHeight = Math.round(newHeight);
            frame.style.left = newLeft + "px";
            frame.style.width = newWidth + "px";
            frame.style.height = newHeight + "px";
            window.document.documentElement.style.setProperty("--bodyHeightScale", (newHeight / vpHeight).toString());
        }
        this.forcedScreenRatio = 1.0 / curRatio;
        this.forcedAspectRatio = 1.0 / refRatio;
        this.forcedAspectRatioSet = true;
    }
    isAspectRatioForced() {
        return this.forcedAspectRatioSet;
    }
    getForcedScreenRatio() {
        return this.forcedScreenRatio;
    }
    getForcedAspectRatio() {
        return this.forcedAspectRatio;
    }
    isComputingAspectRatio() {
        if (this.aspectRatioElement != null && !this.forcedAspectRatioSet) {
            return false;
        }
        return true;
    }
    onSoundEnd(_eventId) {
        for (let i = 0; i < this.pageGroups.length; i++) {
            for (let j = 0; j < this.pageGroups[i].pages.length; j++) {
                this.pageGroups[i].pages[j].onSoundEnd(_eventId);
            }
        }
        for (let i = 0; i < this.IndependentsElements.length; i++) {
            this.IndependentsElements[i].onSoundEnd(_eventId);
        }
        for (let i = 0; i < this.eventLinkedPopUpElements.length; i++) {
            this.eventLinkedPopUpElements[i].onSoundEnd(_eventId);
        }
    }
    onShutDown() {
        console.log("System Turned Off");
        this.hasBeenOff = true;
        this.isStarted = false;
        this.initAcknowledged = false;
        for (let i = 0; i < this.pageGroups.length; i++) {
            for (let j = 0; j < this.pageGroups[i].pages.length; j++) {
                this.pageGroups[i].pages[j].onShutDown();
            }
        }
        for (let i = 0; i < this.IndependentsElements.length; i++) {
            this.IndependentsElements[i].onShutDown();
        }
        for (let i = 0; i < this.eventLinkedPopUpElements.length; i++) {
            this.eventLinkedPopUpElements[i].onShutDown();
        }
        this.alwaysUpdateList.splice(0, this.alwaysUpdateList.length);
    }
    onPowerOn() {
        console.log("System Turned ON");
        this.startTime = Date.now();
        this.isStarted = true;
        this.budgetedItemId = 0;
        for (let i = 0; i < this.pageGroups.length; i++) {
            for (let j = 0; j < this.pageGroups[i].pages.length; j++) {
                this.pageGroups[i].pages[j].onPowerOn();
            }
        }
        for (let i = 0; i < this.IndependentsElements.length; i++) {
            this.IndependentsElements[i].onPowerOn();
        }
        for (let i = 0; i < this.eventLinkedPopUpElements.length; i++) {
            this.eventLinkedPopUpElements[i].onPowerOn();
        }
    }
    isBootProcedureComplete() {
        if (((Date.now() - this.startTime > this.initDuration) || !this.hasBeenOff) && (this.initAcknowledged || !this.needValidationAfterInit)) {
            return true;
        }
        return false;
    }
    acknowledgeInit() {
        this.initAcknowledged = true;
    }
    isInReversionaryMode() {
        return this.reversionaryMode;
    }
    wasTurnedOff() {
        return this.hasBeenOff;
    }
    hasWeatherRadar() {
        if (this.instrumentXmlConfig) {
            const elem = this.instrumentXmlConfig.getElementsByTagName("WeatherRadar");
            if (elem.length > 0 && (elem[0].textContent.toLowerCase() == "off" || elem[0].textContent.toLowerCase() == "none")) {
                return false;
            }
        }
        return true;
    }
    alwaysUpdate(_element, _val) {
        for (let i = 0; i < this.alwaysUpdateList.length; i++) {
            if (this.alwaysUpdateList[i] == _element) {
                if (!_val) {
                    this.alwaysUpdateList.splice(i, 1);
                }
                return;
            }
        }
        if (_val) {
            this.alwaysUpdateList.push(_element);
        }
    }
}
NavSystem.maxTimeUpdateAllTime = 0;
NavSystem.maxTimeUpdate = 0;
NavSystem.mediumMaxTimeUpdate = 0;
NavSystem.mediumTimeUpdate = 0;
NavSystem.maxTimeMapUpdateAllTime = 0;
NavSystem.maxTimeMapUpdate = 0;
NavSystem.mediumMaxTimeMapUpdate = 0;
NavSystem.mediumTimeMapUpdate = 0;
NavSystem._iterations = 0;
class NavSystemPageGroup {
    constructor(_name, _gps, _pages) {
        this._updatingWithBudget = false;
        this.name = _name;
        this.gps = _gps;
        this.pages = _pages;
        this.pageIndex = 0;
        for (let i = 0; i < _pages.length; i++) {
            _pages[i].pageGroup = this;
            _pages[i].gps = this.gps;
        }
    }
    getCurrentPage() {
        return this.pages[this.pageIndex];
    }
    onEnter() {
        this.pages[this.pageIndex].onEnter();
    }
    onUpdate(_deltaTime) {
        if (!this._updatingWithBudget) {
            this.pages[this.pageIndex].onUpdate(_deltaTime);
        }
    }
    onUpdateSpecificItem(_deltaTime, _itemId) {
        if (_itemId == 0) {
            this._updatingWithBudget = true;
            {
                this.onUpdate(_deltaTime);
            }
            this._updatingWithBudget = false;
        }
        return this.pages[this.pageIndex].onUpdateSpecificItem(_deltaTime, _itemId);
    }
    onExit() {
        this.pages[this.pageIndex].onExit();
    }
    nextPage() {
        if (this.pages.length > 1) {
            this.pages[this.pageIndex].onExit();
            this.pageIndex = (this.pageIndex + 1) % this.pages.length;
            this.pages[this.pageIndex].onEnter();
        }
    }
    prevPage() {
        if (this.pages.length > 1) {
            this.pages[this.pageIndex].onExit();
            this.pageIndex = (this.pageIndex + this.pages.length - 1) % this.pages.length;
            this.pages[this.pageIndex].onEnter();
        }
    }
    goToPage(_name, _skipExit = false) {
        if (!_skipExit) {
            this.pages[this.pageIndex].onExit();
        }
        for (let i = 0; i < this.pages.length; i++) {
            if (this.pages[i].name == _name) {
                this.pageIndex = i;
            }
        }
        this.onEnter();
    }
}
class NavSystemEventLinkedPageGroup {
    constructor(_pageGroup, _event) {
        this.pageGroup = _pageGroup;
        this.popUpEvent = _event;
    }
}
class NavSystemElementContainer {
    constructor(_name, _htmlElemId, _element) {
        this.name = "";
        this.htmlElemId = "";
        this.isInitialized = false;
        this._updatingWithBudget = false;
        this.name = _name;
        this.htmlElemId = _htmlElemId;
        this.element = _element;
        if (_element) {
            _element.container = this;
        }
    }
    getDefaultMenu() {
        return this.defaultMenu;
    }
    init() {
    }
    checkInit() {
        if (this.element) {
            if (this.element.isReady()) {
                if (!this.element.isInitialized) {
                    this.element.container = this;
                    this.element.setGPS(this.gps);
                    this.element.init(this.gps.getChildById(this.htmlElemId));
                    this.element.isInitialized = true;
                }
            } else {
                return false;
            }
        }
        if (!this.isInitialized) {
            this.init();
            this.isInitialized = true;
        }
        return this.isInitialized;
    }
    onEnter() {
        if (!this.checkInit()) {
            return;
        }
        if (this.element) {
            this.element.onEnter();
        }
    }
    onUpdate(_deltaTime) {
        if (!this._updatingWithBudget) {
            if (!this.checkInit()) {
                return;
            }
            if (this.element) {
                this.element.onUpdate(_deltaTime);
            }
        }
    }
    onUpdateSpecificItem(_deltaTime, _itemId) {
        if (!this.checkInit()) {
            return;
        }
        if (_itemId == 0) {
            this._updatingWithBudget = true;
            {
                this.onUpdate(_deltaTime);
            }
            this._updatingWithBudget = false;
        }
        if (this.element) {
            return this.element.onUpdateSpecificItem(_deltaTime, _itemId);
        }
        return false;
    }
    onExit() {
        if (this.element) {
            this.element.onExit();
        }
    }
    onEvent(_event) {
        if (this.element) {
            this.element.onEvent(_event);
        }
    }
    onSoundEnd(_eventId) {
        if (this.element) {
            this.element.onSoundEnd(_eventId);
        }
    }
    onShutDown() {
        if (this.element) {
            this.element.onShutDown();
        }
    }
    onPowerOn() {
        if (this.element) {
            this.element.onPowerOn();
        }
    }
    setGPS(_gps) {
        this.gps = _gps;
        if (this.element) {
            this.element.setGPS(_gps);
        }
    }
    getElementOfType(c) {
        if (this.element) {
            return this.element.getElementOfType(c);
        }
        return null;
    }
}
class NavSystemEventLinkedPopUpWindow extends NavSystemElementContainer {
    constructor(_name, _htmlElemId, _element, _popUpEvent) {
        super(_name, _htmlElemId, _element);
        this.popUpEvent = _popUpEvent;
    }
}
class SoftKeyHandler {
}
class NavSystemPage extends NavSystemElementContainer {
    constructor() {
        super(...arguments);
        this.softKeys = new SoftKeysMenu();
    }
    getSoftKeyMenu() {
        return this.softKeys;
    }
}
class Updatable {
}
class NavSystemElement extends Updatable {
    constructor() {
        super(...arguments);
        this.isInitialized = false;
        this.defaultSelectables = [];
        this._alwaysUpdate = false;
    }
    set alwaysUpdate(_val) {
        this._alwaysUpdate = _val;
        if (this.gps) {
            this.gps.alwaysUpdate(this, _val);
        }
    }
    isReady() {
        return true;
    }
    onSoundEnd(_eventId) {
    }
    onShutDown() {
    }
    onPowerOn() {
    }
    onUpdateSpecificItem(_deltaTime, _itemId) {
        if (_itemId == 0) {
            this.onUpdate(_deltaTime);
        }
        return false;
    }
    getDefaultSelectables() {
        return this.defaultSelectables;
    }
    setGPS(_gps) {
        if (this.gps && !_gps && this._alwaysUpdate) {
            this.gps.alwaysUpdate(this, false);
        }
        this.gps = _gps;
        if (this.gps) {
            this.gps.alwaysUpdate(this, this._alwaysUpdate);
        }
    }
    getElementOfType(c) {
        if (this instanceof c) {
            return this;
        } else {
            return null;
        }
    }
    redraw() {
    }
}
class NavSystemIFrameElement extends NavSystemElement {
    constructor(_frameName) {
        super();
        this.iFrame = this.gps.getChildById(_frameName);
    }
    isReady() {
        if (this.iFrame) {
            this.canvas = this.iFrame.contentWindow;
            if (this.canvas) {
                const readyToSet = this.canvas["readyToSet"];
                if (readyToSet) {
                    return true;
                }
            }
        }
        return false;
    }
}
class NavSystemElementGroup extends NavSystemElement {
    constructor(_elements) {
        super();
        this._updatingWithBudget = false;
        this.elements = _elements;
    }
    isReady() {
        for (let i = 0; i < this.elements.length; i++) {
            if (!this.elements[i].isReady()) {
                return false;
            }
        }
        return true;
    }
    init(_root) {
        this.defaultSelectables = [];
        for (let i = 0; i < this.elements.length; i++) {
            if (!this.elements[i].isInitialized) {
                this.elements[i].container = this.container;
                this.elements[i].setGPS(this.gps);
                this.elements[i].init(_root);
                this.elements[i].isInitialized = true;
                this.defaultSelectables.concat(this.elements[i].getDefaultSelectables());
            }
        }
    }
    onEnter() {
        for (let i = 0; i < this.elements.length; i++) {
            this.elements[i].onEnter();
        }
    }
    onUpdate(_deltaTime) {
        if (!this._updatingWithBudget) {
            for (let i = 0; i < this.elements.length; i++) {
                this.elements[i].onUpdate(_deltaTime);
            }
        }
    }
    onUpdateSpecificItem(_deltaTime, _itemId) {
        if (_itemId == 0) {
            this._updatingWithBudget = true;
            {
                this.onUpdate(_deltaTime);
            }
            this._updatingWithBudget = false;
        }
        if (_itemId < this.elements.length) {
            this.elements[_itemId].onUpdate(_deltaTime);
            if (_itemId + 1 < this.elements.length) {
                return true;
            }
        }
        return false;
    }
    onExit() {
        for (let i = 0; i < this.elements.length; i++) {
            this.elements[i].onExit();
        }
    }
    onEvent(_event) {
        for (let i = 0; i < this.elements.length; i++) {
            this.elements[i].onEvent(_event);
        }
    }
    onSoundEnd(_eventId) {
        for (let i = 0; i < this.elements.length; i++) {
            this.elements[i].onSoundEnd(_eventId);
        }
    }
    onShutDown() {
        for (let i = 0; i < this.elements.length; i++) {
            this.elements[i].onShutDown();
        }
    }
    onPowerOn() {
        for (let i = 0; i < this.elements.length; i++) {
            this.elements[i].onPowerOn();
        }
    }
    getDefaultSelectables() {
        this.defaultSelectables = [];
        for (let i = 0; i < this.elements.length; i++) {
            this.defaultSelectables.concat(this.elements[i].getDefaultSelectables());
        }
        return this.defaultSelectables;
    }
    setGPS(_gps) {
        this.gps = _gps;
        for (let i = 0; i < this.elements.length; i++) {
            this.elements[i].setGPS(_gps);
        }
    }
    getElementOfType(c) {
        for (let i = 0; i < this.elements.length; i++) {
            const elem = this.elements[i].getElementOfType(c);
            if (elem) {
                return elem;
            }
        }
        return null;
    }
    addElement(elem) {
        this.elements.push(elem);
    }
}
class NavSystemElementSelector extends NavSystemElement {
    constructor() {
        super(...arguments);
        this.elements = [];
        this.index = 0;
    }
    isReady() {
        for (let i = 0; i < this.elements.length; i++) {
            if (!this.elements[i].isReady()) {
                return false;
            }
        }
        return true;
    }
    init(_root) {
        for (let i = 0; i < this.elements.length; i++) {
            if (!this.elements[i].isInitialized) {
                this.elements[i].container = this.container;
                this.elements[i].setGPS(this.gps);
                this.elements[i].init(_root);
                this.elements[i].isInitialized = true;
            }
        }
    }
    onEnter() {
        this.elements[this.index].onEnter();
    }
    onUpdate(_deltaTime) {
        this.elements[this.index].onUpdate(_deltaTime);
    }
    onExit() {
        this.elements[this.index].onExit();
    }
    onEvent(_event) {
        this.elements[this.index].onEvent(_event);
    }
    onSoundEnd(_eventId) {
        for (let i = 0; i < this.elements.length; i++) {
            this.elements[i].onSoundEnd(_eventId);
        }
    }
    onShutDown() {
        for (let i = 0; i < this.elements.length; i++) {
            this.elements[i].onShutDown();
        }
    }
    onPowerOn() {
        for (let i = 0; i < this.elements.length; i++) {
            this.elements[i].onPowerOn();
        }
    }
    selectElement(_name) {
        for (let i = 0; i < this.elements.length; i++) {
            if (this.elements[i].name == _name) {
                this.index = i;
            }
        }
    }
    addElement(_elem) {
        this.elements.push(_elem);
    }
    getDefaultSelectables() {
        return this.elements[this.index].getDefaultSelectables();
    }
    setGPS(_gps) {
        this.gps = _gps;
        for (let i = 0; i < this.elements.length; i++) {
            this.elements[i].setGPS(_gps);
        }
    }
    getElementOfType(c) {
        for (let i = 0; i < this.elements.length; i++) {
            const elem = this.elements[i].getElementOfType(c);
            if (elem) {
                return elem;
            }
        }
        return null;
    }
    switchToIndex(_index) {
        if (this.index < this.elements.length) {
            this.elements[this.index].onExit();
        }
        if (_index < this.elements.length) {
            this.index = _index;
            this.elements[this.index].onEnter();
        }
    }
}
class SoftKeyElement {
    constructor(_name = "", _callback = null, _stateCB = null) {
        this.callback = _callback;
        this.name = _name;
        this.state = "None";
        this.stateCallback = _stateCB;
        if (!_callback) {
            this.state = "Greyed";
        }
    }
}
class SoftKeysMenu {
}
class NavSystemEventAlias {
    constructor(_from, _to) {
        this.source = _from;
        this.output = _to;
    }
}
class CDIElement extends NavSystemElement {
    init(_root) {
        this.cdiCursor = this.gps.getChildById("CDICursor");
        this.toFrom = this.gps.getChildById("ToFrom");
    }
    onEnter() {
    }
    onUpdate(_deltaTime) {
        const CTD = SimVar.GetSimVarValue("GPS WP CROSS TRK", "nautical mile");
        this.cdiCursor.setAttribute("style", "left:" + ((CTD <= -1 ? -1 : CTD >= 1 ? 1 : CTD) * 50 + 50) + "%");
    }
    onExit() {
    }
    onEvent(_event) {
    }
}
var EMapDisplayMode;
(function (EMapDisplayMode) {
    EMapDisplayMode[EMapDisplayMode["GPS"] = 0] = "GPS";
    EMapDisplayMode[EMapDisplayMode["RADAR"] = 1] = "RADAR";
})(EMapDisplayMode || (EMapDisplayMode = {}));
var ERadarMode;
(function (ERadarMode) {
    ERadarMode[ERadarMode["HORIZON"] = 0] = "HORIZON";
    ERadarMode[ERadarMode["VERTICAL"] = 1] = "VERTICAL";
})(ERadarMode || (ERadarMode = {}));
class MapInstrumentElement extends NavSystemElement {
    constructor() {
        super();
        this.displayMode = EMapDisplayMode.GPS;
        this.nexradOn = false;
        this.radarMode = ERadarMode.HORIZON;
    }
    init(root) {
        this.instrument = root.querySelector("map-instrument");
        if (this.instrument) {
            TemplateElement.callNoBinding(this.instrument, () => {
                this.onTemplateLoaded();
            });
        }
    }
    onTemplateLoaded() {
        this.instrument.init(this.gps);
        this.instrumentLoaded = true;
        // This makes the black parts of the bing map blend perfectly with the backlight in weather and terrain modes
        this.instrument.bingMap.m_imgElement.style.mixBlendMode = 'lighten';
    }
    setGPS(_gps) {
        super.setGPS(_gps);
        if (this.instrument) {
            this.instrument.init(this.gps);
        }
    }
    onEnter() {
    }
    onUpdate(_deltaTime) {
        if (this.instrumentLoaded) {
            this.instrument.update(_deltaTime);
            if (this.weatherTexts) {
                const range = this.instrument.getWeatherRange();
                const ratio = 1.0 / this.weatherTexts.length;
                for (let i = 0; i < this.weatherTexts.length; i++) {
                    this.weatherTexts[i].textContent = fastToFixed(range * ratio * (i + 1), 2) + "NM";
                }
            }
        }
    }
    onExit() {
    }
    onEvent(_event) {
        if (this.instrument) {
            this.instrument.onEvent(_event);
        }
    }
    toggleDisplayMode() {
        if (this.displayMode == EMapDisplayMode.GPS) {
            this.gps.getCurrentPage().name = "WEATHER RADAR";
            this.displayMode = EMapDisplayMode.RADAR;
        } else {
            this.gps.getCurrentPage().name = "NAVIGATION MAP";
            this.displayMode = EMapDisplayMode.GPS;
        }
        this.updateWeather();
    }
    setDisplayMode(_mode) {
        this.displayMode = _mode;
        if (_mode == EMapDisplayMode.GPS) {
            this.gps.getCurrentPage().name = "NAVIGATION MAP";
        } else {
            this.gps.getCurrentPage().name = "WEATHER RADAR";
        }
        this.updateWeather();
    }
    getDisplayMode() {
        return this.displayMode;
    }
    toggleIsolines() {
        if (this.instrument) {
            if (this.instrument.getIsolines() == true) {
                this.instrument.showIsolines(false);
            } else {
                this.instrument.showIsolines(true);
            }
        }
    }
    getIsolines() {
        return this.instrument.getIsolines();
    }
    toggleNexrad() {
        this.nexradOn = !this.nexradOn;
        this.updateWeather();
    }
    getNexrad() {
        return this.nexradOn;
    }
    setRadar(_mode) {
        this.radarMode = _mode;
        this.updateWeather();
    }
    getRadarMode() {
        return this.radarMode;
    }
    updateWeather() {
        if (this.instrument) {
            if (this.displayMode == EMapDisplayMode.GPS) {
                if (this.nexradOn) {
                    this.setWeather(EWeatherRadar.TOPVIEW);
                } else {
                    this.setWeather(EWeatherRadar.OFF);
                }
            } else {
                if (this.radarMode == ERadarMode.HORIZON) {
                    this.setWeather(EWeatherRadar.HORIZONTAL);
                } else {
                    this.setWeather(EWeatherRadar.VERTICAL);
                }
            }
        }
    }
    setWeather(_mode) {
        this.instrument.showWeather(_mode);
        const svgRoot = this.instrument.weatherSVG;
        if (svgRoot) {
            Utils.RemoveAllChildren(svgRoot);
            this.weatherTexts = null;
            if (_mode == EWeatherRadar.HORIZONTAL || _mode == EWeatherRadar.VERTICAL) {
                const circleRadius = 575;
                const dashNbRect = 10;
                const dashWidth = 8;
                const dashHeight = 6;
                if (_mode == EWeatherRadar.HORIZONTAL) {
                    this.instrument.setBingMapStyle("10.3%", "-13.3%", "127%", "157%");
                    var coneAngle = 90;
                    svgRoot.setAttribute("viewBox", "0 0 400 400");
                    var trsGroup = document.createElementNS(Avionics.SVG.NS, "g");
                    trsGroup.setAttribute("transform", "translate(-125, 29) scale(1.63)");
                    svgRoot.appendChild(trsGroup);
                    const viewBox = document.createElementNS(Avionics.SVG.NS, "svg");
                    viewBox.setAttribute("viewBox", "-600 -600 1200 1200");
                    trsGroup.appendChild(viewBox);
                    var circleGroup = document.createElementNS(Avionics.SVG.NS, "g");
                    circleGroup.setAttribute("id", "Circles");
                    viewBox.appendChild(circleGroup);
                    {
                        const rads = [0.25, 0.50, 0.75, 1.0];
                        for (let r = 0; r < rads.length; r++) {
                            const rad = circleRadius * rads[r];
                            let startDegrees = -coneAngle * 0.5;
                            const endDegrees = coneAngle * 0.5;
                            while (Math.floor(startDegrees) <= endDegrees) {
                                const line = document.createElementNS(Avionics.SVG.NS, "rect");
                                const degree = (180 + startDegrees + 0.5);
                                line.setAttribute("x", "0");
                                line.setAttribute("y", rad.toString());
                                line.setAttribute("width", dashWidth.toString());
                                line.setAttribute("height", dashHeight.toString());
                                line.setAttribute("transform", "rotate(" + degree + " 0 0)");
                                line.setAttribute("fill", "white");
                                circleGroup.appendChild(line);
                                startDegrees += coneAngle / dashNbRect;
                            }
                        }
                    }
                    var lineGroup = document.createElementNS(Avionics.SVG.NS, "g");
                    lineGroup.setAttribute("id", "Lines");
                    viewBox.appendChild(lineGroup);
                    {
                        var coneStart = 180 - coneAngle * 0.5;
                        var coneStartLine = document.createElementNS(Avionics.SVG.NS, "line");
                        coneStartLine.setAttribute("x1", "0");
                        coneStartLine.setAttribute("y1", "0");
                        coneStartLine.setAttribute("x2", "0");
                        coneStartLine.setAttribute("y2", circleRadius.toString());
                        coneStartLine.setAttribute("transform", "rotate(" + coneStart + " 0 0)");
                        coneStartLine.setAttribute("stroke", "white");
                        coneStartLine.setAttribute("stroke-width", "3");
                        lineGroup.appendChild(coneStartLine);
                        var coneEnd = 180 + coneAngle * 0.5;
                        var coneEndLine = document.createElementNS(Avionics.SVG.NS, "line");
                        coneEndLine.setAttribute("x1", "0");
                        coneEndLine.setAttribute("y1", "0");
                        coneEndLine.setAttribute("x2", "0");
                        coneEndLine.setAttribute("y2", circleRadius.toString());
                        coneEndLine.setAttribute("transform", "rotate(" + coneEnd + " 0 0)");
                        coneEndLine.setAttribute("stroke", "white");
                        coneEndLine.setAttribute("stroke-width", "3");
                        lineGroup.appendChild(coneEndLine);
                    }
                    var textGroup = document.createElementNS(Avionics.SVG.NS, "g");
                    textGroup.setAttribute("id", "Texts");
                    viewBox.appendChild(textGroup);
                    {
                        this.weatherTexts = [];
                        var text = document.createElementNS(Avionics.SVG.NS, "text");
                        text.setAttribute("x", "100");
                        text.setAttribute("y", "-85");
                        text.setAttribute("fill", "white");
                        text.setAttribute("font-size", "20");
                        textGroup.appendChild(text);
                        this.weatherTexts.push(text);
                        var text = document.createElementNS(Avionics.SVG.NS, "text");
                        text.setAttribute("x", "200");
                        text.setAttribute("y", "-185");
                        text.setAttribute("fill", "white");
                        text.setAttribute("font-size", "20");
                        textGroup.appendChild(text);
                        this.weatherTexts.push(text);
                        var text = document.createElementNS(Avionics.SVG.NS, "text");
                        text.setAttribute("x", "300");
                        text.setAttribute("y", "-285");
                        text.setAttribute("fill", "white");
                        text.setAttribute("font-size", "20");
                        textGroup.appendChild(text);
                        this.weatherTexts.push(text);
                        var text = document.createElementNS(Avionics.SVG.NS, "text");
                        text.setAttribute("x", "400");
                        text.setAttribute("y", "-385");
                        text.setAttribute("fill", "white");
                        text.setAttribute("font-size", "20");
                        textGroup.appendChild(text);
                        this.weatherTexts.push(text);
                    }
                } else if (_mode == EWeatherRadar.VERTICAL) {
                    this.instrument.setBingMapStyle("-75%", "-88%", "201%", "250%");
                    var coneAngle = 51.43;
                    svgRoot.setAttribute("viewBox", "0 0 400 400");
                    var trsGroup = document.createElementNS(Avionics.SVG.NS, "g");
                    trsGroup.setAttribute("transform", "translate(402, -190) scale(1.95) rotate(90)");
                    svgRoot.appendChild(trsGroup);
                    const viewBox = document.createElementNS(Avionics.SVG.NS, "svg");
                    viewBox.setAttribute("viewBox", "-600 -600 1200 1200");
                    trsGroup.appendChild(viewBox);
                    var circleGroup = document.createElementNS(Avionics.SVG.NS, "g");
                    circleGroup.setAttribute("id", "Circles");
                    viewBox.appendChild(circleGroup);
                    {
                        const rads = [0.25, 0.50, 0.75, 1.0];
                        for (let r = 0; r < rads.length; r++) {
                            const rad = circleRadius * rads[r];
                            let startDegrees = -coneAngle * 0.5;
                            const endDegrees = coneAngle * 0.5;
                            while (Math.floor(startDegrees) <= endDegrees) {
                                const line = document.createElementNS(Avionics.SVG.NS, "rect");
                                const degree = (180 + startDegrees + 0.5);
                                line.setAttribute("x", "0");
                                line.setAttribute("y", rad.toString());
                                line.setAttribute("width", dashWidth.toString());
                                line.setAttribute("height", dashHeight.toString());
                                line.setAttribute("transform", "rotate(" + degree + " 0 0)");
                                line.setAttribute("fill", "white");
                                circleGroup.appendChild(line);
                                startDegrees += coneAngle / dashNbRect;
                            }
                        }
                    }
                    const limitGroup = document.createElementNS(Avionics.SVG.NS, "g");
                    limitGroup.setAttribute("id", "Limits");
                    viewBox.appendChild(limitGroup);
                    {
                        const endPosY = circleRadius + 50;
                        let posX = -130;
                        let posY = 50;
                        while (posY <= endPosY) {
                            const line = document.createElementNS(Avionics.SVG.NS, "rect");
                            line.setAttribute("x", posX.toString());
                            line.setAttribute("y", (-posY).toString());
                            line.setAttribute("width", dashHeight.toString());
                            line.setAttribute("height", dashWidth.toString());
                            line.setAttribute("fill", "white");
                            limitGroup.appendChild(line);
                            posY += dashWidth * 2;
                        }
                        posX = 130;
                        posY = 50;
                        while (posY <= endPosY) {
                            const line = document.createElementNS(Avionics.SVG.NS, "rect");
                            line.setAttribute("x", posX.toString());
                            line.setAttribute("y", (-posY).toString());
                            line.setAttribute("width", dashHeight.toString());
                            line.setAttribute("height", dashWidth.toString());
                            line.setAttribute("fill", "white");
                            limitGroup.appendChild(line);
                            posY += dashWidth * 2;
                        }
                    }
                    var lineGroup = document.createElementNS(Avionics.SVG.NS, "g");
                    lineGroup.setAttribute("id", "Lines");
                    viewBox.appendChild(lineGroup);
                    {
                        var coneStart = 180 - coneAngle * 0.5;
                        var coneStartLine = document.createElementNS(Avionics.SVG.NS, "line");
                        coneStartLine.setAttribute("x1", "0");
                        coneStartLine.setAttribute("y1", "0");
                        coneStartLine.setAttribute("x2", "0");
                        coneStartLine.setAttribute("y2", circleRadius.toString());
                        coneStartLine.setAttribute("transform", "rotate(" + coneStart + " 0 0)");
                        coneStartLine.setAttribute("stroke", "white");
                        coneStartLine.setAttribute("stroke-width", "3");
                        lineGroup.appendChild(coneStartLine);
                        var coneEnd = 180 + coneAngle * 0.5;
                        var coneEndLine = document.createElementNS(Avionics.SVG.NS, "line");
                        coneEndLine.setAttribute("x1", "0");
                        coneEndLine.setAttribute("y1", "0");
                        coneEndLine.setAttribute("x2", "0");
                        coneEndLine.setAttribute("y2", circleRadius.toString());
                        coneEndLine.setAttribute("transform", "rotate(" + coneEnd + " 0 0)");
                        coneEndLine.setAttribute("stroke", "white");
                        coneEndLine.setAttribute("stroke-width", "3");
                        lineGroup.appendChild(coneEndLine);
                    }
                    var textGroup = document.createElementNS(Avionics.SVG.NS, "g");
                    textGroup.setAttribute("id", "Texts");
                    viewBox.appendChild(textGroup);
                    {
                        var text = document.createElementNS(Avionics.SVG.NS, "text");
                        text.textContent = "+60000FT";
                        text.setAttribute("x", "50");
                        text.setAttribute("y", "-150");
                        text.setAttribute("fill", "white");
                        text.setAttribute("font-size", "20");
                        text.setAttribute("transform", "rotate(-90)");
                        textGroup.appendChild(text);
                        var text = document.createElementNS(Avionics.SVG.NS, "text");
                        text.textContent = "-60000FT";
                        text.setAttribute("x", "50");
                        text.setAttribute("y", "160");
                        text.setAttribute("fill", "white");
                        text.setAttribute("font-size", "20");
                        text.setAttribute("transform", "rotate(-90)");
                        textGroup.appendChild(text);
                        this.weatherTexts = [];
                        var text = document.createElementNS(Avionics.SVG.NS, "text");
                        text.setAttribute("x", "85");
                        text.setAttribute("y", "85");
                        text.setAttribute("fill", "white");
                        text.setAttribute("font-size", "18");
                        text.setAttribute("transform", "rotate(-90)");
                        textGroup.appendChild(text);
                        this.weatherTexts.push(text);
                        var text = document.createElementNS(Avionics.SVG.NS, "text");
                        text.setAttribute("x", "215");
                        text.setAttribute("y", "160");
                        text.setAttribute("fill", "white");
                        text.setAttribute("font-size", "18");
                        text.setAttribute("transform", "rotate(-90)");
                        textGroup.appendChild(text);
                        this.weatherTexts.push(text);
                        var text = document.createElementNS(Avionics.SVG.NS, "text");
                        text.setAttribute("x", "345");
                        text.setAttribute("y", "220");
                        text.setAttribute("fill", "white");
                        text.setAttribute("font-size", "18");
                        text.setAttribute("transform", "rotate(-90)");
                        textGroup.appendChild(text);
                        this.weatherTexts.push(text);
                        var text = document.createElementNS(Avionics.SVG.NS, "text");
                        text.setAttribute("x", "475");
                        text.setAttribute("y", "280");
                        text.setAttribute("fill", "white");
                        text.setAttribute("font-size", "18");
                        text.setAttribute("transform", "rotate(-90)");
                        textGroup.appendChild(text);
                        this.weatherTexts.push(text);
                    }
                }
                const legendGroup = document.createElementNS(Avionics.SVG.NS, "g");
                legendGroup.setAttribute("id", "legendGroup");
                svgRoot.appendChild(legendGroup);
                {
                    const x = -5;
                    const y = 325;
                    const w = 70;
                    const h = 125;
                    const titleHeight = 20;
                    const scaleOffsetX = 5;
                    const scaleOffsetY = 5;
                    const scaleWidth = 13;
                    const scaleHeight = 24;
                    const left = x - w * 0.5;
                    const top = y - h * 0.5;
                    let rect = document.createElementNS(Avionics.SVG.NS, "rect");
                    rect.setAttribute("x", left.toString());
                    rect.setAttribute("y", top.toString());
                    rect.setAttribute("width", w.toString());
                    rect.setAttribute("height", h.toString());
                    rect.setAttribute("stroke", "white");
                    rect.setAttribute("stroke-width", "2");
                    rect.setAttribute("stroke-opacity", "1");
                    legendGroup.appendChild(rect);
                    rect = document.createElementNS(Avionics.SVG.NS, "rect");
                    rect.setAttribute("x", left.toString());
                    rect.setAttribute("y", top.toString());
                    rect.setAttribute("width", w.toString());
                    rect.setAttribute("height", titleHeight.toString());
                    rect.setAttribute("stroke", "white");
                    rect.setAttribute("stroke-width", "2");
                    rect.setAttribute("stroke-opacity", "1");
                    legendGroup.appendChild(rect);
                    var text = document.createElementNS(Avionics.SVG.NS, "text");
                    text.textContent = "SCALE";
                    text.setAttribute("x", x.toString());
                    text.setAttribute("y", (top + titleHeight * 0.5).toString());
                    text.setAttribute("fill", "white");
                    text.setAttribute("font-size", "11");
                    text.setAttribute("text-anchor", "middle");
                    legendGroup.appendChild(text);
                    let scaleIndex = 0;
                    rect = document.createElementNS(Avionics.SVG.NS, "rect");
                    rect.setAttribute("x", (left + scaleOffsetX).toString());
                    rect.setAttribute("y", (top + titleHeight + scaleOffsetY + scaleIndex * scaleHeight).toString());
                    rect.setAttribute("width", scaleWidth.toString());
                    rect.setAttribute("height", scaleHeight.toString());
                    rect.setAttribute("fill", "red");
                    rect.setAttribute("stroke", "white");
                    rect.setAttribute("stroke-width", "2");
                    rect.setAttribute("stroke-opacity", "1");
                    legendGroup.appendChild(rect);
                    text = document.createElementNS(Avionics.SVG.NS, "text");
                    text.textContent = "HEAVY";
                    text.setAttribute("x", (left + scaleOffsetX + scaleWidth + 5).toString());
                    text.setAttribute("y", (top + titleHeight + scaleOffsetY + scaleIndex * scaleHeight + scaleHeight * 0.5).toString());
                    text.setAttribute("fill", "white");
                    text.setAttribute("font-size", "11");
                    legendGroup.appendChild(text);
                    scaleIndex++;
                    rect = document.createElementNS(Avionics.SVG.NS, "rect");
                    rect.setAttribute("x", (left + scaleOffsetX).toString());
                    rect.setAttribute("y", (top + titleHeight + scaleOffsetY + scaleIndex * scaleHeight).toString());
                    rect.setAttribute("width", scaleWidth.toString());
                    rect.setAttribute("height", scaleHeight.toString());
                    rect.setAttribute("fill", "yellow");
                    rect.setAttribute("stroke", "white");
                    rect.setAttribute("stroke-width", "2");
                    rect.setAttribute("stroke-opacity", "1");
                    legendGroup.appendChild(rect);
                    scaleIndex++;
                    rect = document.createElementNS(Avionics.SVG.NS, "rect");
                    rect.setAttribute("x", (left + scaleOffsetX).toString());
                    rect.setAttribute("y", (top + titleHeight + scaleOffsetY + scaleIndex * scaleHeight).toString());
                    rect.setAttribute("width", scaleWidth.toString());
                    rect.setAttribute("height", scaleHeight.toString());
                    rect.setAttribute("fill", "green");
                    rect.setAttribute("stroke", "white");
                    rect.setAttribute("stroke-width", "2");
                    rect.setAttribute("stroke-opacity", "1");
                    legendGroup.appendChild(rect);
                    text = document.createElementNS(Avionics.SVG.NS, "text");
                    text.textContent = "LIGHT";
                    text.setAttribute("x", (left + scaleOffsetX + scaleWidth + 5).toString());
                    text.setAttribute("y", (top + titleHeight + scaleOffsetY + scaleIndex * scaleHeight + scaleHeight * 0.5).toString());
                    text.setAttribute("fill", "white");
                    text.setAttribute("font-size", "11");
                    legendGroup.appendChild(text);
                    scaleIndex++;
                    rect = document.createElementNS(Avionics.SVG.NS, "rect");
                    rect.setAttribute("x", (left + scaleOffsetX).toString());
                    rect.setAttribute("y", (top + titleHeight + scaleOffsetY + scaleIndex * scaleHeight).toString());
                    rect.setAttribute("width", scaleWidth.toString());
                    rect.setAttribute("height", scaleHeight.toString());
                    rect.setAttribute("fill", "black");
                    rect.setAttribute("stroke", "white");
                    rect.setAttribute("stroke-width", "2");
                    rect.setAttribute("stroke-opacity", "1");
                    legendGroup.appendChild(rect);
                }
            }
        }
    }
}
class SoftKeyHtmlElement {
    constructor(_elem) {
        this.Element = _elem;
        this.Value = "";
    }
    fillFromElement(_elem) {
        const val = _elem.name;
        if (this.Value != val) {
            this.Element.innerHTML = val;
            this.Value = val;
        }
        if (_elem.stateCallback) {
            _elem.state = _elem.stateCallback();
        } else if (!_elem.callback) {
            _elem.state = "Greyed";
        }
        if (_elem.state) {
            Avionics.Utils.diffAndSetAttribute(this.Element, "state", _elem.state);
        }
    }
}
class SoftKeys extends NavSystemElement {
    constructor(_softKeyHTMLClass = SoftKeyHtmlElement) {
        super();
        this.softKeys = [];
        this.softKeyHTMLClass = _softKeyHTMLClass;
    }
    init(root) {
        for (let i = 1; i <= 12; i++) {
            const name = "Key" + i.toString();
            const child = this.gps.getChildById(name);
            if (child) {
                const e = new this.softKeyHTMLClass(child);
                this.softKeys.push(e);
            }
        }
        this.isInitialized = true;
    }
    onEnter() {
    }
    onUpdate(_deltaTime) {
        const currentPage = this.gps.getCurrentPage();
        if (currentPage) {
            this.currentMenu = currentPage.getSoftKeyMenu();
            if (this.currentMenu && this.currentMenu.elements && this.currentMenu.elements.length > 0) {
                for (let i = 0; i < this.currentMenu.elements.length; i++) {
                    this.softKeys[i].fillFromElement(this.currentMenu.elements[i]);
                }
            }
        }
    }
    onExit() {
    }
    onEvent(_event) {
        switch (_event) {
            case "SOFTKEYS_1":
                this.activeSoftKey(0);
                break;
            case "SOFTKEYS_2":
                this.activeSoftKey(1);
                break;
            case "SOFTKEYS_3":
                this.activeSoftKey(2);
                break;
            case "SOFTKEYS_4":
                this.activeSoftKey(3);
                break;
            case "SOFTKEYS_5":
                this.activeSoftKey(4);
                break;
            case "SOFTKEYS_6":
                this.activeSoftKey(5);
                break;
            case "SOFTKEYS_7":
                this.activeSoftKey(6);
                break;
            case "SOFTKEYS_8":
                this.activeSoftKey(7);
                break;
            case "SOFTKEYS_9":
                this.activeSoftKey(8);
                break;
            case "SOFTKEYS_10":
                this.activeSoftKey(9);
                break;
            case "SOFTKEYS_11":
                this.activeSoftKey(10);
                break;
            case "SOFTKEYS_12":
                this.activeSoftKey(11);
                break;
        }
    }
    activeSoftKey(_number) {
        if (this.currentMenu.elements[_number].callback) {
            this.currentMenu.elements[_number].callback();
        }
    }
}
var Annunciation_MessageType;
(function (Annunciation_MessageType) {
    Annunciation_MessageType[Annunciation_MessageType["WARNING"] = 0] = "WARNING";
    Annunciation_MessageType[Annunciation_MessageType["CAUTION"] = 1] = "CAUTION";
    Annunciation_MessageType[Annunciation_MessageType["ADVISORY"] = 2] = "ADVISORY";
    Annunciation_MessageType[Annunciation_MessageType["SAFEOP"] = 3] = "SAFEOP";
})(Annunciation_MessageType || (Annunciation_MessageType = {}));
;
class Annunciation_Message {
    constructor() {
        this.Visible = false;
        this.Acknowledged = false;
    }
}
;
class XMLCondition {
    constructor() {
        this.suffix = "";
    }
}
class Annunciation_Message_XML extends Annunciation_Message {
    constructor() {
        super();
        this.lastConditionIndex = -1;
        this.conditions = [];
        this.Handler = this.getHandlersValue.bind(this);
    }
    getHandlersValue() {
        for (let i = 0; i < this.conditions.length; i++) {
            if (this.conditions[i].logic.getValue() != 0) {
                if (i != this.lastConditionIndex) {
                    this.lastConditionIndex = i;
                    this.Text = this.baseText + (this.conditions[i].suffix ? this.conditions[i].suffix : "");
                    return false;
                }
                return true;
            }
        }
        return false;
    }
}
class Annunciation_Message_Timed extends Annunciation_Message {
}
;
class Annunciation_Message_Switch extends Annunciation_Message {
    get Text() {
        const index = this.Handler();
        if (index > 0) {
            return this.Texts[index - 1];
        } else {
            return "";
        }
    }
}
class Condition {
    constructor(_handler, _time = 0) {
        this.beginTime = 0;
        this.Handler = _handler;
        this.Time = _time;
    }
}
class Annunciator_Message_MultipleConditions extends Annunciation_Message {
    constructor() {
        super();
        this.Handler = this.getHandlersValue;
    }
    getHandlersValue() {
        let result = false;
        for (let i = 0; i < this.conditions.length; i++) {
            if (this.conditions[i].Handler()) {
                if (this.conditions[i].beginTime == 0) {
                    this.conditions[i].beginTime = Date.now() + 1000 * this.conditions[i].Time;
                } else if (this.conditions[i].beginTime <= Date.now()) {
                    result = true;
                }
            } else {
                this.conditions[i].beginTime = 0;
            }
        }
        return result;
    }
}
class Annunciations extends NavSystemElement {
    constructor() {
        super(...arguments);
        this.allMessages = [];
        this.alertLevel = 0;
        this.alert = false;
        this.needReload = true;
        this.rootElementName = "Annunciations";
    }
    init(root) {
        this.engineType = Simplane.getEngineType();
        if (this.rootElementName != "") {
            this.annunciations = this.gps.getChildById(this.rootElementName);
        }
        if (this.gps.xmlConfig) {
            const annunciationsRoot = this.gps.xmlConfig.getElementsByTagName("Annunciations");
            if (annunciationsRoot.length > 0) {
                const annunciations = annunciationsRoot[0].getElementsByTagName("Annunciation");
                for (let i = 0; i < annunciations.length; i++) {
                    this.addXmlMessage(annunciations[i]);
                }
            }
        }
    }
    onEnter() {
    }
    onExit() {
    }
    addMessage(_type, _text, _handler) {
        const msg = new Annunciation_Message();
        msg.Type = _type;
        msg.Text = _text;
        msg.Handler = _handler.bind(msg);
        this.allMessages.push(msg);
    }
    addXmlMessage(_element) {
        const msg = new Annunciation_Message_XML();
        switch (_element.getElementsByTagName("Type")[0].textContent) {
            case "Warning":
                msg.Type = Annunciation_MessageType.WARNING;
                break;
            case "Caution":
                msg.Type = Annunciation_MessageType.CAUTION;
                break;
            case "Advisory":
                msg.Type = Annunciation_MessageType.ADVISORY;
                break;
            case "SafeOp":
                msg.Type = Annunciation_MessageType.SAFEOP;
                break;
        }
        msg.baseText = _element.getElementsByTagName("Text")[0].textContent;
        const conditions = _element.getElementsByTagName("Condition");
        for (let i = 0; i < conditions.length; i++) {
            const condition = new XMLCondition();
            condition.logic = new CompositeLogicXMLElement(this.gps, conditions[i]);
            condition.suffix = conditions[i].getAttribute("Suffix");
            msg.conditions.push(condition);
        }
        this.allMessages.push(msg);
    }
    addMessageTimed(_type, _text, _handler, _time) {
        const msg = new Annunciation_Message_Timed();
        msg.Type = _type;
        msg.Text = _text;
        msg.Handler = _handler.bind(msg);
        msg.timeNeeded = _time;
        this.allMessages.push(msg);
    }
    addMessageSwitch(_type, _texts, _handler) {
        const msg = new Annunciation_Message_Switch();
        msg.Type = _type;
        msg.Texts = _texts;
        msg.Handler = _handler.bind(msg);
        this.allMessages.push(msg);
    }
    addMessageMultipleConditions(_type, _text, _conditions) {
        const msg = new Annunciator_Message_MultipleConditions();
        msg.Type = _type;
        msg.Text = _text;
        msg.conditions = _conditions;
        this.allMessages.push(msg);
    }
}
class Cabin_Annunciations extends Annunciations {
    constructor() {
        super(...arguments);
        this.displayWarning = [];
        this.displayCaution = [];
        this.displayAdvisory = [];
        //this.warningToneNameZ = new Name_Z("CRC");
        this.cautionToneNameZ = new Name_Z("improved_tone_caution");
        this.warningTone = false;
        this.firstAcknowledge = true;
        this.offStart = false;
    }
    init(root) {
        super.init(root);
        this.alwaysUpdate = true;
        this.isPlayingWarningTone = false;
        for (let i = 0; i < this.allMessages.length; i++) {
            const message = this.allMessages[i];
            let value = false;
            if (message.Handler) {
                value = message.Handler() != 0;
            }
            if (value != message.Visible) {
                this.needReload = true;
                message.Visible = value;
                message.Acknowledged = !this.offStart;
                if (value) {
                    switch (message.Type) {
                        case Annunciation_MessageType.WARNING:
                            this.displayWarning.push(message);
                            break;
                        case Annunciation_MessageType.CAUTION:
                            this.displayCaution.push(message);
                            break;
                        case Annunciation_MessageType.ADVISORY:
                            this.displayAdvisory.push(message);
                            break;
                    }
                }
            }
        }
    }
    onEnter() {
    }
    onUpdate(_deltaTime) {
        for (let i = 0; i < this.allMessages.length; i++) {
            const message = this.allMessages[i];
            let value = false;
            if (message.Handler) {
                value = message.Handler() != 0;
            }
            if (value != message.Visible) {
                this.needReload = true;
                message.Visible = value;
                message.Acknowledged = (this.gps.getTimeSinceStart() < 10000 && !this.offStart);
                if (value) {
                    switch (message.Type) {
                        case Annunciation_MessageType.WARNING:
                            this.displayWarning.push(message);
                            break;
                        case Annunciation_MessageType.CAUTION:
                            this.displayCaution.push(message);
                            if (!message.Acknowledged && !this.isPlayingWarningTone && this.gps.isPrimary) {
                                const res = this.gps.playInstrumentSound("improved_tone_caution");
                                if (res) {
                                    this.isPlayingWarningTone = true;
                                }
                            }
                            break;
                        case Annunciation_MessageType.ADVISORY:
                            this.displayAdvisory.push(message);
                            break;
                    }
                } else {
                    switch (message.Type) {
                        case Annunciation_MessageType.WARNING:
                            for (let i = 0; i < this.displayWarning.length; i++) {
                                if (this.displayWarning[i].Text == message.Text) {
                                    this.displayWarning.splice(i, 1);
                                    break;
                                }
                            }
                            break;
                        case Annunciation_MessageType.CAUTION:
                            for (let i = 0; i < this.displayCaution.length; i++) {
                                if (this.displayCaution[i].Text == message.Text) {
                                    this.displayCaution.splice(i, 1);
                                    break;
                                }
                            }
                            break;
                        case Annunciation_MessageType.ADVISORY:
                            for (let i = 0; i < this.displayAdvisory.length; i++) {
                                if (this.displayAdvisory[i].Text == message.Text) {
                                    this.displayAdvisory.splice(i, 1);
                                    break;
                                }
                            }
                            break;
                    }
                }
            }
        }
        if (this.annunciations) {
            this.annunciations.setAttribute("state", this.gps.blinkGetState(800, 400) ? "Blink" : "None");
        }
        if (this.needReload) {
            let warningOn = 0;
            let cautionOn = 0;
            let messages = "";
            for (let i = this.displayWarning.length - 1; i >= 0; i--) {
                messages += '<div class="Warning';
                if (!this.displayWarning[i].Acknowledged) {
                    messages += '_Blink';
                    warningOn = 1;
                }
                messages += '">' + this.displayWarning[i].Text + "</div>";
            }
            for (let i = this.displayCaution.length - 1; i >= 0; i--) {
                messages += '<div class="Caution';
                if (!this.displayCaution[i].Acknowledged) {
                    messages += '_Blink';
                    cautionOn = 1;
                }
                messages += '">' + this.displayCaution[i].Text + "</div>";
            }
            for (let i = this.displayAdvisory.length - 1; i >= 0; i--) {
                messages += '<div class="Advisory">' + this.displayAdvisory[i].Text + "</div>";
            }
            this.warningTone = warningOn > 0;
            if (this.gps.isPrimary) {
                SimVar.SetSimVarValue("L:Generic_Master_Warning_Active", "Bool", warningOn);
                SimVar.SetSimVarValue("L:Generic_Master_Caution_Active", "Bool", cautionOn);
            }
            if (this.annunciations) {
                this.annunciations.innerHTML = messages;
            }
            this.needReload = false;
        }
        /*if (this.warningTone && !this.isPlayingWarningTone && this.gps.isPrimary) {
            this.isPlayingWarningTone = true;
        }*/
    }
    onEvent(_event) {
        switch (_event) {
            case "Master_Caution_Push":
                for (let i = 0; i < this.allMessages.length; i++) {
                    if (this.allMessages[i].Type == Annunciation_MessageType.CAUTION && this.allMessages[i].Visible) {
                        this.allMessages[i].Acknowledged = true;
                        this.needReload = true;
                    }
                }
                break;
            case "Master_Warning_Push":
                for (let i = 0; i < this.allMessages.length; i++) {
                    if (this.allMessages[i].Type == Annunciation_MessageType.WARNING && this.allMessages[i].Visible) {
                        this.allMessages[i].Acknowledged = true;
                        this.needReload = true;
                    }
                }
                if (this.needReload && this.firstAcknowledge && this.gps.isPrimary) {
                    const res = this.gps.playInstrumentSound("aural_warning_ok");
                    if (res) {
                        this.firstAcknowledge = false;
                    }
                }
                break;
        }
    }
    onSoundEnd(_eventId) {
        if (Name_Z.compare(_eventId, this.warningToneNameZ) || Name_Z.compare(_eventId, this.cautionToneNameZ)) {
            this.isPlayingWarningTone = false;
        }
    }
    onShutDown() {
        for (let i = 0; i < this.allMessages.length; i++) {
            this.allMessages[i].Acknowledged = false;
            this.allMessages[i].Visible = false;
        }
        this.displayCaution = [];
        this.displayWarning = [];
        this.displayAdvisory = [];
        if (!this.gps || this.gps.isPrimary) {
            SimVar.SetSimVarValue("L:Generic_Master_Warning_Active", "Bool", 0);
            SimVar.SetSimVarValue("L:Generic_Master_Caution_Active", "Bool", 0);
        }
        this.firstAcknowledge = true;
        this.needReload = true;
    }
    onPowerOn() {
        this.offStart = true;
    }
    hasMessages() {
        for (let i = 0; i < this.allMessages.length; i++) {
            if (this.allMessages[i].Visible) {
                return true;
            }
        }
        return false;
    }
}
class Engine_Annunciations extends Cabin_Annunciations {
    init(root) {
        super.init(root);
        switch (this.engineType) {
            case EngineType.ENGINE_TYPE_PISTON:
                this.addMessage(Annunciation_MessageType.WARNING, "OIL PRESSURE", this.OilPressure);
                this.addMessage(Annunciation_MessageType.WARNING, "LOW VOLTS", this.LowVoltage);
                this.addMessage(Annunciation_MessageType.WARNING, "HIGH VOLTS", this.HighVoltage);
                this.addMessage(Annunciation_MessageType.WARNING, "CO LVL HIGH", this.COLevelHigh);
                this.addMessage(Annunciation_MessageType.CAUTION, "STBY BATT", this.StandByBattery);
                this.addMessage(Annunciation_MessageType.CAUTION, "LOW VACUUM", this.LowVaccum);
                this.addMessage(Annunciation_MessageType.CAUTION, "LOW FUEL R", this.LowFuelR);
                this.addMessage(Annunciation_MessageType.CAUTION, "LOW FUEL L", this.LowFuelL);
                break;
            case EngineType.ENGINE_TYPE_TURBOPROP:
            case EngineType.ENGINE_TYPE_JET:
                this.addMessage(Annunciation_MessageType.WARNING, "FUEL OFF", this.fuelOff);
                this.addMessage(Annunciation_MessageType.WARNING, "FUEL PRESS", this.fuelPress);
                this.addMessage(Annunciation_MessageType.WARNING, "OIL PRESS", this.oilPressWarning);
                this.addMessageMultipleConditions(Annunciation_MessageType.WARNING, "ITT", [
                    new Condition(this.itt.bind(this, "1000")),
                    new Condition(this.itt.bind(this, "870"), 5),
                    new Condition(this.itt.bind(this, "840"), 20)
                ]);
                this.addMessage(Annunciation_MessageType.WARNING, "FLAPS ASYM", this.flapsAsym);
                this.addMessage(Annunciation_MessageType.WARNING, "ELEC FEATH FAULT", this.elecFeathFault);
                this.addMessage(Annunciation_MessageType.WARNING, "BLEED TEMP", this.bleedTemp);
                this.addMessage(Annunciation_MessageType.WARNING, "CABIN ALTITUDE", this.cabinAltitude);
                this.addMessage(Annunciation_MessageType.WARNING, "EDM", this.edm);
                this.addMessage(Annunciation_MessageType.WARNING, "CABIN DIFF PRESS", this.cabinDiffPress);
                this.addMessage(Annunciation_MessageType.WARNING, "DOOR", this.door);
                this.addMessage(Annunciation_MessageType.WARNING, "USP ACTIVE", this.uspActive);
                this.addMessage(Annunciation_MessageType.WARNING, "GEAR UNSAFE", this.gearUnsafe);
                this.addMessage(Annunciation_MessageType.WARNING, "PARK BRAKE", this.parkBrake);
                this.addMessage(Annunciation_MessageType.WARNING, "OXYGEN", this.oxygen);
                this.addMessage(Annunciation_MessageType.CAUTION, "OIL PRESS", this.oilPressCaution);
                this.addMessage(Annunciation_MessageType.CAUTION, "CHIP", this.chip);
                this.addMessage(Annunciation_MessageType.CAUTION, "OIL TEMP", this.oilTemp);
                this.addMessage(Annunciation_MessageType.CAUTION, "AUX BOOST PMP ON", this.auxBoostPmpOn);
                this.addMessageSwitch(Annunciation_MessageType.CAUTION, ["FUEL LOW L", "FUEL LOW R", "FUEL LOW L-R"], this.fuelLowSelector);
                this.addMessage(Annunciation_MessageType.CAUTION, "AUTO SEL", this.autoSel);
                this.addMessageTimed(Annunciation_MessageType.CAUTION, "FUEL IMBALANCE", this.fuelImbalance, 30);
                this.addMessageSwitch(Annunciation_MessageType.CAUTION, ["LOW LVL FAIL L", "LOW LVL FAIL R", "LOW LVL FAIL L-R"], this.lowLvlFailSelector);
                this.addMessage(Annunciation_MessageType.CAUTION, "BAT OFF", this.batOff);
                this.addMessage(Annunciation_MessageType.CAUTION, "BAT AMP", this.batAmp);
                this.addMessage(Annunciation_MessageType.CAUTION, "MAIN GEN", this.mainGen);
                this.addMessage(Annunciation_MessageType.CAUTION, "LOW VOLTAGE", this.lowVoltage);
                this.addMessage(Annunciation_MessageType.CAUTION, "BLEED OFF", this.bleedOff);
                this.addMessage(Annunciation_MessageType.CAUTION, "USE OXYGEN MASK", this.useOxygenMask);
                this.addMessage(Annunciation_MessageType.CAUTION, "VACUUM LOW", this.vacuumLow);
                this.addMessage(Annunciation_MessageType.CAUTION, "PROP DEICE FAIL", this.propDeiceFail);
                this.addMessage(Annunciation_MessageType.CAUTION, "INERT SEP FAIL", this.inertSepFail);
                this.addMessageSwitch(Annunciation_MessageType.CAUTION, ["PITOT NO HT L", "PITOT NO HT R", "PITOT NO HT L-R"], this.pitotNoHtSelector);
                this.addMessageSwitch(Annunciation_MessageType.CAUTION, ["PITOT HT ON L", "PITOT HT ON R", "PITOT HT ON L-R"], this.pitotHtOnSelector);
                this.addMessage(Annunciation_MessageType.CAUTION, "STALL NO HEAT", this.stallNoHeat);
                this.addMessage(Annunciation_MessageType.CAUTION, "STALL HEAT ON", this.stallHeatOn);
                this.addMessage(Annunciation_MessageType.CAUTION, "FRONT CARGO DOOR", this.frontCargoDoor);
                this.addMessage(Annunciation_MessageType.CAUTION, "GPU DOOR", this.gpuDoor);
                this.addMessage(Annunciation_MessageType.CAUTION, "IGNITION", this.ignition);
                this.addMessage(Annunciation_MessageType.CAUTION, "STARTER", this.starter);
                this.addMessage(Annunciation_MessageType.CAUTION, "MAX DIFF MODE", this.maxDiffMode);
                this.addMessage(Annunciation_MessageType.CAUTION, "CPCS BACK UP MODE", this.cpcsBackUpMode);
                break;
        }
    }
    sayTrue() {
        return true;
    }
    SafePropHeat() {
        return false;
    }
    CautionPropHeat() {
        return false;
    }
    StandByBattery() {
        return false;
    }
    LowVaccum() {
        return SimVar.GetSimVarValue("WARNING VACUUM", "Boolean");
    }
    LowPower() {
        return false;
    }
    LowFuelR() {
        return SimVar.GetSimVarValue("FUEL RIGHT QUANTITY", "gallon") < 5;
    }
    LowFuelL() {
        return SimVar.GetSimVarValue("FUEL LEFT QUANTITY", "gallon") < 5;
    }
    FuelTempFailed() {
        return false;
    }
    ECUMinorFault() {
        return false;
    }
    PitchTrim() {
        return false;
    }
    StartEngage() {
        return false;
    }
    OilPressure() {
        return SimVar.GetSimVarValue("WARNING OIL PRESSURE", "Boolean");
    }
    LowFuelPressure() {
        const pressure = SimVar.GetSimVarValue("ENG FUEL PRESSURE", "psi");
        if (pressure <= 1) {
            return true;
        }
        return false;
    }
    LowVoltage() {
        const voltage = SimVar.GetSimVarValue("ELECTRICAL MAIN BUS VOLTAGE", "volts");
        if (voltage < 24) {
            return true;
        }
        return false;
    }
    HighVoltage() {
        const voltage = SimVar.GetSimVarValue("ELECTRICAL MAIN BUS VOLTAGE", "volts");
        if (voltage > 32) {
            return true;
        }
        return false;
    }
    FuelTemperature() {
        return false;
    }
    ECUMajorFault() {
        return false;
    }
    COLevelHigh() {
        return false;
    }
    fuelOff() {
        return (SimVar.GetSimVarValue("FUEL TANK SELECTOR:1", "number") == 0);
    }
    fuelPress() {
        return (SimVar.GetSimVarValue("GENERAL ENG FUEL PRESSURE:1", "psi") <= 10);
    }
    oilPressWarning() {
        return (SimVar.GetSimVarValue("ENG OIL PRESSURE:1", "psi") <= 60);
    }
    itt(_limit = 840) {
        const itt = SimVar.GetSimVarValue("TURB ENG ITT:1", "celsius");
        return (itt > _limit);
    }
    flapsAsym() {
        return false;
    }
    elecFeathFault() {
        return false;
    }
    bleedTemp() {
        return false;
    }
    cabinAltitude() {
        return SimVar.GetSimVarValue("PRESSURIZATION CABIN ALTITUDE", "feet") > 10000;
    }
    edm() {
        return false;
    }
    cabinDiffPress() {
        return SimVar.GetSimVarValue("PRESSURIZATION PRESSURE DIFFERENTIAL", "psi") > 6.2;
    }
    door() {
        return SimVar.GetSimVarValue("EXIT OPEN:0", "percent") > 0;
    }
    uspActive() {
        return false;
    }
    gearUnsafe() {
        return false;
    }
    parkBrake() {
        return SimVar.GetSimVarValue("BRAKE PARKING INDICATOR", "Bool");
    }
    oxygen() {
        return false;
    }
    oilPressCaution() {
        const press = SimVar.GetSimVarValue("ENG OIL PRESSURE:1", "psi");
        return (press <= 105 && press >= 60);
    }
    chip() {
        return false;
    }
    oilTemp() {
        const temp = SimVar.GetSimVarValue("GENERAL ENG OIL TEMPERATURE:1", "celsius");
        return (temp <= 0 || temp >= 104);
    }
    auxBoostPmpOn() {
        return SimVar.GetSimVarValue("GENERAL ENG FUEL PUMP ON:1", "Bool");
    }
    fuelLowSelector() {
        const left = SimVar.GetSimVarValue("FUEL TANK LEFT MAIN QUANTITY", "gallon") < 9;
        const right = SimVar.GetSimVarValue("FUEL TANK RIGHT MAIN QUANTITY", "gallon") < 9;
        if (left && right) {
            return 3;
        } else if (left) {
            return 1;
        } else if (right) {
            return 2;
        } else {
            return 0;
        }
    }
    autoSel() {
        return false;
    }
    fuelImbalance() {
        const left = SimVar.GetSimVarValue("FUEL TANK LEFT MAIN QUANTITY", "gallon");
        const right = SimVar.GetSimVarValue("FUEL TANK RIGHT MAIN QUANTITY", "gallon");
        return Math.abs(left - right) > 15;
    }
    lowLvlFailSelector() {
        return false;
    }
    batOff() {
        return !SimVar.GetSimVarValue("ELECTRICAL MASTER BATTERY", "Bool");
    }
    batAmp() {
        return SimVar.GetSimVarValue("ELECTRICAL BATTERY BUS AMPS", "amperes") > 50;
    }
    mainGen() {
        return !SimVar.GetSimVarValue("GENERAL ENG GENERATOR SWITCH:1", "Bool");
    }
    lowVoltage() {
        return SimVar.GetSimVarValue("ELECTRICAL MAIN BUS VOLTAGE", "volts") < 24.5;
    }
    bleedOff() {
        return SimVar.GetSimVarValue("BLEED AIR SOURCE CONTROL", "Enum") == 1;
    }
    useOxygenMask() {
        return SimVar.GetSimVarValue("PRESSURIZATION CABIN ALTITUDE", "feet") > 10000;
    }
    vacuumLow() {
        return SimVar.GetSimVarValue("PARTIAL PANEL VACUUM", "Enum") == 1;
    }
    propDeiceFail() {
        return false;
    }
    inertSepFail() {
        return false;
    }
    pitotNoHtSelector() {
        return 0;
    }
    pitotHtOnSelector() {
        return 0;
    }
    stallNoHeat() {
        return false;
    }
    stallHeatOn() {
        return false;
    }
    frontCargoDoor() {
        return false;
    }
    gpuDoor() {
        return false;
    }
    ignition() {
        return SimVar.GetSimVarValue("TURB ENG IS IGNITING:1", "Bool");
    }
    starter() {
        return SimVar.GetSimVarValue("GENERAL ENG STARTER ACTIVE:1", "Bool");
    }
    maxDiffMode() {
        return SimVar.GetSimVarValue("BLEED AIR SOURCE CONTROL", "Enum") == 3;
    }
    cpcsBackUpMode() {
        return false;
    }
}
class Warning_Data {
    constructor(_shortText, _longText, _soundEvent, _Level, _callback, _once = false) {
        this.hasPlayed = false;
        this.shortText = _shortText;
        this.longText = _longText;
        this.soundEvent = _soundEvent;
        this.soundEventId = new Name_Z(this.soundEvent);
        this.level = _Level;
        this.callback = _callback;
        this.once = _once;
    }
}
class Warning_Data_XML extends Warning_Data {
    constructor(_gps, _shortText, _longText, _soundEvent, _Level, _logicElement, _once = false) {
        super(_shortText, _longText, _soundEvent, _Level, null, _once);
        this.xmlLogic = new CompositeLogicXMLElement(_gps, _logicElement);
        this.callback = this.getXMLBoolean.bind(this);
    }
    getXMLBoolean() {
        return this.xmlLogic.getValue() != 0;
    }
}
class Warnings extends NavSystemElement {
    constructor() {
        super(...arguments);
        this.warnings = [];
        this.playingSounds = [];
        this.pullUp_sinkRate_Points = [
            [1160, 0, 0],
            [2320, 1070, 1460],
            [4930, 2380, 2980],
            [11600, 4285, 5360]
        ];
    }
    init(_root) {
        let alertsFromXML = false;
        if (this.gps.xmlConfig) {
            const alertsGroup = this.gps.xmlConfig.getElementsByTagName("VoicesAlerts");
            if (alertsGroup.length > 0) {
                alertsFromXML = true;
                const alerts = alertsGroup[0].getElementsByTagName("Alert");
                for (let i = 0; i < alerts.length; i++) {
                    const typeParam = alerts[i].getElementsByTagName("Type");
                    let type = 0;
                    if (typeParam.length > 0) {
                        switch (typeParam[0].textContent) {
                            case "Warning":
                                type = 3;
                                break;
                            case "Caution":
                                type = 2;
                                break;
                            case "Test":
                                type = 1;
                                break;
                            case "SoundOnly":
                                type = 0;
                                break;
                        }
                    }
                    let shortText = "";
                    let longText = "";
                    if (type != 0) {
                        const shortTextElem = alerts[i].getElementsByTagName("ShortText");
                        if (shortTextElem.length > 0) {
                            shortText = shortTextElem[0].textContent;
                        }
                        const longTextElem = alerts[i].getElementsByTagName("LongText");
                        if (longTextElem.length > 0) {
                            longText = longTextElem[0].textContent;
                        }
                    }
                    let soundEvent = "";
                    const soundEventElem = alerts[i].getElementsByTagName("SoundEvent");
                    if (soundEventElem.length > 0) {
                        soundEvent = soundEventElem[0].textContent;
                    }
                    const condition = alerts[i].getElementsByTagName("Condition")[0];
                    let once = false;
                    const onceElement = alerts[i].getElementsByTagName("Once");
                    if (onceElement.length > 0 && onceElement[0].textContent == "True") {
                        once = true;
                    }
                    this.warnings.push(new Warning_Data_XML(this.gps, shortText, longText, soundEvent, type, condition, once));
                }
            }
        }
        if (!alertsFromXML) {
            this.warnings.push(new Warning_Data("", "", "Garmin_Stall_f", 0, this.stallCallback.bind(this)));
            this.warnings.push(new Warning_Data("PULL UP", "PULL UP", "Garmin_Pull_Up_f", 3, this.pullUpCallback.bind(this)));
            this.warnings.push(new Warning_Data("TERRAIN", "SINK RATE", "Garmin_Sink_Rate_f", 2, this.sinkRateCallback.bind(this)));
            this.warnings.push(new Warning_Data("", "", "Garmin_landing_gear_f", 0, this.landingGearCallback.bind(this)));
            this.warnings.push(new Warning_Data("TAWS TEST", "", "", 1, this.tawsTestCallback.bind(this)));
            this.warnings.push(new Warning_Data("", "", "Garmin_TAWS_System_Test_OK_f", 0, this.tawsTestFinishedCallback.bind(this), true));
        }
        this.UID = parseInt(this.gps.getAttribute("Guid")) + 1;
        SimVar.SetSimVarValue("L:AS1000_Warnings_Master_Set", "number", 0);
    }
    onUpdate(_deltaTime) {
        const masterSet = SimVar.GetSimVarValue("L:AS1000_Warnings_Master_Set", "number");
        if (masterSet == 0) {
            SimVar.SetSimVarValue("L:AS1000_Warnings_Master_Set", "number", this.UID);
        } else if (masterSet == this.UID) {
            let found = false;
            let foundText = false;
            let bestWarning = 0;
            for (let i = 0; i < this.warnings.length; i++) {
                const warning = this.warnings[i];
                if (!warning.once || !warning.hasPlayed) {
                    if (warning.callback()) {
                        if (warning.soundEvent != "") {
                            if ((this.playingSounds.length <= 0) || (i < this.playingSounds[this.playingSounds.length - 1])) {
                                const res = this.gps.playInstrumentSound(warning.soundEvent);
                                if (res) {
                                    this.playingSounds.push(i);
                                    warning.hasPlayed = true;
                                }
                            }
                        }
                        if (!foundText) {
                            bestWarning = i;
                        }
                        if (warning.shortText || warning.longText) {
                            foundText = true;
                        }
                        found = true;
                    }
                }
            }
            if (found) {
                SimVar.SetSimVarValue("L:AS1000_Warnings_WarningIndex", "number", bestWarning + 1);
            } else {
                SimVar.SetSimVarValue("L:AS1000_Warnings_WarningIndex", "number", 0);
            }
        }
    }
    onSoundEnd(_eventId) {
        let i = 0;
        while (i < this.playingSounds.length) {
            const soundId = this.playingSounds[i];
            if (Name_Z.compare(this.warnings[soundId].soundEventId, _eventId)) {
                this.playingSounds.splice(i, 1);
                continue;
            }
            i++;
        }
    }
    onShutDown() {
        for (let i = 0; i < this.warnings.length; i++) {
            this.warnings[i].hasPlayed = false;
        }
        this.playingSounds = [];
    }
    linearMultiPointsEvaluation(_points, _valueX, _valueY) {
        let lastLowerIndex = -1;
        for (let i = 0; i < _points.length; i++) {
            if (_valueX > _points[i][0]) {
                lastLowerIndex = i;
            } else {
                break;
            }
        }
        if (lastLowerIndex == _points.length - 1) {
            for (let i = 1; i < _points[lastLowerIndex].length; i++) {
                if (_valueY < _points[lastLowerIndex][i]) {
                    return i;
                }
            }
            return _points[lastLowerIndex].length;
        } else if (lastLowerIndex == -1) {
            for (let i = 1; i < _points[0].length; i++) {
                if (_valueY < _points[0][i]) {
                    return i;
                }
            }
            return _points[0].length;
        } else {
            const factorLower = (_valueX - _points[lastLowerIndex][0]) / _points[lastLowerIndex + 1][0];
            for (let i = 1; i < _points[lastLowerIndex].length; i++) {
                const limit = _points[lastLowerIndex][i] * factorLower + _points[lastLowerIndex + 1][i] * (1 - factorLower);
                if (_valueY < limit) {
                    return i;
                }
            }
            return _points[lastLowerIndex].length;
        }
    }
    pullUpCallback() {
        const height = SimVar.GetSimVarValue("PLANE ALT ABOVE GROUND", "feet");
        const descentRate = -SimVar.GetSimVarValue("VERTICAL SPEED", "feet per minute");
        return this.linearMultiPointsEvaluation(this.pullUp_sinkRate_Points, descentRate, height) == 1;
    }
    sinkRateCallback() {
        const height = SimVar.GetSimVarValue("PLANE ALT ABOVE GROUND", "feet");
        const descentRate = -SimVar.GetSimVarValue("VERTICAL SPEED", "feet per minute");
        return this.linearMultiPointsEvaluation(this.pullUp_sinkRate_Points, descentRate, height) == 2;
    }
    landingGearCallback() {
        const gear = !SimVar.GetSimVarValue("IS GEAR RETRACTABLE", "Boolean") || SimVar.GetSimVarValue("GEAR HANDLE POSITION", "Boolean");
        const throttle = SimVar.GetSimVarValue("GENERAL ENG THROTTLE LEVER POSITION:1", "percent");
        const flaps = SimVar.GetSimVarValue("FLAPS HANDLE INDEX", "number");
        return !gear && (flaps > 1 || (throttle == 0));
    }
    stallCallback() {
        return SimVar.GetSimVarValue("STALL WARNING", "Boolean");
    }
    tawsTestCallback() {
        return this.gps.getTimeSinceStart() < 30000;
    }
    tawsTestFinishedCallback() {
        return this.gps.getTimeSinceStart() >= 30000;
    }
}
class Cabin_Warnings extends Warnings {
    constructor() {
        super(...arguments);
        this.currentWarningLevel = 0;
        this.currentWarningText = "";
    }
    init(root) {
        super.init(root);
        this.alwaysUpdate = true;
    }
    onUpdate(_deltaTime) {
        super.onUpdate(_deltaTime);
        const warningIndex = SimVar.GetSimVarValue("L:AS1000_Warnings_WarningIndex", "number");
        let warningText;
        let warningLevel;
        if (warningIndex <= 0 || warningIndex >= this.warnings.length) {
            warningText = "";
            warningLevel = 0;
        } else {
            warningText = this.warnings[warningIndex - 1].shortText;
            warningLevel = this.warnings[warningIndex - 1].level;
        }
        if (this.currentWarningLevel != warningLevel || this.currentWarningText != warningText) {
            this.currentWarningText = warningText;
            this.currentWarningLevel = warningLevel;
            if (this.warningBox && this.warningContent) {
                this.warningContent.textContent = warningText;
                switch (warningLevel) {
                    case 0:
                        this.warningBox.setAttribute("state", "Hidden");
                        break;
                    case 1:
                        this.warningBox.setAttribute("state", "White");
                        break;
                    case 2:
                        this.warningBox.setAttribute("state", "Yellow");
                        break;
                    case 3:
                        this.warningBox.setAttribute("state", "Red");
                        break;
                }
            }
        }
    }
    getCurrentWarningLevel() {
        return this.currentWarningLevel;
    }
    getCurrentWarningText() {
        return this.currentWarningText;
    }
    onEnter() { }
    onExit() { }
    onEvent(_event) { }
}
class GlassCockpit_XMLEngine extends NavSystemElement {
    constructor() {
        super(...arguments);
        this.xmlEngineDisplay = null;
        this._t = 0;
    }
    init(_root) {
        if (this.gps.xmlConfig) {
            const engineRoot = this.gps.xmlConfig.getElementsByTagName("EngineDisplay");
            if (engineRoot.length > 0) {
                this.xmlEngineDisplay = _root.querySelector("glasscockpit-xmlenginedisplay");
                this.xmlEngineDisplay.setConfiguration(this.gps, engineRoot[0]);
            }
        }
    }
    onEnter() {
    }
    onExit() {
    }
    onUpdate(_deltaTime) {
        if (this.xmlEngineDisplay) {
            this.xmlEngineDisplay.update(_deltaTime);
        }
        this._t++;
        if (this._t > 30) {
            this.gps.currFlightPlanManager.updateFlightPlan();
            this._t = 0;
        }
    }
    onEvent(_event) {
        this.xmlEngineDisplay.onEvent(_event);
    }
    onSoundEnd(_eventId) {
        super.onSoundEnd(_eventId);
        this.xmlEngineDisplay.onSoundEnd(_eventId);
    }
}
