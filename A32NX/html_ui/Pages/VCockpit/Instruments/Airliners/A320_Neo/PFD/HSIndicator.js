class Jet_PFD_HSIndicator extends HTMLElement {
    constructor() {
        super(...arguments);
        this.cursorOpacity = "1.0";
        this.strokeOpacity = "0.75";
        this.strokeColor = "rgb(255,255,255)";
        this.strokeSize = 6;
        this.fontSize = 25;
        this.refStartX = 0;
        this.refWidth = 0;
        this.graduationScrollPosX = 0;
        this.graduationScrollPosY = 0;
        this.nbPrimaryGraduations = 7;
        this.nbSecondaryGraduations = 1;
        this.totalGraduations = this.nbPrimaryGraduations + ((this.nbPrimaryGraduations - 1) * this.nbSecondaryGraduations);
        this.graduationSpacing = 50;
        this._showILS = false;
        this._aircraft = Aircraft.A320_NEO;
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
    connectedCallback() {
        this.graduationScroller = new Avionics.Scroller(this.nbPrimaryGraduations, 10, true, 360);
        this.construct();
    }
    showILS(_val) {
        this._showILS = _val;
        if (!this._showILS) {
            this.ILSBeaconGroup.setAttribute("visibility", "hidden");
            this.ILSOffscreenGroup.setAttribute("visibility", "hidden");
        }
    }

    construct() {
        Utils.RemoveAllChildren(this);
        this.construct_A320_Neo();
    }

    construct_A320_Neo() {
        this.rootSVG = document.createElementNS(Avionics.SVG.NS, "svg");
        this.rootSVG.setAttribute("id", "ViewBox");
        this.rootSVG.setAttribute("viewBox", "0 0 550 250");
        this.refStartX = 25;
        this.refWidth = 500;
        const posX = this.refStartX;
        const posY = 5;
        const width = this.refWidth;
        const height = 100;
        if (!this.rootGroup) {
            this.rootGroup = document.createElementNS(Avionics.SVG.NS, "g");
            this.rootGroup.setAttribute("id", "HS");
        } else {
            Utils.RemoveAllChildren(this.rootGroup);
        }
        this.rootSVG.appendChild(this.rootGroup);
        if (!this.centerSVG) {
            this.centerSVG = document.createElementNS(Avionics.SVG.NS, "svg");
            this.centerSVG.setAttribute("id", "CenterGroup");
        } else {
            Utils.RemoveAllChildren(this.centerSVG);
        }
        this.centerSVG.setAttribute("x", posX.toString());
        this.centerSVG.setAttribute("y", posY.toString());
        this.centerSVG.setAttribute("width", width.toString());
        this.centerSVG.setAttribute("height", height.toString());
        this.centerSVG.setAttribute("overflow", "hidden");
        this.centerSVG.setAttribute("viewBox", "0 0 " + width + " " + height);
        this.rootGroup.appendChild(this.centerSVG);
        {
            const _top = 35;
            const _left = 0;
            const _width = width;
            const _height = 80;
            const bg = document.createElementNS(Avionics.SVG.NS, "rect");
            bg.setAttribute("x", _left.toString());
            bg.setAttribute("y", _top.toString());
            bg.setAttribute("width", _width.toString());
            bg.setAttribute("height", _height.toString());
            bg.setAttribute("fill", "#5A6573");
            bg.setAttribute("stroke", this.strokeColor);
            bg.setAttribute("stroke-width", this.strokeSize.toString());
            bg.setAttribute("stroke-opacity", this.strokeOpacity);
            this.centerSVG.appendChild(bg);
            const graduationGroup = document.createElementNS(Avionics.SVG.NS, "g");
            graduationGroup.setAttribute("id", "Graduations");
            {
                this.graduationScrollPosX = _left + _width * 0.5;
                this.graduationScrollPosY = _top;
                if (!this.graduations) {
                    this.graduations = [];
                    for (var i = 0; i < this.totalGraduations; i++) {
                        var line = new Avionics.SVGGraduation();
                        line.IsPrimary = (i % (this.nbSecondaryGraduations + 1)) ? false : true;
                        const lineWidth = line.IsPrimary ? 5 : 5;
                        const lineHeight = line.IsPrimary ? 25 : 12;
                        const linePosY = 0;
                        line.SVGLine = document.createElementNS(Avionics.SVG.NS, "rect");
                        line.SVGLine.setAttribute("y", linePosY.toString());
                        line.SVGLine.setAttribute("width", lineWidth.toString());
                        line.SVGLine.setAttribute("height", lineHeight.toString());
                        line.SVGLine.setAttribute("fill", "white");
                        if (line.IsPrimary) {
                            line.SVGText1 = document.createElementNS(Avionics.SVG.NS, "text");
                            line.SVGText1.setAttribute("y", (linePosY + lineHeight + 20).toString());
                            line.SVGText1.setAttribute("fill", "white");
                            line.SVGText1.setAttribute("font-size", (this.fontSize * 1.35).toString());
                            line.SVGText1.setAttribute("font-family", "Roboto-Bold");
                            line.SVGText1.setAttribute("text-anchor", "middle");
                            line.SVGText1.setAttribute("alignment-baseline", "central");
                        }
                        this.graduations.push(line);
                    }
                }
                for (var i = 0; i < this.totalGraduations; i++) {
                    var line = this.graduations[i];
                    graduationGroup.appendChild(line.SVGLine);
                    if (line.SVGText1) {
                        graduationGroup.appendChild(line.SVGText1);
                    }
                }
                this.centerSVG.appendChild(graduationGroup);
            }
            this.selectedHeadingGroup = document.createElementNS(Avionics.SVG.NS, "g");
            this.selectedHeadingGroup.setAttribute("id", "Heading");
            {
                const headingPosX = _left + _width * 0.5 - 2;
                const headingPosY = posY;
                const headingWidth = 35;
                const headingHeight = _height;
                const headingSVG = document.createElementNS(Avionics.SVG.NS, "svg");
                headingSVG.setAttribute("x", (headingPosX - headingWidth * 0.5).toString());
                headingSVG.setAttribute("y", headingPosY.toString());
                headingSVG.setAttribute("width", headingWidth.toString());
                headingSVG.setAttribute("height", headingHeight.toString());
                headingSVG.setAttribute("viewBox", "0 0 " + headingWidth + " " + headingHeight);
                {
                    const headingShape = document.createElementNS(Avionics.SVG.NS, "path");
                    headingShape.setAttribute("fill", "transparent");
                    headingShape.setAttribute("stroke", "#00FFFF");
                    headingShape.setAttribute("stroke-width", "4");
                    headingShape.setAttribute("d", "M20 24 l -12 -20 l 24 0 z");
                    headingSVG.appendChild(headingShape);
                }
                this.selectedHeadingGroup.appendChild(headingSVG);
            }
            this.centerSVG.appendChild(this.selectedHeadingGroup);
            this.currentTrackGroup = document.createElementNS(Avionics.SVG.NS, "g");
            this.currentTrackGroup.setAttribute("id", "CurrentTrack");
            {
                const trackPosX = _left + _width * 0.5;
                const trackPosY = posY + 30;
                const trackWidth = 28;
                const trackHeight = _height;
                const trackSVG = document.createElementNS(Avionics.SVG.NS, "svg");
                trackSVG.setAttribute("x", (trackPosX - trackWidth * 0.5).toString());
                trackSVG.setAttribute("y", trackPosY.toString());
                trackSVG.setAttribute("width", trackWidth.toString());
                trackSVG.setAttribute("height", trackHeight.toString());
                trackSVG.setAttribute("viewBox", "0 0 " + trackWidth + " " + trackHeight);
                {
                    const trackShape = document.createElementNS(Avionics.SVG.NS, "path");
                    trackShape.setAttribute("fill", "transparent");
                    trackShape.setAttribute("stroke", "#00FF00");
                    trackShape.setAttribute("stroke-width", "4");
                    trackShape.setAttribute("d", "M13 0 l-13 17 l13 17 l13 -17 Z");
                    trackSVG.appendChild(trackShape);
                }
                this.currentTrackGroup.appendChild(trackSVG);
            }
            this.centerSVG.appendChild(this.currentTrackGroup);
            this.ILSBeaconGroup = document.createElementNS(Avionics.SVG.NS, "g");
            this.ILSBeaconGroup.setAttribute("id", "ILSBeacon");
            {
                const ilsPosX = _left + _width * 0.5 + 2.5;
                const ilsPosY = posY + 45;
                const ilsWidth = 30;
                const ilsHeight = _height;
                const ilsSVG = document.createElementNS(Avionics.SVG.NS, "svg");
                ilsSVG.setAttribute("x", (ilsPosX - ilsWidth * 0.5).toString());
                ilsSVG.setAttribute("y", ilsPosY.toString());
                ilsSVG.setAttribute("width", ilsWidth.toString());
                ilsSVG.setAttribute("height", ilsHeight.toString());
                ilsSVG.setAttribute("viewBox", "0 0 " + ilsWidth + " " + ilsHeight);
                {
                    const ilsShape = document.createElementNS(Avionics.SVG.NS, "path");
                    ilsShape.setAttribute("fill", "transparent");
                    ilsShape.setAttribute("stroke", "#FF94FF");
                    ilsShape.setAttribute("stroke-width", "5");
                    ilsShape.setAttribute("d", "M15 0 l0 50 M0 40 l30 0");
                    ilsSVG.appendChild(ilsShape);
                }
                this.ILSBeaconGroup.appendChild(ilsSVG);
            }
            this.centerSVG.appendChild(this.ILSBeaconGroup);
            const cursorPosX = _left + _width * 0.5;
            const cursorPosY = posY;
            const cursorWidth = 35;
            const cursorHeight = _height;
            if (!this.cursorSVG) {
                this.cursorSVG = document.createElementNS(Avionics.SVG.NS, "svg");
                this.cursorSVG.setAttribute("id", "CursorGroup");
            } else {
                Utils.RemoveAllChildren(this.cursorSVG);
            }
            this.cursorSVG.setAttribute("x", (cursorPosX - cursorWidth * 0.5).toString());
            this.cursorSVG.setAttribute("y", cursorPosY.toString());
            this.cursorSVG.setAttribute("width", cursorWidth.toString());
            this.cursorSVG.setAttribute("height", cursorHeight.toString());
            this.cursorSVG.setAttribute("viewBox", "0 0 " + cursorWidth + " " + cursorHeight);
            {
                const cursorShape = document.createElementNS(Avionics.SVG.NS, "path");
                cursorShape.setAttribute("fill", "yellow");
                cursorShape.setAttribute("fill-opacity", this.cursorOpacity);
                cursorShape.setAttribute("d", "M 15 2 L 25 2 L 25 53 L 15 53 L 15 2 Z");
                this.cursorSVG.appendChild(cursorShape);
            }
            this.centerSVG.appendChild(this.cursorSVG);
        }
        const rectWidth = 70;
        this.ILSOffscreenGroup = document.createElementNS(Avionics.SVG.NS, "g");
        this.ILSOffscreenGroup.setAttribute("id", "ILSOffscreen");
        this.rootSVG.appendChild(this.ILSOffscreenGroup);
        {
            const rect = document.createElementNS(Avionics.SVG.NS, "rect");
            rect.setAttribute("x", (-rectWidth * 0.5).toString());
            rect.setAttribute("y", "60");
            rect.setAttribute("width", rectWidth.toString());
            rect.setAttribute("height", "40");
            rect.setAttribute("fill", "url(#Backlight)");
            rect.setAttribute("stroke", "white");
            rect.setAttribute("stroke-width", "3");
            this.ILSOffscreenGroup.appendChild(rect);
            this.ILSOffscreenText = document.createElementNS(Avionics.SVG.NS, "text");
            this.ILSOffscreenText.setAttribute("x", "0");
            this.ILSOffscreenText.setAttribute("y", (60 + 20).toString());
            this.ILSOffscreenText.setAttribute("fill", "#FF94FF");
            this.ILSOffscreenText.setAttribute("font-size", (this.fontSize * 1.35).toString());
            this.ILSOffscreenText.setAttribute("font-family", "Roboto-Light");
            this.ILSOffscreenText.setAttribute("text-anchor", "middle");
            this.ILSOffscreenText.setAttribute("alignment-baseline", "central");
            this.ILSOffscreenGroup.appendChild(this.ILSOffscreenText);
        }
        this.appendChild(this.rootSVG);
    }
    update(dTime) {
        this.updateRibbon();
    }
    updateRibbon() {
        const compass = SimVar.GetSimVarValue("PLANE HEADING DEGREES MAGNETIC", "degree");
        const trkfpaMode = SimVar.GetSimVarValue("L:A32NX_TRK_FPA_MODE_ACTIVE", "Bool");
        const selectedHeading = SimVar.GetSimVarValue("L:A32NX_AUTOPILOT_HEADING_SELECTED", "Degrees");
        const track = SimVar.GetSimVarValue("GPS GROUND MAGNETIC TRACK", "degree");
        if (this.graduations) {
            this.graduationScroller.scroll(compass);
            let currentVal = this.graduationScroller.firstValue;
            let currentX = this.graduationScrollPosX - this.graduationScroller.offsetY * this.graduationSpacing * (this.nbSecondaryGraduations + 1);
            for (let i = 0; i < this.totalGraduations; i++) {
                var posX = currentX;
                const posY = this.graduationScrollPosY;
                this.graduations[i].SVGLine.setAttribute("transform", "translate(" + posX.toString() + " " + posY.toString() + ")");
                if (this.graduations[i].SVGText1) {
                    const roundedVal = Math.floor(currentVal / 10);
                    this.graduations[i].SVGText1.textContent = roundedVal.toString();
                    this.graduations[i].SVGText1.setAttribute("transform", "translate(" + posX.toString() + " " + posY.toString() + ")");
                    currentVal = this.graduationScroller.nextValue;
                }
                currentX += this.graduationSpacing;
            }
        }
        if (this.selectedHeadingGroup) {
            const showSelectedHeading = SimVar.GetSimVarValue("L:A320_FCU_SHOW_SELECTED_HEADING", "number") === 1;
            if (showSelectedHeading) {
                var delta = selectedHeading - compass;
                if (delta > 180) {
                    delta = delta - 360;
                } else if (delta < -180) {
                    delta = delta + 360;
                }
                var posX = delta * this.graduationSpacing * (this.nbSecondaryGraduations + 1) / this.graduationScroller.increment;
                this.selectedHeadingGroup.setAttribute("transform", "translate(" + posX.toString() + " 0)");
                this.selectedHeadingGroup.setAttribute("visibility", "visible");
            } else {
                this.selectedHeadingGroup.setAttribute("visibility", "hidden");
            }
        }
        if (this.currentTrackGroup) {
            var delta = track - compass;
            if (delta > 180) {
                delta = delta - 360;
            } else if (delta < -180) {
                delta = delta + 360;
            }
            var posX = delta * this.graduationSpacing * (this.nbSecondaryGraduations + 1) / this.graduationScroller.increment;
            this.currentTrackGroup.setAttribute("transform", "translate(" + posX.toString() + " 0)");
        }
        if (this._showILS) {
            if (this.ILSBeaconGroup && this.ILSOffscreenGroup) {
                const localizer = this.gps.radioNav.getBestILSBeacon();
                if (localizer.id > 0) {
                    var delta = localizer.course - compass;
                    if (delta > 180) {
                        delta = delta - 360;
                    } else if (delta < -180) {
                        delta = delta + 360;
                    }
                    var posX = delta * this.graduationSpacing * (this.nbSecondaryGraduations + 1) / this.graduationScroller.increment;
                    if (posX > -(this.refWidth * 0.5) && posX < (this.refWidth * 0.5)) {
                        this.ILSBeaconGroup.setAttribute("visibility", "visible");
                        this.ILSBeaconGroup.setAttribute("transform", "translate(" + posX.toString() + " 0)");
                        this.ILSOffscreenGroup.setAttribute("visibility", "hidden");
                    } else {
                        let pos;
                        if (posX <= -(this.refWidth * 0.5)) {
                            pos = this.refStartX + 15;
                        } else {
                            pos = this.refStartX + this.refWidth - 15;
                        }
                        const rounded = Math.round(localizer.course);
                        this.ILSOffscreenText.textContent = Utils.leadingZeros(rounded, 3);
                        this.ILSOffscreenGroup.setAttribute("transform", "translate(" + pos + " 0)");
                        this.ILSOffscreenGroup.setAttribute("visibility", "visible");
                        this.ILSBeaconGroup.setAttribute("visibility", "hidden");
                    }
                } else {
                    this.ILSOffscreenGroup.setAttribute("visibility", "hidden");
                    this.ILSBeaconGroup.setAttribute("visibility", "hidden");
                }
            }
        }
    }
}
customElements.define("jet-pfd-hs-indicator", Jet_PFD_HSIndicator);
