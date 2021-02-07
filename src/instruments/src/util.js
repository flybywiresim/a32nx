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

import { useEffect, useState, useRef } from 'react';

export const renderTarget = document.getElementById('A32NX_REACT_MOUNT');
export const customElement = renderTarget.parentElement;

// @param {() => void} handler
export function useInteractionEvent(event, handler) {
    // Logic based on https://usehooks.com/useEventListener/
    const savedHandler = useRef(handler);
    useEffect(() => {
        savedHandler.current = handler;
    }, [handler]);

    useEffect(() => {
        const wrappedHandler = (e) => {
            if (event === '*') {
                savedHandler.current(e.detail);
            } else {
                savedHandler.current();
            }
        };
        customElement.addEventListener(event, wrappedHandler);
        return () => {
            customElement.removeEventListener(event, wrappedHandler);
        };
    }, [event]);
}

// @param {(deltaTime: number) => void} handler
export function useUpdate(handler) {
    // Logic based on https://usehooks.com/useEventListener/
    const savedHandler = useRef(handler);
    useEffect(() => {
        savedHandler.current = handler;
    }, [handler]);

    useEffect(() => {
        const wrappedHandler = (event) => {
            savedHandler.current(event.detail);
        };
        customElement.addEventListener('update', wrappedHandler);
        return () => {
            customElement.removeEventListener('update', wrappedHandler);
        };
    });
}

const SIMVAR_TYPES = {
    '__proto__': null,
    'GPS POSITION LAT': 'degrees latitude',
    'L:APU_GEN_ONLINE': 'Bool',
    'EXTERNAL POWER AVAILABLE:1': 'Bool',
    'EXTERNAL POWER ON': 'Bool',
    'L:A32NX_COLD_AND_DARK_SPAWN': 'Bool',
    'TOTAL AIR TEMPERATURE': 'Celsius',
    'AMBIENT TEMPERATURE': 'Celsius',
    'L:A320_Neo_ADIRS_STATE': 'Enum',
};

const SIMVAR_CACHE = new Map();
customElement.addEventListener('update', () => {
    SIMVAR_CACHE.clear();
});

export function getSimVar(name, type) {
    if (!SIMVAR_CACHE.has(name)) {
        SIMVAR_CACHE.set(name, SimVar.GetSimVarValue(name, type || SIMVAR_TYPES[name]));
    }

    return SIMVAR_CACHE.get(name);
}

export function getGlobalVar(name, type) {
    if (!SIMVAR_CACHE.has(name)) {
        SIMVAR_CACHE.set(name, SimVar.GetGlobalVarValue(name, type || SIMVAR_TYPES[name]));
    }
    return SIMVAR_CACHE.get(name);
}

export function setSimVar(name, value, type = SIMVAR_TYPES[name]) {
    SIMVAR_CACHE.delete(name);
    return SimVar.SetSimVarValue(name, type, value);
}

export function useSimVar(name, type) {
    const [value, updater] = useState(0);

    useUpdate(() => updater(getSimVar(name, type)));

    return value;
}

export function useGlobalVar(name, type) {
    const [value, updater] = useState(0);

    useUpdate(() => updater(getGlobalVar(name, type)));

    return value;
}

export const createDeltaTimeCalculator = (startTime = Date.now()) => {
    let lastTime = startTime;

    return () => {
        const nowTime = Date.now();
        const deltaTime = nowTime - lastTime;
        lastTime = nowTime;

        return deltaTime;
    };
};
