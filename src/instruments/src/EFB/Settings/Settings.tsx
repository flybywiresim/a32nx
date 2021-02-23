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

import React, { useState } from 'react';
import { Toggle } from '../Components/Form/Toggle';
import { Select, SelectGroup, SelectItem } from '../Components/Form/Select';
import { Slider } from '../Components/Form/Slider';
import { useSimVarSyncedPersistentProperty } from '../../Common/persistence';
import Button from '../Components/Button/Button';
import ThrottleConfig from './ThrottleConfig/ThrottleConfig';

const PlaneSettings: React.FC = () => (
    <div className="bg-gray-800 opacity-40 rounded-xl px-6 py-4 shadow-lg">
        <h1 className="text-xl font-medium text-white mb-3">Realism</h1>

        <div className="divide-y divide-gray-700 flex flex-col">
            <div className="mb-3.5 flex flex-row justify-between items-center">
                <span className="text-lg text-gray-300">ADIRS Align Time</span>
                <SelectGroup>
                    <SelectItem>Instant</SelectItem>
                    <SelectItem>Fast</SelectItem>
                    <SelectItem selected>Real</SelectItem>
                </SelectGroup>
            </div>
            <div className="mb-4 pt-3 flex flex-row justify-between items-center">
                <span className="text-lg text-gray-300">DMC self-test</span>
                <SelectGroup>
                    <SelectItem>Instant</SelectItem>
                    <SelectItem>Fast</SelectItem>
                    <SelectItem selected>Real</SelectItem>
                </SelectGroup>
            </div>
        </div>

        <h1 className="text-xl text-white font-medium mt-4 mb-3">ATSU/AOC</h1>

        <div className="divide-y divide-gray-700 flex flex-col">
            <div className="mb-3.5 flex flex-row justify-between items-center">
                <span className="text-lg text-gray-300">ATIS Source</span>
                <SelectGroup>
                    <SelectItem>FAA (US)</SelectItem>
                    <SelectItem>PilotEdge</SelectItem>
                    <SelectItem>IVAO</SelectItem>
                    <SelectItem selected>VATSIM</SelectItem>
                </SelectGroup>
            </div>
            <div className="mb-3.5 pt-3 flex flex-row justify-between items-center">
                <span className="text-lg text-gray-300">METAR Source</span>
                <SelectGroup>
                    <SelectItem>MeteoBlue</SelectItem>
                    <SelectItem>IVAO</SelectItem>
                    <SelectItem>PilotEdge</SelectItem>
                    <SelectItem selected>VATSIM</SelectItem>
                </SelectGroup>
            </div>
            <div className="pt-3 flex flex-row justify-between items-center">
                <span className="text-lg text-gray-300">TAF Source</span>
                <SelectGroup>
                    <SelectItem>IVAO</SelectItem>
                    <SelectItem selected>NOAA</SelectItem>
                </SelectGroup>
            </div>
        </div>

        <h1 className="text-xl text-white font-medium mt-5 mb-3">FMGC</h1>

        <div className="divide-y divide-gray-700 flex flex-col">
            <div className="mb-3.5 flex flex-row justify-between items-center">
                <span className="text-lg text-gray-300">Thrust Reduction Altitude</span>
                <div className="flex flex-row">
                    <Select>1500 ft</Select>
                </div>
            </div>
            <div className="mb-3.5 pt-3 flex flex-row justify-between items-center">
                <span className="text-lg text-gray-300">Acceleration Altitude </span>
                <div className="flex flex-row">
                    <Select>1500 ft</Select>
                </div>
            </div>
            <div className="mb-3.5 pt-3 flex flex-row justify-between items-center">
                <span className="text-lg text-gray-300">Transition Level</span>
                <div className="flex flex-row">
                    <Select>FL180</Select>
                </div>
            </div>

            <div className="w-full pt-2 flex flex-row justify-between">
                <div className="pt-2 pr-4 flex-grow flex flex-row justify-between items-center">
                    <span className="text-lg text-gray-300">Default Baro</span>
                    <SelectGroup>
                        <SelectItem selected>Auto</SelectItem>
                        <SelectItem>in Hg</SelectItem>
                        <SelectItem>hPa</SelectItem>
                    </SelectGroup>
                </div>
                <div className="pt-2 pl-4 flex-grow flex flex-row justify-between items-center">
                    <span className="text-lg text-gray-300">Weight Unit</span>
                    <SelectGroup>
                        <SelectItem selected>Kg</SelectItem>
                        <SelectItem>lbs</SelectItem>
                    </SelectGroup>
                </div>
            </div>
        </div>
    </div>
);

const SoundSettings: React.FC = () => {
    const [ptuAudible, setPtuAudible] = useSimVarSyncedPersistentProperty('L:A32NX_SOUND_PTU_AUDIBLE_COCKPIT', 'Bool', 'SOUND_PTU_AUDIBLE_COCKPIT');
    const [exteriorVolume, setExteriorVolume] = useSimVarSyncedPersistentProperty('L:A32NX_SOUND_EXTERIOR_MASTER', 'number', 'SOUND_EXTERIOR_MASTER');
    const [engineVolume, setEngineVolume] = useSimVarSyncedPersistentProperty('L:A32NX_SOUND_INTERIOR_ENGINE', 'number', 'SOUND_INTERIOR_ENGINE');
    const [windVolume, setWindVolume] = useSimVarSyncedPersistentProperty('L:A32NX_SOUND_INTERIOR_WIND', 'number', 'SOUND_INTERIOR_WIND');

    return (
        <div className="bg-gray-800 rounded-xl px-6 py-4 shadow-lg">
            <div className="divide-y divide-gray-700 flex flex-col">
                <div className="mb-4 flex flex-row justify-between items-center">
                    <span>
                        <span className="text-lg text-gray-300">PTU Audible in Cockpit</span>
                        <span className="text-lg text-gray-500 ml-2">(unrealistic)</span>
                    </span>
                    <Toggle value={!!ptuAudible} onToggle={(value) => setPtuAudible(value ? 1 : 0)} />
                </div>
                <div className="mb-4 pt-3 flex flex-row justify-between items-center">
                    <span className="text-lg text-gray-300">Exterior Master Volume</span>
                    <div className="flex flex-row items-center">
                        <span className="text-base pr-3">{exteriorVolume}</span>
                        <Slider className="w-60" value={exteriorVolume + 50} onInput={(value) => setExteriorVolume(value - 50)} />
                    </div>
                </div>
                <div className="mb-4 pt-3 flex flex-row justify-between items-center">
                    <span className="text-lg text-gray-300">Engine Interior Volume</span>
                    <div className="flex flex-row items-center">
                        <span className="text-base pr-3">{engineVolume}</span>
                        <Slider className="w-60" value={engineVolume + 50} onInput={(value) => setEngineVolume(value - 50)} />
                    </div>
                </div>
                <div className="pt-3 flex flex-row justify-between items-center">
                    <span className="text-lg text-gray-300">Wind Interior Volume</span>
                    <div className="flex flex-row items-center">
                        <span className="text-base pr-3">{windVolume}</span>
                        <Slider className="w-60" value={windVolume + 50} onInput={(value) => setWindVolume(value - 50)} />
                    </div>
                </div>
            </div>
        </div>
    );
};

const ControlSettings = ({ setShowSettings }) => (
    <div className="bg-gray-800 divide-y divide-gray-700 flex flex-col rounded-xl px-6 py-4 shadow-lg">
        <div className="flex flex-row justify-between items-center">
            <span className="text-lg text-gray-300">Detents</span>
            <Button text="Calibrate" onClick={() => setShowSettings(true)} />
        </div>

    </div>
);

const FlyPadSettings: React.FC = () => (
    <div className="bg-gray-800 divide-y divide-gray-700 flex flex-col rounded-xl px-6 py-4 shadow-lg">
        <div className="flex flex-row justify-between items-center">
            <span className="text-lg text-gray-300">Brightness</span>
            <Slider value={90} onInput={() => {}} className="w-60" />
        </div>
    </div>
);

const Settings: React.FC = () => {
    const [showThrottleSettings, setShowThrottleSettings] = useState(false);

    return (

        <div className="w-full h-full flex flex-col">
            { !showThrottleSettings
        && (
            <div className="flex-grow m-6 rounded-xl flex flex-row">
                <div className="w-1/2 pr-3">
                    <div className="opacity-40">
                        <h1 className="text-2xl text-white mb-4">Plane Settings</h1>

                        <PlaneSettings />
                    </div>
                </div>
                <div className="w-1/2 pl-3">
                    <h1 className="text-2xl text-white mb-4">Audio Settings</h1>
                    <SoundSettings />

                    <h1 className="text-2xl text-white mt-5 mb-4">Throttle Settings</h1>
                    <ControlSettings setShowSettings={setShowThrottleSettings} />

                    <div className="opacity-40">
                        <h1 className="text-2xl text-white mt-5 mb-4">flyPad Settings</h1>
                        <FlyPadSettings />
                    </div>

                    <h1 className="text-4xl text-center text-gray-700 pt-10">flyPadOS</h1>
                    <h1 className="text-xl text-center text-gray-600 py-2">vAlpha</h1>
                    <h1 className="text-md text-center text-gray-700 py-2">Copyright &copy; 2020-2021 FlyByWire Simulations</h1>
                </div>
            </div>
        )}
            { showThrottleSettings && <ThrottleConfig isShown={showThrottleSettings} onClose={() => setShowThrottleSettings(false)} />}

        </div>

    );
};

export default Settings;
