import React from 'react';
import { useSimVar } from '../../../Common/simVars';
import DetentConfig from './DetentConfig';

interface Props {
    throttleNumber: number,
    throttleCount: number,
}

class ThrottleSimvar {
    readableName: string;

    technicalName: string;

    hiSetter: Array<any>;

    hiGetter: Array<any>;

    lowSetter: Array<any>;

    lowGetter: Array<any>;

    constructor(readableName: string, technicalName: string) {
        this.readableName = readableName;
        this.technicalName = technicalName;
        this.hiSetter = [];
        this.hiGetter = [];
        this.lowGetter = [];
        this.lowSetter = [];
    }
}

const BaseThrottleConfig: React.FC<Props> = (props: Props) => {
    const mappings: Array<ThrottleSimvar> = [
        new ThrottleSimvar('Reverse', 'L:A32NX_THROTTLE_MAPPING_REVERSE_'),
        new ThrottleSimvar('Reverse Idle', 'L:A32NX_THROTTLE_MAPPING_REVERSE_IDLE_'),
        new ThrottleSimvar('Idle', 'L:A32NX_THROTTLE_MAPPING_IDLE_'),
        new ThrottleSimvar('Climb', 'L:A32NX_THROTTLE_MAPPING_CLIMB_'),
        new ThrottleSimvar('Flex', 'L:A32NX_THROTTLE_MAPPING_FLEXMCT_'),
        new ThrottleSimvar('Toga', 'L:A32NX_THROTTLE_MAPPING_TOGA_'),
    ];

    for (const mapped of mappings) {
        for (let index = 1; index <= props.throttleCount; index++) {
            const throttleNumber: number = props.throttleCount > 1 ? index : props.throttleNumber;

            mapped.hiGetter.push(useSimVar(`${mapped.technicalName}HIGH:${throttleNumber}`, 'number', 100)[0]);
            mapped.hiSetter.push(useSimVar(`${mapped.technicalName}HIGH:${throttleNumber}`, 'number', 100)[1]);

            mapped.lowGetter.push(useSimVar(`${mapped.technicalName}LOW:${throttleNumber}`, 'number', 100)[0]);
            mapped.lowSetter.push(useSimVar(`${mapped.technicalName}LOW:${throttleNumber}`, 'number', 100)[1]);
        }
    }

    return (

        <div className="bg-gray-800 px-4 pt-5 pb-4 sm:p-6 sm:pb-4 mt-4 mb-4">
            <div className="sm:flex sm:items-start">
                <div className="justify-between items-center">
                    { mappings.map((m) => (
                        <div className="divide-y divide-gray-700 ">
                            <span className="text-lg text-gray-300">{`${m.lowGetter[0].toFixed(2)} -> ${m.hiGetter[0].toFixed(2)}`}</span>
                            <DetentConfig text={`${m.readableName}`} fromVar={m.lowSetter} toVar={m.hiSetter} throttleNumber={props.throttleNumber} />
                        </div>
                    ))}
                </div>
            </div>
        </div>

    );
};

export default BaseThrottleConfig;
