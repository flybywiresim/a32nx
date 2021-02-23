import React, { useState } from 'react';
import { useSimVar } from '../../../Common/simVars';
import Button, { BUTTON_TYPE } from '../../Components/Button/Button';
import { Toggle } from '../../Components/Form/Toggle';

import BaseThrottleConfig from './BaseThrottleConfig';

interface Props {
    isShown: boolean,
    onClose: any
}

const ThrottleConfig: React.FC<Props> = (props: Props) => {
    if (!props.isShown) {
        return null;
    }

    const [dualAxis, setDualAxis] = useState(1);

    const [reverserOnAxis1, setReverserOnAxis1] = useSimVar('L:A32NX_THROTTLE_MAPPING_USE_REVERSE_ON_AXIS:1', 'number', 1000);
    const [, setReverserOnAxis2] = useSimVar('L:A32NX_THROTTLE_MAPPING_USE_REVERSE_ON_AXIS:2', 'number', 1000);

    const [, syncToDisk] = useSimVar('K:A32NX.THROTTLE_MAPPING_SAVE_TO_FILE', 'number', 1000);
    const [, syncToThrottle] = useSimVar('K:A32NX.THROTTLE_MAPPING_LOAD_FROM_FILE', 'number', 1000);
    const [, applyLocalVar] = useSimVar('K:A32NX.THROTTLE_MAPPING_LOAD_FROM_LOCAL_VARIABLES', 'number', 1000);

    const setReversOnAxis = (reverserOnAxis: number) => {
        setReverserOnAxis1(reverserOnAxis);
        setReverserOnAxis2(reverserOnAxis);
    };

    return (
        <div className="flex flex-row justify-center min-h-screen pt-4 px-4 pb-20 text-center sm:block sm:p-0">
            <div className="bg-gray-800 rounded-xl px-6 py-4 shadow-lg mt-4 mb-4">
                <div className=" flex flex-row w-full top-1/2">
                    <div className="mb-4 flex flex-col right-0 w-1/2" />
                </div>
                <span />

                { dualAxis !== 0

                    && (
                        <div className="flex flex-row justify-center">
                            <BaseThrottleConfig throttleNumber={1} throttleCount={1} />
                            <BaseThrottleConfig throttleNumber={2} throttleCount={1} />
                        </div>
                    )}

                { dualAxis === 0
                    && (

                        <div className="flex flex-row justify-center">
                            <BaseThrottleConfig throttleNumber={1} throttleCount={2} />
                        </div>
                    )}
            </div>
            <div className="bg-gray-800 flex flex-row-reverse">
                <Button
                    text="Close"
                    type={BUTTON_TYPE.RED}
                    onClick={props.onClose}
                    className="ml-2"

                />
                <Button
                    text="Persist"
                    type={BUTTON_TYPE.GREEN}
                    onClick={() => syncToDisk(1)}
                    className="ml-2"
                />
                <Button
                    text="Apply"
                    type={BUTTON_TYPE.BLUE}
                    onClick={() => applyLocalVar(1)}
                    className="ml-2"
                />
                <Button
                    text="Load "
                    type={BUTTON_TYPE.BLUE}
                    onClick={() => syncToThrottle(1)}
                    className="ml-2"
                />
                <div className="flex left-0 mt-auto mb-auto">

                    <span className="text-lg text-gray-300 mr-2">Reverser On Axis</span>
                    <Toggle value={!!reverserOnAxis1} onToggle={(value) => setReversOnAxis(value ? 1 : 0)} />
                    <div className="flex flex-row-reverse divide-y divide-gray-700" />
                    <span>
                        <span className="text-lg text-gray-300 mr-2 ml-2">Dual Axis</span>
                    </span>
                    <Toggle value={!!dualAxis} onToggle={(value) => setDualAxis(value ? 1 : 0)} />
                </div>
            </div>
        </div>
    );
};

export default ThrottleConfig;
