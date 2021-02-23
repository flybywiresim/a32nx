import React, { useState } from 'react';

import { useSimVar } from '../../../Common/simVars';
import Button, { BUTTON_TYPE } from '../../Components/Button/Button';

interface Props {
    fromVar,
    toVar,
    throttleNumber,
    text,
}

type DetentConfigState = {
    DETENT_FROM_SET: boolean,
    DETENT_TO_SET: boolean,
    RESET,
}

const DetentConfig: React.FC<Props> = (props: Props) => {
    const [state, setState] = useState<DetentConfigState>({ DETENT_FROM_SET: false, DETENT_TO_SET: false, RESET: false });

    const [throttle1Position] = useSimVar(`L:A32NX_THROTTLE_MAPPING_INPUT:${props.throttleNumber}`, 'number', 500);

    const applyButtonStateStyling = (state: DetentConfigState) => {
        if (state.DETENT_FROM_SET && !state.RESET) {
            return 'w-full bg-yellow-500 border-yellow-500 hover:bg-yellow-600 hover:border-yellow-600';
        }
        if (state.RESET) {
            return 'w-full border-green-500 bg-green-500 hover:bg-grey-600 hover:border-grey-600';
        }
        return 'w-full border-blue-500 bg-blue-500 hover:bg-blue-600 hover:border-blue-600';
    };

    const setFromTo = (throttle1Position, fromVar, toVar) => {
        if (!state.DETENT_FROM_SET) {
            fromVar.forEach((f) => f(throttle1Position));
            setState({ ...state, DETENT_FROM_SET: true, RESET: false });
        } else if (!state.DETENT_TO_SET) {
            toVar.forEach((f) => f(throttle1Position));
            setState({ ...state, DETENT_TO_SET: true, RESET: true });
        } else {
            setState({ DETENT_FROM_SET: false, DETENT_TO_SET: false, RESET: false });
        }
    };

    return (
        <div className="mb-4 w-56 justify-between items-center">
            <Button
                className={applyButtonStateStyling(state)}
                text={!state.DETENT_FROM_SET ? `${props.text} - low` : `${props.text} - high`}
                onClick={() => setFromTo(throttle1Position, props.fromVar, props.toVar)}
                type={BUTTON_TYPE.NONE}
            />
        </div>
    );
};

export default DetentConfig;
