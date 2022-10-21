/// <reference types="node" />
import { EventEmitter } from 'events';
import { INodeHandle, SubscribeOptions, AdvertiseOptions } from '../types/NodeHandle';
import { ActionMsgs } from '../types/Message';
export declare type ActionServerInterfaceOptions = {
    type: string;
    actionServer: string;
    nh: INodeHandle;
    goal?: SubscribeOptions;
    cancel?: SubscribeOptions;
    status?: AdvertiseOptions;
    feedback?: AdvertiseOptions;
    result?: AdvertiseOptions;
};
export default class ActionServerInterface<G, F, R> extends EventEmitter {
    private _actionType;
    private _actionServer;
    private _goalSub;
    private _cancelSub;
    private _statusPub;
    private _feedbackPub;
    private _resultPub;
    private _nh;
    constructor(options: ActionServerInterfaceOptions);
    getType(): string;
    generateGoalId(): ActionMsgs.GoalID;
    shutdown(): Promise<void>;
    _handleGoal(msg: ActionMsgs.ActionGoal<G>): void;
    _handleCancel(msg: ActionMsgs.GoalID): void;
    publishResult(resultMsg: ActionMsgs.ActionResult<R>): void;
    publishFeedback(feedbackMsg: ActionMsgs.ActionFeedback<F>): void;
    publishStatus(statusMsg: ActionMsgs.GoalStatusArray): void;
}
