import type { MsgSpec, SrvSpec } from './MessageSpec';
export declare function createMessageClass(msgSpec: MsgSpec): string;
export declare function createServiceClass(srvSpec: SrvSpec): string;
export declare function generateActionGoalMessage(messageName: string): string;
export declare function generateActionResultMessage(messageName: string): string;
export declare function generateActionFeedbackMessage(messageName: string): string;
export declare function generateActionMessage(messageName: string): string;
