import * as fieldsUtil from './fields';
import * as MessageSpec from './MessageSpec';
import { Message, MessageConstructor, ServiceConstructor } from '../../types/Message';
/** get message handler class from registry */
export declare function getPackageFromRegistry(packagename: string): OnTheFlyPackageEntry;
/** get all message and service definitions, from all packages */
export declare function getAll(): Promise<void>;
declare type MessageInfo = {
    messageType: string;
    fields: fieldsUtil.Field[];
    fieldTypeInfo: FieldTypeInfo;
    spec: MessageSpec.MsgSpec;
};
interface OnTheFlyMessageConstructor<T extends Message> extends MessageConstructor<T> {
    _info: MessageInfo;
}
interface OnTheFlyServiceConstructor<Req, Resp> extends ServiceConstructor<Req, Resp> {
    Request: OnTheFlyMessageConstructor<Req>;
    Response: OnTheFlyMessageConstructor<Resp>;
}
declare type FieldTypeInfo = {
    [key: string]: {
        spec: MessageSpec.MsgSpec;
        constructor?: OnTheFlyMessageConstructor<any>;
    };
};
declare type OnTheFlyPackageEntry = {
    msg?: {
        [key: string]: OnTheFlyMessageConstructor<any>;
    };
    srv?: {
        [key: string]: OnTheFlyServiceConstructor<any, any>;
    };
};
/**
   @param messageType is the ROS message or service type, e.g.
   'std_msgs/String'
   @param type is from the set
   ["msg", "srv"]
*/
export declare function getMessageFromRegistry(messageType: string): MessageConstructor<any>;
export declare function getServiceFromRegistry(messageType: string): ServiceConstructor<any, any>;
export {};
