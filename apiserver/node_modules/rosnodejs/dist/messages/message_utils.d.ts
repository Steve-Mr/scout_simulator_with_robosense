import { MessageRegistryPackageEntry, MessageConstructor, ServiceConstructor } from '../types/Message';
export declare function getTopLevelMessageDirectory(): string;
export declare function loadMessagePackage(msgPackage: string): MessageRegistryPackageEntry;
export declare function getPackage(msgPackage: string): MessageRegistryPackageEntry;
export declare function requireMsgPackage(msgPackage: string): MessageRegistryPackageEntry;
export declare function getAvailableMessagePackages(): {
    [key: string]: string;
};
export declare function getHandlerForMsgType<T extends MessageConstructor<any>>(rosDataType: string, loadIfMissing?: boolean): T;
export declare function getHandlerForSrvType<T extends ServiceConstructor<any, any>>(rosDataType: string, loadIfMissing?: boolean): T;
