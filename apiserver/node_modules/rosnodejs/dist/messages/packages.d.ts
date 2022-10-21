declare type MessageType = 'message' | 'service' | 'action';
declare type MessageEntry = {
    type: MessageType;
    name: string;
    file: string;
};
declare type PackageEntry = {
    directory: string;
    messages: {
        [key: string]: MessageEntry;
    };
    services: {
        [key: string]: MessageEntry;
    };
    actions: {
        [key: string]: MessageEntry;
    };
};
export declare type MsgPackageCache = {
    [key: string]: PackageEntry;
};
export declare function findPackage(packageName: string): Promise<string>;
export declare function findMessagePackages(): Promise<void>;
export declare function getMessagePackageCache(): MsgPackageCache;
export {};
