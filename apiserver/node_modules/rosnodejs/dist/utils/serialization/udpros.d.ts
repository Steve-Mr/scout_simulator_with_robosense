/// <reference types="node" />
export declare function createSubHeader(callerId: string, md5sum: string, topic: string, type: string): Buffer;
export declare function createPubHeader(callerId: string, md5sum: string, type: string, messageDefinition: string): Buffer;
export declare function createUdpRosError(str: string): Buffer;
export declare function deserializeHeader(buff: Buffer): {
    connectionId: number;
    opCode: number;
    msgId: number;
    blkN: number;
};
export declare function serializeUdpHeader(connectionId: number, opCode: number, msgId: number, blkN: number): Buffer;
