/// <reference types="node" />
import * as BN from 'bn.js';
import { RosTime } from './encoding_utils';
declare function StringSerializer(value: string, buffer: Buffer, bufferOffset: number): number;
declare function UInt8Serializer(value: number, buffer: Buffer, bufferOffset: number): number;
declare function UInt16Serializer(value: number, buffer: Buffer, bufferOffset: number): number;
declare function UInt32Serializer(value: number, buffer: Buffer, bufferOffset: number): number;
declare function UInt64Serializer(value: number | BN, buffer: Buffer, bufferOffset: number): number;
declare function Int8Serializer(value: number, buffer: Buffer, bufferOffset: number): number;
declare function Int16Serializer(value: number, buffer: Buffer, bufferOffset: number): number;
declare function Int32Serializer(value: number, buffer: Buffer, bufferOffset: number): number;
declare function Int64Serializer(value: number | BN, buffer: Buffer, bufferOffset: number): number;
declare function Float32Serializer(value: number, buffer: Buffer, bufferOffset: number): number;
declare function Float64Serializer(value: number, buffer: Buffer, bufferOffset: number): number;
declare function TimeSerializer(value: RosTime, buffer: Buffer, bufferOffset: number): number;
declare function BoolSerializer(value: boolean, buffer: Buffer, bufferOffset: number): number;
export declare type SerializeT<T = any> = (data: T, buffer: Buffer, offset: number) => number;
/**
 * Specialized array serialization for UInt8 Arrays
 */
declare function UInt8ArraySerializer(array: number[], buffer: Buffer, bufferOffset: number, specArrayLen?: number | null): number;
declare const Serialize: {
    string: typeof StringSerializer;
    float32: typeof Float32Serializer;
    float64: typeof Float64Serializer;
    bool: typeof BoolSerializer;
    int8: typeof Int8Serializer;
    int16: typeof Int16Serializer;
    int32: typeof Int32Serializer;
    int64: typeof Int64Serializer;
    uint8: typeof UInt8Serializer;
    uint16: typeof UInt16Serializer;
    uint32: typeof UInt32Serializer;
    uint64: typeof UInt64Serializer;
    char: typeof UInt8Serializer;
    byte: typeof Int8Serializer;
    time: typeof TimeSerializer;
    duration: typeof TimeSerializer;
    Array: {
        string: any;
        float32: any;
        float64: any;
        bool: any;
        int8: any;
        int16: any;
        int32: any;
        int64: any;
        uint8: typeof UInt8ArraySerializer;
        uint16: any;
        uint32: any;
        uint64: any;
        char: typeof UInt8ArraySerializer;
        byte: any;
        time: any;
        duration: any;
    };
};
export default Serialize;
