/// <reference types="node" />
import * as BN from 'bn.js';
import { RosTime } from './encoding_utils';
declare function StringDeserializer(buffer: Buffer, bufferOffset: number[]): string;
declare function UInt8Deserializer(buffer: Buffer, bufferOffset: number[]): number;
declare function UInt16Deserializer(buffer: Buffer, bufferOffset: number[]): number;
declare function UInt32Deserializer(buffer: Buffer, bufferOffset: number[]): number;
declare function UInt64Deserializer(buffer: Buffer, bufferOffset: number[]): BN;
declare function Int8Deserializer(buffer: Buffer, bufferOffset: number[]): number;
declare function Int16Deserializer(buffer: Buffer, bufferOffset: number[]): number;
declare function Int32Deserializer(buffer: Buffer, bufferOffset: number[]): number;
declare function Int64Deserializer(buffer: Buffer, bufferOffset: number[]): BN;
declare function Float32Deserializer(buffer: Buffer, bufferOffset: number[]): number;
declare function Float64Deserializer(buffer: Buffer, bufferOffset: number[]): number;
declare function TimeDeserializer(buffer: Buffer, bufferOffset: number[]): RosTime;
declare function BoolDeserializer(buffer: Buffer, bufferOffset: number[]): boolean;
export declare type DeserializeT<T> = (b: Buffer, o: number[]) => T;
/**
 * Specialized array deserialization for UInt8 Arrays
 * We return the raw buffer when deserializing uint8 arrays because it's much faster
  */
declare function UInt8ArrayDeserializer(buffer: Buffer, bufferOffset: number[], arrayLen?: number | null): Buffer;
declare const Deserialize: {
    string: typeof StringDeserializer;
    float32: typeof Float32Deserializer;
    float64: typeof Float64Deserializer;
    bool: typeof BoolDeserializer;
    int8: typeof Int8Deserializer;
    int16: typeof Int16Deserializer;
    int32: typeof Int32Deserializer;
    int64: typeof Int64Deserializer;
    uint8: typeof UInt8Deserializer;
    uint16: typeof UInt16Deserializer;
    uint32: typeof UInt32Deserializer;
    uint64: typeof UInt64Deserializer;
    char: typeof UInt8Deserializer;
    byte: typeof Int8Deserializer;
    time: typeof TimeDeserializer;
    duration: typeof TimeDeserializer;
    Array: {
        string: any;
        float32: any;
        float64: any;
        bool: any;
        int8: any;
        int16: any;
        int32: any;
        int64: any;
        uint8: typeof UInt8ArrayDeserializer;
        uint16: any;
        uint32: any;
        uint64: any;
        char: typeof UInt8ArrayDeserializer;
        byte: any;
        time: any;
        duration: any;
    };
};
export default Deserialize;
