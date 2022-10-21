/// <reference types="node" />
export declare const primitiveTypes: string[];
export declare function getDefaultValue(type: string): any;
export declare function isString(type: string): boolean;
export declare function isTime(type: string): boolean;
export declare function isBool(type: string): boolean;
export declare function isFloat(type: string): boolean;
export declare function isInteger(type: string): boolean;
export declare function isPrimitive(fieldType: string): boolean;
export declare function isArray(fieldType: string, details?: any): boolean;
export declare function isMessage(fieldType: string): boolean;
export declare function getTypeOfArray(arrayType: string): string;
export declare function getLengthOfArray(arrayType: string): number | null;
export declare class Field {
    name: string;
    type: string;
    isHeader: boolean;
    isBuiltin: boolean;
    isArray: boolean;
    baseType: string;
    arrayLen: number | null;
    constructor(name: string, type: string);
    getPackage(): string | null;
    getMessage(): string | null;
    static isHeader(type: string): boolean;
    static isBuiltin(type: string): boolean;
}
export declare function parsePrimitive(fieldType: string, fieldValue: any): any;
export declare function serializePrimitive<T = any>(fieldType: string, fieldValue: T, buffer: Buffer, bufferOffset: number): any;
export declare function deserializePrimitive(fieldType: string, buffer: Buffer, bufferOffset: number[]): any;
export declare function getPrimitiveSize(fieldType: string, fieldValue?: any): number;
export declare function getArraySize(field: Field, array: any[], msgSpec: any): number;
export declare function getMessageSize(message: any, msgSpec: any): number;
export declare function getMessageNameFromMessageType(messageType: string): string;
export declare function getPackageNameFromMessageType(messageType: string): string;
export declare function splitMessageType(messageType: string): string[];
