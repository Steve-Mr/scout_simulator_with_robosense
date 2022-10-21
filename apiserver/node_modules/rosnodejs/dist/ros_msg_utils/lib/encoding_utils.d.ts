/**
 * Returns the number of bytes in a string (using utf8 encoding),
 * in order to know the length needed to serialize it into a Buffer
 * @param strValue
 * @return {Number}
 */
export declare function getByteLength(strValue: string): number;
export declare type RosTime = {
    secs: number;
    nsecs: number;
};
