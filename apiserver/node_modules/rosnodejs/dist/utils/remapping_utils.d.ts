export declare enum SPECIAL_KEYS {
    name = "__name",
    log = "__log",
    ip = "__ip",
    hostname = "__hostname",
    master = "__master",
    ns = "__ns"
}
export declare type RemapT = {
    [SPECIAL_KEYS.name]?: string;
    [SPECIAL_KEYS.log]?: string;
    [SPECIAL_KEYS.ip]?: string;
    [SPECIAL_KEYS.hostname]?: string;
    [SPECIAL_KEYS.master]?: string;
    [SPECIAL_KEYS.ns]?: string;
    [key: string]: string;
};
export declare function processRemapping(args: string[]): RemapT;
