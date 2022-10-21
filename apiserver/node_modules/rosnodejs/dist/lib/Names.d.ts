export declare type Remappings = {
    [key: string]: string;
};
declare class Names {
    _remappings: Remappings;
    _namespace: string;
    init(remappings: Remappings, namespace: string): void;
    validate(name: string, throwError?: boolean): boolean;
    clean(name: string): string;
    append(left: string, right: string): string;
    remap(name: string): string;
    /**
     * @param [namespace] {string} namespace to resolve name to. If not provided, node's namespace will be used
     * @param name {string} name to resolve
     * @param [remap] {bool} flag indicating if we should also attempt to remap the name
     */
    resolve(...args: ResolveArgs): string;
    parentNamespace(name: string): string;
    _remap(name: string): string;
    _parseResolveArgs(args: ResolveArgs): [string, string, boolean];
}
declare type ResolveArgs = [] | [string] | [string, string] | [string, boolean] | [string, string, boolean];
declare const _default: Names;
export default _default;
