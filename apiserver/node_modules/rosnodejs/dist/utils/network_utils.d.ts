import { RemapT } from './remapping_utils';
export declare function init(remappings: RemapT): void;
export declare function getHost(): string;
export declare function getAddressAndPortFromUri(uriString: string): {
    host: string;
    port: number;
};
export declare function formatServiceUri(port: number): string;
