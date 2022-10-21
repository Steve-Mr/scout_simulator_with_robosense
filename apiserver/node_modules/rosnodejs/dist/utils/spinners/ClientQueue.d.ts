/// <reference types="node" />
import { EventEmitter } from 'events';
/**
 * @class ClientQueue
 * Queue of messages to handle for an individual client (subscriber or publisher)
 */
export default class ClientQueue extends EventEmitter {
    _queue: any[];
    _queueSize: number;
    throttleMs: number;
    _handleTime: number | null;
    _client: any;
    constructor(client: any, queueSize: number, throttleMs: number);
    destroy(): void;
    push(item: any): void;
    get length(): number;
    handleClientMessages(time: number): boolean;
}
