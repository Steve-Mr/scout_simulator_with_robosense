/// <reference types="node" />
import { EventEmitter } from 'events';
import type Spinner from '../../types/Spinner';
import type { SpinnerOptions } from '../../types/RosNode';
/**
 * @class GlobalSpinner
 * Clients (subscribers and publishers) will register themselves with the node's spinner
 * when they're created. Clients will disconnect from the spinner whenever they're shutdown.
 * Whenever they receive a new message to handle, those clients will "ping" the spinner,
 * which will push the new message onto that client's queue and add the client to a list
 * of clients to be handled on the next spin. While spinning, the spinner is locked and
 * ping and disconnect operations are cached in order to ensure that changes aren't
 * made to the spinner during its execution (e.g. subscriber callback publishes a message,
 * publisher pings the spinner which queues the new message and adds the client to its callback
 * list, the client list is cleared at the end of the spin and this client has a
 * message hanging in its queue that will never be handled). Once all of the messages
 * received since the last spin are handled the Spinner is unlocked and all cached
 * ping and disconnect operations are replayed in order.
 */
export default class GlobalSpinner extends EventEmitter implements Spinner {
    private _spinTime;
    private _spinTimer;
    private _clientCallQueue;
    private _clientQueueMap;
    private _queueLocked;
    private _lockedOpCache;
    private _emit;
    constructor(options?: GlobalSpinnerOptions);
    clear(): void;
    addClient(client: any, clientId: string, queueSize: number, throttleMs: number): void;
    /**
     * When subscribers/publishers receive new messages to handle, they will
     * "ping" the spinner.
     * @param clientId
     * @param msg
     */
    ping(clientId?: string, msg?: any): void;
    disconnect(clientId: string): void;
    _queueMessage(clientId: string, message: any): void;
    _handleLockedOpCache(): void;
    _setTimer(): void;
    _handleQueue(): void;
}
interface GlobalSpinnerOptions extends SpinnerOptions {
    spinRate?: number;
    emit?: boolean;
}
export {};
