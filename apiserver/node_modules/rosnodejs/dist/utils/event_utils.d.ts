/// <reference types="node" />
import type { EventEmitter } from 'events';
import Ultron = require('ultron');
export declare function rebroadcast<TArgs extends any[]>(evt: string, emitter: Ultron | EventEmitter, rebroadcaster: EventEmitter): void;
