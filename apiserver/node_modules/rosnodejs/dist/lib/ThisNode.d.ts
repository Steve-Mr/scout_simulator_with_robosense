import IRosNode from "../types/RosNode";
interface ThisNodeT {
    node: IRosNode | null;
    getNodeName(): string;
    ok(): boolean;
    shutdown(): Promise<void>;
    on(evt: string, listener: (d: any) => void): void;
    once(evt: string, listener: (d: any) => void): void;
    removeListener(evt: string, listener: (d: any) => void): void;
}
declare const ThisNode: ThisNodeT;
export default ThisNode;
