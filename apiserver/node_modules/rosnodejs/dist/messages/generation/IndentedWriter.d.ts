export default class IndentedWriter {
    private _str;
    private _indentation;
    write(...args: any[]): IndentedWriter;
    newline(indentDir?: number | undefined): IndentedWriter;
    indent(...args: any[]): IndentedWriter;
    isIndented(): boolean;
    dedent(...args: any[]): IndentedWriter;
    resetIndent(): IndentedWriter;
    dividingLine(): IndentedWriter;
    get(): string;
}
