class DebugPrinter {
    final indentations = [
        '',
        '  ',
        '    ',
        '      ',
        '        ',
    ];
    int indentation = 0;

    DebugPrinter();
    
    String get indents => indentations[indentation];
    
    DebugPrinter writeln(String s) {
        print('$indents$s');
        return this;
    }
    
    DebugPrinter start(String s) {
        writeln(s);
        indentation++;
        return this;
    }
    DebugPrinter end() {
        indentation--;
        return this;
    }
}