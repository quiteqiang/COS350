import 'dart:io';

class FileLoader {
    List<int> data;
    int i;

    FileLoader(String path) {
        data = File(path).readAsBytesSync().toList();
        i = 0;
    }

    bool get isEOF => i >= data.length;

    String peekCharASCII([int n=1]) {
        if(isEOF) return null;
        if(n == 1) return String.fromCharCode(data[i]);
        String s = '';
        for(var _ = 0; _ < n && (i+_) < data.length; _++) {
            s += String.fromCharCode(data[i+_]);
        }
        return s;
    }

    String readCharASCII([int n=1]) {
        if(isEOF) return null;
        if(n == 1) return String.fromCharCode(data[i++]);
        String s = '';
        while(s.length < n && !isEOF) {
            s += peekCharASCII();
            i++;
        }
        return s;
    }

    bool peekThenReadIfMatch(String s) {
        if(peekCharASCII(s.length) != s) return false;
        i += s.length;
        return true;
    }

    String readToNewLine() {
        String s = '';
        while(!isEOF && peekCharASCII() != '\n') {
            s += readCharASCII();
        }
        return s;
    }

    bool get isNumber {
        String c = peekCharASCII();
        return
            c=='0' || c=='1' || c=='2' || c=='3' || c=='4' ||
            c=='5' || c=='6' || c=='7' || c=='8' || c=='9';
    }

    bool get isWhitespace {
        String c = peekCharASCII();
        return c == ' ' || c == '\n' || c == '\t' || c == '\r';
    }

    bool get isComment {
        return peekCharASCII() == '#';
    }

    int readIntASCII() {
        if(!isNumber) return null;
        int v = 0;
        while(isNumber) v = v * 10 + int.parse(readCharASCII());
        return v;
    }
    int readIntBinary() {
        return data[i++];
    }

    bool eatWhitespace([int max=0]) {
        if(!isWhitespace) return false;
        if(max == 0) while(isWhitespace) i++;
        else for(var _ = 0; _ < max && isWhitespace; _++) i++;
        return true;
    }

    bool eatComment() {
        if(!isComment) return false;
        while(readCharASCII() != '\n');
        return true;
    }
}
