import 'dart:io';
import 'dart:convert';

class JsonLoader {
    dynamic data;

    JsonLoader({String path, dynamic data}) {
        this.data = path==null ? data : jsonDecode(File(path).readAsStringSync());
    }
    String loadString(String p) {
        return data[p];
    }
    double loadDouble(String p) {
        return data[p]?.toDouble();
    }
    int loadInt(String p) {
        return data[p]?.toInt();
    }
    dynamic loadObject(String p, Function newFromJson) {
        return data[p]==null ? null : newFromJson(JsonLoader(data:data[p]));
    }
    List<T> loadListOf<T>(String p, Function newFromJson) {
        if(data[p] == null) return null;
        if(data[p] is! List) throw Exception('Expected List in json for $p, but found ${data[p]}');
        var l = <T>[];
        for(var d in data[p]) {
            l.add(newFromJson(JsonLoader(data:d)));
        }
        return l;
    }
}

class Str {
    static String fromJson(JsonLoader loader) => loader.data;
}

class NumInt {
    static int fromJson(JsonLoader loader) => loader.data.toInt();
}

class NumDouble {
    static double fromJson(JsonLoader loader) => loader.data.toDouble();
}

