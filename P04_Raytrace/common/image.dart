import 'dart:io';
import 'dart:math';

import 'maths.dart';
import 'fileloader.dart';

class Image {
    List<Color> _image;
    int _width;
    int _height;

    Image(this._width, this._height) {
        clear();
    }

    // Image.fromFile supports only Netpbm/PNM images (.pbm, .pgm, .ppm, pam)
    Image.fromFile(String path, {String pathMask, bool withMask=false}) {
        loadImage(path);
        if(withMask) pathMask = Image.getMaskPath(path);
        if(pathMask != null) loadMaskImage(pathMask);
    }

    Image.generateImageA() {
        resize(512, 512);
        var cOutside = Color(0.0, 1.0, 0.0, 0.0);
        var cBorder  = Color(0.0, 0.0, 0.0, 1.0);
        var cRing0   = Color(1.0, 1.0, 1.0, 1.0);
        var cRing1   = Color(0.2, 0.2, 0.8, 0.5);
        var cInner0  = Color(1.0, 1.0, 1.0, 1.0);
        var cInner1  = Color(1.0, 1.0, 1.0, 0.0);
        Color c;
        for(var x = 0; x < 512; x++) {
            for(var y = 0; y < 512; y++) {
                var dx = x - 256;
                var dy = y - 256;
                var d = sqrt(dx*dx + dy*dy);
                     if(d > 205) c = cOutside;
                else if(d > 200) c = cBorder;
                else if(d > 150) c = cRing0;
                else if(d > 145) c = cBorder;
                else if(d > 100) c = cRing1;
                else             c = (x+y)%2==0 ? cInner0 : cInner1;
                setPixel(x, y, c);
            }
        }
    }

    Image.generateImageB({withAlpha=false}) {
        resize(512, 512);
        var c0 = Color(1.0, 0.1, 0.1);
        var c1 = Color(0.1, 1.0, 0.1);
        Color c;
        for(var x = 0; x < 512; x++) {
            for(var y = 0; y < 512; y++) {
                var cx = (x*20/512).floor();
                var cy = (asin((y-256)/256)*20 / (pi/2)).floor() % 2;
                c = (cx+cy)%2==0 ? c0 : c1;
                if(withAlpha) c.alpha = pow(sin((y-256)/256 * (pi/2)), 2);
                setPixel(x, y, c);
            }
        }
    }

    Image.composite(Image imageA, Image imageB, Color fnComposite(Color a, Color c)) {
        assert(imageA.width == imageB.width && imageA.height == imageB.height);
        resize(imageA.width, imageA.height);
        for(var x = 0; x < _width; x++) {
            for(var y = 0; y < _height; y++) {
                var a = imageA.getPixel(x, y);
                var b = imageB.getPixel(x, y);
                setPixel(x, y, fnComposite(a, b));
            }
        }
    }


    static String getMaskPath(String path) => path.replaceAll(new RegExp(r'\.p[bgpna]m'), '_mask.ppm');


    int get width => _width;
    int get height => _height;


    // resizes image to given width and height, and clears image
    void resize(int width, int height) {
        _width = width;
        _height = height;
        clear();
    }

    // replaces all pixels in image with clearColor (default: black)
    void clear([Color clearColor]) {
        clearColor ??= Color.black();
        _image  = List<Color>(width * height);
        for(var i = 0; i < width*height; i++) {
            _image[i] = clearColor.copy();
        }
    }


    operator [](dynamic coord) {
        if(coord is List) return getPixel(coord[0], coord[1]);
        return getPixelSafe(coord.x, coord.y);
    }
    operator []=(dynamic coord, dynamic c) {
        if(coord is List) setPixel(coord[0], coord[1], c);
        else setPixelSafe(coord.x, coord.y, c);
    }

    void setPixel(num x, num y, dynamic c) {
        _image[y.toInt() * width + x.toInt()] = c.toColor();
    }
    void setPixelSafe(num x, num y, dynamic c) {
        if(y < 0 || x < 0 || y >= height || x >= width) return;
        _image[y.toInt() * width + x.toInt()] = c.toColor();
    }

    Color getPixel(num x, num y) => _image[y.toInt() * width + x.toInt()].copy();
    Color getPixelSafe(num x, num y) {
        if(y < 0 || x < 0 || y >= height || x >= width) return null;
        return getPixel(x, y);
    }


    // loadImage supports only Netpbm/PNM images (.pbm, .pgm, .ppm, pam)
    // Netpbm references:
    // - https://en.wikipedia.org/wiki/Netpbm
    // - http://netpbm.sourceforge.net/doc/pam.html
    // - https://people.cs.clemson.edu/~dhouse/courses/405/notes/ppm-files.pdf
    void loadImage(String path) {
        var loader = FileLoader(path);

        String magic = loader.readCharASCII(2);
        if(magic[0] != 'P') throw Exception('Invalid PNM header in $path: expected "P" but saw "$magic"');
        if(magic!='P1' && magic!='P2' && magic!='P3' && magic!='P4' && magic!='P5' && magic!='P6' && magic!='P7')
            throw Exception('Invalid PNM header $magic in $path');

        int    pver = int.parse(magic[1]);
        bool   isBinary;
        bool   isRGB;
        bool   hasAlpha;
        int    depth;
        double maxv;
        String type;

        // load header
        if(pver == 7) {
            // PAM loader
            while(true) {
                loader.eatWhitespace();
                if(loader.peekThenReadIfMatch('ENDHDR\n'))  break;
                if(loader.peekThenReadIfMatch('WIDTH '))    _width  = loader.readIntASCII();
                if(loader.peekThenReadIfMatch('HEIGHT '))   _height = loader.readIntASCII();
                if(loader.peekThenReadIfMatch('DEPTH '))    depth   = loader.readIntASCII();
                if(loader.peekThenReadIfMatch('MAXVAL '))   maxv    = loader.readIntASCII().toDouble();
                if(loader.peekThenReadIfMatch('TUPLTYPE ')) type    = loader.readToNewLine();
            }
            if(_width==null || _height==null || maxv==null || depth==null)
                throw Exception('Invalid PAM header in $path');
            hasAlpha = type != null && type.endsWith('_ALPHA');
            isBinary = true;
        } else {
            // PBM/PGM/PPM
            isBinary = pver>=4 && pver<=6;
            isRGB    = pver==3 || pver==6;
            depth    = isRGB ? 3 : 1;
            hasAlpha = false;  // PBM/PGM/PPM do not have alpha
            if(pver == 1 || pver == 4) type = 'BLACKANDWHITE';
            if(pver == 2 || pver == 5) type = 'GRAYSCALE';
            if(pver == 3 || pver == 6) type = 'RGB';

            // Note: bitmap images (P1 and P4) do not have max value stored in header (always 1)
            var valsToLoad = (pver==1 || pver==4) ? 2 : 3;
            var vals = <int>[];
            while(vals.length < valsToLoad) {
                if(loader.isWhitespace) loader.eatWhitespace();
                if(loader.isComment)    loader.eatComment();
                if(loader.isNumber)     vals.add(loader.readIntASCII());
            }
            // eat single whitespace that separates header from data
            loader.eatWhitespace(1);

            maxv    = vals[2]?.toDouble() ?? 1;
            _width  = vals[0];
            _height = vals[1];
        }

        // right now, this loader does not handle all cases :(
        if(isBinary && maxv > 255)      // 2 byte binary values: need to combine 2bytes
            throw Exception('Cannot handle binary PNM with max values > 255');
        if(isBinary && pver == 1)       // binary bitmaps: need to split 1byte into 8bits
            throw Exception('Cannot handle binary bitmap PNM');
        if(depth==1 && hasAlpha)         // make sure all settings are as expected
            throw Exception('Cannot handle configuration: depth:$depth, hasAlpha:$hasAlpha');
        if(depth==2 && !hasAlpha)
            throw Exception('Cannot handle configuration: depth:$depth, hasAlpha:$hasAlpha');
        if(depth==3 && hasAlpha)
            throw Exception('Cannot handle configuration: depth:$depth, hasAlpha:$hasAlpha');
        if(depth==4 && !hasAlpha)
            throw Exception('Cannot handle configuration: depth:$depth, hasAlpha:$hasAlpha');
        if(depth<1 || depth>4)
            throw Exception('Cannot handle configuration: depth:$depth, hasAlpha:$hasAlpha');

        _image      = <Color>[];

        // print('Loading ${width}x${height} ${isBinary?"Binary":"ASCII"} $type image');
        for(var idx = 0; idx < width*height; idx++) {
            var vs = <double>[];
            for(var d = 0; d < depth; d++) {
                if(isBinary) vs.add(loader.readIntBinary().toDouble() / maxv);
                else {
                    if(loader.isWhitespace) loader.eatWhitespace();
                    vs.add(loader.readIntASCII().toDouble() / maxv);
                }
            }
            if(depth == 1 && !hasAlpha) {
                _image.add(Color(vs[0], vs[0], vs[0]));
            } else if(depth == 2 && hasAlpha) {
                _image.add(Color(vs[0], vs[0], vs[0], vs[1]));
            } else if(depth == 3 && !hasAlpha) {
                _image.add(Color(vs[0], vs[1], vs[2]));
            } else if(depth == 4 && hasAlpha) {
                _image.add(Color(vs[0], vs[1], vs[2], vs[3]));
            }
        }
    }

    void loadMaskImage(String pathMask) {
        var mask = Image.fromFile(pathMask);
        for(var i = 0; i < width * height; i++) {
            _image[i].alpha = mask._image[i].red;
        }
    }

    // saveImage supports only Netpbm/PNM images (.pbm, .pgm, .ppm, pam)
    void saveImage(String path, {bool withAlpha=false, bool asBinary=true, int maxv=255, Color background, bool saveMask=false}) {
        background ??= Color.black();
        var br = background.red;
        var bg = background.green;
        var bb = background.blue;

        if(withAlpha || asBinary) {
            var data = <int>[];
            String header;
            if(withAlpha) {
                header = 'P7\nWIDTH $width\nHEIGHT $height\nDEPTH 4\nMAXVAL 255\nTUPLTYPE RGB_ALPHA\nENDHDR\n';
            } else {
                header = 'P6\n${width} ${height}\n255\n';
            }
            for(var i = 0; i < header.length; i++) {
                data.add(header.codeUnitAt(i));
            }
            for(Color c in _image) {
                if(withAlpha) {
                    data.add(c.redAsInt(maxv:maxv));
                    data.add(c.greenAsInt(maxv:maxv));
                    data.add(c.blueAsInt(maxv:maxv));
                    data.add(c.alphaAsInt(maxv:maxv));
                } else if(saveMask) {
                    var a = c.alpha;
                    data.add(c.redAsInt(maxv:maxv));
                    data.add(c.greenAsInt(maxv:maxv));
                    data.add(c.blueAsInt(maxv:maxv));
                } else {
                    var a = c.alpha;
                    c = Color(
                        c.red   * a + br * (1 - a),
                        c.green * a + bg * (1 - a),
                        c.blue  * a + bb * (1 - a),
                    );
                    data.add(c.redAsInt(maxv:maxv));
                    data.add(c.greenAsInt(maxv:maxv));
                    data.add(c.blueAsInt(maxv:maxv));
                }
            }
            File(path).writeAsBytesSync(data);
        } else {
            var data = StringBuffer();
            var header = 'P3\n${width} ${height}\n255';
            data.writeln(header);
            for(Color c in _image) {
                var a = c.alpha;
                c = Color(
                    c.red   * a + br * (1 - a),
                    c.green * a + bg * (1 - a),
                    c.blue  * a + bb * (1 - a),
                );
                data.writeln('${c.redAsInt(maxv:maxv)} ${c.greenAsInt(maxv:maxv)} ${c.blueAsInt(maxv:maxv)}');
            }
            File(path).writeAsStringSync(data.toString());
        }

        if(saveMask) {
            String pathMask = Image.getMaskPath(path);
            saveMaskImage(pathMask, asBinary:asBinary, maxv:maxv);
        }
    }

    void saveMaskImage(String pathMask, {bool asBinary=true, int maxv=255}) {
        var mask = Image(width, height);
        for(var i = 0; i < width * height; i++) {
            mask._image[i].red   = _image[i].alpha;
            mask._image[i].green = _image[i].alpha;
            mask._image[i].blue  = _image[i].alpha;
            mask._image[i].alpha = 1;
        }
        mask.saveImage(pathMask, asBinary:asBinary, maxv:maxv);
    }
}
