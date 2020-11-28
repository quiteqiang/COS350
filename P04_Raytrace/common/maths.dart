import 'dart:math';

import 'jsonloader.dart';

// Num contains convenient number-related functions
class Num {
    static double toDouble(num v) {
        return v.toDouble();
    }
    static int toInt(num v) {
        return v.toInt();
    }
    static int sqrtInt(num v) {
        return sqrt(v.toDouble()).toInt();
    }
    static double sqrtDouble(num v) {
        return sqrt(v.toDouble());
    }
    static double toRadians(num degrees) {
        return degrees / 180 * pi;
    }
}


class Vector2 {
    double x;
    double y;

    Vector2([this.x = 0, this.y = 0]);
    Vector2.fromVector2(Vector2 v) : this(v.x, v.y);
    Vector2.fromList(List list) : this(list[0].toDouble(), list[1].toDouble());
    Vector2.fromJson(JsonLoader loader) : this.fromList(loader.data);
    Vector2.fromPolar(double radians, num radius) : this(cos(radians) * radius, sin(radians) * radius);

    String toString() => 'Vector2($x, $y)';

    Vector2 setTo(Vector2 that) {
        this.x = that.x;
        this.y = that.y;
        return this;
    }
    Vector2 setXYZ(double x, double y) {
        this.x = x;
        this.y = y;
        return this;
    }

    List<double> asList() => [this.x, this.y];

    // various ways of computing length
    double get length        => sqrt(lengthSquared);
    double get lengthSquared => this.dot(this);
    double get l1norm        => x.abs() + y.abs();
    double get l2norm        => length;
    double get maxnorm       => max(x.abs(), y.abs());

    Vector2 operator -() => Vector2(-this.x, -this.y);
    Vector2 operator +(Vector2 that) => Vector2(this.x + that.x, this.y + that.y);
    Vector2 operator -(Vector2 that) => Vector2(this.x - that.x, this.y - that.y);
    Vector2 operator *(dynamic that) {
        if(that is num) {
            double s = that.toDouble();
            return Vector2(this.x * s, this.y * s);
        }
        return Vector2(this.x * that.x, this.y * that.y);
    }

    Vector2 operator /(dynamic that) {
        if(that is num) {
            double s = that.toDouble();
            return Vector2(this.x / s, this.y / s);
        }
        return Vector2(this.x / that.x, this.y / that.y);
    }

    Vector2 normalize() {
        double l = this.length;
        return this.setXYZ(x / l, y / l);
    }
    Vector2 asNormalized() => this / this.length;

    // computes dot (inner) product
    double dot(Vector2 that) => this.x * that.x + this.y * that.y;

    // linearly interpolates between two Vectors
    // when factor = 0, from is returned
    // when factor = 1, to is returned
    // when factor = 0.5, the average of from and to is returned
    static Vector2 lerp(Vector2 from, Vector2 to, double factor) => Vector2(
        from.x * (1 - factor) + to.x * factor,
        from.y * (1 - factor) + to.y * factor,
    );
}

class Point2 extends Vector2 {
    Point2(double x, double y) : super(x, y);
    Point2.fromList(List p) : super.fromList(p);
    Point2.fromVector2(Vector2 v) : super.fromVector2(v);
    Point2.fromJson(JsonLoader loader) : super.fromList(loader.data);
    Point2.fromPolar(double radians, num radius) : this(cos(radians) * radius, sin(radians) * radius);

    String toString() => 'Point2($x, $y)';

    static Point2 average(Point2 a, Point2 b) => Point2((a.x + b.x) / 2, (a.y + b.y) / 2);

    static Point2 lerp(Point2 a, Point2 b, double factor) => Point2(
        a.x * (1 - factor) + b.x * factor,
        a.y * (1 - factor) + b.y * factor,
    );

    Point2 operator +(Vector2 that) {
        return Point2(this.x + that.x, this.y + that.y);
    }
    operator -(Vector2 that) {
        return Vector2(this.x - that.x, this.y - that.y);
    }
}

class Point2i extends Vector2i {
    Point2i(num x, num y) : super(x.toInt(), y.toInt());
    Point2i.fromList(List p) : super.fromList(p);
    Point2i.fromVector2i(Vector2i v) : super.fromVector2i(v);
    Point2i.fromJson(JsonLoader loader) : super.fromList(loader.data);
    Point2i.fromPolar(double radians, num radius) : this(cos(radians) * radius, sin(radians) * radius);

    String toString() => 'Point2i($x, $y)';

    Point2i operator +(Vector2i that) {
        return Point2i(this.x + that.x, this.y + that.y);
    }
    operator -(Vector2i that) {
        return Vector2i(this.x - that.x, this.y - that.y);
    }
}

class LineSegment2i {
    Point2i p0;
    Point2i p1;

    LineSegment2i(this.p0, this.p1);

    Vector2i get delta => p1 - p0;
}


class Vector2i {
    int x;
    int y;

    Vector2i(num x, num y) {
        this.x = x.toInt();
        this.y = y.toInt();
    }
    Vector2i.fromList(List list) : this(list[0].toInt(), list[1].toInt());
    Vector2i.fromJson(JsonLoader loader) : this.fromList(loader.data);
    Vector2i.fromVector2i(Vector2i v) : this(v.x, v.y);
    Vector2i.fromPolar(double radians, num radius) : this(cos(radians) * radius, sin(radians) * radius);

    String toString() => 'Vector2i($x, $y)';

    Vector2i operator +(Vector2i that) {
        return Vector2i(this.x + that.x, this.y + that.y);
    }

    Vector2i operator -(Vector2i that) {
        return Vector2i(this.x - that.x, this.y - that.y);
    }
}

class Size2 {
    double width;
    double height;

    double get area => width * height;

    Size2([this.width = 0, this.height = 0]);
    Size2.fromList(List list) : this(list[0].toDouble(), list[1].toDouble());
    Size2.fromJson(JsonLoader loader) : this.fromList(loader.data);

    String toString() => 'Size2($width, $height)';
}

class Size2i {
    int width;
    int height;

    int get area => width * height;

    Size2i([this.width = 0, this.height = 0]);
    Size2i.fromList(List list) : this(list[0].toInt(), list[1].toInt());
    Size2i.fromJson(JsonLoader loader) : this.fromList(loader.data);

    String toString() => 'Size2i($width, $height)';
}

// Vector stores three double values (x,y,z) to represent a vector in 3D
class Vector {
    double x;
    double y;
    double z;

    Vector(this.x, this.y, this.z);

    Vector.fromList(List list) : this(
        list[0].toDouble(),
        list[1].toDouble(),
        list[2].toDouble(),
    );

    Vector.fromJson(JsonLoader loader) : this.fromList(loader.data);

    String toString() => 'Vector($x, $y, $z)';

    Vector setTo(Vector that) {
        this.x = that.x;
        this.y = that.y;
        this.z = that.z;
        return this;
    }
    Vector setXYZ(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
        return this;
    }

    List<double> asList() => [this.x, this.y, this.z];

    // various ways of computing length
    double get length        => sqrt(lengthSquared);
    double get lengthSquared => this.dot(this);
    double get l1norm        => x.abs() + y.abs() + z.abs();
    double get l2norm        => length;
    double get maxnorm       => max(max(x.abs(), y.abs()), z.abs());

    Vector operator -() => Vector(-this.x, -this.y, -this.z);
    Vector operator +(Vector that) => Vector(this.x + that.x, this.y + that.y, this.z + that.z);
    Vector operator -(Vector that) => Vector(this.x - that.x, this.y - that.y, this.z - that.z);
    Vector operator *(dynamic that) {
        if(that is num) {
            double s = that.toDouble();
            return Vector(this.x * s, this.y * s, this.z * s);
        }
        return Vector(this.x * that.x, this.y * that.y, this.z * that.z);
    }

    Vector operator /(dynamic that) {
        if(that is num) {
            double s = that.toDouble();
            return Vector(this.x / s, this.y / s, this.z / s);
        }
        return Vector(this.x / that.x, this.y / that.y, this.z / that.z);
    }

    Vector normalize() {
        double l = this.length;
        return this.setXYZ(x / l, y / l, z / l);
    }
    Vector asNormalized() => this / this.length;

    // computes dot (inner) product
    double dot(Vector that) => this.x * that.x + this.y * that.y + this.z * that.z;

    // computes cross (outer) product
    Vector cross(Vector that) => Vector(
        this.y * that.z - this.z * that.y,
        this.z * that.x - this.x * that.z,
        this.x * that.y - this.y * that.x,
    );

    // linearly interpolates between two Vectors
    // when factor = 0, from is returned
    // when factor = 1, to is returned
    // when factor = 0.5, the average of from and to is returned
    static Vector lerp(Vector from, Vector to, double factor) => Vector(
        from.x * (1 - factor) + to.x * factor,
        from.y * (1 - factor) + to.y * factor,
        from.z * (1 - factor) + to.z * factor,
    );

    // returns this reflected about given Direction
    Vector reflected(Direction about) {
        var d = about.dot(this);
        return about * (d * 2) - this;
    }
}


class Point extends Vector {
    Point(double x, double y, double z) : super(x, y, z);
    Point.fromList(List p) : super(p[0].toDouble(), p[1].toDouble(), p[2].toDouble());
    Point.fromVector(Vector v) : super(v.x, v.y, v.z);
    Point.fromJson(JsonLoader loader) : this.fromList(loader.data);

    static Point average(Point a, Point b) => Point(
        (a.x + b.x) / 2,
        (a.y + b.y) / 2,
        (a.z + b.z) / 2,
    );

    static Point lerp(Point a, Point b, double factor) => Point(
        a.x * (1 - factor) + b.x * factor,
        a.y * (1 - factor) + b.y * factor,
        a.z * (1 - factor) + b.z * factor,
    );

    String toString() => 'Point($x, $y, $z)';

    Point operator +(Vector that) {
        return Point(this.x + that.x, this.y + that.y, this.z + that.z);
    }
    operator -(Vector that) {
        return Vector(this.x - that.x, this.y - that.y, this.z - that.z);
    }
}


class Direction extends Vector {
    Direction(double x, double y, double z) : super(0,0,0) {
        double l = sqrt(x*x + y*y + z*z);
        this.x = x / l;
        this.y = y / l;
        this.z = z / l;
    }
    Direction.fromVector(Vector that) : super(0,0,0) {
        setTo(that);
    }
    Direction.fromDirection(Direction that) : super(that.x, that.y, that.z);
    Direction.fromNormal(Normal that) : super(that.x, that.y, that.z);
    Direction.fromList(List p) : this(p[0].toDouble(), p[1].toDouble(), p[2].toDouble());
    Direction.fromJson(JsonLoader loader) : this.fromList(loader.data);
    Direction.fromPoints(Point from, Point to) : this.fromVector(to - from);

    Direction.xAxis() : super(1,0,0);
    Direction.yAxis() : super(0,1,0);
    Direction.zAxis() : super(0,0,1);

    String toString() => 'Direction($x, $y, $z)';

    double get length => 1;
    double get lengthSquared => 1;

    Direction normalize() => this;
    Direction asNormalized() => Direction.fromDirection(this);

    Direction setTo(dynamic that) {
        if(that is Direction || that is Normal) {
            // no need to normalize
            this.x = that.x;
            this.y = that.y;
            this.z = that.z;
            return this;
        }
        if(that is Vector) {
            double l = that.length;
            this.x = that.x / l;
            this.y = that.y / l;
            this.z = that.z / l;
            return this;
        }
        if(that is List<double>) {
            double x = that[0];
            double y = that[1];
            double z = that[2];
            double l = sqrt(x*x + y*y + z*z);
            this.x = x / l;
            this.y = y / l;
            this.z = z / l;
            return this;
        }
        throw Exception("Unhandled parameter type");
    }

    Direction operator -() => Direction(-this.x, -this.y, -this.z);
    Vector operator *(dynamic that) {
        if(that is num) {
            return Vector(this.x * that, this.y * that, this.z * that);
        }
        if(that is Vector) {
            return Vector(this.x * that.x, this.y * that.y, this.z * that.z);
        }
        throw Exception("Unhandled parameter type");
    }

    // returns this reflected about given Direction
    Direction reflected(Direction about) {
        var d = about.dot(this);
        return Direction.fromVector(about * (d * 2) - this);
    }
}


class Normal extends Vector {
    Normal(double x, double y, double z) : super(0,0,0) {
        double l = sqrt(x*x + y*y + z*z);
        this.x = x / l;
        this.y = y / l;
        this.z = z / l;
    }
    Normal.fromVector(Vector that) : super(0,0,0) {
        setTo(that);
    }
    Normal.fromDirection(Direction that) : super(that.x, that.y, that.z);
    Normal.fromNormal(Normal that) : super(that.x, that.y, that.z);
    Normal.fromList(List p) : this(p[0].toDouble(), p[1].toDouble(), p[2].toDouble());
    Normal.fromJson(JsonLoader loader) : this.fromList(loader.data);
    Normal.fromPoints(Point from, Point to) : this.fromVector(to - from);

    String toString() => 'Normal($x, $y, $z)';

    double get length => 1;
    double get lengthSquared => 1;
    Normal normalize() => this;
    Normal asNormalized() => Normal.fromNormal(this);

    Normal setTo(dynamic that) {
        if(that is Direction || that is Normal) {
            this.x = that.x;
            this.y = that.y;
            this.z = that.z;
            return this;
        }
        if(that is Vector) {
            double l = that.length;
            this.x = that.x / l;
            this.y = that.y / l;
            this.z = that.z / l;
            return this;
        }
        if(that is List) {
            double x = that[0];
            double y = that[1];
            double z = that[2];
            double l = sqrt(x*x + y*y + z*z);
            this.x = x / l;
            this.y = y / l;
            this.z = z / l;
            return this;
        }
        throw Exception("Unhandled parameter type");
    }

    // returns this reflected about given Direction
    Normal reflected(Direction about) {
        var d = about.dot(this);
        return Normal.fromVector(about * (d * 2) - this);
    }
}


class Ray {
    static const ray_epsilon  = 10e-5;
    static const ray_infinity = double.infinity;

    Point e;
    Direction d;
    double t_min;
    double t_max;

    Ray(this.e, this.d, {this.t_min=ray_epsilon, this.t_max=ray_infinity});

    String toString() => 'Ray(e:$e, d:$d)';

    Ray.fromPoints(Point from, Point to) {
        Vector delta = to - from;
        this.e = from;
        this.d = Direction.fromVector(delta);
        this.t_min = ray_epsilon;
        this.t_max = delta.length;
    }

    Point eval(double t) => e + d * t;
    Point evalClamped(double t) => eval(max(this.t_min, min(this.t_max, t)));

    bool valid(double t) => this.t_min <= t && t <= this.t_max;
}


class Color {
    double red   = 0;
    double green = 0;
    double blue  = 0;
    double alpha = 1;

    Color(this.red, this.green, this.blue, [this.alpha=1]);
    Color.fromList(List list) {
        this.red   = list[0]?.toDouble() ?? 0;
        this.green = list[1]?.toDouble() ?? 0;
        this.blue  = list[2]?.toDouble() ?? 0;
        this.alpha = list.length >= 4 ? list[3].toDouble() : 1;
    }
    Color.fromColor(Color that) {
        this.red   = that.red;
        this.green = that.green;
        this.blue  = that.blue;
        this.alpha = that.alpha;
    }
    Color.fromRGBColor(RGBColor that, {double alpha}) {
        this.red   = that.red;
        this.green = that.green;
        this.blue  = that.blue;
        this.alpha = alpha ?? 1;
    }
    Color.fromHSL(double hue, double saturation, double lightness, [this.alpha=1]) {
        // hue in [0, 360), saturation in [0, 1], lightness in [0, 1]
        double c = (1.0 - (2.0 * lightness - 1.0).abs()) * saturation;
        double x = c * (1.0 - ((hue / 60.0) % 2.0 - 1.0).abs());
        double m = lightness - c / 2.0;
        double r = 0.0, g = 0.0, b = 0.0;
             if(hue <  60) { r = c; g = x; b = 0; }
        else if(hue < 120) { r = x; g = c; b = 0; }
        else if(hue < 180) { r = 0; g = c; b = x; }
        else if(hue < 240) { r = 0; g = x; b = c; }
        else if(hue < 300) { r = x; g = 0; b = c; }
        else               { r = c; g = 0; b = x; }
        this.red   = r + m;
        this.green = g + m;
        this.blue  = b + m;
    }
    Color.fromJson(JsonLoader loader) : this.fromList(loader.data);

    // a few standard colors
    Color.black()       : this(0, 0, 0, 1);
    Color.white()       : this(1, 1, 1, 1);
    Color.red()         : this(1, 0, 0, 1);
    Color.yellow()      : this(1, 1, 0, 1);
    Color.green()       : this(0, 1, 0, 1);
    Color.cyan()        : this(0, 1, 1, 1);
    Color.blue()        : this(0, 0, 1, 1);
    Color.magenta()     : this(1, 0, 1, 1);
    Color.transparent() : this(0, 0, 0, 0);

    Color copy()    => Color(red, green, blue, alpha);
    Color toColor() => Color(red, green, blue, alpha);

    static Color average(Color a, Color b) => Color(
        (a.red   + b.red)   / 2,
        (a.green + b.green) / 2,
        (a.blue  + b.blue)  / 2,
        (a.alpha + b.alpha) / 2,
    );

    static Color lerp(Color a, Color b, double factor) => Color(
        a.red   * (1 - factor) + b.red   * factor,
        a.green * (1 - factor) + b.green * factor,
        a.blue  * (1 - factor) + b.blue  * factor,
        a.alpha * (1 - factor) + b.alpha * factor,
    );

    String toString() => 'Color($red, $green, $blue, $alpha)';

    int redAsInt({int maxv=255})   => max(0, min(maxv, (red   * (maxv + 0.999)).toInt()));
    int greenAsInt({int maxv=255}) => max(0, min(maxv, (green * (maxv + 0.999)).toInt()));
    int blueAsInt({int maxv=255})  => max(0, min(maxv, (blue  * (maxv + 0.999)).toInt()));
    int alphaAsInt({int maxv=255}) => max(0, min(maxv, (alpha * (maxv + 0.999)).toInt()));

    double get r => red;
    double get g => green;
    double get b => blue;
    double get a => alpha;

    double get hue {
        var cmax = max(max(red, green), blue);
        var cmin = min(min(red, green), blue);
        var delta = cmax - cmin;
        if(delta == 0)      return 0;
        if(cmax == red)     return 60 * ((green - blue)  / delta % 6);
        if(cmax == green)   return 60 * ((blue  - red)   / delta + 2);
                            return 60 * ((red   - green) / delta + 4);
    }
    double get saturation {
        var cmax = max(max(red, green), blue);
        var cmin = min(min(red, green), blue);
        var delta = cmax - cmin;
        if(delta == 0) return 0;
        return delta / (1.0 - (2.0 * lightness - 1.0).abs());
    }
    double get lightness => 0.5 * (
        max(max(red, green), blue) + min(min(red, green), blue)
    );

    // returns true if this is (nearly) black
    bool get isBlack       => red <= 10e-5 && green <= 10e-5 && blue <= 10e-5;

    // returns true if this is (nearly) perfectly transparent
    bool get isTransparent => alpha <= 10e-5;

    // returns true if this is (nearly) perfectly opaque
    bool get isOpaque      => alpha >= 1 - 10e-5;


    Color operator +(Color that) => Color(
        this.red   + that.red,
        this.green + that.green,
        this.blue  + that.blue,
        this.alpha + that.alpha,
    );
    Color operator *(dynamic that) {
        if(that is Color) {
            return Color(
                this.red   * that.red,
                this.green * that.green,
                this.blue  * that.blue,
                this.alpha * that.alpha,
            );
        }
        if(that is num) {
            return Color(
                this.red   * that,
                this.green * that,
                this.blue  * that,
                this.alpha * that,
            );
        }
        throw Exception("Unhandled parameter type");
    }
    Color operator /(dynamic that) {
        if(that is Color) {
            return Color(
                this.red   / that.red,
                this.green / that.green,
                this.blue  / that.blue,
                this.alpha / that.alpha,
            );
        }
        if(that is num) {
            return Color(
                this.red   / that,
                this.green / that,
                this.blue  / that,
                this.alpha / that,
            );
        }
        throw Exception("Unhandled parameter type");
    }
}

class RGBColor {
    double red   = 0;
    double green = 0;
    double blue  = 0;

    RGBColor(this.red, this.green, this.blue);
    RGBColor.fromList(List list) {
        this.red   = list[0]?.toDouble() ?? 0;
        this.green = list[1]?.toDouble() ?? 0;
        this.blue  = list[2]?.toDouble() ?? 0;
    }
    RGBColor.fromColor(Color that) {
        this.red   = that.red;
        this.green = that.green;
        this.blue  = that.blue;
    }
    RGBColor.fromRGBColor(RGBColor that) {
        this.red   = that.red;
        this.green = that.green;
        this.blue  = that.blue;
    }
    RGBColor.fromHSL(double hue, double saturation, double lightness) {
        // hue in [0, 360), saturation in [0, 1], lightness in [0, 1]
        double c = (1.0 - (2.0 * lightness - 1.0).abs()) * saturation;
        double x = c * (1.0 - ((hue / 60.0) % 2.0 - 1.0).abs());
        double m = lightness - c / 2.0;
        double r = 0.0, g = 0.0, b = 0.0;
             if(hue <  60) { r = c; g = x; b = 0; }
        else if(hue < 120) { r = x; g = c; b = 0; }
        else if(hue < 180) { r = 0; g = c; b = x; }
        else if(hue < 240) { r = 0; g = x; b = c; }
        else if(hue < 300) { r = x; g = 0; b = c; }
        else               { r = c; g = 0; b = x; }
        this.red   = r + m;
        this.green = g + m;
        this.blue  = b + m;
    }
    RGBColor.fromJson(JsonLoader loader) : this.fromList(loader.data);

    // a few standard colors
    RGBColor.black()       : this(0, 0, 0);
    RGBColor.white()       : this(1, 1, 1);
    RGBColor.red()         : this(1, 0, 0);
    RGBColor.yellow()      : this(1, 1, 0);
    RGBColor.green()       : this(0, 1, 0);
    RGBColor.cyan()        : this(0, 1, 1);
    RGBColor.blue()        : this(0, 0, 1);
    RGBColor.magenta()     : this(1, 0, 1);

    RGBColor copy() => RGBColor(red, green, blue);
    Color toColor() => Color(red, green, blue, 1);

    static RGBColor average(RGBColor a, RGBColor b) => RGBColor(
        (a.red   + b.red)   / 2,
        (a.green + b.green) / 2,
        (a.blue  + b.blue)  / 2,
    );

    static RGBColor lerp(RGBColor a, RGBColor b, double factor) => RGBColor(
        a.red   * (1 - factor) + b.red   * factor,
        a.green * (1 - factor) + b.green * factor,
        a.blue  * (1 - factor) + b.blue  * factor,
    );

    String toString() => 'RGBColor($red, $green, $blue)';

    int redAsInt({int maxv=255})   => max(0, min(maxv, (red   * (maxv + 0.999)).toInt()));
    int greenAsInt({int maxv=255}) => max(0, min(maxv, (green * (maxv + 0.999)).toInt()));
    int blueAsInt({int maxv=255})  => max(0, min(maxv, (blue  * (maxv + 0.999)).toInt()));
    int alphaAsInt({int maxv=255}) => maxv;

    double get alpha => 1;

    double get r => red;
    double get g => green;
    double get b => blue;
    double get a => alpha;

    double get hue {
        var cmax = max(max(red, green), blue);
        var cmin = min(min(red, green), blue);
        var delta = cmax - cmin;
        if(delta == 0)      return 0;
        if(cmax == red)     return 60 * ((green - blue)  / delta % 6);
        if(cmax == green)   return 60 * ((blue  - red)   / delta + 2);
                            return 60 * ((red   - green) / delta + 4);
    }
    double get saturation {
        var cmax = max(max(red, green), blue);
        var cmin = min(min(red, green), blue);
        var delta = cmax - cmin;
        if(delta == 0) return 0;
        return delta / (1.0 - (2.0 * lightness - 1.0).abs());
    }
    double get lightness => 0.5 * (
        max(max(red, green), blue) + min(min(red, green), blue)
    );

    // returns true if this is (nearly) black
    bool get isBlack       => red <= 10e-5 && green <= 10e-5 && blue <= 10e-5;


    RGBColor operator +(RGBColor that) => RGBColor(
        this.red   + that.red,
        this.green + that.green,
        this.blue  + that.blue,
    );
    RGBColor operator *(dynamic that) {
        if(that is RGBColor) {
            return RGBColor(
                this.red   * that.red,
                this.green * that.green,
                this.blue  * that.blue,
            );
        }
        if(that is num) {
            return RGBColor(
                this.red   * that,
                this.green * that,
                this.blue  * that,
            );
        }
        throw Exception("Unhandled parameter type");
    }
    RGBColor operator /(dynamic that) {
        if(that is RGBColor) {
            return RGBColor(
                this.red   / that.red,
                this.green / that.green,
                this.blue  / that.blue,
            );
        }
        if(that is num) {
            return RGBColor(
                this.red   / that,
                this.green / that,
                this.blue  / that,
            );
        }
        throw Exception("Unhandled parameter type");
    }
}


class Frame {
    Point o = Point(0, 0, 0);
    Direction x = Direction.xAxis();
    Direction y = Direction.yAxis();
    Direction z = Direction.zAxis();

    Frame({Point o, Direction x, Direction y, Direction z, Normal n}) {
        if(n != null) z = Direction.fromNormal(n);

        // count how many axes (x,y,z) were provided
        int axes = (x == null ? 0 : 1) + (y == null ? 0 : 1) + (z == null ? 0 : 1);
        // make sure that axes (x,y,z) are orthonormal
        if(axes == 0) {
            x = Direction.xAxis();
            y = Direction.yAxis();
            z = Direction.zAxis();
        } else if(axes == 1) {
            if(x != null) {
                y = Direction(-x.x + 3.14, x.y + 42, x.z - 1.61);
                z = Direction.fromVector(x.cross(y));
                y = Direction.fromVector(z.cross(x));
            } else if(y != null) {
                x = Direction(-y.x + 3.14, y.y + 42, y.z - 1.61);
                z = Direction.fromVector(x.cross(y));
                x = Direction.fromVector(y.cross(z));
            } else {
                x = Direction(-z.x + 3.14, z.y + 42, z.z - 1.61);
                y = Direction.fromVector(-x.cross(z));
                x = Direction.fromVector(y.cross(z));
            }
        } else if(axes == 2) {
            if(x != null && y != null) {
                z = Direction.fromVector(x.cross(y));
                y = Direction.fromVector(z.cross(x));
            } else if(x != null && z != null) {
                y = Direction.fromVector(z.cross(x));
                x = Direction.fromVector(y.cross(z));
            } else {
                x = Direction.fromVector(y.cross(z));
                y = Direction.fromVector(z.cross(x));
            }
        } else {
            x = Direction.fromVector(y.cross(z));
            y = Direction.fromVector(z.cross(x));
        }
        this.o = o ?? Point(0, 0, 0);
        this.x = x;
        this.y = y;
        this.z = z;
    }

    Frame.lookAt(Point eye, Point target, Direction up) {
        o = eye;
        z = Direction.fromVector(eye - target);
        x = Direction.fromVector(up.cross(z));
        y = Direction.fromVector(z.cross(x));
    }

    Frame.fromJson(JsonLoader loader) : this(
        o:loader.loadObject('o', (d)=>Point.fromJson(d)),
        x:loader.loadObject('x', (d)=>Direction.fromJson(d)),
        y:loader.loadObject('y', (d)=>Direction.fromJson(d)),
        z:loader.loadObject('z', (d)=>Direction.fromJson(d)),
    );

    String toString() => 'Frame(o:$o, x:$x, y:$y, z:$z)';

    // the following functions transform entities from
    // world to local (w2l) or local to world (l2w)

    Point w2lPoint(Point p) {
        Vector d = p - o;
        return Point(x.dot(d), y.dot(d), z.dot(d));
    }
    Point l2wPoint(Point p) {
        return o + x * p.x + y * p.y + z * p.z;
    }

    Vector w2lVector(Vector v) {
        return Vector(x.dot(v), y.dot(v), z.dot(v));
    }
    Vector l2wVector(Vector v) {
        return x * v.x + y * v.y + z * v.z;
    }

    Direction w2lDirection(Direction v) {
        return Direction(x.dot(v), y.dot(v), z.dot(v));
    }
    Direction l2wDirection(Direction v) {
        return Direction.fromVector(x * v.x + y * v.y + z * v.z);
    }

    Normal w2lNormal(Normal v) {
        return Normal(x.dot(v), y.dot(v), z.dot(v));
    }
    Normal l2wNormal(Normal v) {
        return Normal.fromVector(x * v.x + y * v.y + z * v.z);
    }

    Ray w2lRay(Ray r) {
        Point e = w2lPoint(r.e);
        Direction d = w2lDirection(r.d);
        double t_max = double.infinity;
        if(r.t_max < double.infinity) {
            Point m = w2lPoint(r.eval(r.t_max));
            t_max = (e - m).length;
        }
        return Ray(e, d, t_max:t_max);
    }
    Ray l2wRay(Ray r) {
        Point e = l2wPoint(r.e);
        Direction d = l2wDirection(r.d);
        double t_max = double.infinity;
        if(r.t_max < double.infinity) {
            Point m = l2wPoint(r.eval(r.t_max));
            t_max = (e - m).length;
        }
        return Ray(e, d, t_max:t_max);
    }
}
