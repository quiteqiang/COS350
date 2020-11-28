import 'dart:math';
import 'jsonloader.dart';
import 'maths.dart';


class Intersection {
    Frame frame;
    Material material;
    double distance;

    Intersection(this.frame, this.material, this.distance);

    // convenience getters
    Point  get o => frame.o;
    Normal get n => Normal.fromDirection(frame.z);
}


class Material {
    var kd = RGBColor.white();
    var ks = RGBColor.black();
    var n  = 10.0;
    var kr = RGBColor.black();
    var uv_mapping = "";

    Material();

    Material.fromJson(JsonLoader loader) {
        kd = loader.loadObject('kd', (d)=>RGBColor.fromJson(d)) ?? kd;
        ks = loader.loadObject('ks', (d)=>RGBColor.fromJson(d)) ?? ks;
        n  = loader.loadDouble('n')                             ?? n;
        kr = loader.loadObject('kr', (d)=>RGBColor.fromJson(d)) ?? kr;
        uv_mapping = loader.loadString('uv_mapping')            ?? uv_mapping;
    }
}


class Surface {
    var type     = 'sphere';
    var size     = 1.0;
    var frame    = Frame();
    var material = Material();

    Surface();

    Surface.fromJson(JsonLoader loader) {
        type       = loader.loadString('type')                                ?? type;
        size       = loader.loadDouble('size')                                ?? size;
        frame      = loader.loadObject('frame',    (d)=>Frame.fromJson(d))    ?? frame;
        material   = loader.loadObject('material', (d)=>Material.fromJson(d)) ?? material;
    }
}


// <!--
class Mesh {
    var frame    = Frame();
    var material = Material();
    var verts    = [Point( 0, 0, 0), Point( 1, 0, 0), Point( 0, 1, 0)];
    var norms    = [Normal(0, 0, 1), Normal(0, 0, 1), Normal(0, 0, 1)];
    var faces    = [0, 1, 2];


    Mesh() {
        recompute_bounding_sphere();
    }

    Mesh.fromJson(JsonLoader loader) {
        frame    = loader.loadObject(        'frame',    (d)=>Frame.fromJson(d))    ?? frame;
        material = loader.loadObject(        'material', (d)=>Material.fromJson(d)) ?? material;
        verts    = loader.loadListOf<Point>( 'verts',    (d)=>Point.fromJson(d))    ?? verts;
        norms    = loader.loadListOf<Normal>('norms',    (d)=>Normal.fromJson(d))   ?? norms;
        faces    = loader.loadListOf<int>(   'faces',    (d)=>NumInt.fromJson(d))   ?? faces;
        recompute_bounding_sphere();
    }

    // determine size of a bounding sphere that fits the mesh
    var bssize = 0.0;
    void recompute_bounding_sphere() {
        bssize = 0.0;
        for(Point p in verts) bssize = max(bssize, p.length);
    }
}
// -->


class Light {
    var frame     = Frame();
    var intensity = RGBColor(1, 1, 1);
    var size      = 0.01;

    Light();

    Light.fromJson(JsonLoader loader) {
        frame     = loader.loadObject('frame',     (d)=>Frame.fromJson(d))    ?? frame;
        intensity = loader.loadObject('intensity', (d)=>RGBColor.fromJson(d)) ?? intensity;
        size      = loader.loadDouble('size')                                 ?? size;
    }
}


class Camera {
    var sensorSize     = Size2(1, 1);
    var sensorDistance = 1.0;
    var eye            = Point(0, 0, 1);
    var target         = Point(0, 0, 0);
    var up             = Direction(0, 1, 0);

    Camera();

    Camera.fromJson(JsonLoader loader) {
        sensorSize     = loader.loadObject('sensorSize',     (d)=>Size2.fromJson(d))     ?? sensorSize;
        sensorDistance = loader.loadObject('sensorDistance', (d)=>NumDouble.fromJson(d)) ?? sensorDistance;
        eye            = loader.loadObject('eye',            (d)=>Point.fromJson(d))     ?? eye;
        target         = loader.loadObject('target',         (d)=>Point.fromJson(d))     ?? target;
        up             = loader.loadObject('up',             (d)=>Direction.fromJson(d)) ?? up;
    }

    // convenience getter (note: _frame is cached, based on eye, target, up)
    Frame _frame;
    Frame get frame {
        return _frame ??= Frame.lookAt(eye, target, up);
    }
}


class Scene {
    var camera              = Camera();
    var resolution          = Size2i(512, 512);
    var pixelSamples        = 1;
    var backgroundIntensity = RGBColor(0.2, 0.2, 0.2);
    var ambientIntensity    = RGBColor(0.2, 0.2, 0.2);
    var lights              = [ Light() ];
    var surfaces            = [ Surface() ];
    // <!--
    var meshes              = [];
    // -->

    Scene();

    Scene.fromJson(JsonLoader loader) {
        camera              = loader.loadObject('camera',              (d)=>Camera.fromJson(d))   ?? camera;
        resolution          = loader.loadObject('resolution',          (d)=>Size2i.fromJson(d))   ?? resolution;
        pixelSamples        = loader.loadInt   ('pixelSamples')                                   ?? pixelSamples;
        backgroundIntensity = loader.loadObject('backgroundIntensity', (d)=>RGBColor.fromJson(d)) ?? backgroundIntensity;
        ambientIntensity    = loader.loadObject('ambientIntensity',    (d)=>RGBColor.fromJson(d)) ?? ambientIntensity;
        lights              = loader.loadListOf<Light>('lights',       (d)=>Light.fromJson(d))    ?? lights;
        surfaces            = loader.loadListOf<Surface>('surfaces',   (d)=>Surface.fromJson(d))  ?? surfaces;
        // <!--
        meshes              = loader.loadListOf<Mesh>('meshes',        (d)=>Mesh.fromJson(d))     ?? meshes;
        // -->
    }
}
