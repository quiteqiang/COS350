import 'dart:io';
import 'dart:math';

import 'common/image.dart';
import 'common/jsonloader.dart';
import 'common/maths.dart';
import 'common/scene.dart';

var writeImageInBinary = true;
var overrideResolution = null; // Size2i(32, 32);
var overrideSamples    = null; // 1

List<String> scenePaths = [
    'scenes/P04_00_triangle.json',
    'scenes/P04_01_scene.json',
    'scenes/P04_02_animation001.json',
    'scenes/P04_02_animation002.json',
    'scenes/P04_02_animation003.json',
    'scenes/P04_02_animation004.json',
];



// Determines if given ray intersects any surface in the scene.
// If ray does not intersect anything, null is returned.
// Otherwise, details of first intersection are returned as an `Intersection` object.
Intersection intersectRayScene(Scene scene, Ray ray) {
    Intersection intersection;
    var t_closest = double.infinity;

    for(var surface in scene.surfaces) {
        Point o = surface.frame.o;

        switch(surface.type) {
            case 'sphere': {
                Vector oe = ray.e - o;
                double a = ray.d.lengthSquared;
                double b = 2.0 * ray.d.dot(oe);
                double c = oe.lengthSquared - surface.size * surface.size;
                double d = b * b - 4 * a * c;
                if(d < 0) continue;     // ray misses sphere

                double sqrtd = sqrt(d);
                double t_min = (-b - sqrtd) / (2 * a);
                double t_max = (-b + sqrtd) / (2 * a);

                if(ray.valid(t_min) && t_min < t_closest) {
                    t_closest = t_min;
                    Point p = ray.eval(t_min);
                    Normal n = Normal.fromPoints(o, p);
                    Frame frame = Frame(o:p, n:n);
                    intersection = Intersection(frame, surface.material, t_closest);
                }

                if(ray.valid(t_max) && t_max < t_closest) {
                    t_closest = t_max;
                    Point p = ray.eval(t_max);
                    Normal n = Normal.fromPoints(o, p);
                    Frame frame = Frame(o:p, n:n);
                    intersection = Intersection(frame, surface.material, t_closest);
                }
            } break;

            case 'quad': {
                double den = ray.d.dot(surface.frame.z);
                if(den.abs() <= 10e-8) continue;    // ray is parallel to plane
                double t = (o - ray.e).dot(surface.frame.z) / den;
                if(ray.valid(t) && t < t_closest) {
                    Point p = ray.eval(t);
                    // determine if p is inside quad
                    Point pl = surface.frame.w2lPoint(p);
                    if(pl.maxnorm > surface.size) continue;
                    t_closest = t;
                    Frame frame = Frame(o:p, x:surface.frame.x, y:surface.frame.y, z:surface.frame.z);
                    intersection = Intersection(frame, surface.material, t_closest);
                }
            } break;
        }
    }

    for(var mesh in scene.meshes) {
        var ray_local = mesh.frame.w2lRay(ray); // transform ray to be local
        var el = ray_local.e;
        var dl = ray_local.d;

        // test if ray intersects bounding sphere
        double b = 2.0 * dl.dot(el);
        double c = el.lengthSquared - mesh.bssize * mesh.bssize;
        double d = b * b - 4.0 * c;
        if(d < 0.0) continue;     // ray misses sphere
        double sqrtd = sqrt(d);
        double t_min = (-b - sqrtd) / 2.0;
        double t_max = (-b + sqrtd) / 2.0;
        if(!ray.valid(t_min) && !ray.valid(t_max)) continue;

        for(var i_face = 0; i_face < mesh.faces.length; i_face += 3) {
            // https://gfx.cse.taylor.edu/courses/cos350/slides/03_Raytracing.md.html?scale#sect029
            var a = mesh.verts[mesh.faces[i_face+0]];
            var b = mesh.verts[mesh.faces[i_face+1]];
            var c = mesh.verts[mesh.faces[i_face+2]];
            var a_ = a  - c;    // a'
            var b_ = b  - c;    // b'
            var e_ = el - c;    // e'
            var t     = e_.cross(a_).dot(b_) / dl.cross(b_).dot(a_);
            var alpha = dl.cross(b_).dot(e_) / dl.cross(b_).dot(a_);
            var beta  = e_.cross(a_).dot(dl) / dl.cross(b_).dot(a_);
            var gamma = 1.0 - alpha - beta;
            if(!ray_local.valid(t) || t >= t_closest) continue;
            if(alpha < 0 || beta < 0 || alpha + beta >= 1) continue;
            t_closest = t;
            Point  pl = ray_local.eval(t);
            Normal nl = Normal.fromVector(
                mesh.norms[mesh.faces[i_face+0]] * alpha +
                mesh.norms[mesh.faces[i_face+1]] * beta  +
                mesh.norms[mesh.faces[i_face+2]] * gamma
            );
            Frame frame = Frame(o:mesh.frame.l2wPoint(pl), n:mesh.frame.l2wNormal(nl));
            intersection = Intersection(frame, mesh.material, t_closest);
        }
    }

    return intersection;
}

// Computes irradiance (as RGBColor) from scene along ray
RGBColor irradiance(Scene scene, Ray ray, int depth) {
    Intersection intersection = intersectRayScene(scene, ray);
    if(intersection == null) return scene.backgroundIntensity;
    if(depth <= 0) return RGBColor.black();

    Point     p  = intersection.o;
    Direction v  = -ray.d;
    RGBColor  kd = intersection.material.kd;
    RGBColor  ks = intersection.material.ks;
    double    n  = intersection.material.n;
    RGBColor  kr = intersection.material.kr;

    // start accumulating irradiance
    RGBColor c = kd * scene.ambientIntensity;

    for(var light in scene.lights) {
        Vector ps = light.frame.o - p;
        double dist = ps.length;
        Direction l = Direction.fromVector(ps);
        Ray shadowRay = Ray(p, l, t_max:dist);
        if(intersectRayScene(scene, shadowRay) != null) continue;
        Direction h = Direction.fromVector(l + v);
        RGBColor L = light.intensity / (dist * dist);
        c += L * (kd + ks * pow(max(0, intersection.n.dot(h)), n) ) * max(intersection.n.dot(l), 0.0);
    }

    if(!kr.isBlack) {
        Direction r = v.reflected(intersection.frame.z);
        Ray reflectRay = Ray(p, r);
        RGBColor rc = irradiance(scene, reflectRay, depth-1);
        c += kr * rc;
    }

    return c;
}

// Computes image of scene using basic Whitted raytracer.
Image raytraceScene(Scene scene) {
    var image = Image(scene.resolution.width, scene.resolution.height);

    Frame cameraFrame = scene.camera.frame;
    int samples = max(Num.sqrtInt(scene.pixelSamples), 1);
    for(var x = 0; x < scene.resolution.width; x++) {
        for(var y = 0; y < scene.resolution.height; y++) {
            RGBColor c = RGBColor.black();
            for(var ii = 0; ii < samples; ii++) {
                for(var jj = 0; jj < samples; jj++) {
                    double u = (x + (ii + 0.5) / samples) / scene.resolution.width;
                    double v = 1.0 - (y + (jj + 0.5) / samples) / scene.resolution.height;
                    Point o = cameraFrame.o;
                    Point q = o
                        + cameraFrame.x * (scene.camera.sensorSize.width  * (u - 0.5))
                        + cameraFrame.y * (scene.camera.sensorSize.height * (v - 0.5))
                        + cameraFrame.z * -scene.camera.sensorDistance;
                    Ray camera_ray = Ray(o, Direction.fromPoints(o, q));
                    c += irradiance(scene, camera_ray, 5);
                }
            }
            c /= samples * samples;
            image.setPixel(x, y, c);
        }
    }

    return image;
}

void main() {
    // Make sure images folder exists, because this is where all generated images will be saved
    Directory('images').createSync();
    print("-------");
    for(String scenePath in scenePaths) {
        // Determine where to write the rendered image.
        // NOTE: the following line is not safe, but it is fine for this project.
        var ppmPath = scenePath.replaceAll('.json', '.ppm').replaceAll('scenes/', 'images/');

        print('Scene: $scenePath');
        print('    output image: $ppmPath');
        print('    loading...');
        var loader = JsonLoader(path:scenePath);    // load json file
        var scene = Scene.fromJson(loader);         // parse json file as Scene


        // override scene's resolution
        if(overrideResolution != null) {
            print('    overriding resolution: $overrideResolution');
            scene.resolution = overrideResolution;
        }
        if(overrideSamples != null) {
            print('    overriding pixelSamples: $overrideSamples');
            scene.pixelSamples = overrideSamples;
        }

        print('    tracing rays...');
        Stopwatch watch = Stopwatch()..start();             // create Stopwatch, then start it (NOTE: keep the two ..)
        var image = raytraceScene(scene);                   // raytrace the scene
        var seconds = watch.elapsedMilliseconds / 1000.0;   // determine elapsed time in seconds

        image.saveImage(ppmPath, asBinary:writeImageInBinary);  // write raytraced image to PPM file

        // report details to console
        print('    time:  $seconds seconds');               // note: includes time for saving file
    }
}
