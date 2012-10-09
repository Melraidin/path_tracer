#include <cmath>
#include <cstdlib>
#include <ctime>
#include <cfloat>
#include <iostream>
#include <sstream>
#include <boost/thread.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>
#include <fstream>

boost::mutex buffer_mutex;

using namespace std;

// Generate a random number between 0 and 1
// return a uniform number in [0,1].
double unifRand()
{
    return rand() / double(RAND_MAX);
}

// Generate a random number in a real interval.
// param a one end point of the interval
// param b the other end of the interval
// return a inform rand numberin [a,b].
double unifRand(double a, double b)
{
    return (b-a)*unifRand() + a;
}

// Generate a random integer between 1 and a given value.
// param n the largest value 
// return a uniform random value in [1,...,n]
long unifRand(long n)
{
    if (n < 0) n = -n;
    if (n==0) return 0;
    /* There is a slight error in that this code can produce a return value of n+1
    **
    **  return long(unifRand()*n) + 1;
    */
    //Fixed code
    long guard = (long) (unifRand() * n) +1;
    return (guard > n)? n : guard;
}


class V3 {
public:
    double x, y, z;

    V3(void) {
        x = 0.0L;
        y = 0.0L;
        z = 0.0L;
    }

    V3(double ix, double iy, double iz) {
        x = ix;
        y = iy;
        z = iz;
    }

    V3 add(V3 v) {
        return V3(x + v.x, y + v.y, z + v.z);
    }

    V3 iadd(V3 v) {
        x += v.x;
        y += v.y;
        z += v.z;
    }

    V3 sub(V3 v) {
        return V3(x - v.x, y - v.y, z - v.z);
    }

    V3 mul(V3 v) {
        return V3(x * v.x, y * v.y, z * v.z);
    }

    V3 div(V3 v) {
        return V3(x / v.x, y / v.y, z / v.z);
    }

    V3 muls(double s) {
        return V3(x * s, y * s, z * s);
    }

    V3 divs(double s) {
        return muls(1.0L / s);
    }

    double dot(V3 v) {
        return x * v.x + y * v.y + z * v.z;
    }

    V3 normalize(void) {
        return divs(sqrt(dot(*this)));
    }
};

V3 getRandomNormalInHemisphere(V3 v) {
    V3 v2(0.0L, 0.0L, 0.0L);
    do {
        v2 = V3(unifRand()*2.0L-1.0L, unifRand()*2.0L-1.0L, unifRand()*2.0L-1.0L);
    } while (v2.dot(v2) > 1.0L);

    v2.normalize();
    if (v2.dot(v) < 0.0L) {
        return v2.muls(-1.0L);
    }

    return v2;
}


class Ray {
public:
    V3 origin;
    V3 direction;

    Ray(V3 iorigin, V3 idirection) {
        origin = iorigin;
        direction = idirection;
    }
};


class Camera {
public:
    V3 origin;
    V3 topleft;
    V3 topright;
    V3 bottomleft;
    V3 xd;
    V3 yd;

    Camera(void) {
    }

    Camera(V3 iorigin, V3 itopleft, V3 itopright, V3 ibottomleft) {
        origin = iorigin;
        topleft = itopleft;
        topright = itopright;
        bottomleft = ibottomleft;

        xd = topright.sub(topleft);
        yd = bottomleft.sub(topleft);
    }

    Ray getRay(double x, double y) {
        V3 p = topleft.add(xd.muls(x)).add(yd.muls(y));
        return Ray(origin, p.sub(origin).normalize());
    }
};


class Shape {
public:
    virtual double intersect(Ray) {
        // cout << "Shape.intersect()" << endl;
    }

    virtual V3 getNormal(V3) {}
};


class Plane : public Shape {
public:
    V3 center;
    V3 normal;

    Plane(V3 icenter, V3 inormal) {
        center = icenter;
        normal = inormal.normalize();
    }

    double intersect(Ray r) {
        double n_dot_u = normal.dot(r.direction);

        if ((n_dot_u > -0.00001L) && (n_dot_u < 0.00001L)) {
            return -1.0L;
        }

        double n_dot_p0 = normal.dot(center.sub(r.origin));

        return n_dot_p0 / n_dot_u;
    }
    V3 getNormal(V3 point) {
        return normal;
    }
};


class Sphere : public Shape {
public:
    V3 center;
    double radius;
    double radius2;

    Sphere(V3 icenter, double iradius) {
        center = icenter;
        radius = iradius;
        radius2 = radius * radius;
    }

    double intersect(Ray r) {
        // cout << "Sphere intersect()" << endl;
        V3 distance = r.origin.sub(center);
        double b = distance.dot(r.direction);
        double c = distance.dot(distance) - radius2;
        double d = (b * b) - c;

        if (d > 0.0L) {
            return -b - sqrt(d);
        } else {
            return -1.0L;
        }
    }

    V3 getNormal(V3 point) {
        return point.sub(center).normalize();
    }
};


class Material {
public:
    V3 color;
    V3 emission;

    Material(void) {
        color = V3(0.0L, 0.0L, 0.0L);
        emission = V3(0.0L, 0.0L, 0.0L);
    };

    Material(V3 icolor, V3 iemission) {
        color = icolor;
        emission = iemission;
    }

    Material(V3 icolor) {
        color = icolor;
        emission = V3(0.0L, 0.0L, 0.0L);
    }

    virtual V3 bounce(Ray ray, V3 inormal) {
        // cout << "material bounce\n";
        return getRandomNormalInHemisphere(inormal);
    }
};


class Chrome : public Material {
public:
    Chrome(V3 icolor) : Material(icolor) {}

    V3 bounce(Ray ray, V3 inormal) {
        // cout << "glass bounce\n";
        double theta1 = fabs(ray.direction.dot(inormal));
        return ray.direction.add(inormal.muls(theta1 * 2.0L));
    }
};


class Glass : public Material {
public:
    double ior;
    double reflection;

    Glass(V3 icolor, double iior, double ireflection) : Material(icolor) {
        ior = iior;
        reflection = ireflection;
    }

    V3 bounce(Ray ray, V3 normal) {
        // cout << "chrome bounce\n";
        double theta1 = fabs(ray.direction.dot(normal));
        double internalIndex, externalIndex;
        if (theta1 >= 0.0L) {
            internalIndex = ior;
            externalIndex = 1.0L;
        } else {
            internalIndex = 1.0L;
            externalIndex = ior;
        }

        double eta = externalIndex/internalIndex;
        double theta2 = sqrt(1.0L - (eta * eta) * (1.0L - (theta1 * theta1)));
        double rs = (externalIndex * theta1 - internalIndex * theta2) / (externalIndex*theta1 + internalIndex * theta2);
        double rp = (internalIndex * theta1 - externalIndex * theta2) / (internalIndex*theta1 + externalIndex * theta2);
        double reflectance = (rs*rs + rp*rp);

        //reflection
        if(unifRand() < reflectance+reflection) {
            return ray.direction.add(normal.muls(theta1*2.0L));
        }

        // refraction
        return (ray.direction.add(normal.muls(theta1)).muls(eta) \
            .add(normal.muls(-theta2)));
    }
};


class Body {
public:
    Shape* shape;
    Material* material;

    Body(Shape* ishape, Material* imaterial) {
        shape = ishape;
        material = imaterial;
    }
};


struct Scene {
    int width;
    int height;
    Camera camera;
    Body** objects;
    int body_count;
};


class Renderer {
public:
    Scene scene;
    V3* buffer;
    int pixels;

    Renderer(Scene iscene) {
        scene = iscene;
        pixels = scene.width * scene.height;
        buffer = new V3[pixels];
        for (int i = 0; i < pixels; i++) {
            buffer[i] = V3(0.0L, 0.0L, 0.0L);
        }
    }

    ~Renderer(void) {
        delete[] buffer;
    }

    void iterate() {
        int i = 0;
        // cout << "Entered iterate()\n";
        for (double y = unifRand() / (double)scene.height, ystep = 1.0L / (double)scene.height;
             y < 0.99999L;
             y += ystep) {
            for (double x = unifRand() / (double)scene.width, xstep = 1.0L / (double)scene.width;
                 x < 0.99999L;
                 x += xstep) {
                // cout << "Iterate! " << x << ", " << y << endl;
                Ray ray = scene.camera.getRay(x, y);
                V3 color = trace(ray, 0);
                // cout << color.x << ", " << color.y << ", " << color.z << endl;
                buffer[i++].iadd(color);
            }
        }
    }

    V3 trace(Ray ray, int n) {
        if (n > 4) {
            return V3();
        }

        Body *hit = NULL;
        double mint = DBL_MAX;
        // cout << "Body count: " << scene.body_count << endl;
        for (int i = 0; i < scene.body_count; i++) {
            Body *candidate = scene.objects[i];
            // cout << candidate->shape->intersect(ray) << endl;
            double t = candidate->shape->intersect(ray);
            if ((t > 0) && (t <= mint)) {
                mint = t;
                hit = candidate;
            }
        }

        if (hit == NULL) {
            return V3();
        }

        // cout << "hit";

        V3 point = ray.origin.add(ray.direction.muls(mint));
        V3 normal = hit->shape->getNormal(point);
        V3 direction = hit->material->bounce(ray, normal);
        if (direction.dot(ray.direction) > 0.0f) {
            // if the ray is refractedmove the intersection point a bit in
            point = ray.origin.add(ray.direction.muls(mint*1.0000001L));
        } else {
            // otherwise move it out to prevent problems with floating point
            // accuracy
            point = ray.origin.add(ray.direction.muls(mint*0.9999999L));
        }

        Ray newray = Ray(point, direction);
        return trace(newray, n+1).mul(hit->material->color).add(hit->material->emission);
    }
};

void save(V3* buffer, int samples, int width, int height) {
    ofstream myfile;

    stringstream filename;
    filename << "output_" << samples << ".ppm";
    cout << "saving to file \"" << filename.str() << "\"" << endl;
    // myfile.open("output" + samples + ".ppm");
    myfile.open(filename.str());

    myfile << "P3\n" << width << " " << height << endl << "255\n";
    V3* pixel = buffer;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int r = (255.0L * pixel->x) / (double)samples;
            int g = (255.0L * pixel->y) / (double)samples;
            int b = (255.0L * pixel->z) / (double)samples;

            if (r > 255) { r = 255; }
            if (g > 255) { g = 255; }
            if (b > 255) { b = 255; }
            myfile << r << " " << g << " " << b << endl;
            pixel++;
        }
    }

    myfile.close();
}

void worker(int worker_num, int iterations, Scene* scene, V3* buffer) {
    Renderer renderer = Renderer(*scene);

    for (int i = 0; i < iterations; i++) {
        cout << "Worker " << worker_num << " iteration " << i << endl;
        renderer.iterate();
    }

    boost::mutex::scoped_lock lock(buffer_mutex);

    V3* src_pixel = renderer.buffer;
    V3* dst_pixel = buffer;
    for (int y = 0; y < scene->height; y++) {
        for (int x = 0; x < scene->width; x++) {
            dst_pixel->x += src_pixel->x;
            dst_pixel->y += src_pixel->y;
            dst_pixel->z += src_pixel->z;

            src_pixel++;
            dst_pixel++;
        }
    }
}

int main(int argc, const char* argv[]) {
    srand(time(NULL));

    if (argc < 5) {
        cout << argv[0] << " <width> <height> <iterations> <thread count>\n";
        return 0;
    }

    int width = strtol(argv[1], NULL, 10);
    int height = strtol(argv[2], NULL, 10);
    int iterations = strtol(argv[3], NULL, 10);
    int thread_count = strtol(argv[4], NULL, 10);

    // int width = 320;
    // int height = 240;
    // int width = 640;
    // int height = 480;

    Scene scene;
    scene.width = width;
    scene.height = height;
    scene.camera = Camera(
        V3(0.0L, -0.5L, 0.0L),
        V3(-1.3L, 1.0L, 1.0L),
        V3(1.3L, 1.0L, 1.0L),
        V3(-1.3L, 1.0L, -1.0L)
    );

    // Sphere glowing_sphere = Sphere(V3(0.0L, 3.0L, 0.0L), 0.5L);
    // Material glowing_mat = Material(V3(0.9L, 0.9L, 0.9L), V3(1.2L, 1.2L, 1.2L));
    // Body glowing = Body(&glowing_sphere, &glowing_mat);

    Sphere glass_sphere = Sphere(V3(1.0L, 2.0L, 0.0L), 0.5L);
    Glass glass_mat = Glass(V3(1.00L, 1.00L, 1.00L), 1.5L, 0.1L);
    Body glass = Body(&glass_sphere, &glass_mat);
    Sphere chrome_sphere = Sphere(V3(-1.1L, 2.8L, 0.0L), 0.5L);
    Chrome chrome_mat = Chrome(V3(0.8L, 0.8L, 0.8L));
    Body chrome = Body(&chrome_sphere, &chrome_mat);

    Sphere green_sphere = Sphere(V3(-0.15L, 1.55L, -0.4L), 0.1L);
    Glass green_mat = Glass(V3(0.0L, 0.8L, 0.0L), 1.0L, 0.2L);
    Body green = Body(&green_sphere, &green_mat);

    Sphere red_sphere = Sphere(V3(0.0L, 1.187867L, -0.4L), 0.1L);
    Glass red_mat = Glass(V3(0.8, 0.0, 0.0), 1.0, 0.2);
    Body red = Body(&red_sphere, &red_mat);

    Sphere blue_sphere = Sphere(V3(0.15, 1.55, -0.4), 0.1);
    Glass blue_mat = Glass(V3(0.0, 0.0, 0.8), 1.0, 0.2);
    Body blue = Body(&blue_sphere, &blue_mat);

    Sphere pyramid_sphere = Sphere(V3(0.0L, 1.4L, -0.28L), 0.1L);
    Glass pyramid_mat = Glass(V3(1.00L, 1.00L, 1.00L), 1.5L, 0.1L);
    Body pyramid = Body(&pyramid_sphere, &pyramid_mat);

    // # White ball
    // Body(Sphere(V3(0.0, 1.4, -0.6828), 0.1), Glass(V3(0.8, 0.8, 0.8), 1.0, 0.2)),

    Material floor_mat = Material(V3(0.9L, 0.9L, 0.9L));
    Plane floor_plane = Plane(V3(0.0L, 3.5L, -0.5L), V3(0.0L, 0.0L, 1.0L));
    Body floor = Body(&floor_plane, &floor_mat);

    Plane back_plane = Plane(V3(0.0L, 4.5L, 0.0L), V3(0.0L, -1.0L, 0.0L));
    Material back_mat = Material(V3(0.9L, 0.9L, 0.9L));
    Body back = Body(&back_plane, &back_mat);
    Plane left_plane = Plane(V3(-1.9L, 0.0L, 0.0L), V3(1.0L, 0.0L, 0.0L));
    Material left_mat = Material(V3(0.9L, 0.5L, 0.5L));
    Body left = Body(&left_plane, &left_mat);
    Plane right_plane = Plane(V3(1.9L, 0.0L, 0.0L), V3(-1.0L, 0.0L, 0.0L));
    Material right_mat = Material(V3(0.5L, 0.5L, 0.9L));
    Body right = Body(&right_plane, &right_mat);

    Plane top_light_plane = Plane(V3(0.0L, 0.0L, 2.5L), V3(0.0L, 0.0L, -1.0L));
    Material top_light_mat = Material(V3(0.0L, 0.0L, 0.0L), V3(1.6L, 1.47L, 1.29L));
    // Material top_light_mat = Material(V3(0.0L, 0.0L, 0.0L), V3(1.0L, 0.87L, 0.69L));
    // Material top_light_mat = Material(V3(0.0L, 0.0L, 0.0L), V3(1.1L, 0.97L, 0.79L));
    Body top_light = Body(&top_light_plane, &top_light_mat);

    Plane front_plane = Plane(V3(0.0L, -2.5L, 0.0L), V3(0.0L, 1.0L, 0.0L));
    Material front_mat = Material(V3(0.9L, 0.9L, 0.9L));
    Body front = Body(&front_plane, &front_mat);

    Plane top_left_divider_plane = Plane(V3(-1.8L, 4.4L, 2.4L), V3(1.0L, -1.0L, -1.0L).normalize());
    Material top_left_divider_mat = Glass(V3(0.0L, 0.8L, 0.0L), 1.0L, 0.2L);
    Body top_left_divider = Body(&top_left_divider_plane, &top_left_divider_mat);

    Sphere right_light_sphere = Sphere(V3(1.9L, 3.625L, 2.1L), 0.1L);
    Material right_light_mat = Material(V3(0.0L, 0.0L, 0.0L), V3(2.0L, 1.87L, 1.69L));
    Body right_light = Body(&right_light_sphere, &right_light_mat);

    Sphere left_light_sphere = Sphere(V3(-1.9L, 3.625L, 2.1L), 0.1L);
    Material left_light_mat = Material(V3(0.0L, 0.0L, 0.0L), V3(2.0L, 1.87L, 1.69L));
    Body left_light = Body(&left_light_sphere, &left_light_mat);

    Sphere right2_light_sphere = Sphere(V3(1.9L, 2.75L, 2.1L), 0.1L);
    Body right2_light = Body(&right2_light_sphere, &right_light_mat);
    Sphere left2_light_sphere = Sphere(V3(-1.9L, 2.75L, 2.1L), 0.1L);
    Body left2_light = Body(&left2_light_sphere, &left_light_mat);
    Sphere right3_light_sphere = Sphere(V3(1.9L, 1.875L, 2.1L), 0.1L);
    Body right3_light = Body(&right3_light_sphere, &right_light_mat);
    Sphere left3_light_sphere = Sphere(V3(-1.9L, 1.875L, 2.1L), 0.1L);
    Body left3_light = Body(&left3_light_sphere, &left_light_mat);
    Sphere right4_light_sphere = Sphere(V3(1.9L, 1.0L, 2.1L), 0.1L);
    Body right4_light = Body(&right4_light_sphere, &right_light_mat);
    Sphere left4_light_sphere = Sphere(V3(-1.9L, 1.0L, 2.1L), 0.1L);
    Body left4_light = Body(&left4_light_sphere, &left_light_mat);
    Sphere right5_light_sphere = Sphere(V3(1.9L, 0.125L, 2.1L), 0.1L);
    Body right5_light = Body(&right5_light_sphere, &right_light_mat);
    Sphere left5_light_sphere = Sphere(V3(-1.9L, 0.125L, 2.1L), 0.1L);
    Body left5_light = Body(&left5_light_sphere, &left_light_mat);
    Sphere right6_light_sphere = Sphere(V3(1.9L, -0.75L, 2.1L), 0.1L);
    Body right6_light = Body(&right6_light_sphere, &right_light_mat);
    Sphere left6_light_sphere = Sphere(V3(-1.9L, -0.75L, 2.1L), 0.1L);
    Body left6_light = Body(&left6_light_sphere, &left_light_mat);
    Sphere right7_light_sphere = Sphere(V3(1.9L, -1.625, 2.1L), 0.1L);
    Body right7_light = Body(&right7_light_sphere, &right_light_mat);
    Sphere left7_light_sphere = Sphere(V3(-1.9L, -1.625L, 2.1L), 0.1L);
    Body left7_light = Body(&left7_light_sphere, &left_light_mat);

    Sphere back_light_sphere = Sphere(V3(-0.63333L, 4.5L, 2.1L), 0.1L);
    Material back_light_mat = Material(V3(0.0L, 0.0L, 0.0L), V3(2.0L, 1.87L, 1.69L));
    Body back_light = Body(&back_light_sphere, &back_light_mat);

    Sphere back2_light_sphere = Sphere(V3(0.63333L, 4.5L, 2.1L), 0.1L);
    Material back2_light_mat = Material(V3(0.0L, 0.0L, 0.0L), V3(2.0L, 1.87L, 1.69L));
    Body back2_light = Body(&back2_light_sphere, &back2_light_mat);

    Sphere front_light_sphere = Sphere(V3(-0.63333L, -2.5L, 2.1L), 0.1L);
    Material front_light_mat = Material(V3(0.0L, 0.0L, 0.0L), V3(2.0L, 1.87L, 1.69L));
    Body front_light = Body(&front_light_sphere, &front_light_mat);

    Sphere front2_light_sphere = Sphere(V3(0.63333L, -2.5L, 2.1L), 0.1L);
    Material front2_light_mat = Material(V3(0.0L, 0.0L, 0.0L), V3(2.0L, 1.87L, 1.69L));
    Body front2_light = Body(&front2_light_sphere, &front2_light_mat);

    // Plane top_right_divider_plane = Plane(V3(1.4L, 4.0L, 2.0L), V3(-1.0L, -1.0L, -1.0L).normalize());
    // Material top_right_divider_mat = Glass(V3(0.8L, 0.0L, 0.0L), 1.0L, 0.2L);
    // Body top_right_divider = Body(&top_right_divider_plane, &top_right_divider_mat);

    Plane top_right_divider_plane = Plane(V3(1.8L, 4.4L, 2.4L), V3(-1.0L, -1.0L, -1.0L).normalize());
    // Material top_right_divider_mat = Material(V3(0.0L, 0.0L, 0.0L), V3(1.6L, 1.47L, 1.29L));
    Material top_right_divider_mat = Glass(V3(0.8L, 0.0L, 0.0L), 1.0L, 0.2L);
    Body top_right_divider = Body(&top_right_divider_plane, &top_right_divider_mat);

    scene.body_count = 32;
    Body* bodies[] = {&glass, &chrome, &green, &red, &blue, &pyramid, &floor, &back, &left, &right, &top_light, &front,
                      &top_left_divider, &top_right_divider, &right_light, &left_light, &back_light, &back2_light, &right2_light, &left2_light,
                      &right3_light, &left3_light, &right4_light, &left4_light,
                      &right5_light, &left5_light, &right6_light, &left6_light, &right7_light, &left7_light,
                      &front_light, &front2_light};
    // scene.body_count = 14;
    // Body* bodies[] = {&glass, &chrome, &green, &red, &blue, &pyramid, &floor, &back, &left, &right, &top_light, &front, &top_left_divider, &top_right_divider};
    scene.objects = bodies;

    V3* buffer = new V3[width * height];
    for (int i = 0; i < (width * height); i++) {
        buffer[i] = V3(0.0L, 0.0L, 0.0L);
    }

    // int iterations_remaining = iterations;
    // boost::thread** threads = NULL;

    // while (iterations_remaining > 0) {
    //     int iteration_step = min(10, max((int)((double)iterations_remaining / (double)thread_count), 1));
    //     cout << "Iterations between output: " << iteration_step << endl;

    //     threads = new boost::thread*[thread_count];
    //     for (int i = 0; i < thread_count; i++) {
    //         threads[i] = new boost::thread(boost::bind(&worker, i, iteration_step, &scene, buffer));
    //         iterations_remaining -= iteration_step;
    //     }

    //     for (int i = 0; i < thread_count; i++) {
    //         threads[i]->join();
    //     }

    //     for (int i = 0; i < thread_count; i++) {
    //         delete threads[i];
    //     }
    
    //     save(buffer, iterations - iterations_remaining, width, height);
    // }

    // delete threads;

    // save(buffer, iterations, width, height);

    boost::thread** threads = new boost::thread*[thread_count];
    for (int i = 0; i < thread_count; i++) {
        threads[i] = new boost::thread(boost::bind(&worker, i, (double)iterations / (double)thread_count, &scene, buffer));
    }

    for (int i = 0; i < thread_count; i++) {
        threads[i]->join();
    }

    for (int i = 0; i < thread_count; i++) {
        delete threads[i];
    }
    delete threads;
    
    save(buffer, iterations, width, height);

    delete[] buffer;
}
