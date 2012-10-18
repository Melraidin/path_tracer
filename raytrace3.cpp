#include <algorithm>
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
#include <vector>
#include <string>

#include "MatusikBRDF.h"

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
// return a inform rand number in [a,b].
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

    return *this;
  }

  V3 sub(V3 v) {
    return V3(x - v.x, y - v.y, z - v.z);
  }

  V3 isub(V3 v) {
    x -= v.x;
    y -= v.y;
    z -= v.z;
    return *this;
  }

  V3 mul(V3 v) {
    return V3(x * v.x, y * v.y, z * v.z);
  }

  V3 imul(V3 v) {
    x *= v.x;
    y *= v.y;
    z *= v.z;
    return *this;
  }

  V3 div(V3 v) {
    return V3(x / v.x, y / v.y, z / v.z);
  }

  V3 muls(double s) {
    return V3(x * s, y * s, z * s);
  }

  V3 imuls(double s) {
    this->x *= s;
    this->y *= s;
    this->z *= s;
    return *this;
  }

  V3 divs(double s) {
    return muls(1.0L / s);
  }

  double dot(V3 v) {
    return x * v.x + y * v.y + z * v.z;
  }

  V3 cross(V3 v) {
    return V3(
        (y * v.z) - (z * v.y),
        (z * v.x) - (x * v.z),
        (x * v.y) - (y * v.x));
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

std::ostream& operator<<(std::ostream &strm, const V3 &v) {
  return strm << "V3(" << v.x << ", " << v.y << ", " << v.z << ")";
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
  virtual V3 getPointOnSurface(void) {}
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

  V3 getPointOnSurface(void) {
    // HACK Only supports planes with normals on the Z axis.
    // HACK Currently limits to -10 to 10 on X and Y axes.
    // TODO
    return V3(unifRand(-10.0L, 10.0L), unifRand(-10.0L, 10.0L), this->center.z);
  }
};

class Rectangle : public Shape {
  V3 p1, p2, p3, p4;
  V3 normal;

  // Collision detection from http://answers.google.com/answers/threadview?id=18979
  V3 v1, v3;

  V3 minimums, maximums;

public:
  Rectangle(V3 p1, V3 p2, V3 p3, V3 p4) {
    this->p1 = p1;
    this->p2 = p2;
    this->p3 = p3;
    this->p4 = p4;

    this->normal = p3.sub(p1).cross(p2.sub(p1)).normalize();
    // this->normal = p4.sub(p1).cross(p2.sub(p1)).normalize();
    // this->normal = V3(0.0L, 0.0L, -1.0L);

    this->v1 = p2.sub(p1).normalize();
    this->v3 = p4.sub(p3).normalize();

    this->minimums = V3(
        min(min(this->p1.x, this->p2.x), min(this->p3.x, this->p4.x)),
        min(min(this->p1.y, this->p2.y), min(this->p3.y, this->p4.y)),
        min(min(this->p1.z, this->p2.z), min(this->p3.z, this->p4.z)));

    this->maximums = V3(
        max(max(this->p1.x, this->p2.x), max(this->p3.x, this->p4.x)),
        max(max(this->p1.y, this->p2.y), max(this->p3.y, this->p4.y)),
        max(max(this->p1.z, this->p2.z), max(this->p3.z, this->p4.z)));
  }

  V3 getNormal(V3 point) {
    // cout << p1 << "\t" << p2 << "\t" << p3 << "\t" << this->normal << endl;
    return this->normal;
  }

  double intersect(Ray r) {
    double n_dot_u = normal.dot(r.direction);

    if ((n_dot_u > -0.00001L) && (n_dot_u < 0.00001L)) {
      return -1.0L;
    }

    double n_dot_p0 = normal.dot(p1.sub(r.origin));
    double t = n_dot_p0 / n_dot_u;

    // Intersection before origin of ray.
    if (t < 1.0L) {
      return -1.0L;
    }

    V3 hit = r.origin.add(r.direction.muls(t));

    V3 v4 = hit.sub(p1).normalize();
    V3 v5 = hit.sub(p3).normalize();

    if ((v1.dot(v4) > 0.0L) && (v3.dot(v5) > 0.0L)) {
      return t;
    } else {
      return -1.0L;
    }
  }

  V3 getPointOnSurface(void) {
    // HACK Hard-coded.
    // V3 point = V3(
    //     unifRand(this->minimums.x, this->maximums.x),
    //     unifRand(this->minimums.y, this->maximums.y),
    //     unifRand(this->minimums.z, this->maximums.z));

    V3 v1 = this->p2.sub(this->p1);
    V3 v2 = this->p3.sub(this->p2);

    return this->p1.add(v1.imuls(unifRand(0.0L, 1.0L)).iadd(v2.imuls(unifRand(0.0L, 1.0L))));

    // cout << "rectangle rand: " << point << endl;
  }

  // V3 getPointOnSurface(void) {
  //     // HACK Likely biased towards center of rectangle.
  //     return p2.sub(p1).muls(unifRand(0.0L, 1.0L)).iadd(
  //         p4.sub(p1).muls(unifRand(0.0L, 1.0L)));
  // }
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

  V3 getPointOnSurface(void) {
    // HACK Is this biased?
    // Find random point within a cube with sides 2*radius then fit to radius.

    // TODO HACK Forcing this to be a negative Z to point into scene for lighting for now.
    V3 point = V3(
        unifRand(-this->radius, this->radius),
        unifRand(-this->radius, this->radius),
        unifRand(-this->radius, 0.0L));

    V3 normal = point.normalize();
    normal.muls(this->radius);
    return normal.iadd(this->center);
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
  vector<Body*> objects;
  int body_count;
};


struct HitRecord {
  V3 hit;
  Body* body;
  V3 color;
  V3 flux;

  HitRecord(V3 hit, Body* body, V3 color) {
    this->hit = hit;
    this->body = body;
    this->color = color;
  }

  HitRecord(V3 hit, Body* body, V3 color, V3 flux) {
    this->hit = hit;
    this->body = body;
    this->color = color;
    this->flux = flux;
  }
};


class Renderer {
public:
  Scene scene;
  V3* buffer;
  int pixels;
  vector<Body*> lights;
  MatusikBRDF brdf;

  Renderer(Scene iscene) {
    scene = iscene;
    pixels = scene.width * scene.height;
    buffer = new V3[pixels];
    for (int i = 0; i < pixels; i++) {
      buffer[i] = V3(0.0L, 0.0L, 0.0L);
    }

    // Find all lights in scene.
    for (vector<Body*>::iterator it = this->scene.objects.begin(); it < this->scene.objects.end(); it++) {
      if (((*it)->material->emission.x > 1.0L) || ((*it)->material->emission.y > 1.0L) || ((*it)->material->emission.z > 1.0L)) {
        this->lights.push_back(*it);
      }
    }

    brdf.read_brdf(string("nickel.binary"));
  }

  ~Renderer(void) {
    delete[] buffer;
  }

  void iterate() {
    int i = 0;
    // cout << "Entered iterate()\n";

    vector<HitRecord> gather_records;
    vector<HitRecord> shoot_records;

    for (double y = unifRand() / (double)scene.height, ystep = 1.0L / (double)scene.height;
         y < 0.99999L;
         y += ystep) {
      for (double x = unifRand() / (double)scene.width, xstep = 1.0L / (double)scene.width;
           x < 0.99999L;
           x += xstep) {

        gather_records.clear();
        // cout << "Iterate! " << x << ", " << y << endl;
        Ray gather_ray = scene.camera.getRay(x, y);
        V3 gather_color = trace(gather_ray, 0, &gather_records);

        // cout << "Iterations: " << gather_records.size() << endl;
        // cout << color.x << ", " << color.y << ", " << color.z << endl;

        shoot_records.clear();

        bool shoot_trace = false;

        Body* light;
        if (shoot_trace) {
          // TODO Only using a single spherical light.
          // light = this->lights[unifRand(0, this->lights.size() - 1)];
          // light = this->lights[unifRand(0, this->lights.size() - 1)];
          //light = this->lights[0, 1];
          // light = this->lights[0];
          // light = this->lights[i % this->lights.size()];
          V3 shoot_origin;
          do {
            light = this->lights[unifRand(0, this->lights.size() - 1)];
            // light = this->lights[i % 3];
            // light = this->lights[2];
            // cout << __LINE__ << endl;
            // cout << "Light index: " << i % 3 << endl;
            shoot_origin = light->shape->getPointOnSurface();
            Ray shoot_ray = Ray(shoot_origin, light->shape->getNormal(shoot_origin).normalize());
            // cout << shoot_ray.origin << " -> " << shoot_ray.direction << endl;
            // V3 shoot_color = trace(shoot_ray, 0, &shoot_records, light->material->emission, true);
            V3 shoot_color = trace(shoot_ray, 0, &shoot_records, light->material->emission, false);
            // cout << shoot_records.size() << ", " << shoot_color << endl;
            // cout << __LINE__ << endl;
            // cout << "shoot_records: " << shoot_records.size() << endl;
          } while (shoot_records.size() == 0);

          // cout << __LINE__ << endl;
          bool add_light = true;

          if (add_light) {
            // Add the light source itself as a shoot hit.
            shoot_records.push_back(
              HitRecord(
                shoot_origin, light, light->material->emission.add(light->material->color)));
          }

          V3 final_color;
          V3 indirect_color;
          V3 direct_color;

          // cout << "Gather hits: " << gather_records.size() << ", shoot hits: " << shoot_records.size() << endl;

          double samples = 0;

          // cout << __LINE__ << endl;
          for (int gather_i = 0; gather_i < gather_records.size(); gather_i++) {
            HitRecord gather_it = gather_records[gather_i];
            for (int shoot_i = 0; shoot_i < shoot_records.size(); shoot_i++) {
              HitRecord shoot_it = shoot_records[shoot_i];
              // cout << __LINE__ << endl;
              // TODO Haven't accounted for visiblity of shoot hit from gather hit.

              // if (shoot_i < shoot_records.size() - 1) {
              if (true) {
                //     HitRecord prev_shoot = shoot_records[shoot_i + 1];
                // if (false) {
                //     // // Previous shoot hit was not the shoot origin.
                //     // // TODO Very uncertain whether using bounce() as the BRDF is reasonable.
                //     V3 light_brdf = shoot_it->body->material->bounce(
                //         Ray(prev_shoot.hit, shoot_it->hit.sub(prev_shoot.hit).normalize()),
                //         gather_it->hit.sub(shoot_it->hit).normalize());

                //     double light_flux = fabs(light_brdf.dot(gather_it->body->shape->getNormal(gather_it->hit)));

                //     // cout << "Old final: " << final_color << endl;
                //     final_color = final_color.mul(gather_it->body->material->color);
                //     // cout << "Final with gather hit: " << final_color << endl;
                //     final_color.iadd(gather_it->body->material->emission);
                //     // cout << "Final with gather emission: " << final_color << endl;
                //     final_color.iadd(shoot_it->color.muls(light_flux));
                //     // cout << "Final with shoot color: " << final_color << endl;
                // cout << __LINE__ << endl;
                //     samples++;
                // } else {
                if (checkHit(shoot_it.hit, gather_it.hit, gather_it.body, shoot_it.body)) {
                  // cout << __LINE__ << endl;
                  // TODO First check these.
                  // indirect_color.iadd(shoot_it->body->material->emission);
                  // indirect_color.iadd(shoot_it->color);
                  // indirect_color.iadd(shoot_it->color.muls(1.0L / (double)shoot_records.size()));
                  // indirect_color.iadd(shoot_it.color.muls(((double)shoot_records.size() - shoot_i) / (double)shoot_records.size()));
                  // samples++;

                  double brdf_scale = 1.0L;

                  // Find probability of path to this point. Assume the first gather hit is guaranteed.
                  V3 prob = V3(1.0L, 1.0L, 1.0L);
                  V3 g_prob;
                  double in_theta, in_phi, out_theta, out_phi;
                  // cout << __LINE__ << endl;
                  vector_to_spherical(gather_records[0].hit.sub(gather_ray.origin).normalize(), in_theta, in_phi);
                  // cout << __LINE__ << endl;
                  // cout << __LINE__ << endl;
                  for (int gpi = 1; gpi <= gather_i; gpi++) {
                    vector_to_spherical(gather_records[gpi].hit.sub(gather_records[gpi - 1].hit).normalize(), out_theta, out_phi);

                    brdf.lookup_brdf_val(in_theta, in_phi, out_theta, out_phi, g_prob.x, g_prob.y, g_prob.z);
                    g_prob.imuls(brdf_scale);
                    prob.imul(g_prob);

                    in_theta = out_theta;
                    in_phi = out_phi;
                    // cout << __LINE__ << endl;
                  }
                  // cout << __LINE__ << endl;

                  // cout << __LINE__ << endl;
                  vector_to_spherical(shoot_records[shoot_i].hit.sub(gather_records[gather_i].hit).normalize(), out_theta, out_phi);
                  // cout << __LINE__ << endl;
                  for (int spi = shoot_i; spi > 0; spi--) {
                    brdf.lookup_brdf_val(in_theta, in_phi, out_theta, out_phi, g_prob.x, g_prob.y, g_prob.z);

                    // cout << "gather probability: " << g_prob << endl;
                    g_prob.imuls(brdf_scale);
                    prob.imul(g_prob);

                    in_theta = out_theta;
                    in_phi = out_phi;
                    // cout << __LINE__ << endl;

                    vector_to_spherical(shoot_records[spi].hit.sub(gather_records[spi + 1].hit).normalize(), out_theta, out_phi);
                    // cout << __LINE__ << endl;
                  }
                  // cout << __LINE__ << endl;

                  // cout << "Shoot color: " << shoot_it.color << endl;
                  // indirect_color.iadd(shoot_it.color);
                  // if ((indirect_color.x != 0.0L) || (indirect_color.y != 0.0L) || (indirect_color.z != 0.0L)) {
                  //   cout << "Indirect color: " << indirect_color << endl;
                  // }

                  // cout << __LINE__ << endl;
                  // cout << "indirect: " << indirect_color << endl;
                  // cout << "emission: " << light->material->emission << endl;
                  // cout << "g_prob: " << g_prob << endl;
                  indirect_color.iadd(light->material->emission.mul(g_prob));
                  // cout << __LINE__ << endl;
                  // cout << "Indirect color: " << indirect_color << endl;
                }
              }
              // cout << __LINE__ << endl;

              // if (checkHit(shoot_origin, gather_it->hit, light)) {
              //     direct_color = light->material->emission;
              // }
            }

            // cout << __LINE__ << endl;
            // cout << "body color: " << gather_it.body->material->color << ", indirect color: " << indirect_color << endl;
            final_color = gather_it.body->material->color.mul(indirect_color);
            // cout << "1\n";
            // final_color = gather_it->body->material->color.mul(indirect_color.iadd(direct_color));
            // cout << "Samples: " << samples << endl;
            // final_color.iadd(gather_it->body->material->color.mul(indirect_color.iadd(direct_color)).iadd(gather_it->body->material->emission));
            // final_color = gather_it.body->material->color.mul(indirect_color.iadd(direct_color).iadd(final_color)).iadd(gather_it.body->material->emission);
            // final_color.iadd(indirect_color);
          }

          buffer[i++].iadd(final_color.iadd(gather_color));
          // buffer[i++].iadd(final_color.divs(samples));
          // buffer[i++].iadd(final_color);
        } else {
          buffer[i++].iadd(gather_color);
        }
      }
    }
  }

  // TODO This should likely have the vector rotated to be along the Z-axis.
  void vector_to_spherical(V3 in, double& theta, double&phi) {
    theta = atan(in.y / in.x);
    phi = acos(in.z / sqrt(pow(in.x, 2) + pow(in.y, 2) + pow(in.z, 2)));
  }

  bool checkHit(V3 origin, V3 target, Body* expected, Body* unexpected) {
    // Returns true if a ray from origin to target hits nothing in
    // between except the expected Body.
    Body* hit = NULL;
    double mint = DBL_MAX;

    Ray ray = Ray(origin, target.sub(origin));

    for (vector<Body*>::iterator it = scene.objects.begin(); it < scene.objects.end(); it++) {
      double t = (*it)->shape->intersect(ray);
      if ((t > 0.0000001L) && (t <= mint)) {
        mint = t;
        hit = *it;
      }
    }

    // if (hit == unexpected) {
    //     cout << "checkHit: unexpected\n";
    // } else if (hit == expected) {
    //     cout << "checkHit: expected\n";
    // } else {
    //     cout << "checkHit: Normal\n";
    // }

    return hit == expected;
  }

  V3 trace(Ray ray, int n, vector<HitRecord>* records) {
    return trace(ray, n, records, V3(), false);
  }

  V3 trace(Ray ray, int n, vector<HitRecord>* records, V3 flux, bool debug) {
    if (n > 4) {
      if (debug) {
        cout << "Too long a path, stopping.\n";
      }

      return V3();
    }

    Body* hit = NULL;
    double mint = DBL_MAX;
    // cout << "Body count: " << scene.body_count << endl;
    for (vector<Body*>::iterator it = scene.objects.begin(); it < scene.objects.end(); it++) {
      // cout << candidate->shape->intersect(ray) << endl;
      double t = (*it)->shape->intersect(ray);
      if ((t > 0.0000001L) && (t <= mint)) {
        mint = t;
        hit = *it;
      }
    }

    if (hit == NULL) {
      if (debug) {
        cout << "No hit, stopping. " << ray.origin << " -> " << ray.direction << endl;
      }
      return V3();
    }

    // V3 ra = hit->shape->getPointOnSurface();
    // cout << "hit, random point: " << ra.x << ", " << ra.y << ", " << ra.z << endl;

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
    V3 color, direct_color;

    bool direct_light = false;
    bool single_random_light = false;

    if (direct_light) {
      if (single_random_light) {
        Body* light = this->lights[unifRand(0, this->lights.size() - 1)];
        V3 light_point = light->shape->getPointOnSurface();

        // TODO Undone for checkHit tests.
        // if (checkHit(light_point, point, hit)) {
        //     // TODO Should this be scaled by the BRDF?
        //     // HACK Magic divisor
        //     direct_color.iadd(light->material->emission.divs(4.0L));
        // }
      } else {
        for (vector<Body*>::iterator light_it = this->lights.begin(); light_it < this->lights.end(); light_it++) {
          V3 light_point = (*light_it)->shape->getPointOnSurface();

          // TODO Undone for checkHit tests.
          // if (checkHit(light_point, point, hit)) {
          //     // TODO Should this be scaled by the BRDF?
          //     // HACK Magic divisor
          //     direct_color.iadd((*light_it)->material->emission.divs(12.0L));
          // }
        }
      }
    }

    // cout << "direct: " << direct_color << endl;

    // color = trace(newray, n+1, records).mul(hit->material->color).mul(direct_color).add(hit->material->emission);
    // color = hit->material->color.mul(trace(newray, n+1, records).add(direct_color).add(hit->material->emission));
    // color = hit->material->color.mul(trace(newray, n+1, records).add(direct_color).add(hit->material->emission));
    color = hit->material->color.mul(trace(newray, n+1, records).add(direct_color)).add(hit->material->emission);

    // if ((hit->material->emission.x < 1.0L) && (hit->material->emission.y < 1.0L) && (hit->material->emission.z < 1.0L)) {
    //     color = trace(newray, n+1, records).mul(hit->material->color).mul(direct_color).add(hit->material->emission);
    // } else {
    //     color = hit->material->color.mul(direct_color).add(hit->material->emission);
    // }

    records->push_back(HitRecord(point, hit, color));

    return color;
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

  Scene scene;
  scene.width = width;
  scene.height = height;
  scene.camera = Camera(
      V3(0.0L, -0.5L, 0.0L),
      V3(-1.3L, 1.0L, 1.0L),
      V3(1.3L, 1.0L, 1.0L),
      V3(-1.3L, 1.0L, -1.0L)
                        );

  vector<Body*> bodies;

  Sphere glass_sphere = Sphere(V3(1.0L, 2.0L, 0.0L), 0.5L);
  Glass glass_mat = Glass(V3(1.00L, 1.00L, 1.00L), 1.5L, 0.1L);
  Body glass = Body(&glass_sphere, &glass_mat);
  bodies.push_back(&glass);

  Sphere chrome_sphere = Sphere(V3(-1.1L, 2.8L, 0.0L), 0.5L);
  Chrome chrome_mat = Chrome(V3(0.8L, 0.8L, 0.8L));
  Body chrome = Body(&chrome_sphere, &chrome_mat);
  bodies.push_back(&chrome);

  Material floor_mat = Material(V3(0.9L, 0.9L, 0.9L));
  Rectangle floor_rectangle = Rectangle(V3(-1.9L, 4.5L, -0.5L), V3(1.9L, 4.5L, -0.5L), V3(1.9L, -2.5L, -0.5L), V3(-1.9L, -2.5L, -0.5L));
  Body floor = Body(&floor_rectangle, &floor_mat);
  bodies.push_back(&floor);
  Material back_mat = Material(V3(0.9L, 0.9L, 0.9L));
  Rectangle back_rectangle = Rectangle(V3(-1.9L, 4.5L, 2.5L), V3(1.9L, 4.5L, 2.5L), V3(1.9L, 4.5L, -0.5L), V3(-1.9L, 4.5L, -0.5L));
  Body back = Body(&back_rectangle, &back_mat);
  bodies.push_back(&back);
  Material left_mat = Material(V3(0.9L, 0.5L, 0.5L));
  Rectangle left_rectangle = Rectangle(V3(-1.9L, -2.5L, 2.5L), V3(-1.9L, 4.5L, 2.5L), V3(-1.9L, 4.5L, -0.5L), V3(-1.9L, -2.5L, 2.5L));
  Body left = Body(&left_rectangle, &left_mat);
  bodies.push_back(&left);
  Material right_mat = Material(V3(0.5L, 0.5L, 0.9L));
  Rectangle right_rectangle = Rectangle(V3(1.9L, 4.5L, 2.5L), V3(1.9L, -2.5L, 2.5L), V3(1.9L, -2.5L, -0.5L), V3(1.9L, 4.5L, -0.5L));
  Body right = Body(&right_rectangle, &right_mat);
  bodies.push_back(&right);
  Material top_mat = Material(V3(0.9L, 0.9L, 0.9L));
  Rectangle top_rectangle = Rectangle(V3(1.9L, 4.5L, 2.5L), V3(-1.9L, 4.5L, 2.5L), V3(-1.9L, -2.5L, 2.5L), V3(1.9L, -2.5L, 2.5L));
  Body top = Body(&top_rectangle, &top_mat);
  bodies.push_back(&top);
  Material front_mat = Material(V3(0.9L, 0.9L, 0.9L));
  Rectangle front_rectangle = Rectangle(V3(1.9L, 4.5L, 2.5L), V3(-1.9L, 4.5L, 2.5L), V3(-1.9L, 4.5L, -0.5L), V3(1.9L, 4.5L, -0.5L));
  Body front = Body(&front_rectangle, &front_mat);
  bodies.push_back(&front);

  // Material light_mat = Material(V3(1.0L, 0.935L, 0.845L), V3(2.0L, 1.87L, 1.69L));
  Material light_mat = Material(V3(0.2L, 0.187L, 0.169L), V3(2.0L, 1.87L, 1.69L));

  Sphere back_light_sphere = Sphere(V3(-0.63333L, 4.5L, 2.1L), 0.1L);
  // Material back_light_mat = Material(V3(0.0L, 0.0L, 0.0L), V3(2.0L, 1.87L, 1.69L));
  Body back_light = Body(&back_light_sphere, &light_mat);
  bodies.push_back(&back_light);

  Sphere back2_light_sphere = Sphere(V3(0.63333L, 4.5L, 2.1L), 0.1L);
  //Material back2_light_mat = Material(V3(0.0L, 0.0L, 0.0L), V3(2.0L, 1.87L, 1.69L));
  Body back2_light = Body(&back2_light_sphere, &light_mat);
  bodies.push_back(&back2_light);

  Rectangle top_light_box = Rectangle(V3(1.4L, 3.5L, 2.5L), V3(-1.4L, 3.5L, 2.5L), V3(-1.4L, -2.5L, 2.5L), V3(1.4L, -2.5L, 2.5L));
  // Rectangle top_light_box = Rectangle(V3(-1.4L, 3.5L, 2.5L), V3(1.4L, 3.5L, 2.5L), V3(1.4L, -2.5L, 2.5L), V3(-1.4L, -2.5L, 2.5L));
  Material top_light_mat = Material(V3(0.0L, 0.0L, 0.0L), V3(2.0L, 1.87L, 1.69L));
  // Material top_light_mat = Material(V3(0.0L, 0.0L, 0.0L), V3(1.1L, 0.935L, 0.845L));
  Body top_light = Body(&top_light_box, &light_mat);
  bodies.push_back(&top_light);

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
