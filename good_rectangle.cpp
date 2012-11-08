////////////////////////////////////////////////////////////////////////
// Working rectangle intersection.
////////////////////////////////////////////////////////////////////////

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
#include <kdtree++/kdtree.hpp>

boost::mutex buffer_mutex;

using namespace std;

#define EPSILON 0.000001L

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

    double total(void) {
        // Returns the sum of the components.
        return this->x + this->y + this->z;
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

    double power_distance(V3 v) {
        // Returns the distance between this point and the given point
        // without converting to real distance. Only useful for
        // comparison.
        return pow(this->x - v.x, 2) + pow(this->y - v.y, 2) + pow(this->z - v.z, 2);
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

    double length() {
        return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
    }
};

std::ostream& operator<<(std::ostream &strm, const V3 &v) {
    return strm << "V3(" << v.x << ", " << v.y << ", " << v.z << ")";
}

class V4 : public V3 {
public:
    int s = 0;

    V4(void) {
        x = 0.0L;
        y = 0.0L;
        z = 0.0L;
        s = 0;
    }

    V4(double ix, double iy, double iz) {
        x = ix;
        y = iy;
        z = iz;
        s = 0;
    }

    V4 iadd(V3 v) {
        x += v.x;
        y += v.y;
        z += v.z;
        s++;

        return *this;
    }
};

std::ostream& operator<<(std::ostream &strm, const V4 &v) {
    return strm << "V4(" << v.x << ", " << v.y << ", " << v.z << ", " << v.s << ")";
}

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

    Ray(void) {}

    Ray(V3 iorigin, V3 idirection) {
        origin = iorigin;
        direction = idirection;
    }
};

std::ostream& operator<<(std::ostream &strm, const Ray &r) {
    return strm << "Ray(" << r.origin << " -> " << r.direction << ")";
}

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
        throw "Cannot get point on plane.";
        return V3(unifRand(-10.0L, 10.0L), unifRand(-10.0L, 10.0L), this->center.z);
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

    V3 getPointOnSurface(void) {
        // HACK Is this biased?
        // Find random point within a cube with sides 2*radius then fit to radius.

        // TODO HACK Forcing this to be a negative Z to point into scene for lighting for now.
        V3 point = V3(
            unifRand(-this->radius, this->radius),
            unifRand(-this->radius, this->radius),
            unifRand(-this->radius, this->radius));

        V3 normal = point.normalize();
        normal.muls(this->radius);
        return normal.iadd(this->center);
    }

    double area(void) {
        return this->radius2 * M_PI;
    }
};


class Material {
public:
    V3 color;
    V3 emission;
    V3 diffuse;
    V3 specular;
    double power;

    Material(void) {
        color = V3(0.0L, 0.0L, 0.0L);
        emission = V3(0.0L, 0.0L, 0.0L);
        diffuse = V3(0.7L, 0.7L, 0.7L);
        specular = V3(0.0L, 0.0L, 0.0L);
    };

    Material(V3 icolor, V3 iemission) {
        color = icolor;
        emission = iemission;
        diffuse = V3(1.0L, 1.0L, 1.0L);
        specular = V3(0.0L, 0.0L, 0.0L);
    }

    Material(V3 icolor, V3 iemission, double power) {
        color = icolor;
        emission = iemission;
        diffuse = V3(1.0L, 1.0L, 1.0L);
        specular = V3(0.0L, 0.0L, 0.0L);
        this->power = power;
    }

    Material(V3 icolor) {
        color = icolor;
        emission = V3(0.0L, 0.0L, 0.0L);
        diffuse = V3(1.0L, 1.0L, 1.0L);
        specular = V3(0.0L, 0.0L, 0.0L);
    }

    virtual V3 bounce(Ray ray, V3 inormal) {
        // cout << "material bounce\n";
        return getRandomNormalInHemisphere(inormal);
    }
};


class Chrome : public Material {
public:
    Chrome(V3 icolor) : Material(icolor) {
        diffuse = V3(0.4L, 0.4L, 0.4L);
        specular = V3(0.4L, 0.4L, 0.4L);
    }

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
        diffuse = V3(0.1L, 0.1L, 0.1L);
        specular = V3(0.6L, 0.6L, 0.6L);
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
    string name;

    Body(string name, Shape* ishape, Material* imaterial) {
        this->name = name;
        shape = ishape;
        material = imaterial;
    }
};

Body* LEFT;
Body* SCREEN;
Body* FLOOR;
Body* BACK;
Body* TOP;

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
        // From http://www.graphics.cornell.edu/pubs/1997/MT97.pdf

        double first_t = this->tri_intersect2(r, this->p1, this->p2, this->p3);

        // if (this == SCREEN->shape) {
        //     cout << "Testing screen: " << r << ", t = " << first_t << endl;
        // }

        if (first_t != -1.0L) {
            if (this == SCREEN->shape) {
                cout << "First hit screen at t = " << first_t << endl;
            }

            return first_t;
        }

        first_t = this->tri_intersect2(r, this->p1, this->p3, this->p4);

        // if (this == SCREEN->shape) {
        //     cout << "Testing screen: " << r << ", t = " << first_t << endl;
        // }

        if (first_t != -1.0L) {
            if (this == SCREEN->shape) {
                cout << "Second hit screen at t = " << first_t << endl;
            }

            return first_t;
        }

        return -1.0L;
    }

    double tri_intersect2(Ray r, V3 p0, V3 p1, V3 p2) {
        // if (this == LEFT->shape) {
        //     cout << "Testing left wall, " << p0 << ", " << p1 << ", " << p2 << endl;
        // }

        // if (this == BACK->shape) {
        //     cout << "Testing back wall, " << p0 << ", " << p1 << ", " << p2 << endl;
        // }

        // if (this == TOP->shape) {
        //     cout << "Testing top wall, " << p0 << ", " << p1 << ", " << p2 << endl;
        // }

        V3 edge1 = p1.sub(p0);
        V3 edge2 = p2.sub(p0);

        V3 pvec = r.direction.cross(edge2);
        // cout << "edge2: " << edge2 << ", pvec: " << pvec << endl;

        double det = edge1.dot(pvec);

        if (det > -EPSILON && det < EPSILON) {
            // cout << "No hit, determinant: " << det << endl;
            return -1.0L;
        }

        double inv_det = 1.0L / det;

        V3 tvec = r.origin.sub(p0);

        double u = tvec.dot(pvec) * inv_det;
        if ((u < 0.0L) || (u > 1.0L)) {
            // cout << "No hit one: " << u << endl;
            return -1.0L;
        }

        V3 qvec = tvec.cross(edge1);

        double v = r.direction.dot(qvec) * inv_det;
        if ((v < 0.0L) || ((u + v) > 1.0L)) {
            // cout << "No hit two, v: " << v << ", u: " << u << endl;
            // cout << "Hit point would have been: " << r.origin.add(r.direction.muls(edge2.dot(qvec) * inv_det)) << endl;
            return -1.0L;
        }

        return edge2.dot(qvec) * inv_det;
    }

    double another_intersect(Ray r) {
        // From http://stackoverflow.com/questions/8812073/ray-and-square-rectangle-intersection-in-3d

        bool DEBUG = this == FLOOR->shape;

        V3 s1 = this->p2.sub(this->p1);
        V3 s2 = this->p4.sub(this->p1);

        double normal_test = r.direction.dot(this->normal);
        if ((normal_test > -0.00001L) && (normal_test < 0.00001L)) {
            // if (DEBUG) {
            //     cout << "Failed normal test\n";
            // }
            return -1.0L;
        }

        // a = ((P0 - R0).N) / (D.N)
        double t = (this->p1.sub(r.origin).dot(this->normal)) / r.direction.dot(this->normal);

        V3 hit = r.origin.add(r.direction.muls(t));

        // Now you need to check if the point is inside the rect or
        // not. To do this, take projection Q1 of P0P along S1 and Q2
        // of P0P along S2. The condition for the point being inside
        // is then 0 <= length(Q1) <= length(S1) and 0 <= length(Q2)
        // <= length(S2)

        double len_q1 = this->p1.sub(hit).dot(s1);
        double len_q2 = this->p1.sub(hit).dot(s2);
        // V3 q1 = this->p1.sub(hit).dot(s1);
        // V3 q2 = this->p1.sub(hit).dot(s2);
        // double len_q1 = q1.length();
        // double len_q2 = q2.length();

        if ((0.0L <= len_q1) && (len_q1 <= s1.length())) {
            // No action.
            // if (DEBUG) {
            //     cout << "Passed q1\n";
            // }
        } else {
            // if (DEBUG) {
            //     cout << "Failed q1\n";
            // }
            return -1.0L;
        }

        if ((0.0L <= len_q2) && (len_q2 <= s2.length())) {
            // if (DEBUG) {
            //     cout << "Passed q2 test, " << hit << endl;
            // }
            return t;
        } else {
            // if (DEBUG) {
            //     cout << "Failed q2\n";
            // }
            return -1.0L;
        }
    }

    double split_tri_intersect(Ray r) {
        // From http://gamedev.stackexchange.com/questions/7331/ray-plane-intersection-to-find-the-z-of-the-intersecting-point

        double first_t = this->tri_intersect(r, this->p1, this->p2, this->p3);

        // if (this == SCREEN->shape) {
        //     cout << "Testing screen: " << r << ", t = " << first_t << endl;
        // }

        if (first_t != -1.0L) {
            if (this == SCREEN->shape) {
                cout << "First hit screen at t = " << first_t << endl;
            }

            return first_t;
        }

        first_t = this->tri_intersect(r, this->p1, this->p3, this->p4);

        // if (this == SCREEN->shape) {
        //     cout << "Testing screen: " << r << ", t = " << first_t << endl;
        // }

        if (first_t != -1.0L) {
            if (this == SCREEN->shape) {
                cout << "Second hit screen at t = " << first_t << endl;
            }
        }

        return -1.0L;
    }

    double tri_intersect(Ray r, V3 p1, V3 p2, V3 p3) {
        //RAY-TRIANGLE test
        V3 e1 = p2.sub(p1);
        V3 e2 = p3.sub(p1);
        V3 s1 = r.direction.cross(e2);

        if (this == SCREEN->shape) {
            cout << "e1: " << e1 << ", s1: " << s1 << ", dot: " << s1.dot(e1) << endl;
        }

        double divisor = s1.dot(e1);
        if (divisor == 0.0L) {
            return -1.0L; //not hit
        }

        double invDivisor = 1.0L / divisor;

        // Compute first barycentric coordinate
        V3 d = r.origin.sub(p1);

        double b1 = d.dot(s1) * invDivisor;
        if ((b1 < 0.0L) || (b1 > 1.0L)) {
            return -1.0L; // not hit
        }

        // Compute second barycentric coordinate
        V3 s2 = d.cross(e1);
        double b2 = r.direction.dot(s2) * invDivisor;
        if ((b2 < 0.0L) || ((b1 + b2) > 1.0L)) {
            return -1.0L;
        }

        // Compute _t_ to intersection distance
        double t = e2.dot(s2) * invDivisor; /// << HERE you go.        
    }

    double intersect3(Ray r) {
        double a = (this->p1.y * (this->p2.z - this->p3.z)) +
            (this->p2.y * (this->p3.z - this->p1.z)) +
            (this->p3.y * (this->p1.z - this->p2.z));
        double b = (this->p1.z * (this->p2.x - this->p3.x)) +
            (this->p2.z * (this->p3.x - this->p1.x)) +
            (this->p3.z * (this->p1.x - this->p2.x));
        double c = (this->p1.x * (this->p2.y - this->p3.y)) +
            (this->p2.x * (this->p3.y - this->p1.y)) +
            (this->p3.x * (this->p1.y - this->p2.y));
        double d = (-1.0L * (this->p1.x * ((this->p2.y * this->p3.z) - (this->p3.y * this->p2.z)))) -
            (this->p2.x * ((this->p3.y * this->p1.z) - (this->p1.y * this->p3.z))) -
            (this->p3.x * ((this->p1.y * this->p2.z) - (this->p2.y * this->p1.z)));

        double t = -((a * r.origin.x) + (b * r.origin.y) + (c * r.origin.z) + d) /
            ((a * r.direction.x) + (b * r.direction.y) + (c * r.direction.z));

        if (t < 0.000001L) {
            return -1.0L;
        }

        V3 hit = r.origin.add(r.direction.muls(t));

        V3 v4 = hit.sub(p1).normalize();
        V3 v5 = hit.sub(p3).normalize();

        V3 myv = p4.sub(p1).normalize();
        V3 myv2 = p3.sub(p2).normalize();

        cout << "Intersection at " << hit << ", t = " << t << endl;
        cout << "v4: " << v4 << ", v1: " << v1 << ", dot: " << v1.dot(v4) << endl;
        cout << "v5: " << v5 << ", v3: " << v3 << ", dot: " << v3.dot(v5) << endl;
        cout << "v4: " << v4 << ", myv: " << myv << ", dot: " << myv.dot(v4) << endl;
        cout << "v5: " << v5 << ", myv2: " << myv2 << ", dot: " << v3.dot(v5) << endl;

        if ((v1.dot(v4) >= 0.0L) && (v3.dot(v5) >= 0.0L) && (myv.dot(v4) >= 0.0L) && (myv2.dot(v5) >= 0.0L)) {
            return t;
        } else {
            // cout << "Intersection outside rectangle\n";
            return -1.0L;
        }
    }

    double intersect2(Ray r) {
        double n_dot_u = normal.dot(r.direction);

        if ((n_dot_u > -0.00001L) && (n_dot_u < 0.00001L)) {
            // cout << "Failed normal test\n";
            return -1.0L;
        }

        double n_dot_p0 = normal.dot(p1.sub(r.origin));
        double t = n_dot_p0 / n_dot_u;

        // Intersection before origin of ray.
        // if (t < 1.0L) {
        if (t < 0.000001L) {
            // cout << "Intersection before ray: " << t << endl;
            return -1.0L;
        }

        V3 hit = r.origin.add(r.direction.muls(t));

        cout << "Intersection at " << hit << endl;

        V3 v4 = hit.sub(p1).normalize();
        V3 v5 = hit.sub(p3).normalize();

        if ((v1.dot(v4) > 0.0L) && (v3.dot(v5) > 0.0L)) {
            return t;
        } else {
            // cout << "Intersection outside rectangle\n";
            return -1.0L;
        }
    }

    V3 getPointOnSurface(void) {
        V3 v1 = this->p2.sub(this->p1);
        V3 v2 = this->p3.sub(this->p2);

        return this->p1.add(v1.imuls(unifRand(0.0L, 1.0L)).iadd(v2.imuls(unifRand(0.0L, 1.0L))));
    }

    double area(void) {
        return this->p1.sub(this->p2).length() * this->p3.sub(this->p2).length();
    }
};


struct Scene {
    int width;
    int height;
    Camera camera;
    Body** objects;
    int body_count;
    Body* screen;
};


struct PhotonHit {
    V3 point;
    V3 flux;
    V3 incoming;

    PhotonHit(V3 point, V3 flux, V3 incoming) {
        this->point = point;
        this->flux = flux;
        this->incoming = incoming;
    }
};


class Renderer {
public:
    Scene scene;
    V4* buffer;
    int pixels;
    vector<Body*> lights;
    vector<PhotonHit> photon_hits;
    double screen_width;
    double screen_height;

    Renderer(Scene iscene) {
        scene = iscene;

        // HACK Hard coded.
        this->screen_width = 0.8L;
        this->screen_height = 0.8L;

        pixels = scene.width * scene.height;
        buffer = new V4[pixels];
        for (int i = 0; i < pixels; i++) {
            buffer[i] = V4(0.0L, 0.0L, 0.0L);
        }

        for (int i = 0; i < this->scene.body_count; i++) {
            Body* body = this->scene.objects[i];
            if ((body->material->emission.x > 1.0L) || (body->material->emission.y > 1.0L) || (body->material->emission.z > 1.0L)) {
                this->lights.push_back(body);
            }
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
                Ray ray = scene.camera.getRay(x, y);
                V3 color = trace(ray, 0);
                buffer[i++].iadd(color);
            }
        }
    }

    // V3 get_photon_color(V3 point, V3 eye, int photon_count) {
    //     // Get the colour of the point given the number of photons to
    //     // check and the direction to the viewer.
        
        

    //     for (Vector<PhotonHit>::iterator it = this->photon_hits.begin(); it < this->photon_hits.end(); it++) {
            
    //     }
    // }

    void trace_all_photons(int photon_count) {
        // Find total light power.
        double total_power = 0.0L;
        for (vector<Body*>::iterator it = this->lights.begin(); it < this->lights.end(); it++) {
            // BUGBUG Not sure about treating material's power as a pointer.
            total_power += (*it)->material->power;
        }

        // Find the number of photons for each light we should trace.
        vector<int> light_photons = vector<int>();
        for (vector<Body*>::iterator it = this->lights.begin(); it < this->lights.end(); it++) {
            light_photons.push_back((int)(((double)(*it)->material->power / total_power) * (double)photon_count));
        }

        // For each light's photon count trace each photon.
        vector<Body*>::iterator l_it = this->lights.begin();
        for (vector<int>::iterator p_it = light_photons.begin();
             p_it < light_photons.end();
             p_it++,
                 l_it++) {
            cout << "Tracing " << *p_it << " photons\n";
            for (int i = 0; i < *p_it; i++) {
                this->start_photon(**l_it);

                // TODO DEBUG
                return;
            }
        }
    }

    void start_photon(Body light) {
        // Note that it's assumed that the scene is closed around the
        // camera. This means that if a photon does not hit any object
        // then it was shot out of the scene and we'll retrace it.

        Ray ray;

        // Trace until we find a path with at least one hit to account
        // for photons not hitting anything.
        while (true) {
            V3 light_origin = light.shape->getPointOnSurface();
            ray = Ray(light_origin, light.shape->getNormal(light_origin));

            // V3 flux = V3(1.0L, 1.0L, 1.0L);
            V3 flux = light.material->emission;

            if (trace_photon(ray, 0, light.material->emission) != 0) {
                break;
            }

            // DEBUG TODO Break anyway.
            break;

            // cout << "Ray " << ray << " hit nothing, retrying" << endl;
        }
    }

    int trace_photon(Ray ray, int hit_count, V3 flux) {
        bool LOG_DEBUG = true;

        if (hit_count > 4000) {
            cout << "Missed screen after " << hit_count << " hits\n";
            return hit_count;
        }

        Body *hit = NULL;
        double mint = DBL_MAX;
        for (int i = 0; i < scene.body_count; i++) {
            Body *candidate = scene.objects[i];
            double t = candidate->shape->intersect(ray);
            if ((t > 0) && (t <= mint)) {
                mint = t;
                hit = candidate;
            }
        }

        if (hit == NULL) {
            cout << "Hit nothing with ray " << ray << endl;
            return hit_count;
        }

        hit_count++;

        V3 point = ray.origin.add(ray.direction.muls(mint));

        if (LOG_DEBUG) {
            cout << "Hit " << hit->name << " at " << point << " with ray " << ray << endl;
        }

        if (hit == this->scene.screen) {
            if (LOG_DEBUG) {
                cout << "Hit screen at " << point << endl;
            }

            int x = (int)(((point.x / this->screen_width) * (this->scene.width / 2)) + (this->scene.width / 2));
            int z = (int)(((point.z / this->screen_height) * (this->scene.height / 2)) + (this->scene.height / 2));

            if ((x < 0) || (x >= this->scene.width)) {
                if (LOG_DEBUG) {
                    cout << "Point outside buffer: " << x << ", " << z << endl;
                }

                return hit_count;
            }

            if ((z < 0) || (z >= this->scene.height)) {
                if (LOG_DEBUG) {
                    cout << "Point outside buffer: " << x << ", " << z << endl;
                }

                return hit_count;
            }

            this->buffer[x + (z * this->scene.height)].iadd(flux);

            V4 current_color = this->buffer[x + (z * this->scene.height)];
            if (LOG_DEBUG) {
                cout << "Ray: " << ray << endl;
                cout << "Hit screen at " << point << ": " << flux << ", buffer[" << x + (z * this->scene.height) << "] = " << current_color << ", (x, z): (" << x << ", " << z << ")\n";
            }

            return hit_count;
        }

        // photon_hits.push_back(PhotonHit(point, flux, ray.direction));

        V3 normal = hit->shape->getNormal(point);
        V3 direction = hit->material->bounce(ray, normal);
        if (direction.dot(ray.direction) > 0.0f) {
            // If the ray is refracted move the intersection point a
            // bit in.
            // point = ray.origin.add(ray.direction.muls(mint * 1.0000001L));
            point = ray.origin.add(ray.direction.muls(mint * 1.00001L));
        } else {
            // Otherwise move it out to prevent problems with floating
            // point accuracy.
            // point = ray.origin.add(ray.direction.muls(mint * 0.9999999L));
            point = ray.origin.add(ray.direction.muls(mint * 0.99999L));
        }

        flux.imuls(0.4L);

        // flux = flux.imul(hit->material->color).iadd(hit->material->emission);
        flux = flux.imul(hit->material->color).iadd(hit->material->emission.muls(0.5));

        Ray newray = Ray(point, direction.normalize());
        return trace_photon(newray, hit_count, flux);
    }

    V3 trace(Ray ray, int n) {
        if (n > 40) {
            return V3();
        }

        Body *hit = NULL;
        double mint = DBL_MAX;
        for (int i = 0; i < scene.body_count; i++) {
            Body *candidate = scene.objects[i];
            double t = candidate->shape->intersect(ray);

            // if (candidate->name == "right") {
            //     cout << "t on right: " << t << endl;
            // }

            if ((t > 0) && (t <= mint)) {
                mint = t;
                hit = candidate;
            }
        }

        if (hit == NULL) {
            // cout << "Miss after " << n << " bounces: " << ray << endl;
            return V3();
        }

        V3 point = ray.origin.add(ray.direction.muls(mint));
        V3 normal = hit->shape->getNormal(point);
        V3 direction = hit->material->bounce(ray, normal);
        if (direction.dot(ray.direction) > 0.0f) {
            // if the ray is refracted move the intersection point a bit in
            point = ray.origin.add(ray.direction.muls(mint*1.0000001L));
            // point = ray.origin.add(ray.direction.muls(mint*1.00001L));
        } else {
            // otherwise move it out to prevent problems with floating point
            // accuracy
            point = ray.origin.add(ray.direction.muls(mint*0.9999999L));
            // point = ray.origin.add(ray.direction.muls(mint*0.99999L));
        }

        if (hit->name == "Ball Light") {
            // cout << "Ball hit\n";
            return hit->material->color.add(hit->material->emission);
        }

        Ray newray = Ray(point, direction);
        V3 value = trace(newray, n+1).mul(hit->material->color).add(hit->material->emission);

        // cout << "Ended after hitting: " << hit->name << ", " << ray << endl;

        return value;
    }
};

void save(V4* buffer, int samples, int width, int height) {
    ofstream myfile;

    samples = 1;

    stringstream filename;
    filename << "light_trace_" << width << "x" << height << "_" << samples << "spp.ppm";
    cout << "saving to file \"" << filename.str() << "\"" << endl;
    myfile.open(filename.str());

    myfile << "P3\n" << width << " " << height << endl << "255\n";
    V4* pixel = buffer;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int r = (255.0L * pixel->x) / (double)pixel->s;
            int g = (255.0L * pixel->y) / (double)pixel->s;
            int b = (255.0L * pixel->z) / (double)pixel->s;

            if (r > 255) { r = 255; }
            if (g > 255) { g = 255; }
            if (b > 255) { b = 255; }
            // cout << "Pixel: " << *pixel << "\t(" << r << ", " << g << ", " << b << ")" << endl;

            myfile << r << " " << g << " " << b << endl;
            pixel++;
        }
    }

    myfile.close();
}

void worker(int worker_num, int photon_count, int iterations, Scene* scene, V4* buffer) {
    Renderer renderer = Renderer(*scene);

    // TODO When it's ready.
    renderer.trace_all_photons(photon_count);

    for (int i = 0; i < iterations; i++) {
        cout << "Worker " << worker_num << " iteration " << i << endl;
        renderer.iterate();
    }

    boost::mutex::scoped_lock lock(buffer_mutex);

    V4* src_pixel = renderer.buffer;
    V4* dst_pixel = buffer;
    for (int y = 0; y < scene->height; y++) {
        for (int x = 0; x < scene->width; x++) {
            dst_pixel->x += src_pixel->x;
            dst_pixel->y += src_pixel->y;
            dst_pixel->z += src_pixel->z;
            dst_pixel->s += src_pixel->s;

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
    int photon_count = strtol(argv[4], NULL, 10);
    int thread_count = strtol(argv[5], NULL, 10);

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
    Body glass = Body("glass", &glass_sphere, &glass_mat);
    Sphere chrome_sphere = Sphere(V3(-1.1L, 2.8L, 0.0L), 0.5L);
    Chrome chrome_mat = Chrome(V3(0.8L, 0.8L, 0.8L));
    Body chrome = Body("chrome", &chrome_sphere, &chrome_mat);

    Sphere green_sphere = Sphere(V3(-0.15L, 1.55L, -0.4L), 0.1L);
    Glass green_mat = Glass(V3(0.0L, 0.8L, 0.0L), 1.0L, 0.2L);
    Body green = Body("green", &green_sphere, &green_mat);

    Sphere red_sphere = Sphere(V3(0.0L, 1.187867L, -0.4L), 0.1L);
    Glass red_mat = Glass(V3(0.8, 0.0, 0.0), 1.0, 0.2);
    Body red = Body("red", &red_sphere, &red_mat);

    Sphere blue_sphere = Sphere(V3(0.15, 1.55, -0.4), 0.1);
    Glass blue_mat = Glass(V3(0.0, 0.0, 0.8), 1.0, 0.2);
    Body blue = Body("blue", &blue_sphere, &blue_mat);

    Sphere pyramid_sphere = Sphere(V3(0.0L, 1.4L, -0.28L), 0.1L);
    Glass pyramid_mat = Glass(V3(1.00L, 1.00L, 1.00L), 1.5L, 0.1L);
    Body pyramid = Body("pyramid", &pyramid_sphere, &pyramid_mat);

    Material floor_mat = Material(V3(0.9L, 0.9L, 0.9L));
    Rectangle floor_rectangle = Rectangle(V3(-1.9L, 4.5L, -0.5L), V3(1.9L, 4.5L, -0.5L), V3(1.9L, -2.5L, -0.5L), V3(-1.9L, -2.5L, -0.5L));
    Body floor = Body("floor", &floor_rectangle, &floor_mat);
    Material back_mat = Material(V3(0.9L, 0.9L, 0.9L));
    Rectangle back_rectangle = Rectangle(V3(-1.9L, 4.5L, 2.5L), V3(1.9L, 4.5L, 2.5L), V3(1.9L, 4.5L, -0.5L), V3(-1.9L, 4.5L, -0.5L));
    Body back = Body("back", &back_rectangle, &back_mat);
    Material left_mat = Material(V3(0.9L, 0.5L, 0.5L));
    Rectangle left_rectangle = Rectangle(V3(-1.9L, -2.5L, 2.5L), V3(-1.9L, 4.5L, 2.5L), V3(-1.9L, 4.5L, -0.5L), V3(-1.9L, -2.5L, -0.5L));
    Body left = Body("left", &left_rectangle, &left_mat);
    Material right_mat = Material(V3(0.5L, 0.5L, 0.9L));
    Rectangle right_rectangle = Rectangle(V3(1.9L, 4.5L, 2.5L), V3(1.9L, -2.5L, 2.5L), V3(1.9L, -2.5L, -0.5L), V3(1.9L, 4.5L, -0.5L));
    Body right = Body("right", &right_rectangle, &right_mat);
    Material top_mat = Material(V3(0.9L, 0.9L, 0.9L));
    Rectangle top_rectangle = Rectangle(V3(1.9L, 4.5L, 2.5L), V3(-1.9L, 4.5L, 2.5L), V3(-1.9L, -2.5L, 2.5L), V3(1.9L, -2.5L, 2.5L));
    Body top = Body("top", &top_rectangle, &top_mat);
    Material front_mat = Material(V3(0.9L, 0.9L, 0.9L));
    // Rectangle front_rectangle = Rectangle(V3(1.9L, 4.5L, 2.5L), V3(-1.9L, 4.5L, 2.5L), V3(-1.9L, 4.5L, -0.5L), V3(1.9L, 4.5L, -0.5L));
    Rectangle front_rectangle = Rectangle(V3(1.9L, -2.5L, 2.5L), V3(-1.9L, -2.5L, 2.5L), V3(-1.9L, -2.5L, -0.5L), V3(1.9L, -2.5L, -0.5L));
    Body front = Body("front", &front_rectangle, &front_mat);

    // Material floor_mat = Material(V3(0.9L, 0.9L, 0.9L));
    // Plane floor_plane = Plane(V3(0.0L, 3.5L, -0.5L), V3(0.0L, 0.0L, 1.0L));
    // Body floor = Body(&floor_plane, &floor_mat);

    // Plane back_plane = Plane(V3(0.0L, 4.5L, 0.0L), V3(0.0L, -1.0L, 0.0L));
    // Material back_mat = Material(V3(0.9L, 0.9L, 0.9L));
    // Body back = Body(&back_plane, &back_mat);
    // Plane left_plane = Plane(V3(-1.9L, 0.0L, 0.0L), V3(1.0L, 0.0L, 0.0L));
    // Material left_mat = Material(V3(0.9L, 0.5L, 0.5L));
    // Body left = Body(&left_plane, &left_mat);
    // Plane right_plane = Plane(V3(1.9L, 0.0L, 0.0L), V3(-1.0L, 0.0L, 0.0L));
    // Material right_mat = Material(V3(0.5L, 0.5L, 0.9L));
    // Body right = Body(&right_plane, &right_mat);

    Rectangle top_light_box = Rectangle(V3(1.4L, 3.5L, 2.5L), V3(-1.4L, 3.5L, 2.5L), V3(-1.4L, -2.5L, 2.5L), V3(1.4L, -2.5L, 2.5L));
    // Rectangle top_light_box = Rectangle(V3(-1.4L, 3.5L, 2.5L), V3(1.4L, 3.5L, 2.5L), V3(1.4L, -2.5L, 2.5L), V3(-1.4L, -2.5L, 2.5L));
    // Material top_light_mat = Material(V3(0.0L, 0.0L, 0.0L), V3(2.0L, 1.87L, 1.69L), 12.0L);
    // Material top_light_mat = Material(V3(0.0L, 0.0L, 0.0L), V3(1.0L, 0.935L, 0.845L), 1.0L);
    Material top_light_mat = Material(V3(0.5L, 0.5L, 0.5L), V3(0.0L, 0.0L, 0.0L));
    // Material top_light_mat = Material(V3(0.0L, 0.0L, 0.0L), V3(1.1L, 0.935L, 0.845L));
    Body top_light = Body("top light", &top_light_box, &top_light_mat);
    // Plane top_light_plane = Plane(V3(0.0L, 0.0L, 2.5L), V3(0.0L, 0.0L, -1.0L));
    // Material top_light_mat = Material(V3(0.0L, 0.0L, 0.0L), V3(1.6L, 1.47L, 1.29L));
    // // Material top_light_mat = Material(V3(0.0L, 0.0L, 0.0L), V3(1.0L, 0.87L, 0.69L));
    // // Material top_light_mat = Material(V3(0.0L, 0.0L, 0.0L), V3(1.1L, 0.97L, 0.79L));
    // Body top_light = Body(&top_light_plane, &top_light_mat);

    // Plane front_plane = Plane(V3(0.0L, -2.5L, 0.0L), V3(0.0L, 1.0L, 0.0L));
    // Material front_mat = Material(V3(0.9L, 0.9L, 0.9L));
    // Body front = Body(&front_plane, &front_mat);

    // Plane top_left_divider_plane = Plane(V3(-1.8L, 4.4L, 2.4L), V3(1.0L, -1.0L, -1.0L).normalize());
    // Material top_left_divider_mat = Glass(V3(0.0L, 0.8L, 0.0L), 1.0L, 0.2L);
    // Body top_left_divider = Body(&top_left_divider_plane, &top_left_divider_mat);

    // Material ball_light_mat = Material(V3(0.0L, 0.0L, 0.0L), V3(2.0L, 1.87L, 1.69L), 8.0L);
    // Material ball_light_mat = Material(V3(0.0L, 0.0L, 0.0L), V3(20.0L, 18.7L, 16.9L), 8.0L);
    Material ball_light_mat = Material(V3(0.0L, 0.0L, 0.0L), V3(1.2L, 1.37L, 1.29L), 1.80L);

    Sphere right_light_sphere = Sphere(V3(1.9L, 3.625L, 2.1L), 0.1L);
    //Material right_light_mat = Material(V3(0.0L, 0.0L, 0.0L), V3(2.0L, 1.87L, 1.69L), 8);
    Body right_light = Body("Ball Light", &right_light_sphere, &ball_light_mat);

    Sphere left_light_sphere = Sphere(V3(-1.9L, 3.625L, 2.1L), 0.1L);
    //Material left_light_mat = Material(V3(0.0L, 0.0L, 0.0L), V3(2.0L, 1.87L, 1.69L));
    Body left_light = Body("Ball Light", &left_light_sphere, &ball_light_mat);

    Sphere right2_light_sphere = Sphere(V3(1.9L, 2.75L, 2.1L), 0.1L);
    Body right2_light = Body("Ball Light", &right2_light_sphere, &ball_light_mat);
    Sphere left2_light_sphere = Sphere(V3(-1.9L, 2.75L, 2.1L), 0.1L);
    Body left2_light = Body("Ball Light", &left2_light_sphere, &ball_light_mat);
    Sphere right3_light_sphere = Sphere(V3(1.9L, 1.875L, 2.1L), 0.1L);
    Body right3_light = Body("Ball Light", &right3_light_sphere, &ball_light_mat);
    Sphere left3_light_sphere = Sphere(V3(-1.9L, 1.875L, 2.1L), 0.1L);
    Body left3_light = Body("Ball Light", &left3_light_sphere, &ball_light_mat);
    Sphere right4_light_sphere = Sphere(V3(1.9L, 1.0L, 2.1L), 0.1L);
    Body right4_light = Body("Ball Light", &right4_light_sphere, &ball_light_mat);
    Sphere left4_light_sphere = Sphere(V3(-1.9L, 1.0L, 2.1L), 0.1L);
    Body left4_light = Body("Ball Light", &left4_light_sphere, &ball_light_mat);
    Sphere right5_light_sphere = Sphere(V3(1.9L, 0.125L, 2.1L), 0.1L);
    Body right5_light = Body("Ball Light", &right5_light_sphere, &ball_light_mat);
    Sphere left5_light_sphere = Sphere(V3(-1.9L, 0.125L, 2.1L), 0.1L);
    Body left5_light = Body("Ball Light", &left5_light_sphere, &ball_light_mat);
    Sphere right6_light_sphere = Sphere(V3(1.9L, -0.75L, 2.1L), 0.1L);
    Body right6_light = Body("Ball Light", &right6_light_sphere, &ball_light_mat);
    Sphere left6_light_sphere = Sphere(V3(-1.9L, -0.75L, 2.1L), 0.1L);
    Body left6_light = Body("Ball Light", &left6_light_sphere, &ball_light_mat);
    Sphere right7_light_sphere = Sphere(V3(1.9L, -1.625, 2.1L), 0.1L);
    Body right7_light = Body("Ball Light", &right7_light_sphere, &ball_light_mat);
    Sphere left7_light_sphere = Sphere(V3(-1.9L, -1.625L, 2.1L), 0.1L);
    Body left7_light = Body("Ball Light", &left7_light_sphere, &ball_light_mat);

    Sphere back_light_sphere = Sphere(V3(-0.63333L, 4.5L, 2.1L), 0.1L);
    // Sphere back_light_sphere = Sphere(V3(-0.63333L, 4.5L, 2.1L), 0.6L);
    Body back_light = Body("Ball Light", &back_light_sphere, &ball_light_mat);

    Sphere back2_light_sphere = Sphere(V3(0.63333L, 4.5L, 2.1L), 0.1L);
    Body back2_light = Body("Ball Light", &back2_light_sphere, &ball_light_mat);

    Sphere front_light_sphere = Sphere(V3(-0.63333L, -2.5L, 2.1L), 0.1L);
    Body front_light = Body("Ball Light", &front_light_sphere, &ball_light_mat);

    Sphere front2_light_sphere = Sphere(V3(0.63333L, -2.5L, 2.1L), 0.1L);
    Body front2_light = Body("Ball Light", &front2_light_sphere, &ball_light_mat);

    // Plane top_right_divider_plane = Plane(V3(1.4L, 4.0L, 2.0L), V3(-1.0L, -1.0L, -1.0L).normalize());
    // Material top_right_divider_mat = Glass(V3(0.8L, 0.0L, 0.0L), 1.0L, 0.2L);
    // Body top_right_divider = Body(&top_right_divider_plane, &top_right_divider_mat);

    // Plane top_right_divider_plane = Plane(V3(1.8L, 4.4L, 2.4L), V3(-1.0L, -1.0L, -1.0L).normalize());
    // // Material top_right_divider_mat = Material(V3(0.0L, 0.0L, 0.0L), V3(1.6L, 1.47L, 1.29L));
    // Material top_right_divider_mat = Glass(V3(0.8L, 0.0L, 0.0L), 1.0L, 0.2L);
    // Body top_right_divider = Body(&top_right_divider_plane, &top_right_divider_mat);

    Rectangle screen_box = Rectangle(V3(-0.4L, 0.1L, 0.4L), V3(0.4L, 0.1L, 0.4L), V3(0.4L, 0.1L, -0.4L), V3(-0.4L, 0.1L, -0.4L));
    Material screen_mat = Material(V3(0.0L, 0.0L, 0.0L), V3(0.1L, 0.1L, 0.1L), 1.0L);
    Body screen = Body("screen", &screen_box, &screen_mat);

    scene.body_count = 32;
    Body* bodies[] = {&glass, &chrome, &green, &red, &blue, &pyramid, &floor, &back, &left, &right, &top, &top_light, &front,
                      &right_light, &left_light, &back_light, &back2_light, &right2_light, &left2_light,
                      &right3_light, &left3_light, &right4_light, &left4_light,
                      &right5_light, &left5_light, &right6_light, &left6_light, &right7_light, &left7_light,
                      &front_light, &front2_light,
                      &screen};
    // scene.body_count = 14;
    // Body* bodies[] = {&glass, &chrome, &green, &red, &blue, &pyramid, &floor, &back, &left, &right, &top_light, &front, &top_left_divider, &top_right_divider};
    scene.objects = bodies;
    scene.screen = &screen;

    SCREEN = &screen;
    FLOOR = &floor;
    LEFT = &left;
    BACK = &back;
    TOP = &top;

    V4* buffer = new V4[width * height];
    for (int i = 0; i < (width * height); i++) {
        buffer[i] = V4(0.0L, 0.0L, 0.0L);
    }

    boost::thread** threads = new boost::thread*[thread_count];
    for (int i = 0; i < thread_count; i++) {
        threads[i] = new boost::thread(boost::bind(&worker, i, photon_count, (double)iterations / (double)thread_count, &scene, buffer));
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
