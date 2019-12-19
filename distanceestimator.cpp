#include "distanceestimator.h"
#include <cmath>
#include "efloat.h"
#include "paramset.h"
#include "shapes/sphere.h"
std::shared_ptr<Shape> CreateDistanceEstimatorShape(const Transform *o2w,
                                                    const Transform *w2o,
                                                    bool reverseOrientation,
                                                    const ParamSet &params) {
    Float radius = params.FindOneFloat("radius", 1.f);
    Float zmin = params.FindOneFloat("zmin", -radius);
    Float zmax = params.FindOneFloat("zmax", radius);
    Float phimax = params.FindOneFloat("phimax", 360.f);
    DEpara de;
    de.hitEpsilon = params.FindOneFloat("hitepsilon", 0.01);
    de.maxIters = params.FindOneInt("maxiters", 1000);
    de.normalEpsilon = params.FindOneFloat("", 0.01);
    de.rayEpsilonMultiplier = params.FindOneFloat("rayepsilonmultiplier", 10);
    return std::make_shared<DistanceEstimator>(o2w, w2o, reverseOrientation,
                                               radius, zmin, zmax, phimax, de);
}

Bounds3f DistanceEstimator::ObjectBound() const {
    return Bounds3f(Point3f(-radius, -radius, zMin),
                    Point3f(radius, radius, zMax));
}
Vector3f DistanceEstimator::CalculateNormal(
    const Point3f &pos, float eps, const Vector3f &defaultNormal) const {
    const Vector3f v1 = Vector3f(1.0, -1.0, -1.0);
    const Vector3f v2 = Vector3f(-1.0, -1.0, 1.0);
    const Vector3f v3 = Vector3f(-1.0, 1.0, -1.0);
    const Vector3f v4 = Vector3f(1.0, 1.0, 1.0);
    
    const Vector3f normal =
        v1 * Evaluate(pos + v1 * eps) + v2 * Evaluate(pos + v2 * eps) +
        v3 * Evaluate(pos + v3 * eps) + v4 * Evaluate(pos + v4 * eps);
    const Float length = normal.Length();

    return length > 0 ? (normal / length) : defaultNormal;
}
bool DistanceEstimator::Intersect(const Ray &r, Float *tHit,
                                  SurfaceInteraction *isect,
                                  bool testAlphaTexture) const {
    Point3f p = r.o;
    float t = 0;
    int iter = 0;
    // stop conditions:1.find 2.ray.tmax 3.maxiter
    while (Evaluate(p + t * r.d) > DEparam.hitEpsilon ) {
if (t >= r.tMax || iter >= DEparam.maxIters) return false;
        float temp = Evaluate(p + t * r.d);
        t +=  temp/ r.d.Length();
        iter++;
    }
    *tHit = t;
    auto n = CalculateNormal(p + t * r.d, DEparam.normalEpsilon, -r.d);
    decltype(n) a, b;
    if (std::abs(n.x) > std::abs(n.y))
        a = Vector3f(-n.z, 0, n.x) / std::sqrt(n.x * n.x + n.z * n.z);
    else
        a = Vector3f(0, n.z, -n.y) / std::sqrt(n.y * n.y + n.z * n.z);
    b = Cross(n, a);
    
    *isect = SurfaceInteraction(p + t * r.d, Vector3f(0.1,0.1,0.1),
        Point2f(0, 0), -r.d, a, b, Normal3f(0,0,0), Normal3f(0,0,0), r.time, this);
    return true;
}

Float DistanceEstimator::Area() const {
    return phiMax * radius * (zMax - zMin);
}

Interaction DistanceEstimator::Sample(const Point2f &u) const {
    return Interaction();
}

Float DistanceEstimator::Evaluate(const Point3f &p) const {
    return sqrt(p.x * p.x + p.y * p.y + p.z * p.z) - radius;
}
