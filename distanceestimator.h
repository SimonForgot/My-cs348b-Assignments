#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_SHAPES_DISTANCEESTIMATOR_H
#define PBRT_SHAPES_DISTANCEESTIMATOR_H

#include "shape.h"
class DEpara {
  public:
    int maxIters;                // ����������
    float hitEpsilon;            // hit������Ķ�ô��
    float rayEpsilonMultiplier;  // how much we multiply hitEpsilon by to get
                                 // pError
    float normalEpsilon;         // CalculateNormal()��eps����
};
class DistanceEstimator : public Shape {
  public:
    DistanceEstimator(const Transform *ObjectToWorld,
                      const Transform *WorldToObject, bool reverseOrientation,
                      Float radius, Float zMin, Float zMax, Float phiMax,DEpara dep)
        : Shape(ObjectToWorld, WorldToObject, reverseOrientation),
          radius(radius),
          zMin(Clamp(std::min(zMin, zMax), -radius, radius)),
          zMax(Clamp(std::max(zMin, zMax), -radius, radius)),
          thetaMin(std::acos(Clamp(std::min(zMin, zMax) / radius, -1, 1))),
          thetaMax(std::acos(Clamp(std::max(zMin, zMax) / radius, -1, 1))),
          phiMax(Radians(Clamp(phiMax, 0, 360))),
			DEparam(dep){}

    Bounds3f ObjectBound() const;
    Vector3f CalculateNormal(const Point3f &pos, float eps,
                             const Vector3f &defaultNormal) const;
    bool Intersect(const Ray &ray, Float *tHit, SurfaceInteraction *isect,
                   bool testAlphaTexture) const;

    Float Area() const;
    Interaction Sample(const Point2f &u) const;
    virtual Float DistanceEstimator::Evaluate(const Point3f &p) const;


  private:
    // Private Data
    const Float radius;
    const Float zMin, zMax;
    const Float thetaMin, thetaMax, phiMax;
    DEpara DEparam; 
    
};

std::shared_ptr<Shape> CreateDistanceEstimatorShape(const Transform *o2w,
                                                    const Transform *w2o,
                                                    bool reverseOrientation,
                                                    const ParamSet &params);
#endif