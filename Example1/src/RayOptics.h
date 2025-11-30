#pragma once
#include "mathStructure.h"
#include <vector>

namespace Optics {
    // Ray-surface hit data embedded in ray structure
    struct RayHit {
        NodeWeft::Vec2 point;        // Intersection point
        NodeWeft::Vec2 normal;       // Surface normal at intersection
        double distance;             // Distance from previous point
        double refractiveIndexBefore; // Refractive index before this interface
        double refractiveIndexAfter;  // Refractive index after this interface

        RayHit()
            : point{0, 0}, normal{0, 1}, distance(0.0),
              refractiveIndexBefore(1.0), refractiveIndexAfter(1.0) {}

        RayHit(const NodeWeft::Vec2& p, const NodeWeft::Vec2& n,
                   double d, double nBefore, double nAfter)
            : point(p), normal(n), distance(d),
              refractiveIndexBefore(nBefore), refractiveIndexAfter(nAfter) {}
    };

    // Ray structure with path history (2D version)
    struct Ray {
        // Path of ray through optical system (starting point + all intersection points)
        std::vector<NodeWeft::Vec2> path;

        // Direction from the last point in path
        NodeWeft::Vec2 direction;

        // Ray properties
        double wavelength;          // Wavelength in nanometers (e.g., 550nm for green)
        double intensity;           // Intensity/power of the ray (0.0 to 1.0)

        // Hit history - stores all intersection data
        std::vector<RayHit> hits;

        // Constructors
        Ray() : direction{0, 1}, wavelength(550.0), intensity(1.0) {
            path.push_back(NodeWeft::Vec2{0, 0});  // Starting point
        }

        Ray(const NodeWeft::Vec2& orig, const NodeWeft::Vec2& dir,
            double wl = 550.0, double intens = 1.0)
            : direction(NodeWeft::normalize(dir)), wavelength(wl), intensity(intens) {
            path.push_back(orig);  // Starting point
        }

        // Get current ray origin (last point in path)
        NodeWeft::Vec2 getOrigin() const {
            return path.empty() ? NodeWeft::Vec2{0, 0} : path.back();
        }

        // Get point along the ray at distance t from current origin
        NodeWeft::Vec2 pointAt(double t) const {
            return getOrigin() + direction * t;
        }

        // Add a hit point to the ray path
        void addHit(const NodeWeft::Vec2& hitPoint, const NodeWeft::Vec2& normal, 
        double nBefore, double nAfter, double distance = 0.0) {
            // Calculate distance if not provided
            if (distance < 0.0001) {
                distance = NodeWeft::length(hitPoint - getOrigin());
            }
            path.push_back(hitPoint);
            hits.emplace_back(hitPoint, normal, distance, nBefore, nAfter);
        }

        // Get the last hit information
        const RayHit* getLastHit() const {
            return hits.empty() ? nullptr : &hits.back();
        }

        // Get the color using wavelength
        NodeWeft::tRGB getWavelengthColor() const;
    };

    // Circular lens/surface structure (2D)
    // In 2D, a sphere becomes a circle
    struct SphereLens {
        NodeWeft::Vec2 center;      // Center of the Lens surface
        double radius;              // Radius of curvature (positive = convex, negative = concave)
        double refractiveIndex;     // Refractive index of the material (at 589nm - sodium D-line)
        bool isEntrance;            // True if ray enters material, false if exits

        // Dispersion model: simplified Cauchy's equation
        // n(£f) = n0 + B / £f^2
        // where n0 is refractiveIndex, B is the dispersion coefficient
        double dispersiveCoefficient; // B coefficient for Cauchy's equation (nm^2)

        SphereLens() : center{0, 0}, radius(100.0), refractiveIndex(1.5), isEntrance(true), dispersiveCoefficient(0.0) {}
        
        SphereLens(const NodeWeft::Vec2& c, double r, double n, bool entrance = true) 
            : center(c), radius(r), refractiveIndex(n), isEntrance(entrance), dispersiveCoefficient(0.0) {}

        // Full constructor with dispersion
        SphereLens(const NodeWeft::Vec2& c, double r, double n, bool entrance, double dispersionCoeff)
        : center(c), radius(r), refractiveIndex(n), isEntrance(entrance),  dispersiveCoefficient(dispersionCoeff) {}

        // Get the surface normal at a point on the circle
        NodeWeft::Vec2 getNormalAt(const NodeWeft::Vec2& point) const {
            NodeWeft::Vec2 normal = NodeWeft::normalize(point - (center + NodeWeft::Vec2{ radius,0 }));
            // For concave surfaces or exit surfaces, flip normal if needed
            return radius > 0 ? normal : -normal;
        }

        // Get wavelength-dependent refractive index using Cauchy's equation
        // n(£f) = n0 + B / £f^2
        // where £f is wavelength in nanometers
        double getRefractiveIndexAtWavelength(double wavelength) const {
            if (wavelength <= 0.0) {
                return refractiveIndex;  // Return base index for invalid wavelength
            }
            // Convert wavelength-dependent variation
            // B is in nm^2, so we need to account for this
            double wavelengthMicrons = wavelength / 1000.0;  // Convert nm to micrometers
            double n_lambda = refractiveIndex + (dispersiveCoefficient / (wavelengthMicrons * wavelengthMicrons));
            return n_lambda;
        }
    };

    // Find intersection between ray and circular surface, returns true if hit occurred
    // Updates ray with hit information if intersection found
    bool intersectAndUpdateRay(Ray& ray, const SphereLens& sphere, double refractiveIndexBefore = 1.0);

    // Calculate refraction of a ray and update ray direction
    // Returns false if total internal reflection occurs (ray is not updated)
    bool refractRay(Ray& ray, const SphereLens& surface);

    // Calculate Fresnel reflectance (fraction of light reflected vs refracted)
    double fresnelReflectance(const NodeWeft::Vec2& incident, const NodeWeft::Vec2& normal, double n1, double n2);

} // namespace Optics
