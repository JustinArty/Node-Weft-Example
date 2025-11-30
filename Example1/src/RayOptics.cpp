#include "RayOptics.h"
#include <cmath>
#include <algorithm>

namespace Optics {

    NodeWeft::tRGB Ray::getWavelengthColor() const
    {
        double wl = wavelength;
        double r = 0.0, g = 0.0, b = 0.0;
        double gamma = 0.8;  // Gamma correction factor for color perception

        // Handle out-of-range wavelengths (infrared or ultraviolet)
        if (wl < 380.0 || wl > 700.0) {
            return NodeWeft::tRGB{ 0, 0, 0 };  // Return black for non-visible wavelengths
        }

        // Calculate RGB components based on wavelength
        // Using a simplified model of the visible spectrum
        if (wl >= 380.0 && wl < 440.0) {
            // Violet to Blue (380-440 nm)
            r = -(wl - 440.0) / (440.0 - 380.0);
            g = 0.0;
            b = 1.0;
        }
        else if (wl >= 440.0 && wl < 490.0) {
            // Blue to Cyan (440-490 nm)
            r = 0.0;
            g = (wl - 440.0) / (490.0 - 440.0);
            b = 1.0;
        }
        else if (wl >= 490.0 && wl < 510.0) {
            // Cyan to Green (490-510 nm)
            r = 0.0;
            g = 1.0;
            b = -(wl - 510.0) / (510.0 - 490.0);
        }
        else if (wl >= 510.0 && wl < 580.0) {
            // Green to Yellow (510-580 nm)
            r = (wl - 510.0) / (580.0 - 510.0);
            g = 1.0;
            b = 0.0;
        }
        else if (wl >= 580.0 && wl < 645.0) {
            // Yellow to Red (580-645 nm)
            r = 1.0;
            g = -(wl - 645.0) / (645.0 - 580.0);
            b = 0.0;
        }
        else if (wl >= 645.0 && wl <= 700.0) {
            // Deep Red (645-700 nm)
            r = 1.0;
            g = 0.0;
            b = 0.0;
        }

        // Apply intensity factor based on eye's photopic luminosity function
        // The eye is most sensitive around 555 nm and less sensitive at spectrum edges
        double factor = 1.0;
        if (wl < 420.0 || wl > 700.0) {
            factor = 0.3;  // Reduced brightness for edges
        }
        else if (wl > 645.0) {
            factor = 0.8;  // Red region has lower luminosity
        }
        else if (wl < 420.0) {
            factor = 0.4;  // Violet region has lower luminosity
        }

        // Apply gamma correction for perceptually uniform color display
        r = std::pow((std::max)(0.0, r), 1.0 / gamma) * factor;
        g = std::pow((std::max)(0.0, g), 1.0 / gamma) * factor;
        b = std::pow((std::max)(0.0, b), 1.0 / gamma) * factor;

        // Scale to 0-255 range and apply ray intensity
        uint8_t r_byte = static_cast<uint8_t>((std::max)(0.0, (std::min)(255.0, r * 255.0 * intensity)));
        uint8_t g_byte = static_cast<uint8_t>((std::max)(0.0, (std::min)(255.0, g * 255.0 * intensity)));
        uint8_t b_byte = static_cast<uint8_t>((std::max)(0.0, (std::min)(255.0, b * 255.0 * intensity)));

        return NodeWeft::tRGB{ r_byte, g_byte, b_byte };
    }

    bool intersectAndUpdateRay(Ray& ray, const SphereLens& sphere, double refractiveIndexBefore) {
        // Vector from ray origin to circle center (2D)
        NodeWeft::Vec2 rayOrigin = ray.getOrigin();
        NodeWeft::Vec2 oc = rayOrigin - (sphere.center + NodeWeft::Vec2{ sphere.radius,0 });
        
        // Solve quadratic equation: t^2*d.d + 2*t*oc.d + oc.oc - r^2 = 0
        double a = NodeWeft::dot(ray.direction, ray.direction);
        double b = 2.0 * NodeWeft::dot(oc, ray.direction);
        double c = NodeWeft::dot(oc, oc) - sphere.radius * sphere.radius;
      
        double discriminant = b * b - 4 * a * c;
    
        // No intersection if discriminant is negative
        if (discriminant < 0) {
            return false;
        }
        
        // Find the nearest intersection point (smallest positive t)
        double sqrtDisc = std::sqrt(discriminant);
        double t1 = (-b - sqrtDisc) / (2.0 * a);
        double t2 = (-b + sqrtDisc) / (2.0 * a);

        // Calculate intersection point 
        bool isHit = false;
		double t = 0.0;
        NodeWeft::Vec2 hitPoint;
        if (t1 > 0.0001) { // try t1 first
			t = t1;
			hitPoint = ray.pointAt(t1);
            if(sphere.radius > 0 && hitPoint.x < sphere.center.x + sphere.radius)
                isHit = true;
            else if(sphere.radius < 0 && hitPoint.x > sphere.center.x + sphere.radius)
				isHit = true;
        }
		if (!isHit && t2 > 0.0001) { // then try t2
			t = t2;
			hitPoint = ray.pointAt(t2);
            if(sphere.radius > 0 && hitPoint.x < sphere.center.x + sphere.radius)
                isHit = true;
			else if (sphere.radius < 0 && hitPoint.x > sphere.center.x + sphere.radius)
				isHit = true;
        }
        if(!isHit)
			return false; // No valid intersection

		// Get surface normal at hit point
        NodeWeft::Vec2 normal = sphere.getNormalAt(hitPoint);
        
        // Add hit to ray path
        double refractiveIndexAfter = sphere.isEntrance ? sphere.refractiveIndex : 1.0;
        ray.addHit(hitPoint, normal, refractiveIndexBefore, refractiveIndexAfter, t);
     
        return true;
    }

    bool refractRay(Ray& ray, const SphereLens& surface) {
        // Get the last hit information
       const RayHit* lastHit = ray.getLastHit();
       if (!lastHit) {
           return false;  // No hit information available
       }

        // Get refractive indices with wavelength dispersion
        double n1_base = lastHit->refractiveIndexBefore;
        double n2_base = lastHit->refractiveIndexAfter;
   
        // Apply wavelength-dependent dispersion
        // If entering material (isEntrance), n2 varies with wavelength
        // If exiting material (!isEntrance), n1 varies with wavelength
        double n1, n2;
        
        if (surface.isEntrance) {
            // Ray entering: n1 = air, n2 = material(wavelength dependent)
            n1 = n1_base;
            n2 = surface.getRefractiveIndexAtWavelength(ray.wavelength);
        }
        else {
            // Ray exiting: n1 = material(wavelength dependent), n2 = air
            n1 = surface.getRefractiveIndexAtWavelength(ray.wavelength);
            n2 = n2_base;
        }
     
        double eta = n1 / n2; // Ratio of refractive indices
        
        NodeWeft::Vec2 normal = lastHit->normal;
        double cosI = -NodeWeft::dot(ray.direction, normal);
        
        // Make sure normal points against incident ray
        if (cosI < 0) {
            cosI = -cosI;
            normal = -normal;
        }
      
        // Calculate refracted direction using Snell's law
        double sinT2 = eta * eta * (1.0 - cosI * cosI);
     
        // Check for total internal reflection
        if (sinT2 > 1.0) {
            return false; 
        }
        
        double cosT = std::sqrt(1.0 - sinT2);
        NodeWeft::Vec2 refractedDir = ray.direction * eta + normal * (eta * cosI - cosT);
 
        // Update ray direction
        ray.direction = NodeWeft::normalize(refractedDir);
   
        return true;
    }

    double fresnelReflectance(const NodeWeft::Vec2& incident, const NodeWeft::Vec2& normal, double n1, double n2) {
        double cosI = std::abs(NodeWeft::dot(incident, normal));

        // Snell's law
        double sinT2 = (n1 / n2) * (n1 / n2) * (1.0 - cosI * cosI);

        // Total internal reflection
        if (sinT2 > 1.0) {
            return 1.0;
        }

        double cosT = std::sqrt(1.0 - sinT2);

        // Fresnel equations
        double rs = ((n1 * cosI - n2 * cosT) / (n1 * cosI + n2 * cosT));
        double rp = ((n1 * cosT - n2 * cosI) / (n1 * cosT + n2 * cosI));

        // Average of s and p polarization
        return (rs * rs + rp * rp) * 0.5;
    }

} // namespace Optics
