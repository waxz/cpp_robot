//
// Created by waxz on 5/31/24.
//

#ifndef LIBROSCPP_POINT_VECTOR3_H
#define LIBROSCPP_POINT_VECTOR3_H
#include <cmath>

namespace geometry{
    // 3D vector
    struct float3
    {
        float x, y, z;

        float3 operator-( float3 v ) const
        {
            return float3{ x - v.x, y - v.y, z - v.z };
        }
        float3 operator+( float3 v ) const
        {
            return float3{ x + v.x, y + v.y, z + v.z };
        }

        void operator -=( float3 v )
        {
            x -= v.x; y -= v.y; z -= v.z;
        }
        float3 operator*( float s ) const
        {
            return float3{ x * s, y * s, z * s };
        }
        [[nodiscard]] float dot( float3 b) const {
            return  x * b.x +  y * b.y +  z * b.z;
        }

        [[nodiscard]] float norm()const{
            return std::sqrt(x*x + y*y +z*z);
        }

        ///
        void normalize(){
            float n = 1.0f/std::sqrt(x*x + y*y +z*z);
            x *= n;
            y *= n;
            z *= n;
        }

    };

// Dot product of 3D vectors
    inline float dot( float3 a, float3 b )
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }
// Length of a 3D vector
    inline float length( float3 v )
    {
        return std::sqrt( dot( v, v ) );
    }
// Distance between line and point in 3D
    inline float distanceToLine( float3 line0, float3 lineDir, float3 point )
    {
        // Translate point making it relative to the start of the line
        point -= line0;
        // Coefficient to get closest point on the line,
        // equal to length(point) * cos(angle) / length(lineDir)
        float pl = dot( lineDir, point ) / dot( lineDir, lineDir );
        // Position of the closest point on the line
        float3 closest = lineDir * pl;
        // Compute distance between two points
        return length( point - closest );
    }
    inline float distanceToLineWithNormDir( float3 line0, float3 lineDir, float3 point )
    {
        // Translate point making it relative to the start of the line
        point -= line0;
        // Coefficient to get closest point on the line,
        // equal to length(point) * cos(angle) / length(lineDir)
        float pl = lineDir.dot(point);
        // Position of the closest point on the line
        float3 closest = lineDir * pl;
        // Compute distance between two points
        return length( point - closest );
    }

}
#endif //LIBROSCPP_POINT_VECTOR3_H
