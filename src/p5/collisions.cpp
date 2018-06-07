/**
 * @author HingOn Miu (hmiu)
 */

#include "p5/collisions.hpp"

namespace _462 {

static const real_t epsilon = 0.1;

/*
 * Change velocity to zero if it is too small.
 * @param v Velocity
 * @return Void.
 */
inline void optimize(Vector3& v) {
    if (length(v) < epsilon) {
        v = Vector3::Zero();
    }
}

/*
 * Detect collision between two spheres.
 * @param body1 The first sphere.
 * @param body2 The second sphere.
 * @param collision_damping The damping factor of collision.
 * @return True if they collide, False otherwise.
 */
bool collides( SphereBody& body1, SphereBody& body2, real_t collision_damping )
{
    // TODO detect collision. If there is one, update velocity
    Vector3 p1 = body1.position;
    Vector3 p2 = body2.position;
    Vector3 v1 = body1.velocity;
    Vector3 v2 = body2.velocity;
    real_t m1 = body1.mass;
    real_t m2 = body2.mass;

    // the spheres are moving away from each other
    if (dot(p2 - p1, v1 - v2) < 0.0) {
        return false; 
    } 
    
    // check if the distance between two centers of spheres is
    // less than the sum of the radii of spheres 
    if (length(p2 - p1) < body1.radius + body2.radius) {
        Vector3 v1p = v1 - v2;
        Vector3 d = (p2 - p1) / length(p2 - p1);
        Vector3 v2pp = 2.0 * d * (m1 / (m1 + m2)) * dot(v1p, d);
        body2.velocity = v2 + v2pp;
        body1.velocity = (m1*v1 + m2*v2 - m2*body2.velocity) / m1;
        body1.velocity *= (1.0 - collision_damping);
        body2.velocity *= (1.0 - collision_damping);
        optimize(body1.velocity);
        optimize(body2.velocity);
        return true;
    }

    return false;
}

/*
 * Check whether the point is inside or outside of the triangle.
 * @param p1 The first point.
 * @param p2 The second point.
 * @param a The triangle vertex.
 * @param b The triangle vertex.
 * @return True if p1 and p2 are both on the same side, False otherwise.
 */
inline bool same_side(Vector3& p1, Vector3& p2, Vector3& a, Vector3& b) {
    Vector3 cp1 = cross(b - a, p1 - a);
    Vector3 cp2 = cross(b - a, p2 - a);
    return (dot(cp1, cp2) >= 0.0);
}

/*
 * Detect collision between sphere and triangle.
 * @param body1 The sphere.
 * @param body2 The triangle.
 * @param collision_damping The damping factor of collision.
 * @return True if they collide, False otherwise.
 */
bool collides( SphereBody& body1, TriangleBody& body2, real_t collision_damping )
{
    // TODO detect collision. If there is one, update velocity
    Vector3 v0 = body2.vertices[0];
    Vector3 v1 = body2.vertices[1];
    Vector3 v2 = body2.vertices[2];
    Vector3 p1 = body1.position;
    Vector3 p2 = body2.position;
    Vector3 a = p1 - p2;
    Vector3 n = normalize(cross(v1 - v0, v2 - v0));
    // get the normal towards the sphere
    n = (dot(a, n) < 0.0) ? -n : n;
    real_t d = dot(a, n);
    d = (d < 0.0) ? -d : d;
    Vector3 v = body1.velocity;

    // the sphere is moving away from triangle
    if (dot(n, v) > 0.0) {
        return false; 
    } 
    
    // project point onto the triangle plane
    Vector3 p1_plane = p1 - d * n;
    
    // Reference: http://www.blackpawn.com/texts/pointinpoly/ 
    // check if the point is within triangle 
    if (same_side(p1_plane, v0, v1, v2) && same_side(p1_plane, v1, v0, v2) &&
        same_side(p1_plane, v2, v0, v1)) {
        // check the distance to triangle if within triangle surface 
        if (d < body1.radius) {
            body1.velocity = v - 2.0 * dot(v, n) * n;
            body1.velocity *= (1.0 - collision_damping);
            optimize(body1.velocity);
            return true;
        }
    }
     
    // project point onto edge v0-v1
    Vector3 p1_edge1 = 
        (dot(p1_plane - v0, v1 - v0) * (v1 - v0))/ squared_length(v1 - v0) + v0;

    // check if touches the edge  
    if (length(p1 - p1_edge1) < body1.radius) {
        body1.velocity = v - 2.0 * dot(v, n) * n;
        body1.velocity *= (1.0 - collision_damping);
        optimize(body1.velocity);
        return true;
    }
    
    // project point onto edge v1-v2
    Vector3 p1_edge2 = 
        (dot(p1_plane - v1, v2 - v1) * (v2 - v1))/ squared_length(v2 - v1) + v1;

    // check if touches the edge  
    if (length(p1 - p1_edge2) < body1.radius) {
        body1.velocity = v - 2.0 * dot(v, n) * n;
        body1.velocity *= (1.0 - collision_damping);
        optimize(body1.velocity);
        return true;
    }

    // project point onto edge v2-v0
    Vector3 p1_edge3 = 
        (dot(p1_plane - v2, v0 - v2) * (v0 - v2))/ squared_length(v0 - v2) + v2;

    // check if touches the edge  
    if (length(p1 - p1_edge3) < body1.radius) {
        body1.velocity = v - 2.0 * dot(v, n) * n;
        body1.velocity *= (1.0 - collision_damping);
        optimize(body1.velocity);
        return true;
    }

    return false;
}

/*
 * Detect collision between sphere and plane.
 * @param body1 The sphere.
 * @param body2 The plane.
 * @param collision_damping The damping factor of collision.
 * @return True if they collide, False otherwise.
 */
bool collides( SphereBody& body1, PlaneBody& body2, real_t collision_damping )
{
    // TODO detect collision. If there is one, update velocity
    Vector3 p1 = body1.position;
    Vector3 p2 = body2.position;
    Vector3 a = p1 - p2;
    Vector3 n = body2.normal;
    // get the normal towards the sphere
    n = (dot(a, n) < 0.0) ? -n : n;
    real_t d = dot(a, n);
    d = (d < 0.0) ? -d : d;
    Vector3 v = body1.velocity;

    // the sphere is moving away from plane
    if (dot(n, v) > 0.0) {
        return false;   
    } 
    
    // check if projected distance on normal is less than sphere radius 
    if (d < body1.radius) {
        body1.velocity = v - 2.0 * dot(v, n) * n;
        body1.velocity *= (1.0 - collision_damping);
        optimize(body1.velocity);
        return true;
    }
    
    return false;
}

}
