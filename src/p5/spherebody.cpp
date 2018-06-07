/**
 * @author HingOn Miu (hmiu)
 */

#include "p5/spherebody.hpp"
#include "math/vector.hpp"
#include "math/matrix.hpp"
#include "scene/sphere.hpp"
#include <SDL.h>
#include <iostream>
#include <exception>
#include <algorithm>

namespace _462 {

SphereBody::SphereBody( Sphere* geom )
{
    sphere = geom;
    position = sphere->position;
    radius = sphere->radius;
    orientation = sphere->orientation;
    mass = 0.0;
    velocity = Vector3::Zero();
    angular_velocity = Vector3::Zero();
    force = Vector3::Zero();
    torque = Vector3::Zero();
}

Vector3 SphereBody::step_position( real_t dt, real_t motion_damping )
{
    // calculate and return the  acceleration 
    return force / mass;
}

Vector3 SphereBody::step_orientation( real_t dt, real_t motion_damping )
{
    // calculate and return the angular acceleration 
    real_t I = (2.0/5.0) * mass * radius * radius;
    return torque / I;
}

void SphereBody::apply_force( const Vector3& f, const Vector3& offset )
{
    if (offset == Vector3::Zero() || cross(f, offset) == Vector3::Zero()) {
        // only apply linear force if the offset from center of mass is zero
        // or if the offset and the force direction are parallel.
        force += f;
    }
    else {
        force += f;
        // apply torque as well
        torque += cross(offset, f);
    }
}

}
