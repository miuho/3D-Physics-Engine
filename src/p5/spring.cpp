/**
 * @author HingOn Miu (hmiu)
 */

#include "math/math.hpp"
#include "math/vector.hpp"
#include "math/quaternion.hpp"
#include "math/matrix.hpp"
#include "p5/spring.hpp"
#include "p5/body.hpp"
#include "p5/spherebody.hpp"
#include <iostream>

namespace _462 {

Spring::Spring()
{
    body1_offset = Vector3::Zero();
    body2_offset = Vector3::Zero();
    damping = 0.0;
}

void Spring::step( real_t dt )
{
    Vector3 o1 = body1->orientation * body1_offset;
    Vector3 o2 = body2->orientation * body2_offset;
    // find the location of attachment in world space
    Vector3 p1 = body1->position + o1;
    Vector3 p2 = body2->position + o2;
    Vector3 dir1 = normalize(p1 - p2);
    Vector3 dir2 = normalize(p2 - p1);
    // get the length of the spring
    real_t len = length(p2 - p1);
    Vector3 dis1 = (len - equilibrium) * dir1;
    Vector3 dis2 = (len - equilibrium) * dir2;
    Vector3 v1 = dot(body1->velocity, dir1) * dir1; 
    Vector3 v2 = dot(body2->velocity, dir2) * dir2; 
    // calculate the spring forces
    Vector3 F1 = - constant * dis1 - damping * v1;
    Vector3 F2 = - constant * dis2 - damping * v2;
    // apply force to attached bodies
    body1->apply_force(F1, o1); 
    body2->apply_force(F2, o2); 
}

}


