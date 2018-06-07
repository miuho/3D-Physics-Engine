/**
 * @author HingOn Miu (hmiu)
 */

#ifndef _462_PHYSICS_PHYSICS_HPP_
#define _462_PHYSICS_PHYSICS_HPP_

#include "math/math.hpp"
#include "math/quaternion.hpp"
#include "math/vector.hpp"
#include "p5/body.hpp"
#include "p5/spherebody.hpp"
#include "p5/trianglebody.hpp"
#include "p5/planebody.hpp"
#include "p5/spring.hpp"
#include "p5/collisions.hpp"
#include <SDL.h>
#include <vector>

namespace _462 {

class Physics
{
public:
    Vector3 gravity;
    real_t collision_damping;

    Physics();
    ~Physics();

    void step( real_t dt );
    void add_sphere( SphereBody* s );
    size_t num_spheres() const;
    void add_plane( PlaneBody* p );
    size_t num_planes() const;
    void add_triangle( TriangleBody* t );
    size_t num_triangles() const;
    void add_spring( Spring* s );
    size_t num_springs() const;

    void reset();
    void detect_collisions();
    void exert_force(SphereBody *sb);
    void integrate(SphereBody *sb,
                   Vector3& x_origin, Vector3& v_origin,
                   Quaternion& w_origin, Vector3& u_origin, 
                   Vector3& prev_dx, Vector3& prev_dv, 
                   Vector3& prev_dw, Vector3& prev_du, 
                   Vector3& curr_dx, Vector3& curr_dv, 
                   Vector3& curr_dw, Vector3& curr_du, 
                   real_t dt, real_t factor); 

private:
    typedef std::vector< Spring* > SpringList;
    typedef std::vector< SphereBody* > SphereList;
    typedef std::vector< PlaneBody* > PlaneList;
    typedef std::vector< TriangleBody* > TriangleList;

    SpringList springs;
    SphereList spheres;
    PlaneList planes;
    TriangleList triangles;
};

}

#endif

