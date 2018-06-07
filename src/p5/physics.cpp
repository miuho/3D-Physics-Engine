/**
 * @author HingOn Miu (hmiu)
 */

#include "p5/physics.hpp"

namespace _462 {

Physics::Physics()
{
    reset();
}

Physics::~Physics()
{
    reset();
}

/*
 * Rotate the quaternion by angle radians.
 * @param axis The rotating axis.
 * @param radians The angle.
 * @param orientation The quaternion.
 * @return Void.
 */
void rotate( const Vector3& axis, real_t radians, Quaternion& orientation ) {
    orientation = normalize( Quaternion( axis, radians ) * orientation );
}

/*
 * Rotate the quaternion by angle radians along the x axis.
 * @param radians The angle.
 * @param orientation The quaternion.
 * @return Void.
 */
void pitch( real_t radians, Quaternion& orientation ) {
    rotate( orientation * Vector3::UnitX(), radians, orientation );
}

/*
 * Rotate the quaternion by angle radians along the z axis.
 * @param radians The angle.
 * @param orientation The quaternion.
 * @return Void.
 */
void roll( real_t radians, Quaternion& orientation ) {
    rotate( orientation * Vector3::UnitZ(), radians, orientation );
}

/*
 * Rotate the quaternion by angle radians along the y axis.
 * @param radians The angle.
 * @param orientation The quaternion.
 * @return Void.
 */
void yaw( real_t radians, Quaternion& orientation ) {
    rotate( orientation * Vector3::UnitY(), radians, orientation );
}

/*
 * Rotate the quaternion.
 * @param dest_w The destination quaternion.
 * @param src_w The source quaternion.
 * @param dw The angle to rotate along each axis.
 * @return Void.
 */
void spin(Quaternion& dest_w, Quaternion& src_w, Vector3 dw) {
    dest_w = src_w;
    pitch(dw.x, dest_w);
    roll(dw.z, dest_w);
    yaw(dw.y, dest_w);
}

/*
 * Detect collisions among geometries.
 * @return Void.
 */
void Physics::detect_collisions() {
    size_t i, j;
    for (i = 0; i < num_spheres(); i++) {
        // able to assume that at most one collision happens at one time
        bool collided = false;
        SphereBody *sb = spheres[i];
        // detect collisions with planes
        for (j = 0; j < num_planes(); j++) {
            PlaneBody *pb = planes[j];
            if (collided == false && 
                collides(*sb, *pb, collision_damping) == true) {
                collided = true;
            }
        }
        // detect collisions with triangles
        for (j = 0; j < num_triangles(); j++) {
            TriangleBody *tb = triangles[j];
            if (collided == false &&
                collides(*sb, *tb, collision_damping) == true) {
                collided = true;
            }
        }
        // detect collisions with other spheres
        for (j = 0; j < num_spheres(); j++) {
            if (i == j) 
                continue;
            SphereBody *sb2 = spheres[j];
            if (collided == false &&
                collides(*sb, *sb2, collision_damping) == true) {
                collided = true;
            }
        }
    }
}

/*
 * Update position, velocity, orientation and angular velocity.
 * @param sb The sphere.
 * @param x_origin The original position of sphere.
 * @param v_origin The original velocity of sphere.
 * @param w_origin The original orientation of sphere.
 * @param u_origin The original angular velocity of sphere.
 * @param dx The change position of sphere.
 * @param dv The change velocity of sphere.
 * @param dw The change orientation of sphere.
 * @param du The change angular velocity of sphere.
 * @return Void.
 */
void update(SphereBody* sb,
            Vector3& x_origin, Vector3& v_origin,
            Quaternion& w_origin, Vector3& u_origin, 
            Vector3& dx, Vector3& dv, 
            Vector3& dw, Vector3& du) { 
    sb->position = x_origin + dx;
    sb->sphere->position = sb->position;
    sb->velocity = v_origin + dv;
    spin(w_origin, w_origin, dw);
    sb->orientation = w_origin;
    sb->sphere->orientation = sb->orientation;
    sb->angular_velocity = u_origin + du;
}

/*
 * Apply gravity and spring force to the sphere.
 * @param sb The sphere.
 * @return Void.
 */
void Physics::exert_force(SphereBody *sb) {
    // apply gravity force
    sb->apply_force(sb->mass * gravity, Vector3::Zero());
    size_t i;
    for (i = 0; i < num_springs(); i++) {
        Spring *s = springs[i];
        if (s->body1->id == sb->id ||
            s->body2->id == sb->id) {
            // apply spring force
            s->step(0.0);
            break;
        }
    }
}

/*
 * Use RK4 method to calculate k.
 * @param sb The sphere.
 * @param x_origin The original position of sphere.
 * @param v_origin The original velocity of sphere.
 * @param w_origin The original orientation of sphere.
 * @param u_origin The original angular velocity of sphere.
 * @param prev_dx The previous k of position.
 * @param prev_dv The previous k of velocity.
 * @param prev_dw The previous k of orientation.
 * @param prev_du The previous k of angular velocity.
 * @param curr_dx The current k of position.
 * @param curr_dv The current k of velocity.
 * @param curr_dw The current k of orientation.
 * @param curr_du The current k of angular velocity.
 * @param h The step size in time.
 * @param factor The time factor.
 * @return Void.
 */
void Physics::integrate(SphereBody *sb,
                        Vector3& x_origin, Vector3& v_origin,
                        Quaternion& w_origin, Vector3& u_origin, 
                        Vector3& prev_dx, Vector3& prev_dv, 
                        Vector3& prev_dw, Vector3& prev_du, 
                        Vector3& curr_dx, Vector3& curr_dv, 
                        Vector3& curr_dw, Vector3& curr_du, 
                        real_t h, real_t factor) {
    // step the current states by factor
    sb->position = x_origin + prev_dx * factor;
    spin(sb->orientation, w_origin, prev_dw * factor);
    sb->velocity = v_origin + prev_dv * factor;
    sb->angular_velocity = u_origin + prev_du * factor;
    // apply forces
    exert_force(sb);
    // https://www.youtube.com/watch?v=smfX0Jt_f0I
    curr_dv = h * sb->step_position(0.0, 0.0);
    curr_dx = h * sb->velocity;
    curr_du = h * sb->step_orientation(0.0, 0.0);
    curr_dw = h * sb->angular_velocity;   
    // clear accumulators
    sb->force = Vector3::Zero(); 
    sb->torque = Vector3::Zero(); 
}

/*
 * Step the geometries in the scene.
 * @param dt The step size in time.
 * @return Void.
 */
void Physics::step( real_t dt )
{
    detect_collisions(); 

    size_t i;
    for (i = 0; i < num_spheres(); i++) {
        SphereBody *sb = spheres[i];
        // record the original state
        Vector3 x_origin = sb->position;
        Vector3 v_origin = sb->velocity;
        Quaternion w_origin = sb->orientation;
        Vector3 u_origin = sb->angular_velocity;        
        // initialize the k variables 
        Vector3 dx1 = Vector3::Zero();
        Vector3 dv1 = Vector3::Zero();
        Vector3 dw1 = Vector3::Zero();
        Vector3 du1 = Vector3::Zero();
        Vector3 dx2 = Vector3::Zero();
        Vector3 dv2 = Vector3::Zero();
        Vector3 dw2 = Vector3::Zero();
        Vector3 du2 = Vector3::Zero();
        Vector3 dx3 = Vector3::Zero();
        Vector3 dv3 = Vector3::Zero();
        Vector3 dw3 = Vector3::Zero();
        Vector3 du3 = Vector3::Zero();
        Vector3 dx4 = Vector3::Zero();
        Vector3 dv4 = Vector3::Zero();
        Vector3 dw4 = Vector3::Zero();
        Vector3 du4 = Vector3::Zero();
        // Use RK4 to calculate k variables 
        integrate(sb, x_origin, v_origin, w_origin, u_origin, 
                  dx1, dv1, dw1, du1, dx1, dv1, dw1, du1, dt, 0.0);
        integrate(sb, x_origin, v_origin, w_origin, u_origin, 
                  dx1, dv1, dw1, du1, dx2, dv2, dw2, du2, dt, 0.5);
        integrate(sb, x_origin, v_origin, w_origin, u_origin, 
                  dx2, dv2, dw2, du2, dx3, dv3, dw3, du3, dt, 0.5);
        integrate(sb, x_origin, v_origin, w_origin, u_origin, 
                  dx3, dv3, dw3, du3, dx4, dv4, dw4, du4, dt, 1.0);
        Vector3 dx = (1.0/6.0)*dx1 + (1.0/3.0)*dx2 + (1.0/3.0)*dx3 + (1.0/6.0)*dx4;
        Vector3 dv = (1.0/6.0)*dv1 + (1.0/3.0)*dv2 + (1.0/3.0)*dv3 + (1.0/6.0)*dv4;
        Vector3 dw = (1.0/6.0)*dw1 + (1.0/3.0)*dw2 + (1.0/3.0)*dw3 + (1.0/6.0)*dw4;
        Vector3 du = (1.0/6.0)*du1 + (1.0/3.0)*du2 + (1.0/3.0)*du3 + (1.0/6.0)*du4;
        
        // update the state of the sphere 
        update(sb, x_origin, v_origin, w_origin, u_origin, dx, dv, dw, du); 
    }
}

/*
 * Add sphere to sphere list.
 * @param b The sphere.
 * @return Void.
 */
void Physics::add_sphere( SphereBody* b )
{
    spheres.push_back( b );
}

/*
 * Get the number of spheres.
 * @return The sphere count.
 */
size_t Physics::num_spheres() const
{
    return spheres.size();
}

/*
 * Add plane to plane list.
 * @param p The plane.
 * @return Void.
 */
void Physics::add_plane( PlaneBody* p )
{
    planes.push_back( p );
}

/*
 * Get the number of planes.
 * @return The plane count.
 */
size_t Physics::num_planes() const
{
    return planes.size();
}

/*
 * Add triangle to triangle list.
 * @param t The triangle.
 * @return Void.
 */
void Physics::add_triangle( TriangleBody* t )
{
    triangles.push_back( t );
}

/*
 * Get the number of triangles.
 * @return The triangle count.
 */
size_t Physics::num_triangles() const
{
    return triangles.size();
}

/*
 * Add spring to spring list.
 * @param s The spring.
 * @return Void.
 */
void Physics::add_spring( Spring* s )
{
    springs.push_back( s );
}

/*
 * Get the number of springs.
 * @return The spring count.
 */
size_t Physics::num_springs() const
{
    return springs.size();
}

/*
 * Reset the physics engine.
 * @return Void.
 */
void Physics::reset()
{
    for ( SphereList::iterator i = spheres.begin(); i != spheres.end(); i++ ) {
        delete *i;
    }
    for ( PlaneList::iterator i = planes.begin(); i != planes.end(); i++ ) {
        delete *i;
    }
    for ( TriangleList::iterator i = triangles.begin(); i != triangles.end(); i++ ) {
        delete *i;
    }
    for ( SpringList::iterator i = springs.begin(); i != springs.end(); i++ ) {
        delete *i;
    }

    spheres.clear();
    planes.clear();
    triangles.clear();
    springs.clear();
    
    gravity = Vector3::Zero();
    collision_damping = 0.0;
}

}
