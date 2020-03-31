#pragma once

#include "scenes/base/base.hpp"
#include "vcl/vcl.hpp"

// Store element corresponding to an object
struct rigid_body {

    // Create a new shape given
    // - A basic mesh model
    // - Its position
    // - Its speed of the center of mass
    // - The angular speed of the object
    // - Its color
    rigid_body( object_type phase,
                particules_t& particules, 
                int id, 
                mu_frcition mu,
                float alpha,
                float plastic_thres,
                const vcl::mesh& m,
                const vcl::vec3& center, 
                const vcl::vec3& speed, 
                const vcl::vec3& angular_speed, 
                const vcl::vec3& color);
    
    rigid_body();

    static vcl::mesh g_plane(int line_particules, float p_radius, vcl::vec3 origin, float offset);


    int m_id=-1;
    vcl::buffer<int> m_particules_idx;

    vcl::buffer<vcl::vec3> p; // Set of positions describing the shape
    vcl::buffer<vcl::vec3> r0; // Storage of the relative coordinate of each vertex in the reference frame (can be updated in the case of plastic deformation)
    float m_alpha=1;
    float m_plastic_thres=0;

    vcl::vec3 COM;  // The Center Of Mass (barycenter of the shape)

    vcl::buffer<vcl::uint3> connectivity; // Connectivity of the surface

    vcl::mesh_drawable visual;       // Visual model to be displayed (updated at each deformation)
    vcl::buffer<vcl::vec3> normals;  // Storage for normals

    void damp_velocity(particules_t& particules);
    void update_center_of_mass(particules_t& particules); // recompute the center of mass (based on positions p and connectivity)
    void update_visual_model(particules_t& particules);   // update the position and normal of the visual model
};
