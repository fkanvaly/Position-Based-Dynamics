#pragma once

#include "scenes/base/base.hpp"

#include "vcl/vcl.hpp"


// Store element corresponding to an object
struct fix_rigid_body {

    // Create a new shape given
    // - A basic mesh model
    // - Its position
    // - Its speed of the center of mass
    // - The angular speed of the object
    // - Its color
    fix_rigid_body();
    fix_rigid_body( object_type phase, 
                                particules_t& particules, 
                                int id, 
                                mu_frcition mu,
                                const vcl::mesh& m, 
                                vcl::vec3& center, 
                                const vcl::vec3& color);

    int m_id=-1;
    float m_p_radius;
    vcl::buffer<int> m_particules_idx;

    vcl::buffer<vcl::uint3> connectivity; // Connectivity of the surface

    vcl::mesh_drawable visual;       // Visual model to be displayed (updated at each deformation)
    vcl::buffer<vcl::vec3> normals;  // Storage for normals

};

