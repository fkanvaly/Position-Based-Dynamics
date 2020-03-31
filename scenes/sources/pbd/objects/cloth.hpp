#pragma once

#include "scenes/base/base.hpp"
#include "vcl/vcl.hpp"
#include "../utils/HalfedgeBuilder.cpp"

// Store element corresponding to an object
struct cloth {

    // Create a new shape given
    // - A basic mesh model
    // - Its position
    // - Its speed of the center of mass
    // - The angular speed of the object
    // - Its color
    cloth( particules_t& particules, 
            int id, 
            cloth_parameter cloth_settings,
            int N_cloth,
            const vcl::vec3& center, 
            const vcl::vec3& color);
    
    cloth();

    int m_id=-1;
    vcl::buffer<int> m_particules_idx;
    cloth_parameter m_cloth_settings;
    int m_N_cloth;

    HalfedgeDS* he;

    vcl::buffer<vcl::vec3> p; // Set of positions describing the shape
    vcl::buffer2D<int> constraint_grid;
    float L0; 

    vcl::buffer<vcl::uint3> connectivity; // Connectivity of the surface
    vcl::mesh_drawable visual;       // Visual model to be displayed (updated at each deformation)
    vcl::buffer<vcl::vec3> normals;  // Storage for normals

    void damp_velocity(particules_t& particules);
    void update_center_of_mass(particules_t& particules); // recompute the center of mass (based on positions p and connectivity)
    void update_visual_model(particules_t& particules);   // update the position and normal of the visual model
};
