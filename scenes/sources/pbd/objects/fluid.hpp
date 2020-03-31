#pragma once

#include "scenes/base/base.hpp"
#include "vcl/vcl.hpp"

// SPH simulation parameters
struct sph_parameters
{
    float h;     // influence distance of a particle
    float rho0;  // rest density
    float m;     // total mass of a particle
    float stiffness; // constant of tait equation (relation density / pression)
    float nu;    // viscosity parameter
};

// Store element corresponding to an object
struct fluid {

    // Create a new shape given
    // - A basic mesh model
    // - Its position
    // - Its speed of the center of mass
    // - The angular speed of the object
    // - Its color
    fluid(
            particules_t& particules, 
            int id, 
            sph_parameters sph_p,
            const vcl::vec3& center, 
            int width,
            int height,
            const vcl::vec3& color);
    
    fluid();

    static vcl::mesh g_plane(int line_particules, vcl::vec3 origin, float offset);



    int m_id=-1;
    vcl::buffer<int> m_particules_idx;
    sph_parameters sph_param;

    vcl::buffer<vcl::vec3> p; // Set of positions describing the shape
    vcl::buffer<vcl::vec3> r0; // Storage of the relative coordinate of each vertex in the reference frame (can be updated in the case of plastic deformation)
    float m_alpha=1;
    float m_plastic_thres=0;

    vcl::vec3 COM;  // The Center Of Mass (barycenter of the shape)

    vcl::buffer<vcl::uint3> connectivity; // Connectivity of the surface

    vcl::mesh_drawable visual;       // Visual model to be displayed (updated at each deformation)
    vcl::buffer<vcl::vec3> normals;  // Storage for normals

    void update_density(particules_t& particules);
    void update_pression(particules_t& particules);
    void update_force(particules_t& particules);

};
