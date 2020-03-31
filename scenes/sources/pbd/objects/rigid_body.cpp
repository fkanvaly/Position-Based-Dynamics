#pragma once
#include "rigid_body.hpp"

using namespace vcl;
using namespace Eigen;

rigid_body::rigid_body() {}

rigid_body::rigid_body(object_type phase,
                       particules_t& particules, 
                       int id, 
                       mu_frcition mu,
                       float alpha,
                       float plastic_thres,
                       const vcl::mesh& m,
                       const vcl::vec3& center, 
                       const vcl::vec3& speed, 
                       const vcl::vec3& angular_speed, 
                       const vcl::vec3& color)
{
    const size_t N = m.position.size();
    const size_t N_particules = particules.size();
    m_id = id;
    m_alpha = alpha;
    m_plastic_thres = plastic_thres;

    p = m.position;
    particules.resize(N_particules + N);
    m_particules_idx.resize(N);
    for(size_t k=0; k<N; ++k)
    {
        const int idx = N_particules + k;
        m_particules_idx[k] = idx;

        p[k] = p[k] + center;
        particules[idx].p = p[k];
        particules[idx].p_i = p[k];
        particules[idx].body = id;
        particules[idx].phase = phase;
        particules[idx].invMass = 1.0f;
        
        particules[idx].mu_k = mu.mu_k;
        particules[idx].mu_s = mu.mu_s;
        
        particules[idx].color = color;

    }

    connectivity = m.connectivity;

    update_center_of_mass(particules);

    for(size_t k=0; k<N; ++k){
        particules[m_particules_idx[k]].v = speed + cross(angular_speed, p[k]-COM);
        particules[m_particules_idx[k]].r0 = p[k] - COM;
    }

    visual = m;
    normals = m.normal;
    visual.uniform.color = color;

}

void rigid_body::update_center_of_mass(particules_t& particules){
    for (size_t k = 0; k < p.size(); k++){
        p[k]=particules[m_particules_idx[k]].p;
    }
    COM = center_of_mass(p, connectivity);
}


void rigid_body::update_visual_model(particules_t& particules){
    for (size_t k = 0; k < p.size(); k++){
        int idx = m_particules_idx[k];
        p[k]=particules[idx].p;
    }
    visual.update_position(p);      // update position
    normal(p,connectivity,normals); // recompute new normals
    visual.update_normal(normals);  // update normals
}

//
// ────────────────────────────────────────────────────────────────────────── I ──────────
//*   :::::: S O M E   R I G I D   B O D I E S : :  :   :    :     :        :          :
// ────────────────────────────────────────────────────────────────────────────────────
//

vcl::mesh rigid_body::g_plane(int line_particules, float p_radius, vcl::vec3 origin, float offset)
{
    // Number of samples of the terrain is N x N
    const size_t N = line_particules;

    float s = N * 1.5*R;


    vcl::mesh terrain; // temporary terrain storage (CPU only)
    terrain.texture_uv.resize(N*N);
    terrain.position.resize(N*N);
    terrain.color.resize(N*N);

    // Fill terrain geometry
    for(size_t ku=0; ku<N; ++ku)
    {
        for(size_t kv=0; kv<N; ++kv)
        {
            // Compute local parametric coordinates (u,v) \in [0,1]
            const float u = ku/(N-1.0f);
            const float v = kv/(N-1.0f);


            // 3D vertex coordinates
            const float x = s * (u-0.5f);
            const float y = s * (v-0.5f);


            //float radious = 8;
            float z = origin.z - offset*ku;

            // Compute coordinates
            terrain.position[kv+N*ku] = {x,y,z};
            terrain.color[kv+N*ku]  = {1,1,1,1.0f};
            terrain.texture_uv[kv+N*ku] = {15*u, 15*v};
            terrain.color[kv+N*ku]  = {1,1,1,0.0f};
        }
    }


    // Generate triangle organization
    //  Parametric surface with uniform grid sampling: generate 2 triangles for each grid cell
    unsigned int Ns = N;
    for(size_t ku=0; ku<Ns-1; ++ku)
    {
        for(size_t kv=0; kv<Ns-1; ++kv)
        {
            const unsigned int idx = kv + Ns*ku; // current vertex offset

            const vcl::uint3 triangle_1 = {idx, idx+1+Ns, idx+1};
            const vcl::uint3 triangle_2 = {idx, idx+Ns, idx+1+Ns};

            terrain.connectivity.push_back(triangle_1);
            terrain.connectivity.push_back(triangle_2);
        }
    }

    return terrain;
}
