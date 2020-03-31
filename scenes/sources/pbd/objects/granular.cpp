#pragma once
#include "granular.hpp"

using namespace vcl;
using namespace Eigen;

granular::granular() {}

granular::granular(
            particules_t& particules, 
            int id, 
            mu_frcition mu,
            const vcl::vec3& center, 
            int width,
            int height,
            const vcl::vec3& color)
{
    for (size_t i = 0; i < height; i++)
    {
        const mesh m = g_plane(width, center + vec3{0,0,2*R*i}, 0);
        const size_t N = m.position.size();
        const size_t N_particules = particules.size();
        m_id = id;

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
            particules[idx].phase = GRANULAR;
            particules[idx].invMass = 20.0;
            particules[idx].color = color;
            particules[idx].v = {.1,.1,.1};
        }
    }
}


// ────────────────────────────────────────────────────────────────────────── I ──────────
//*   :::::: S O M E   R I G I D   B O D I E S : :  :   :    :     :        :          :
// ────────────────────────────────────────────────────────────────────────────────────
//

vcl::mesh granular::g_plane(int line_particules, vcl::vec3 origin, float offset)
{
    // Number of samples of the terrain is N x N
    const size_t N = line_particules;

    float s = N * 2*R;


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
