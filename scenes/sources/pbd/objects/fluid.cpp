#pragma once
#include "fluid.hpp"

using namespace vcl;
using namespace Eigen;

fluid::fluid() {}

fluid::fluid(
            particules_t& particules, 
            int id, 
            sph_parameters sph_p,
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
        sph_param = sph_p;

        particules.resize(N_particules + N);
        m_particules_idx.resize(N*(i+1) );
        
        for(size_t k=0; k<N; ++k)
        {
            const int idx = N_particules + k;
            m_particules_idx[N*i + k] = idx;
            
            particules[idx].p = m.position[k] + center;
            particules[idx].p_i = m.position[k] + center;
            particules[idx].body = id;
            particules[idx].phase = FLUID;
            particules[idx].invMass = 1.0;
            particules[idx].color = color;
            particules[idx].v = {.1,.1,.1};
        }

    }
}


void fluid::update_density(particules_t& particules)
{
    // Fill particles[i].rho = ...
    const float h = sph_param.h; 
    const float s = 315/(64*M_PI*pow(sph_param.h,3));

    auto Wh = [h,s](vcl::vec3 p){
        return (norm(p)<h) ? s * pow(1 - pow( norm(p) / h, 2), 3) : 0;
    };
    // printf("here");
    for (size_t idx = 0; idx < m_particules_idx.size(); idx++)
    {
        // printf("rho_%i:\n", idx);
        int i = m_particules_idx[idx];
        // printf("%i:%i\n",i,idx);
        // int i = idx;
        const vec3& p = particules[i].p;
        particules[i].rho = 0;
        for (size_t j = 0; j < particules.size(); j++)
        {
            const vec3& pj = particules[j].p;
            particules[i].rho += sph_param.m * Wh(p - pj);
            // if(Wh(p - pj)!=0)
            //     printf("->||p-pj||:%f, Wh:%f \n", norm(p-pj), Wh(p - pj));
        }
        // printf("rho_%i:%f\n", i, particules[i].rho);
    }
    
}

void fluid::update_pression(particules_t& particules)
{
    // Fill particles[i].pression = ...
    const float rho0 = sph_param.rho0;
    const float K = sph_param.stiffness;
    const float alpha = 0.001;
    // std::cout<<"end"<<std::endl;
    for (size_t idx = 0; idx < m_particules_idx.size(); idx++)
    {
        int i = m_particules_idx[idx];
        particules[i].pression = (particules[i].rho>rho0) ? K * pow( particules[i].rho/rho0 - 1, alpha) : 0;
        // printf("rho_%i:%f ? rho-0:%f\n", i, particules[i].rho, rho0);
        // printf("pression_%i:%f\n\n", i, particules[i].pression);

    }
}


void fluid::update_force(particules_t& particules)
{
    // gravity
    const size_t N = particules.size();

    // Add contribution of SPH forces
    // particules[i].a += ... (contribution from pression and viscosity)
    const float h = sph_param.h; 
    const float s = 315/(64*M_PI*pow(h,3));
    auto delta_Wh = [s, h](vcl::vec3 p){
        return (norm(p)<h) ? -(6*s/(h*h)) * pow(1 - pow( norm(p) / h, 2), 2) * p : 0*p;
    };

    float e = 1e-2;
    for (size_t idx = 0; idx < m_particules_idx.size(); idx++)
    {
        int i = m_particules_idx[idx];
        vec3 pi = particules[i].p;
        vec3 vi = particules[i].v;
        float P_i = particules[i].pression;
        float rho_i = particules[i].rho;
        for (size_t j = 0; j < particules.size(); j++){
            vec3 pj = particules[j].p;
            vec3 vj = particules[j].v;
            float P_j = particules[j].pression;
            float rho_j = particules[j].rho;

            particules[i].forces -= sph_param.m * (P_i/(rho_i*rho_i) + P_j/(rho_j*rho_j)) * delta_Wh(pi-pj);
            // particules[i].forces += 2*sph_param.nu * (sph_param.m/rho_j) * (dot(pi-pj, vi-vj) * delta_Wh(pi-pj) / (pow(norm(pi-pj),2) + e*h*h)) ;

        }

        // std::cout<< particules[i].forces <<std::endl;
    }
    
}



// ────────────────────────────────────────────────────────────────────────── I ──────────
//*   :::::: S O M E   R I G I D   B O D I E S : :  :   :    :     :        :          :
// ────────────────────────────────────────────────────────────────────────────────────
//

vcl::mesh fluid::g_plane(int line_particules, vcl::vec3 origin, float offset)
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
