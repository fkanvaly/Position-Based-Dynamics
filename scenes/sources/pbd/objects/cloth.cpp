#pragma once
#include "cloth.hpp"


using namespace vcl;
using namespace Eigen;

cloth::cloth() {}

cloth::cloth(   particules_t& particules, 
                int id, 
                cloth_parameter cloth_settings,
                int N_cloth,
                const vcl::vec3& center, 
                const vcl::vec3& color  )
{
    L0 = 2*R * N_cloth * 1.0f/float(N_cloth-1);
    m_cloth_settings = cloth_settings;
    m_N_cloth = N_cloth;

    const mesh m = mesh_primitive_grid(N_cloth,N_cloth,center,{N_cloth*2*R,0,0},{0,0,N_cloth*2*R});
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
        particules[idx].phase = CLOTH;
        particules[idx].invMass = 1.0;
        particules[idx].color = color;
        particules[idx].v = {1,1,1};

        if(k==0 || k==N_cloth*(N_cloth-1)){
            particules[idx].v = {0,0,0};
            particules[idx].invMass = 0;}
    }



    constraint_grid = buffer2D_from_vector(m_particules_idx, N_cloth, N_cloth);

    connectivity = m.connectivity;

    Eigen::MatrixXi F (connectivity.size(),3);
    for (size_t i = 0; i < connectivity.size(); i++)
    {
        vcl::uint3 u = connectivity[i];
        F(i,0)=u[0]; F(i,1)=u[1]; F(i,2)=u[2];
    }
    
    HalfedgeBuilder *builder = new HalfedgeBuilder();
	HalfedgeDS he_DS = (builder->createMeshWithFaces(N, F)); // create the half-edge representation
    he = &he_DS;

    visual = m;
    normals = m.normal;
    visual.uniform.color = color;

}


void cloth::update_visual_model(particules_t& particules){
    for (size_t k = 0; k < p.size(); k++){
        int idx = m_particules_idx[k];
        p[k]=particules[idx].p;
    }
    visual.update_position(p);      // update position
    normal(p,connectivity,normals); // recompute new normals
    visual.update_normal(normals);  // update normals
}
