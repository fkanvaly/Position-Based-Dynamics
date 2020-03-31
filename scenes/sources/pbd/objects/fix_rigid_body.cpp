#include "fix_rigid_body.hpp"

using namespace vcl;

fix_rigid_body::fix_rigid_body() {}

fix_rigid_body::fix_rigid_body( object_type phase, 
                                particules_t& particules, 
                                int id, 
                                mu_frcition mu,
                                const vcl::mesh& m, 
                                vcl::vec3& center, 
                                const vcl::vec3& color)
{

    const size_t N = m.position.size();
    const size_t N_particules = particules.size();
    m_id = id;

    particules.resize(N_particules + N);
    m_particules_idx.resize(N);
    for(size_t k=0; k<N; ++k)
    {
        const int idx = N_particules + k;
        m_particules_idx[k] = idx;
        
        particules[idx].invMass = 0 ;
        particules[idx].p = m.position[k];
        particules[idx].p_i = m.position[k];
        particules[idx].body = id;
        particules[idx].phase = phase;
        particules[idx].mu_k = mu.mu_k;
        particules[idx].mu_s = mu.mu_s;
        particules[idx].color = color;

    }

    connectivity = m.connectivity;

    visual = m;
    normals = m.normal;
    visual.uniform.color = color;

}