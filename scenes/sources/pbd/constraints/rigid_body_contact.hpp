#pragma once

#include "scenes/base/base.hpp"
#include "vcl/vcl.hpp"

struct rigid_body_contact_constraint_t 
{
    int m_idx_1;
    int m_idx_2;
    bool m_statbilized;

    rigid_body_contact_constraint_t(int idx_1, int idx_2, bool stabilized)
        : m_idx_1(idx_1), m_idx_2(idx_2), m_statbilized(stabilized)
    {}

    void project(particules_t& particules){
        const vcl::vec3& p1 = particules[m_idx_1].p;
        const float w1 = particules[m_idx_1].invMass;
        const float n1 = particules[m_idx_1].n;

        const vcl::vec3& p2 = particules[m_idx_2].p;
        const float w2 = particules[m_idx_2].invMass;
        const float n2 = particules[m_idx_2].n;

        float w12 = w1 + w2;

        if (w12 ==0){return;}

        if (n1 ==0){printf("n==0 !!");}
        if (n2 ==0){printf("n==0 !!");}

        const vcl::vec3 n12 = (p2-p1)/vcl::norm(p2-p1);
        const float d = 2*R - vcl::norm(p2-p1);

        if(d<0) return;

        vcl::vec3 delta_p1 = -(w1/w12) * d * n12;
        vcl::vec3 delta_p2 = (w2/w12) * d * n12;

        if (m_statbilized){
            particules[m_idx_1].p_i += delta_p1;
            particules[m_idx_2].p_i += delta_p2;
        }        
        
        particules[m_idx_1].p += delta_p1;
        particules[m_idx_2].p += delta_p2;


        //!Friction
        const vcl::vec3 v = (particules[m_idx_1].p - particules[m_idx_1].p_i) 
                            - (particules[m_idx_2].p - particules[m_idx_2].p_i);

        const vcl::vec3 fric_v = v - vcl::dot(v, n12) * n12;
        float frict_amount = vcl::norm(fric_v);

        if(frict_amount<EPSILON) return;

        float mu_k1 = particules[m_idx_1].mu_k;
        float mu_s1 = particules[m_idx_1].mu_s;

        float mu_k2 = particules[m_idx_2].mu_k;
        float mu_s2 = particules[m_idx_2].mu_s;

        float mu_s = mu_s1 * mu_s2;
        float mu_k = mu_k1 * mu_k2;
        
        vcl::vec3 val_fric;
    
        if (vcl::norm(fric_v)<mu_s*d){
            val_fric =  fric_v ;
            particules[m_idx_1].p -= (w1/w12) * val_fric / n1;
            particules[m_idx_2].p += (w2/w12) * val_fric / n2;
            if (m_statbilized) 
            {
                particules[m_idx_1].p_i -= (w1/w12) * val_fric / n1;
                particules[m_idx_2].p_i += (w2/w12) * val_fric / n2;
            }
        }else{
            val_fric =  fric_v * std::min((float)(mu_k*d/norm(fric_v)), 1.0f);
            particules[m_idx_1].p -= (w1/w12) * val_fric / n1;
            particules[m_idx_2].p += (w2/w12) * val_fric / n2;
            if (m_statbilized) 
            {
                particules[m_idx_1].p_i -= (w1/w12) * val_fric / n1;
                particules[m_idx_2].p_i += (w2/w12) * val_fric / n2;
            }
        }
    }
};
