#pragma once

#include "scenes/base/base.hpp"
#include "vcl/vcl.hpp"


struct contact_constraint_t 
{
    int m_idx_1;
    int m_idx_2;
    bool m_statbilized;

    contact_constraint_t(int idx_1, int idx_2, bool stabilized)
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

        if (w1 ==0 && w2 == 0 ){return;}

        if (n1 ==0){printf("n==0 !!");}
        if (n2 ==0){printf("n==0 !!");}

        const vcl::vec3 n12 = (p2-p1)/vcl::norm(p2-p1);
        const float d = 2*R - vcl::norm(p2-p1);

        if(d<0) return;

        vcl::vec3 delta_p1 = -(w1/w12) * d * n12;
        vcl::vec3 delta_p2 = (w2/w12) * d * n12;

        if (m_statbilized){
            particules[m_idx_1].p_i += delta_p1/n1;
            particules[m_idx_2].p_i += delta_p2/n2;
        }        
        
        particules[m_idx_1].p += delta_p1/n1;
        particules[m_idx_2].p += delta_p2/n2;

    }
};


