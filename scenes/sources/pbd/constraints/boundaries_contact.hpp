#pragma once

#include "scenes/base/base.hpp"
#include "vcl/vcl.hpp"
#include "../definition.hpp"

struct boundaries_constraint_t 
{
    int m_idx;
    boundary m_bndary;
    bool m_statbilized;

    boundaries_constraint_t(int idx, boundary bndary, bool stabilized)
        : m_idx(idx), m_bndary(bndary), m_statbilized(stabilized)
    {}

    void project(particules_t& particules){
        const float n1 = particules[m_idx].n;

        float d = vcl::dot( ( particules[m_idx].p - m_bndary.pts), m_bndary.normal );

        if (d > R) return;
        float d2plane = R - d;
        particules[m_idx].p += d2plane * m_bndary.normal;

        if(m_statbilized) particules[m_idx].p_i += d2plane * m_bndary.normal;
        if (m_statbilized) return;

        //!friction
        vcl::vec3 v = particules[m_idx].p - particules[m_idx].p_i;
        vcl::vec3 v_fric = v - vcl::dot(v, m_bndary.normal)*m_bndary.normal;
        float frict_amount = vcl::norm(v_fric);

        if(frict_amount<EPSILON) return;

        float mu_s = particules[m_idx].mu_s  * m_bndary.mu_s;
        float mu_k = particules[m_idx].mu_k  * m_bndary.mu_k;

        if (frict_amount < mu_s * d)
            particules[m_idx].p -= v_fric / n1;
        else{
            particules[m_idx].p -= v_fric * std::min((float)(mu_k*d)/frict_amount, 1.0f) / n1;
        }

    }
};

