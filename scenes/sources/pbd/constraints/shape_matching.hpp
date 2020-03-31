#pragma once

#include "scenes/base/base.hpp"
#include "vcl/vcl.hpp"
#include "../objects/rigid_body.hpp"



struct shape_maching_constraint_t 
{
    rigid_body* m_obj;

    shape_maching_constraint_t(rigid_body& obj)
    {
        m_obj = &obj;
    }

    void project(particules_t& particules){
        const int N_pts = (*m_obj).m_particules_idx.size();
        (*m_obj).update_center_of_mass(particules);

        //*plastic
        for (size_t i = 0; i < N_pts; i++)
        {
            int idx = (*m_obj).m_particules_idx[i];
            float thres  = (*m_obj).m_plastic_thres ;
            float k = abs(norm(particules[idx].r0) - norm(particules[idx].p-(*m_obj).COM));
            // if (k>thres)
            //     particules[idx].r0 = particules[idx].p-(*m_obj).COM;
        }
        


        //Covariance matrix
        Eigen::Matrix3f M = Eigen::Matrix3f::Zero();
        for (int i = 0; i < N_pts; i++)
        {
            int idx = (*m_obj).m_particules_idx[i];
            const vcl::vec3 r_ = particules[idx].p - (*m_obj).COM;
            Eigen::MatrixXf r (1,3); r << r_.x, r_.y, r_.z;
            Eigen::MatrixXf r0 (1,3); r0 << particules[idx].r0.x, particules[idx].r0.y, particules[idx].r0.z;
            M += r.transpose() * r0;
        }

        //Polar decomposition
        Eigen::Affine3f A ; A = M;
        Eigen::Matrix3f R = A.rotation();
        vcl::mat4 T = {R(0,0), R(0,1), R(0,2), (*m_obj).COM.x,
                    R(1,0), R(1,1), R(1,2), (*m_obj).COM.y,
                    R(2,0), R(2,1), R(2,2), (*m_obj).COM.z,
                    0,      0,      0,      1};

        //Correct the vertex and velocity
        for(int i = 0; i < N_pts; i++)
        {
            int idx = (*m_obj).m_particules_idx[i];
            vcl::vec4 r0 = {particules[idx].r0.x, particules[idx].r0.y, particules[idx].r0.z, 1};
            vcl::vec4 p_ = T * r0 ;

            //*Elastic
            float alpha  = (*m_obj).m_alpha ;
            particules[idx].p =  (1-alpha) * vcl::vec3(p_.x, p_.y, p_.z) + alpha * particules[idx].p; 

        }
    }
};

