#pragma once

#include "scenes/base/base.hpp"

#include "vcl/vcl.hpp"
#include "rigid_body_contact.hpp"
#include "particules_contact.hpp"
#include "shape_matching.hpp"
#include "boundaries_contact.hpp"
#include "strech_constraint.hpp"


struct constraintsSolver{
    //*Contact constraints
    std::vector<rigid_body_contact_constraint_t> rigid_body_contact_constraints;
    std::vector<contact_constraint_t> contact_constraints;
    std::vector<boundaries_constraint_t> boundaries_constraints;

    void add_constraint(rigid_body_contact_constraint_t& mesh_const){
        rigid_body_contact_constraints.push_back(mesh_const);
    }

    void add_constraint(contact_constraint_t& sphere_const){
        contact_constraints.push_back(sphere_const);
    }

    void add_constraint(boundaries_constraint_t& rigid_const){
        boundaries_constraints.push_back(rigid_const);
    }

    //*Group constraints
    std::vector<shape_maching_constraint_t> shape_maching_constraints;
    std::vector<strech_constraint_t> strech_constraints;

    void add_constraint(shape_maching_constraint_t& rigid_const){
        shape_maching_constraints.push_back(rigid_const);
    }

    void add_constraint(strech_constraint_t& dist_const){
        strech_constraints.push_back(dist_const);
    }

    //* solve contacts
    void project_all_contact(particules_t& particules, int N_iter, bool stabilized){
        for (size_t i = 0; i < N_iter; i++)
        {
#pragma omp parallel for
            for (auto &&constraint : boundaries_constraints)
                if (constraint.m_statbilized == stabilized)
                    constraint.project(particules);

#pragma omp parallel for
            for (auto &&constraint : rigid_body_contact_constraints)
                if (constraint.m_statbilized == stabilized)
                    constraint.project(particules);

#pragma omp parallel for
            for (auto &&constraint : contact_constraints)
                if (constraint.m_statbilized == stabilized)
                    constraint.project(particules);   
        }    
    }

    //*  solve group
    void project_all_group(particules_t& particules, int N_iter, bool stabilized){
        for (size_t i = 0; i < N_iter; i++)
        {         
#pragma omp parallel for
            for (auto &&constraint : strech_constraints)
                constraint.project(particules);

 #pragma omp parallel for
            for (auto &&constraint : shape_maching_constraints)
                constraint.project(particules);
        }    
    }

    void clear(){
        rigid_body_contact_constraints.clear();
        contact_constraints.clear();
        contact_constraints.clear();
        boundaries_constraints.clear();

        shape_maching_constraints.clear();
        strech_constraints.clear();
    }

};

