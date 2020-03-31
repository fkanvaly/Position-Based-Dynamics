#pragma once

#include "scenes/base/base.hpp"
#include "objects/objets.hpp"

struct scene_objects_t{
    std::vector<rigid_body> rigid_bodies;
    std::vector<fix_rigid_body> fix_rigid_bodies;
    std::vector<cloth> clothes;
    std::vector<boundary> boundaries;
    std::vector<granular> granulars;
    std::vector<fluid> fluids;
    int No = 0;

    void add_rigid_body(rigid_body Rb){
        rigid_bodies.push_back( Rb );
        No++;
    }

    void add_fix_rigid_body(fix_rigid_body fRb){
        fix_rigid_bodies.push_back( fRb );
        No++;
    }

    void add_cloth(cloth Clth ){
        clothes.push_back( Clth );
        No++;
    }

    void add_boundary(boundary bnd){
        boundaries.push_back(bnd);
    }

    void add_fluid(fluid fld){
        fluids.push_back(fld);
    }

    void add_granular(granular gr){
        granulars.push_back(gr);
    }
};

