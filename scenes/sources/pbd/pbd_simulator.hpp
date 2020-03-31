#pragma once

#include "scenes/base/base.hpp"

#include "definition.hpp"
#include "objects/objets.hpp"
#include "constraints/constraints.hpp"
#include "utils/collision_grid.hpp"

#include <fstream>
#include <map>
#include <utility>
#include <set>

struct pbd_simulator
{
    int N_iter_solver;
    int N_iter_stab;
    constraintsSolver constraints_solver;
    
    pbd_simulator();
    pbd_simulator(int N_solver, int N_stab, constraintsSolver& solverIter);

    void simulate(float dt, particules_t& particules, collision_grid& col_grid, 
                            scene_objects_t& scene_obj);
    
    void position_integration(float dt, particules_t& particules);
    void velocity_integration(float dt, particules_t& particules);
    void position_as_speed(float dt, particules_t& particules);
    
};



