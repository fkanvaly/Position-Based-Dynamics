#include "pbd_simulator.hpp"

using namespace vcl;

pbd_simulator::pbd_simulator() {};

pbd_simulator::pbd_simulator(int N_solver, int N_stab, constraintsSolver& solverIter){
        N_iter_solver = N_solver;
        N_iter_stab = N_stab; 
        constraints_solver = solverIter;
    };

void pbd_simulator::simulate(float dt, particules_t& particules, collision_grid& col_grid, scene_objects_t& scene_obj){
    //*update forces
    //*---> Winds for clothes
    const int No_clothes = scene_obj.clothes.size();
#pragma omp parallel for
    for (int ko = 0; ko < No_clothes; ko++)
    {
        cloth& cloth_k = scene_obj.clothes[ko];
        const vec3 wind = vec3(cloth_k.m_cloth_settings.wind,0,0);
        const vec3 wind_u = normalize(wind);
        for(size_t k=0; k<cloth_k.m_particules_idx.size(); ++k)
        {
            const vec3& n = cloth_k.normals[k];
            const float wind_magnitude = std::abs(dot(wind,n));
            const vec3 f = wind_magnitude * wind_u * cloth_k.L0*cloth_k.L0;
            int idx = cloth_k.m_particules_idx[k];
            particules[idx].forces = f;
        }
    }

    //* FLUID
    int N_fluid = scene_obj.fluids.size();
    for (size_t i = 0; i < N_fluid; i++)
    {
        scene_obj.fluids[i].update_density(particules);
        scene_obj.fluids[i].update_pression(particules);
        scene_obj.fluids[i].update_force(particules);
    }
    
    
    velocity_integration(dt, particules);
    position_integration(dt, particules);


    //
    // ──────────────────────────────────────────────────────────────────────────────────────── I ──────────
    //*   :::::: F I N D   C O N T A C T   C O N S T R A I N T S & S O L V E : :  :   :    :     :        :          :
    // ──────────────────────────────────────────────────────────────────────────────────────────────────
    //

    constraints_solver.clear();

    //* clear constraint count
#pragma omp parallel for
    for (size_t i = 0; i < particules.size(); i++){
        particules[i].forces = {0,0,0};
        particules[i].n=0;
    }
    

    //! Update collision grid
    col_grid.update(particules);

    // * Contact with cube
#pragma omp parallel for
    for (size_t i = 0; i < scene_obj.boundaries.size(); i++)
    {
        const vec3 normal = scene_obj.boundaries[i].normal;
        const vec3 pts = scene_obj.boundaries[i].pts; 
        for (size_t k = 0; k < particules.size(); k++)
        {
            float proj = dot( (particules[k].p-pts), normal );

            if (proj < R){ //detected
                boundaries_constraint_t c (k, scene_obj.boundaries[i], false);
                boundaries_constraint_t c_stab (k, scene_obj.boundaries[i], true);
                constraints_solver.add_constraint(c);
                constraints_solver.add_constraint(c_stab);

                particules[k].n++;
            }
        }
    }

    // //* Contact with other sphere
    const float r = R;
    std::set<std::pair<int,int>> points_pair; //! to avoid doublon
#pragma omp parallel for
    for (size_t i = 0; i < particules.size(); i++)
    {
        int id_1 = particules[i].body;
        vec3& p1 = particules[i].p;
        object_type obj_type_1 = particules[i].phase;

        buffer<int> neighboor = col_grid.neigh(particules[i].p);
        for (size_t j = 0; j < neighboor.size(); j++)
        {
            const size_t k = neighboor[j];
            int id_2 = particules[k].body;
            object_type obj_type_2 = particules[k].phase;

            if((obj_type_1==SOLID && obj_type_2==SOLID) || (obj_type_1==FLUID && obj_type_2==FLUID)){
                if (id_1 == id_2) continue;
            }
            if (k==i ) continue;

            vec3& p2 = particules[k].p;

            //detect 
            if (norm(p1-p2) < 2*r){
                const std::pair<int,int> p(i,k);
                const std::pair<int,int> p_inv(k,i);

                auto it = points_pair.find(p_inv); //* if we find the inverted relation we skip
                if (it == points_pair.end()){
                    points_pair.insert(p);

                    if ((obj_type_1 == SOLID && obj_type_2 == SOLID) 
                            || (obj_type_1 == GRANULAR && obj_type_2 == GRANULAR)){
                        rigid_body_contact_constraint_t c (i,k, false);
                        constraints_solver.add_constraint(c);

                        rigid_body_contact_constraint_t c_stab (i,k, true);
                        constraints_solver.add_constraint(c_stab);

                        particules[i].n++;
                        particules[k].n++;
                    }else {
                        contact_constraint_t c (i,k, false);
                        constraints_solver.add_constraint(c);

                        contact_constraint_t c_stab (i,k, true);
                        constraints_solver.add_constraint(c_stab);

                        particules[i].n++;
                        particules[k].n++;
                    }
                }
            }
        }
    }

    //* Solve all contraints
    constraints_solver.project_all_contact(particules, N_iter_stab, false); //* solve contact
    constraints_solver.project_all_contact(particules, N_iter_stab, true); //*stabilization

    //
    // ────────────────────────────────────────────────────────────────────────────────────────────────────────── I ──────────
    //*   :::::: G R O U P   I T E R A T I O N   C O N S T R A I N T S   S O L V E R : :  :   :    :     :        :          :
    // ────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
    //

#pragma omp parallel for
    for (size_t i = 0; i < particules.size(); i++)
        particules[i].n=0;

    //* Shape Matching
    const int No = scene_obj.rigid_bodies.size();
#pragma omp parallel for
    for (int ko = 0; ko < No; ko++)
    {
        shape_maching_constraint_t c (scene_obj.rigid_bodies[ko]);
        constraints_solver.add_constraint(c);
    }

    //* spring distance constraints
    for (int ko = 0; ko < No_clothes; ko++)
    {
        cloth& cloth_k = scene_obj.clothes[ko];
        int N_cloth_dim = cloth_k.m_N_cloth;

        std::set<std::pair<int,int>> points_pair_cloth; //! to avoid doublon
        points_pair_cloth.clear();

        const int N_neighbors = 1;
#pragma omp parallel for
            for(int ku=0; ku < N_cloth_dim; ++ku) {
                for(int kv=0; kv < N_cloth_dim; ++kv) {
                    // Loops over neighbors
                    for(int du=-N_neighbors; du<=N_neighbors; ++du) {
                        for(int dv=-N_neighbors; dv<=N_neighbors; ++dv) {
                            if(du!=0 || dv!=0)
                            {
                                // Neighbors indices
                                const int ku_n = ku+du;
                                const int kv_n = kv+dv;
                                const float alpha = float(std::sqrt(du*du + dv*dv)); // rest-length

                                if( ku_n>=0 && ku_n<N_cloth_dim && kv_n>=0 && kv_n<N_cloth_dim){
                                    int idx1 = cloth_k.constraint_grid(ku,kv);
                                    int idx2 = cloth_k.constraint_grid(ku_n,kv_n);
                                    
                                    const std::pair<int,int> p(idx1,idx2);
                                    const std::pair<int,int> p_inv(idx2,idx1);

                                    auto it = points_pair_cloth.find(p_inv); //* if we find the inverted relation we skip
                                    if (it == points_pair_cloth.end()){
                                        points_pair_cloth.insert(p);
                                        float d = vcl::norm(particules[idx1].p - particules[idx2].p);

                                        if (d>cloth_k.L0 + 1e-3 || d < cloth_k.L0 - 1e-3 ){
                                            strech_constraint_t c (idx1, idx2, alpha*cloth_k.L0);
                                            constraints_solver.add_constraint(c);

                                            particules[idx1].n++;
                                            particules[idx2].n++;

                                        }
                                    }
                                }

                            }
                        }
                    }
                }
            }

    }


    constraints_solver.project_all_group(particules, N_iter_solver, false); //* solve group constraint

       
    //* Update velocity
    position_as_speed(dt, particules);


    //* Particle sleeping
    // for (size_t i = 0; i < particules.size(); i++)
    //     if( norm(particules[i].p-particules[i].p_i) < EPSILON )
    //         particules[i].v = {0,0,0};
        


};


//* Integrate speed
void pbd_simulator::velocity_integration(float dt, particules_t& particules)
{
    const vec3 g = {0,0,-9.81f};

    for(int k=0; k<particules.size(); ++k) {
        particules[k].p_i = particules[k].p;

        particules[k].v = 0.999*particules[k].v + dt * particules[k].invMass * g + particules[k].invMass*particules[k].forces;
    }

}

//* Integration position
void pbd_simulator::position_integration(float dt, particules_t& particules)
{
    for(int k=0; k<particules.size(); ++k) {
        particules[k].p = particules[k].p + dt*particules[k].v;
    }

}

//* Convert displacement into speed
void pbd_simulator::position_as_speed(float dt, particules_t& particules)
{
    for(int k=0; k<particules.size(); ++k) {
        particules[k].v = 1/dt*(particules[k].p-particules[k].p_i);
    }

}



