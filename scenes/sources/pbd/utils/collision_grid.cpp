#pragma once

#include "collision_grid.hpp"
#include "scenes/base/base.hpp"

void collision_grid::init(float grid_size, float p_size, const vcl::vec3& origin,  particules_t& particules)
{
    float grid_extent = 0.2;
    float origin_offset = 0.1;
    m_p_size = 2*p_size;
    m_grid_size = grid_size + grid_extent;
    n_points = m_grid_size/(2*p_size);
    m_grid.resize(n_points,n_points,n_points);
    offset = {p_size, p_size, p_size};
    m_origin = origin - vcl::vec3 {origin_offset, origin_offset, origin_offset} ;

    m_grid.fill(-1);
    for (size_t idx = 0; idx < particules.size(); idx++)
    {
        vcl::vec3 local_cord = (particules[idx].p - m_origin);
        int i = abs(local_cord.x/m_p_size); 
        int j = abs(local_cord.y/m_p_size); 
        int k = abs(local_cord.z/m_p_size); 
        
        if ( ! is_valid(i,j,k) ){
            printf("particle at grid not valid pos : %i,%i,%i\n \n",i,j,k);
            continue;  
        } 

        m_grid(i,j,k) = idx;
        particules[idx].grid_pos = {i,j,k};
    }

    //* Visual representation of the grid position
    segment_drawer.init();
    segment_drawer.uniform_parameter.color = {0,0,0};

    sphere_grid = vcl::mesh_primitive_sphere(0.015f);
    sphere_grid.uniform.color_alpha = 0;
}

vcl::uint3 collision_grid::get_grid_pos(particules_t& particules, int idx){
    vcl::vec3 local_cord = (particules[idx].p - m_origin);
    int i = abs(local_cord.x/m_p_size); 
    int j = abs(local_cord.y/m_p_size); 
    int k = abs(local_cord.z/m_p_size); 
    
    return vcl::uint3 {i,j,k};
}


void collision_grid::update(particules_t& particules)
{
    //* Solution 1 
    for (size_t idx = 0; idx < particules.size(); idx++)
    {
        vcl::vec3 local_cord = (particules[idx].p - m_origin);
        int i = abs(local_cord.x/m_p_size); 
        int j = abs(local_cord.y/m_p_size); 
        int k = abs(local_cord.z/m_p_size); 
        
        if ( ! is_valid(i,j,k) ){
            printf("particle at grid not valid pos : %i,%i,%i\n \n",i,j,k);
            vcl::uint3 t = particules[idx].grid_pos;
            if (is_valid(t[0], t[1], t[2]))
                if (m_grid(t[0], t[1], t[2]) == idx) //it is still me there
                    m_grid(t[0], t[1], t[2]) = -1;
            continue;  
        } 

        vcl::uint3 t = particules[idx].grid_pos;
        bool changed = !(t[0] == i 
                        && t[1] == j
                        && t[2] == k ) ;

        if (is_valid(t[0], t[1], t[2]))
            if (m_grid(t[0], t[1], t[2]) == idx) //it is still me there
                m_grid(t[0], t[1], t[2]) = -1;
                
        m_grid(i,j,k) = idx;
        particules[idx].grid_pos = {i,j,k};
    }

    //* Solution 2
    // m_grid.fill(-1);

    // for (size_t idx = 0; idx < particules.size(); idx++)
    // {
    //     vcl::vec3 local_cord = (particules[idx].p - m_origin);
    //     int i = abs(local_cord.x/m_p_size); 
    //     int j = abs(local_cord.y/m_p_size); 
    //     int k = abs(local_cord.z/m_p_size);

    //     if ( ! is_valid(i,j,k) ){
    //         printf("particle at grid not valid pos : %i,%i,%i\n \n",i,j,k);
    //         continue;  
    //     } 

    //     m_grid(i,j,k) = idx;
    //     particules[idx].grid_pos = {i,j,k};
    // }
    
}

bool collision_grid::is_valid(int i, int j, int k){
    return i<n_points && j<n_points && k<n_points
            && i>=0 && j>=0 && k>=0;
}

vcl::buffer<int> collision_grid::neigh(const vcl::vec3& point)
{
    vcl::vec3 local_cord = (point - m_origin);
    int i = abs(local_cord.x/m_p_size); 
    int j = abs(local_cord.y/m_p_size); 
    int k = abs(local_cord.z/m_p_size); 
    
    vcl::buffer<int> neighboor;
    vcl::buffer<int> idx = {-1,0,1};
    for (auto &&ki : idx)
    {
        for (auto &&kj : idx)
        {
            for (auto &&kk : idx)
            {
                int i_n = i + ki;
                int j_n = j + kj;
                int k_n = k + kk;

                bool valid = is_valid(i_n, j_n, k_n) ;
                        valid = valid && !(i_n==i && j_n==j && k_n==k);

                if (valid) 
                {
                    if (m_grid(i_n, j_n, k_n) != -1)
                        neighboor.push_back( m_grid(i_n, j_n, k_n) );
                }
            }
        }
    }

    return neighboor;
}

//* Use for neighboorhood debug
vcl::buffer<vcl::uint3> collision_grid::neigh_3(const vcl::vec3& point)
{
    vcl::vec3 local_cord = (point - m_origin);
    int i = abs(local_cord.x/m_p_size); 
    int j = abs(local_cord.y/m_p_size); 
    int k = abs(local_cord.z/m_p_size); 
    
    vcl::buffer<vcl::uint3> neighboor;
    vcl::buffer<int> idx = {-1,0,1};
    for (auto &&ki : idx)
    {
        for (auto &&kj : idx)
        {
            for (auto &&kk : idx)
            {
                int i_n = i + ki;
                int j_n = j + kj;
                int k_n = k + kk;

                bool valid = is_valid(i_n, j_n, k_n) ;
                        valid = valid && !(i_n==i && j_n==j && k_n==k);

                if (valid) 
                {
                    if (m_grid(i_n, j_n, k_n) != -1)
                        neighboor.push_back( vcl::uint3{i_n, j_n, k_n} );
                }
            }
        }
    }

    return neighboor;
}

void collision_grid::display_grid(particules_t& particules, std::map<std::string,GLuint>& shaders, scene_structure& scene, bool neighb_pts, bool grid_pts)
{
    // Display spheres
    sphere_grid.shader     = shaders["mesh"];
    sphere_grid.uniform.color_alpha = 0;

    for (size_t i = 0; i < n_points; i++)
    {
        for (size_t j = 0; j < n_points; j++)
        {
            for (size_t k = 0; k < n_points; k++)
            {
                vcl::vec3 v {i * m_p_size, j * m_p_size, k * m_p_size};
                sphere_grid.uniform.transform.translation = v + m_origin + offset;

                if(m_grid(i,j,k) != -1) {
                    sphere_grid.uniform.color = {1,0,0};
                    draw(sphere_grid, scene.camera);


                    // //! Show each particle neighboors
                    if (neighb_pts){
                    vcl::buffer<vcl::uint3> neighboor = neigh_3(v + m_origin + offset);
                    for (auto&& idx : neighboor)    
                    {
                        if (particules[m_grid(idx[0], idx[1], idx[2])].body == particules[m_grid(i,j,k)].body) continue;

                        vcl::vec3 v_neigh {idx[0] * m_p_size, idx[1] * m_p_size, idx[2] * m_p_size};
                        sphere_grid.uniform.transform.translation = v_neigh + m_origin + offset;
                        sphere_grid.uniform.transform.scaling = 1.2;
                        sphere_grid.uniform.color = {0,1,0};
                        draw(sphere_grid, scene.camera);
                    }
                    sphere_grid.uniform.transform.scaling = 1;
                    }
                }

                else if (grid_pts){
                    sphere_grid.uniform.color = {0,0,1};
                    // continue;
                    draw(sphere_grid, scene.camera);
                }

                
            }
            
        }
    }
}
