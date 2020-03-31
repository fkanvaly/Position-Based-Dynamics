#pragma once

#include "scenes/base/base.hpp"

struct collision_grid
{
    float m_p_size;
    vcl::vec3 m_origin;
    vcl::vec3 m_p_radius;
    float m_grid_size;
    int n_points;
    vcl::buffer3D<int> m_grid;
    vcl::vec3 offset;

    vcl::segment_drawable_immediate_mode segment_drawer; // Visual representation of the grid edges
    vcl::mesh_drawable sphere_grid;            // Visual representation of the grid positions

    collision_grid(){};
    void init(float grid_size, float p_size, const vcl::vec3& origin,  particules_t& particules);
    void update(particules_t& particules);
    vcl::uint3 get_grid_pos(particules_t& particules, int idx);
    bool is_valid(int i, int j, int k);
    vcl::buffer<int> neigh(const vcl::vec3& point);
    vcl::buffer<vcl::uint3> neigh_3(const vcl::vec3& point);
    
    void display_grid(particules_t& particules, std::map<std::string,GLuint>& shaders, scene_structure& scene, bool neighb_pts, bool grid_pts);
};