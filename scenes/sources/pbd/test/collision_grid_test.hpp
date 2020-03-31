#pragma once

#include "scenes/base/base.hpp"
#ifdef COLLISION_GRID_TEST

#include "../definition.hpp"
#include "../objects/objets.hpp"
#include "../constraints/constraints.hpp"
#include "../utils/collision_grid.hpp"

#include <map>


#include <fstream>
#include <map>
#include <utility>
#include <set>


// Define the three possible surfaces: Cube, Cylinder, Torus
enum surface_type_enum {surface_cube, surface_cylinder, surface_torus};



struct gui_scene_structure{
    bool sphere;
    bool wireframe;
    bool grid;
    bool neighboor;
    float speed_accumulator;
    float speed_accumulator_max;
    bool speed_loading;
    float radius_bounding_sphere;
    surface_type_enum surface_type;
};


struct scene_model : scene_base
{

    void setup_data(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);
    void initialize_shapes();
    void initialize_particules();
    vcl::mesh g_plane(int line_particules, vcl::vec3 origin);

    void frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);

    // Can insert new shape using space key
    void keyboard_input(scene_structure& scene, GLFWwindow* window, int key, int scancode, int action, int mods);
    void throw_new_shape(const scene_structure& scene);

    void set_gui();


    // Helper functions
    void position_integration(float dt);
    void velocity_integration(float dt);
    void damping_velocity();
    void position_as_speed(float dt);
    void display_shapes_surface(std::map<std::string,GLuint>& shaders, scene_structure& scene);
    void display_bounding_spheres(std::map<std::string,GLuint>& shaders, scene_structure& scene);
    void display_grid(std::map<std::string,GLuint>& shaders, scene_structure& scene);

    // The set of shapes
    std::vector<rigid_body> shapes;
    float object_length; // Caracteristic length of each shape

    // Storage for all the possible basic shapes (cube, cylinder, torus)
    std::map<surface_type_enum, vcl::mesh> mesh_basic_model;

    constraintsSolver solverIter;


    // Visual model of the bounding spheres
    vcl::mesh_drawable sphere_visual;
    float r;

    // Timer for the simulation
    vcl::timer_basic timer;

    // Visual model for the border
    vcl::segments_drawable border_visual;
    float border_length;
    std::vector<vcl::vec3> borders_pts;

    collision_grid col_grid;
    vcl::segment_drawable_immediate_mode segment_drawer; // Visual representation of the grid edges
    vcl::mesh_drawable sphere_grid_occupied;            // Visual representation of the grid positions
    vcl::mesh_drawable sphere_grid_empty;            // Visual representation of the grid positions

    particules_t particules;

    // Specific GUI for this scene
    gui_scene_structure gui_scene;
};

#endif


