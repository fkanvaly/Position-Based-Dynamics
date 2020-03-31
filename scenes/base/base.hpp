#pragma once


#include "vcl/vcl.hpp"
#include "scenes/current_scene.hpp"
#include "main/helper_scene/helper_scene.hpp"

#include <Eigen/Eigen>
#include <string>
#include <map>
#include <vector>
#include <memory>


struct scene_structure;
struct gui_structure;

enum object_type{
    SOLID,
    FLUID,
    CLOTH, 
    SMOKE,
    GRANULAR
};

static float EPSILON = 1e-4;
// static const float R = 0.037f;
static float R = 0.05f;

struct mu_frcition {
    float mu_k=std::sqrt(.3);
    float mu_s=std::sqrt(.5);
    mu_frcition();
    mu_frcition(float k, float s) : mu_k(k), mu_s(s) {};
};

struct cloth_parameter
{
    float K = 100;    // Global stiffness (to be divided by the number of particles)
    float mu = 0.02;   // Damping
    float wind = 0; // Wind magnitude;
};


struct particule{
    vcl::vec3 p;
    vcl::vec3 p_i;
    vcl::vec3 v;
    vcl::vec3 r0;
    float mu_k=std::sqrt(.3);
    float mu_s=std::sqrt(.5);
    vcl::uint3 grid_pos;
    float invMass = 1.0f;
    vcl::vec3 forces {0,0,0};
    object_type phase;
    int body;
    int n=0;
    vcl::vec3 color {1,1,1};

    // local density and pression
    float rho;
    float pression;
};

typedef vcl::buffer<particule> particules_t;


struct scene_base
{
    /** A empty scene defining basic functions used to display a 3D scene
     * Every scene can derivate from this empty scene
     *
     * Fonctions:
     * - setup_data: called once to setup data before starting the animation loop
     * - frame_display: called at every displayed frame within the animation loop
     * - mouse_click: called every time the user click on the mouse
     * - mouse_move: called every time the user move the mouse
     * - mouse_scroll: called every time the user scroll the mouse
     * - keyboard_input: called every time a key is pressed/released on the keyboard
     *
     * These functions receive the following parameters
     * - shaders: A set of shaders.
     * - scene: Contains general common object to define the 3D scene. Contains in particular the camera.
     * - gui: The GUI structure allowing to create/display buttons to interact with the scene.
     * - window, button, action, mods: are GLFW related parameters to handle window and mouse state
    */

    void setup_data(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);
    void frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);

    void mouse_click(scene_structure& scene, GLFWwindow* window, int button, int action, int mods);
    void mouse_move(scene_structure& scene, GLFWwindow* window);
    void mouse_scroll(scene_structure& scene, GLFWwindow* window, float x_offset, float y_offset);
    void keyboard_input(scene_structure& scene, GLFWwindow* window, int key, int scancode, int action, int mods);
};


