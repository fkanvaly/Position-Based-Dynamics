#include "fluid_test.hpp"

#ifdef FLUID_TEST
using namespace vcl;
using namespace Eigen;




void scene_model::setup_data(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& )
{
    // Initialize global gui values
    gui_scene.neighboor                 = true;
    gui_scene.grid                 = false;
    gui_scene.sphere                 = true;
    gui_scene.wireframe              = true;
    gui_scene.speed_loading          = false;
    gui_scene.speed_accumulator_max  = 25.0f;
    object_length = 0.25f;

    // Create the basic mesh models: Cube, Cylinder, Torus
    mesh_basic_model[surface_cube]     = mesh_primitive_bar_grid(4,4,4,{-object_length/2,-object_length/2,-object_length/2},{object_length,0,0},{0,object_length,0},{0,0,object_length});
    mesh_basic_model[surface_cylinder] = mesh_primitive_cylinder(object_length/2.0f, {-object_length,0.0f,0.0f}, {object_length,0.0f,0.0f}, 9,5);
    mesh_basic_model[surface_torus]    = mesh_primitive_torus(object_length, object_length*0.5f, {0,0,0}, {0,0,object_length},8,15);
    
    
    // Load specific shaders
    shaders["wireframe_quads"] = create_shader_program("scenes/shared_assets/shaders/wireframe_quads/shader.vert.glsl","scenes/shared_assets/shaders/wireframe_quads/shader.geom.glsl","scenes/shared_assets/shaders/wireframe_quads/shader.frag.glsl"); // Shader for quad meshes
    shaders["normals"] = create_shader_program("scenes/shared_assets/shaders/normals/shader.vert.glsl","scenes/shared_assets/shaders/normals/shader.geom.glsl","scenes/shared_assets/shaders/normals/shader.frag.glsl"); // Shader to display normals


    // Initialize drawable parameters
    sphere_visual = mesh_primitive_sphere(R);
    sphere_visual.shader = shaders["mesh"];


    // Create borders
    border_length = 2;
    const float a = border_length;

    borders =  {    
                    // boundary( {0, 0, -a/2}, {0, 0,  1}, 0.1 * 1e-2, 0.003 * 1e-2), //* Ice on ice
                    // boundary( {0, 0, -a/2}, {0, 0,  1}, 0.25 * 1e-2, 0.2 * 1e-2), //* Wood on wood
                    boundary( {0, 0, -a/2}, {0, 0,  1}, 1, 1), //* very rough surfaces
                    boundary( {0, 0, a/2}, {0, 0, -1},  0,0),
                    boundary( {0, a/2,0} ,  {0, -1,0} ,  0,0),
                    boundary( {0, -a/2,0} , {0, 1,0} , 0,0),
                    boundary( {a/2, 0, 0} , {-1, 0, 0} ,  0,0),
                    boundary( {-a/2,0, 0.0}, {1,0, 0.0}, 0,0)
                };
    
    scene_objects.boundaries = borders;

    std::vector<vec3> borders_segments = {{-a/2,-a/2,-a/2},{a/2,-a/2,-a/2}, {a/2,-a/2,-a/2},{a/2,a/2,-a/2}, {a/2,a/2,-a/2},{-a/2,a/2,-a/2}, {-a/2,a/2,-a/2},{-a/2,-a/2,-a/2},
                                          {-a/2,-a/2,a/2} ,{a/2,-a/2,a/2},  {a/2,-a/2,a/2}, {a/2,a/2,a/2},  {a/2,a/2,a/2}, {-a/2,a/2,a/2},  {-a/2,a/2,a/2}, {-a/2,-a/2,a/2},
                                          {-a/2,-a/2,-a/2},{-a/2,-a/2,a/2}, {a/2,-a/2,-a/2},{a/2,-a/2,a/2}, {a/2,a/2,-a/2},{a/2,a/2,a/2},   {-a/2,a/2,-a/2},{-a/2,a/2,a/2}};

    border_visual = borders_segments;
    border_visual.uniform.color = {0,0,0};
    border_visual.shader = shaders["curve"];


    //* COLLISION GRID DEBUG
    col_grid.init(border_length, R, {-a/2,-a/2,-a/2}, particules);

    // Initialize a initial scene
    printf("a:%f\n", a);
    create_rigid_body();
    initialize_sph();
    

    int N_iter_solver = 4; 
    int N_iter_stab = 2; 
    physics_simulator = pbd_simulator(N_iter_solver, N_iter_stab, solverIter);

    // Initialize camera
    scene.camera.scale = 4.0f;
    scene.camera.orientation = rotation_from_axis_angle_mat3({0,0,1},-M_PI/6.0f) * rotation_from_axis_angle_mat3({1,0,0},M_PI/2.0f-M_PI/6.0f);

    timer.stop();
}


void scene_model::initialize_sph()
{
    // Influence distance of a particle (size of the kernel)
    const float h = 5*R;

    // Rest density (consider 1000 Kg/m^3)
    const float rho0 = 1000.0f;

    // Stiffness (consider ~2000 - used in tait equation)
    const float stiffness = 2000.0f;

    // Viscosity parameter
    const float nu = 2.0f;

    // Total mass of a particle (consider rho0 h^2)
    const float m = rho0*h*h;

    sph_param.h    = h;
    sph_param.rho0 = rho0;
    sph_param.nu   = nu;
    sph_param.m    = m;
    sph_param.stiffness = stiffness;    

    // Shorthand notation to setup the basic shape models
    const float a = border_length;                    // size of the object

    // Create the scene
    const vec3& color = {1,1,0.7f};
    int width = 10;
    int height = 10;
    vec3 origin {0,0,-a/4+R};
    std::cout<<origin<<endl;
    scene_objects.add_fluid( fluid(particules, scene_objects.No, sph_param, origin, width, height, color_LUT[ scene_objects.No%color_LUT.size() ] ) );
}

void scene_model::create_rigid_body()
{
    // Shorthand notation to setup the basic shape models
    const float a = border_length;                    // size of the object
    r = R; // radius of the bounding sphere

    // Create the scene
    vec3 origin {0,0,-a/4+R};
    // scene_objects.add_rigid_body( rigid_body(SOLID, particules, scene_objects.No, mu_frcition(0.0001,0.001), 0, 1, mesh_basic_model[surface_cube], {0,0,0.23}, {5,0,0}, {0,0,0}, color_LUT[ scene_objects.No%color_LUT.size() ] ) );
    
}


void scene_model::frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& )
{
    set_gui();
    const float elapsed = timer.update();

    // Accumulate force when press on space
    if(gui_scene.speed_loading)
        gui_scene.speed_accumulator =std::min( gui_scene.speed_accumulator + 0.1f, gui_scene.speed_accumulator_max);

    // Display rigid_bodies
    display_shapes_surface(shaders, scene);
    if(gui_scene.sphere)
        display_bounding_spheres(shaders, scene);
    draw(border_visual, scene.camera);

    // col_grid.display_grid(particules, shaders, scene, gui_scene.neighboor, gui_scene.grid);
    
    // Perform iterations of shape matching simulation
    if(elapsed>1e-6f) { // Do not evolve if simulation is stopped
        const float dt = 1/25.0f*0.04*timer.scale;
        physics_simulator.simulate(dt, particules, col_grid, scene_objects);
    }

    // Update visual models
    for(int ko=0; ko<scene_objects.rigid_bodies.size(); ++ko)
        scene_objects.rigid_bodies[ko].update_visual_model(particules);
    
    for(int ko=0; ko<scene_objects.clothes.size(); ++ko)
        scene_objects.clothes[ko].update_visual_model(particules);
}


void scene_model::display_shapes_surface(std::map<std::string,GLuint>& shaders, scene_structure& scene)
{
    const size_t No_R = scene_objects.rigid_bodies.size();
    for(size_t ko=0; ko<No_R; ++ko)
    {
        draw(scene_objects.rigid_bodies[ko].visual, scene.camera, shaders.at("mesh_bf"));
        if(gui_scene.wireframe)
            draw(scene_objects.rigid_bodies[ko].visual, scene.camera, shaders.at("wireframe"));
    }

    const size_t No_fR = scene_objects.fix_rigid_bodies.size();
    for(size_t ko=0; ko<No_fR; ++ko)
    {
        draw(scene_objects.fix_rigid_bodies[ko].visual, scene.camera, shaders.at("mesh_bf"));
        if(gui_scene.wireframe)
            draw(scene_objects.fix_rigid_bodies[ko].visual, scene.camera, shaders.at("wireframe"));
    }

    const size_t No_Clth = scene_objects.clothes.size();
    for(size_t ko=0; ko<No_Clth; ++ko)
    {
        draw(scene_objects.clothes[ko].visual, scene.camera, shaders.at("mesh_bf"));
        if(gui_scene.wireframe)
            draw(scene_objects.clothes[ko].visual, scene.camera, shaders.at("wireframe"));
    }

}
void scene_model::display_bounding_spheres(std::map<std::string,GLuint>& , scene_structure& scene)
{
    // sphere_visual.uniform.transform.scaling = R;
    for (size_t i = 0; i < particules.size(); i++)
    {
        sphere_visual.uniform.transform.translation = particules[i].p;
        draw(sphere_visual, scene.camera);
    }

}


void scene_model::throw_new_shape(const scene_structure& scene)
{
    // Camera direction
    const vec3 forward = -scene.camera.orientation.col(2);
    const vec3 up = scene.camera.orientation.col(1);
    const vec3 c0 = scene.camera.camera_position();

    // Parameter of the new shape
    const vec3 position = c0 + 0.8*forward;                               // position a little bit in front of the camera
    const vec3 speed = gui_scene.speed_accumulator * (forward + 0.05*up); // initial speed going forward, slightly in the up direction
    // Angular speed with magnitude given by the accumulated power and arbitrary axis direction
    const vec3 angular_speed = gui_scene.speed_accumulator*normalize(vec3(rand_interval(),rand_interval(),rand_interval()));

    // Can only throw new shape from inside the cube (to avoid problem of collision)
    const float a = 0.9; // security factor to be inside the cube
    if( position.x<a*border_length && position.x>-a*border_length && position.y<a*border_length && position.y>-a*border_length && position.z>-a)
    {
        // Create the new shape and add it to the vector
        mesh mesh_model = mesh_basic_model[gui_scene.surface_type];
        scene_objects.add_rigid_body( rigid_body(SOLID, particules, scene_objects.No, mu_frcition(0.5,0.4), 0, 1, mesh_model, position, speed, angular_speed, {0,0,0}) );
        scene_objects.rigid_bodies.back().visual.uniform.color = color_LUT[ scene_objects.No%color_LUT.size() ];
    }
    else
        std::cout<<"Impossible to throw new object from position outside the cube :"<<position<<std::endl;
}



void scene_model::keyboard_input(scene_structure& scene, GLFWwindow* , int key, int , int action, int )
{
    // Press on space key to increase power
    if(key==GLFW_KEY_SPACE && action==GLFW_PRESS)
        gui_scene.speed_loading = true;

    if(key==GLFW_KEY_SPACE && action==GLFW_RELEASE)
    {
        // Add new shape when release the space key
        throw_new_shape(scene);

        gui_scene.speed_loading = false;
        gui_scene.speed_accumulator = 0;
    }
}


/** Part specific GUI drawing */
/** Part specific GUI drawing */
void scene_model::set_gui()
{
    // Display
    // ************************************************ //
    ImGui::Text("\nDisplay");
    ImGui::Checkbox("Sphere", &gui_scene.sphere); ImGui::SameLine();
    ImGui::Checkbox("Wireframe", &gui_scene.wireframe);
    ImGui::Checkbox("grid", &gui_scene.grid);
    ImGui::Checkbox("neighboor", &gui_scene.neighboor);
    ImGui::SliderFloat("Radius bounding sphere", &R, 0.001f, 0.15f, "%.3f");

    // Throwing new shape
    // ************************************************ //
    ImGui::Text("\nThrow new shape");

    ImGui::Text("Surface Type:"); ImGui::SameLine(); // Select type of deformation
    int* ptr_surface_type = (int*)&gui_scene.surface_type;
    ImGui::RadioButton("Cube",ptr_surface_type, surface_cube); ImGui::SameLine();
    ImGui::RadioButton("Cylinder",ptr_surface_type, surface_cylinder); ImGui::SameLine();
    ImGui::RadioButton("Torus",ptr_surface_type, surface_torus);

    ImGui::SliderFloat("Accumulated Power", &gui_scene.speed_accumulator, 0, gui_scene.speed_accumulator_max, "%.2f");


    // Simulation Parameters
    // ************************************************ //
    ImGui::Text("\nSimulation parameters");

    ImGui::SliderFloat("Time scale", &timer.scale, 0.05, 2.0f);

    // Restart, Start and Stop animation
    const bool restart = ImGui::Button("Restart"); ImGui::SameLine();
    const bool start = ImGui::Button("Start"); ImGui::SameLine();
    const bool stop  = ImGui::Button("Stop");
    if(restart) create_rigid_body();
    if(start) timer.start();
    if(stop) timer.stop();
}



#endif

