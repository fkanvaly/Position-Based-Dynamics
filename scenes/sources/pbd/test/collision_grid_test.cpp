#include "collision_grid_test.hpp"

#ifdef COLLISION_GRID_TEST


using namespace vcl;
using namespace Eigen;




void scene_model::setup_data(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& )
{
    // Initialize global gui values
    gui_scene.neighboor                 = true;
    gui_scene.grid                 = false;
    gui_scene.sphere                 = true;
    R = 0.042f;
    gui_scene.wireframe              = true;
    gui_scene.speed_loading          = false;
    gui_scene.speed_accumulator_max  = 25.0f;
    object_length = 0.25f;

    // Create the basic mesh models: Cube, Cylinder, Torus
    mesh_basic_model[surface_cube]     = mesh_primitive_bar_grid(4,4,4,{-object_length/2,-object_length/2,-object_length/2},{object_length,0,0},{0,object_length,0},{0,0,object_length});
    mesh_basic_model[surface_cylinder] = mesh_primitive_cylinder(object_length/2.0f, {-object_length,0.0f,0.0f}, {object_length,0.0f,0.0f}, 9,5);
    mesh_basic_model[surface_torus]    = mesh_primitive_torus(object_length, object_length*0.5f, {0,0,0}, {0,0,object_length},8,15);
    
    
    // Initialize a initial scene
    initialize_shapes();
    initialize_particules();

    // Load specific shaders
    shaders["wireframe_quads"] = create_shader_program("scenes/shared_assets/shaders/wireframe_quads/shader.vert.glsl","scenes/shared_assets/shaders/wireframe_quads/shader.geom.glsl","scenes/shared_assets/shaders/wireframe_quads/shader.frag.glsl"); // Shader for quad meshes
    shaders["normals"] = create_shader_program("scenes/shared_assets/shaders/normals/shader.vert.glsl","scenes/shared_assets/shaders/normals/shader.geom.glsl","scenes/shared_assets/shaders/normals/shader.frag.glsl"); // Shader to display normals


    // Initialize drawable parameters
    sphere_visual = mesh_primitive_sphere();
    sphere_visual.shader = shaders["mesh"];


    // Create borders
    border_length = 6;
    const float a = border_length;

    borders_pts =  { {0.0f,0.0f,-a/2}, {0.0f,0.0f,a/2}, 
                     {0.0f,a/2,0.0f}, {0.0f,-a/2,0.0f},
                     {a/2, 0.0f,0.0f}, {-a/2,0.0f,0.0}};

    std::vector<vec3> borders_segments = {{-a/2,-a/2,-a/2},{a/2,-a/2,-a/2}, {a/2,-a/2,-a/2},{a/2,a/2,-a/2}, {a/2,a/2,-a/2},{-a/2,a/2,-a/2}, {-a/2,a/2,-a/2},{-a/2,-a/2,-a/2},
                                          {-a/2,-a/2,a/2} ,{a/2,-a/2,a/2},  {a/2,-a/2,a/2}, {a/2,a/2,a/2},  {a/2,a/2,a/2}, {-a/2,a/2,a/2},  {-a/2,a/2,a/2}, {-a/2,-a/2,a/2},
                                          {-a/2,-a/2,-a/2},{-a/2,-a/2,a/2}, {a/2,-a/2,-a/2},{a/2,-a/2,a/2}, {a/2,a/2,-a/2},{a/2,a/2,a/2},   {-a/2,a/2,-a/2},{-a/2,a/2,a/2}};

    border_visual = borders_segments;
    border_visual.uniform.color = {0,0,0};
    border_visual.shader = shaders["curve"];


    //* COLLISION GRID DEBUG
    col_grid.init(border_length, R, {-a/2,-a/2,-a/2}, particules);


    // Initialize camera
    scene.camera.scale = 4.0f;
    scene.camera.orientation = rotation_from_axis_angle_mat3({0,0,1},-M_PI/6.0f) * rotation_from_axis_angle_mat3({1,0,0},M_PI/2.0f-M_PI/6.0f);
}

void scene_model::initialize_shapes()
{
    // Clear in case there is existing shapes
    shapes.clear();

    // Shorthand notation to setup the basic shape models
    const float& a = object_length;                    // size of the object
    r = R; // radius of the bounding sphere

    // Create the scene
    const vec3& color = {1,1,0.7f};
    // shapes.push_back( rigid_body(particules, shapes.size(), r, mesh_basic_model[surface_cube], {0,0,1}, {0,0,0}, {0,0,0}, color ) );
    // shapes.push_back( rigid_body(particules, shapes.size(), r, mesh_basic_model[surface_cube], {a+2*r,0,-1+a/2+r}, {0,0,0}, {0,0,0},color ) );
    // shapes.push_back( rigid_body(particules, shapes.size(), r, mesh_basic_model[surface_cube], {-a-2*r,0,-1+a/2+r}, {0,0,0}, {0,0,0},color ) );
    shapes.push_back( rigid_body(particules, shapes.size(), r, mesh_basic_model[surface_cube], {0,0,1} ,{0,0,0} , {0,0,0}, color ) );
    shapes.push_back( rigid_body(particules, shapes.size(), r, mesh_basic_model[surface_cylinder], {0,1,1.5} ,{0,0,0} , {0,0,0}, color ) );
    shapes.push_back( rigid_body(particules, shapes.size(), r, mesh_basic_model[surface_torus], {0,0,0} ,{0,0,0} , {0,0,0}, color ) );
    // shapes.push_back( rigid_body(particules, shapes.size(), r, bunny, {0,2,1} ,{0,0,0} , {0,0,0}, color ) );
}

void scene_model::initialize_particules()
{
    r = R; // radius of the bounding sphere

    // Create the scene
    int N_particules = 10;
    const vec3& color = {1,1,0.7f};

    particule p ;
    p.p = {0,0,0};    
    p.p_i = p.p;     
    p.v = {1,1,0};     
    p.radius = r;     

    particules.push_back(p);
}


void scene_model::frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& )
{
    set_gui();
    const float elapsed = timer.update();

    // Accumulate force when press on space
    if(gui_scene.speed_loading)
        gui_scene.speed_accumulator =std::min( gui_scene.speed_accumulator + 0.1f, gui_scene.speed_accumulator_max);

    // Display shapes
    // display_shapes_surface(shaders, scene);
    if(gui_scene.sphere)
        display_bounding_spheres(shaders, scene);
    draw(border_visual, scene.camera);

    col_grid.display_grid(particules, shaders, scene, gui_scene.neighboor, gui_scene.grid);

    // Perform iterations of shape matching simulation
    const int No = shapes.size();
    if(elapsed>1e-6f) { // Do not evolve if simulation is stopped

        const float dt = 1/25.0f*0.04*timer.scale;

        velocity_integration(dt);
        position_integration(dt);

        int N_iter_solver = 1; 
        int N_iter_stab = 1; 
        solverIter.clear();
       
        // * Contact with cube
        for (size_t i = 0; i < borders_pts.size(); i++)
        {
            vec3 normal = -borders_pts[i]/norm(borders_pts[i]);
            vec3 pts = borders_pts[i]; 
            for (size_t k = 0; k < particules.size(); k++)
            {
                float proj = abs(dot( (particules[k].p-pts), normal ));

                if (proj < R){ //detected
                    //Position correction 
                    boundaries_constraint_t c (boundaries_constraint_t(k, pts, normal, false));
                    boundaries_constraint_t c_stab (boundaries_constraint_t(k, pts, normal, true));
                    solverIter.add_constraint(c);
                    solverIter.add_constraint(c_stab);
                }
            }
        }

        // //* Shape Matching
        for (int ko = 0; ko < No; ko++)
        {
            shape_maching_constraint_t c (shapes[ko]);
            solverIter.add_constraint(c);
        }

        // * Solve all contraints
        solverIter.project_all(particules, N_iter_stab, true);
        solverIter.project_all(particules, N_iter_solver, false);
        
        col_grid.update(particules);
        

        // Propagate new position as speed
        position_as_speed(dt);
        for (size_t i = 0; i < particules.size(); i++)
            if( norm(particules[i].p-particules[i].p_i) < EPSILON )
                particules[i].p = particules[i].p_i;
        
    }

    // Update visual models
    for(int ko=0; ko<No; ++ko)
        shapes[ko].update_visual_model(particules);
}


void scene_model::display_shapes_surface(std::map<std::string,GLuint>& shaders, scene_structure& scene)
{
    const size_t No = shapes.size();
    for(size_t ko=0; ko<No; ++ko)
    {
        draw(shapes[ko].visual, scene.camera, shaders.at("mesh_bf"));
        if(gui_scene.wireframe)
            draw(shapes[ko].visual, scene.camera, shaders.at("wireframe"));
    }
}
void scene_model::display_bounding_spheres(std::map<std::string,GLuint>& , scene_structure& scene)
{
    sphere_visual.uniform.transform.scaling = R;
    for (size_t i = 0; i < particules.size(); i++)
    {
        sphere_visual.uniform.transform.translation = particules[i].p;
        draw(sphere_visual, scene.camera);
    }

}


void scene_model::velocity_integration(float dt)
{
    const vec3 g = {0,0,-9.81f};

    for(int k=0; k<particules.size(); ++k) {
        particules[k].p_i = particules[k].p;

        // Integrate speed
        particules[k].v = 0.999*particules[k].v + dt * particules[k].invMass * g;
    }

}



void scene_model::position_integration(float dt)
{
    for(int k=0; k<particules.size(); ++k) {
        // Integration position
        particules[k].p = particules[k].p + dt*particules[k].v;
    }

}

void scene_model::position_as_speed(float dt)
{
    for(int k=0; k<particules.size(); ++k) {
        // Convert displacement into speed
        particules[k].v = 1/dt*(particules[k].p-particules[k].p_i);
    }

}


void scene_model::throw_new_shape(const scene_structure& scene)
{
    const std::vector<vec3> color_LUT =  {{1,0.7f,0.7f},{0.7f,1,0.7f},{0.7f,0.7f,1},{1,1,0.7f},{1,0.7f,1},{0.7f,1,1}};

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
        shapes.push_back( rigid_body(particules, shapes.size(), r, mesh_model, position, speed, angular_speed, {0,0,0}) );
        shapes.back().visual.uniform.color = color_LUT[ shapes.size()%color_LUT.size() ];
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
    if(restart) initialize_shapes();
    if(start) timer.start();
    if(stop) timer.stop();
}



#endif

