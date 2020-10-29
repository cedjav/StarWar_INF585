#pragma once

#include "scenes/base/base.hpp"
#ifdef XWING

#include "quaternion.hpp"

enum gui_parameters_display_type { display_lave, display_noise, display_nasa,display_wireframe };

struct gui_parameters
{
    bool display_lave;
    bool display_nasa;  
    bool display_noise; 
    bool display_wireframe;
    int display_type;
};

struct scene_model : scene_base
{

    void setup_data(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);
    void frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);
    void set_gui();


    vcl::segment_drawable_immediate_mode segment_drawer;
    vcl::mesh_drawable sphere;
    GLuint shader_mesh;

    vcl::mesh_drawable mesh_xwing;
    GLuint shader_xwing;

    vcl::mesh_drawable frame;

    gui_parameters gui_param;

    vcl::timer_interval timer;
};



#endif
