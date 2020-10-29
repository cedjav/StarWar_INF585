
#include "skinning.hpp"
#ifdef XWING

#include "skinning_loader.hpp"


using namespace vcl;


float anglederotation;
vcl::buffer<vcl::vec3> points, di, diplusun;
vcl::buffer<float> temps;
vcl::vec3 lastp=vec3(0.,0.,0.);
vcl::mesh shape;
vcl::mesh_drawable shape_visual;
GLuint texture_nasa,texture_noise,texture_lave;

void create_vectors_for_xwing_trajectory() {
    // Brut force implementation for the xwing "circuit"
    points.push_back(vec3(-10., -10., +6.));
    points.push_back(vec3(-9., -0., +6.));
    points.push_back(vec3(0, +8., +6.));
    points.push_back(vec3(+7., 0., +6.));
    points.push_back(vec3(0., -6., +5.5));
    points.push_back(vec3(-6., -0., +5));
    points.push_back(vec3(0., 6., +4.5));
    points.push_back(vec3(+6., 0., +4));
    points.push_back(vec3(0., -6., +3.5));
    points.push_back(vec3(-6., 0., +3.));
    points.push_back(vec3(0., 6., +2.5));
    points.push_back(vec3(6., -0., +2.));
    points.push_back(vec3(2., -6., +1.));
    points.push_back(vec3(0., -2., 0.));
    points.push_back(vec3(0., +2., 0.));
    points.push_back(vec3(0., +2, 0.));
    points.push_back(vec3(0., +2, 0.));
    points.push_back(vec3(0., +2, 0.));
    points.push_back(vec3(0., +100, 10.));
    points.push_back(vec3(0., +200, 20.));
    points.push_back(vec3(0., +400, 40.));
    points.push_back(vec3(0., +800, 80.));
    for (int i = 0;i < 19;i++)
        temps.push_back(i * 2);
    temps.push_back(52);
    temps.push_back(52);
    float mu = 2.f;
    float sigma;
    di.resize(temps.size() - 1);
    diplusun.resize(temps.size() - 1);
    for (int i = 1; i < temps.size() - 2; i++) {
        sigma = temps[i + 1] - temps[i];
        di[i] = mu * sigma * (points[i + 1] - points[i - 1]) / (temps[i + 1] - temps[i - 1]);
        diplusun[i] = mu * sigma * (points[i + 2] - points[i]) / (temps[i + 2] - temps[i]);
    }
}


vec3 locatisation_xwing(float t) {
    // D'abord trouver dans quel intervalle [t_i,t_i+1] on est
    int i0 = 1;
    for (int i = 2;i < temps.size() - 2;i++)
        if (t > temps[i]) i0 = i;

    if (t > temps[temps.size() - 2])
        t = temps[temps.size() - 2];    // Time limit

    // On commence par calculer step by step l'interpolation linéaire
    float s = (t - temps[i0]) / (temps[i0 + 1] - temps[i0]);
    vec3 p1 = (2 * s * s * s - 3 * s * s + 1) * points[i0];
    vec3 p2 = (s * s * s * -2 * s * s + s) * di[i0];
    vec3 p3 = (-2 * s * s * s + 3 * s * s) * points[i0 + 1];
    vec3 p4 = (s * s * s - s * s) * diplusun[i0];
    vec3 p5 = points[i0];

    return (2 * s * s * s - 3 * s * s + 1) * points[i0] + (s * s * s - 2 * s * s + s) * di[i0] + (-2 * s * s * s + 3 * s * s) * points[i0 + 1] + (s * s * s - s * s) * diplusun[i0];
}



void scene_model::setup_data(std::map<std::string,GLuint>& shaders, scene_structure& , gui_structure& )
{
    // Mise en place du XWing
    create_vectors_for_xwing_trajectory();
    shader_xwing = shaders["mesh"];
    load_xwing(mesh_xwing, shader_xwing);

    // Mise en place du sol avec un bruit de Perlin pour modéliser le relief
    const size_t N = 300;
    shape = mesh_primitive_grid(N, N,vec3(-30, -30, 0), vec3(60, 0, 0), vec3(0, 60, 0));
    for (size_t x = 0; x < N; x++)
        for (size_t y = 0; y < N; y++) {
            shape.position[y * N + x].z = -4+perlin(((float)x) /N, ((float)y) / N, 8, .7f, 2.0f)*2;
        }

    texture_nasa= create_texture_gpu(image_load_png("../../../DATA/Nasa.png"));
    texture_lave = create_texture_gpu(image_load_png("../../../DATA/Lave.png"));
    texture_noise = create_texture_gpu(image_load_png("../../../DATA/Noise.png"));
    shape_visual = shape;
    shape_visual.uniform.color = { 1,1,1 };
    shape_visual.texture_id = texture_lave;        // Et une texture de sol à améliorer
    shape_visual.shader = shaders["mesh"];

    //segment_drawer.init();
    //segment_drawer.uniform_parameter.color = {0.0f,0.0f,0.0f};
    //glEnable(GL_POLYGON_OFFSET_FILL);

    gui_param.display_lave=true;
    gui_param.display_nasa = false;
    gui_param.display_noise = false;
    gui_param.display_wireframe = false;
    
    shader_xwing = shaders["mesh"];
    load_xwing(mesh_xwing, shader_xwing);

    // Gestion du timer
    timer.t_min = 0;
    timer.t_max = 38;
    timer.scale = 1.f;
    timer.start();

}

void scene_model::frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& )
{
    const mat3 remisedanslebonsens = mat3(vec3(1, 0, 0), vec3(0, 0, 1), vec3(0, 1, 0));     // Le vaisseau spatial chargé est "à plat" pour y=cte mais je veux z=cte
    vec3 p;
    timer.update();
    set_gui();
    float t = timer.t;
    float angle;

    //On commence par gérer le XWing
    if (t < 40) {
        p = locatisation_xwing(t);     //Après il ne bouge plus
        vec3 direction = p - lastp;
        lastp = p;
        float a = direction[0];
        float b = direction[1];
        if (a != 0) {     // Sinon l'angle de rotation garde son ancienne valeur pour éviter les discontinuités
            angle = atan(b / a);
            anglederotation = 3.14159 / 2 - angle;
            if ((a < 0) && (abs(b) + abs(a)) > 0.0003)
                anglederotation += 3.14159;       // Pour gérer le problème que atan ne donne une réponse qu'en -pi/2 et +pi/2 et qu'on a 2pi en tout
        }
        mat3 rotationpourremettredanslaxe = mat3(vec3(cos(anglederotation), -sin(anglederotation), 0.), vec3(sin(anglederotation), cos(anglederotation), 0), vec3(0, 0, 1));
        mesh_xwing.uniform.transform.rotation = (rotationpourremettredanslaxe * remisedanslebonsens);
    }
    else {
        p = lastp;
        mesh_xwing.uniform.transform.rotation = remisedanslebonsens;
    }
    mesh_xwing.uniform.transform.translation = p;
    draw(mesh_xwing, scene.camera, mesh_xwing.shader, mesh_xwing.texture_id);
    glBindTexture(GL_TEXTURE_2D, scene.texture_white);

    // Maintenant le sol
   
    glPolygonOffset(1.0, 1.0);
    if (gui_param.display_type == display_wireframe)
        draw(shape_visual, scene.camera, shaders["wireframe_quads"]);
    else
        draw(shape_visual, scene.camera);
   
}


void scene_model::set_gui()
{
    ImGui::SliderFloat("Timer",  &timer.t, timer.t_min, timer.t_max, "%.2f s");
    ImGui::SliderFloat("Time scale", &timer.scale, 0.05f, 2.0f, "%.2f s");

    ImGui::Text("Sol Texture:");
    bool click_lave = ImGui::RadioButton("Lave", &gui_param.display_type,display_lave);  ImGui::SameLine();
    bool click_nasa = ImGui::RadioButton("Nasa", &gui_param.display_type,display_nasa);  ImGui::SameLine();
    bool click_noise = ImGui::RadioButton("Noise", &gui_param.display_type,display_noise); ImGui::SameLine();
    bool click_wireframe = ImGui::RadioButton("Wireframe", &gui_param.display_type,display_wireframe);

    if (click_lave) shape_visual.texture_id = texture_lave;
    if (click_nasa) shape_visual.texture_id = texture_nasa;
    if (click_noise) shape_visual.texture_id = texture_noise;

    // Start and stop animation
    bool const stop  = ImGui::Button("Stop"); ImGui::SameLine();
    bool const start = ImGui::Button("Start"); ImGui::SameLine();

    if(stop) timer.stop();
    if(start) timer.start();
}

#endif
