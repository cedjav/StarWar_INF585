
#include "skinning.hpp"
#ifdef ST2

#include "skinning_loader.hpp"


using namespace vcl;


vcl::buffer<vcl::vec3> points, di, diplusun;
vcl::buffer<float> temps;
vcl::vec3 lastp = vec3(0., 0., 0.);
float anglederotation;

void scene_model::setup_data(std::map<std::string,GLuint>& shaders, scene_structure& , gui_structure& )
{

    
    shader_mesh = shaders["mesh"];

    segment_drawer.init();
    segment_drawer.uniform_parameter.color = {0.0f,0.0f,0.0f};
    glEnable(GL_POLYGON_OFFSET_FILL);

    // Init gui parameters
    gui_param.display_mesh      = false;
    gui_param.display_wireframe = false;
    gui_param.display_rest_pose = false;
    gui_param.display_skeleton_bones  = true;
    gui_param.display_skeleton_joints = true;
    gui_param.display_texture = true;

    //gui_param.display_type = display_bar;

    // Sphere used to display joints
    sphere = mesh_primitive_sphere(0.005f);
    sphere.shader = shader_mesh;

    frame = mesh_primitive_frame();
    frame.uniform.transform.scaling = 0.02f;
    frame.shader = shaders["mesh"];

    load_character_data_ST2(skeleton_ST2, skinning_ST2, character_visual_ST2, timer, shader_mesh);
    load_character_data(skeleton, skinning, character_visual, timer, shader_mesh);
     
    timer.t_min = 0.0f;
    timer.t_max = 0.733f;
    timer.scale = 0.05f;

}


buffer<joint_geometry> interpolate_skeleton_at_time(float time, const buffer< buffer<joint_geometry_time> >& animation)
{
    // Compute skeleton corresponding to a given time from key poses
    // - animation[k] corresponds to the kth animated joint (vector of joint geometry at given time)
    // - animation[k][i] corresponds to the ith pose of the kth joint
    //      - animation[k][i].time         -> corresponding time
    //      - animation[k][i].geometry.p/r -> corresponding position, rotation

    size_t N_joint = animation.size();
    buffer<joint_geometry> skeleton;
    skeleton.resize(N_joint);

    for(size_t k_joint=0; k_joint<N_joint; ++k_joint)
    {
        const buffer<joint_geometry_time>& joint_anim = animation[k_joint];

        // Find the index corresponding to the current time
        size_t k_current = 0;
        assert_vcl_no_msg(joint_anim.size()>k_current+1);
        if (time > joint_anim[joint_anim.size() - 1].time) 
            time = 0;
        while( time>joint_anim[k_current+1].time ) {
            ++k_current;
            assert_vcl_no_msg(joint_anim.size()>k_current+1);
        }

        const joint_geometry current_geometry1 = joint_anim[k_current].geometry;
        const joint_geometry current_geometry2 = joint_anim[k_current + 1].geometry;
        float time1 = joint_anim[k_current].time;
        float time2 = joint_anim[k_current + 1].time;
        float t = (time-time1) / (time2 - time1);
        quaternion q1 = current_geometry1.r;
        quaternion q2 = current_geometry2.r;
        quaternion q = slerp(q1, q2, t);

        vec3 p1 = current_geometry1.p;
        vec3 p2 = current_geometry2.p;
        vec3 p = p1 * (1-t) + t * p2;

        joint_geometry current_geometry;
        current_geometry.r = q;
        current_geometry.p = p;
        skeleton[k_joint] = current_geometry;
    }

    return skeleton;
}

void compute_skinning(skinning_structure& skinning,
                      const buffer<joint_geometry>& skeleton_current,
                      const buffer<joint_geometry>& skeleton_rest_pose)
{
    const size_t N_vertex = skinning.rest_pose.size();
    for(size_t k=0; k<N_vertex; ++k) {

        skinning.deformed.position[k] = vec3(0,0,0);
        vcl::buffer<skinning_influence> current_influence = skinning.influence[k];

        for (size_t l = 0; l < current_influence.size(); l++) {
            skinning_influence current_skinning_influence = current_influence[l];
            float w = current_skinning_influence.weight;
            int number = current_skinning_influence.joint;

            joint_geometry current_geometry = skeleton_current[number];
            joint_geometry current_geometry0 = skeleton_rest_pose[number];

            mat3 r1 = current_geometry.r.matrix();
            mat3 r10inv = conjugate(current_geometry0.r).matrix();
            vec3 t1 = current_geometry.p;
            vec3 t10 = current_geometry0.p;
            vec3 p0 = skinning.rest_pose[k];

            vec3 result = r1 * r10inv * p0 - r1 * r10inv * t10 + t1;
            skinning.deformed.position[k] += w * result;
        }
    }
}


// Convert skeleton from local to global coordinates
buffer<joint_geometry> local_to_global(const buffer<joint_geometry>& local, const buffer<joint_connectivity>& connectivity)
{
    const size_t N = connectivity.size();
    assert(local.size()==connectivity.size());
    std::vector<joint_geometry> global;
    global.resize(N);
    global[0] = local[0];

    for(size_t k=1; k<N; ++k)
    {
        const int parent = connectivity[k].parent;
        global[k].r = global[parent].r * local[k].r;
        global[k].p = global[parent].r.apply(local[k].p) + global[parent].p;
    }

    return global;
}


void display_skeleton(const buffer<joint_geometry>& skeleton_geometry,
                      const buffer<joint_connectivity>& skeleton_connectivity,
                      const std::map<std::string,GLuint>& shaders,
                      const scene_structure& scene,
                      segment_drawable_immediate_mode& segment_drawer)
{
    const size_t N = skeleton_geometry.size();
    for(size_t k=1; k<N; ++k)
    {
        int parent = skeleton_connectivity[k].parent;
        const vec3& p1 = skeleton_geometry[parent].p;
        const vec3& p2 = skeleton_geometry[k].p;

        segment_drawer.uniform_parameter.p1 = p1;
        segment_drawer.uniform_parameter.p2 = p2;
        segment_drawer.draw(shaders.at("segment_im"),scene.camera);
    }
}

void display_joints(const buffer<joint_geometry>& skeleton_geometry,
                    const scene_structure& scene,
                    mesh_drawable& sphere)
{
    const size_t N = skeleton_geometry.size();
    for(size_t k=0; k<N; ++k)
    {
        sphere.uniform.transform.translation = skeleton_geometry[k].p;
        draw(sphere, scene.camera);
    }

}

void display_frames(const buffer<joint_geometry>& skeleton_geometry,
                    const scene_structure& scene,
                    mesh_drawable& frame)
{
    const size_t N = skeleton_geometry.size();
    for(size_t k=0; k<N; ++k)
    {
        frame.uniform.transform.rotation = skeleton_geometry[k].r.matrix();
        frame.uniform.transform.translation = skeleton_geometry[k].p;
        draw(frame, scene.camera);
    }
}


void scene_model::frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& )
{
    timer.update();
    set_gui();
    float t = timer.t;

    const auto skeleton_geometry_local  = interpolate_skeleton_at_time(t, skeleton.anim);
    const auto skeleton_rest_pose = local_to_global(skeleton.rest_pose, skeleton.connectivity);
    auto skeleton_rest_pose_ST2 = local_to_global(skeleton_ST2.rest_pose, skeleton.connectivity);
    auto skeleton_current = local_to_global(skeleton_geometry_local, skeleton.connectivity);

    // Maintenant, il faut passer skeleton_geometry_local au Stoormtrooper dans sa géométrie à lui !
    const size_t N = skeleton.connectivity.size();
    buffer<joint_geometry> st2_current;
    st2_current.resize(N);
    st2_current[0].r = skeleton_geometry_local[0].r;
    st2_current[0].p = skeleton_ST2.rest_pose[0].p;

    for (size_t k = 1; k < N; ++k)
    {
        const int parent = skeleton.connectivity[k].parent;
        float taille_bone_st2 = norm(skeleton_ST2.rest_pose[k].p);
        float taille_bone_pTD = norm(skeleton.rest_pose[k].p);
        float ratio = taille_bone_st2 / taille_bone_pTD;
        st2_current[k].r = st2_current[parent].r * skeleton_geometry_local[k].r;
        st2_current[k].p = st2_current[parent].r.apply(ratio * skeleton_geometry_local[k].p) + st2_current[parent].p;

        // On en profite pour copier l'orientation des quaternions des rest_postions
        skeleton_rest_pose_ST2.data[k].r = skeleton_rest_pose.data[k].r;
    }


    if (gui_param.display_rest_pose) {
        skeleton_current = skeleton_rest_pose;
        st2_current = skeleton_rest_pose_ST2;
    }

    compute_skinning(skinning, skeleton_current, skeleton_rest_pose);
    character_visual.update_position(skinning.deformed.position);
    character_visual.update_normal(skinning.deformed.normal);

    compute_skinning(skinning_ST2, st2_current, skeleton_rest_pose_ST2);
    character_visual_ST2.update_position(skinning_ST2.deformed.position);
    character_visual_ST2.update_normal(skinning_ST2.deformed.normal);


    if (gui_param.display_skeleton_bones) {
        display_skeleton(skeleton_current, skeleton.connectivity, shaders, scene, segment_drawer);
        display_skeleton(st2_current, skeleton.connectivity, shaders, scene, segment_drawer);
    }
    if (gui_param.display_skeleton_joints) {
        display_joints(skeleton_current, scene, sphere);
        display_joints(st2_current, scene, sphere);
    }
    if (gui_param.display_skeleton_frames) {
        display_frames(skeleton_current, scene, frame);
        display_frames(st2_current, scene, frame);
    }

    if(gui_param.display_mesh) {
        glPolygonOffset( 1.0, 1.0 );
        GLuint const texture_id = (gui_param.display_texture? character_visual.texture_id : scene.texture_white);
        draw(character_visual, scene.camera, character_visual.shader, texture_id);
        GLuint const texture_id_ST2 = (gui_param.display_texture ? character_visual_ST2.texture_id : scene.texture_white);
        draw(character_visual_ST2, scene.camera, character_visual_ST2.shader, texture_id_ST2);

    }
    if(gui_param.display_wireframe) {
        glPolygonOffset( 1.0, 1.0 );
        draw(character_visual, scene.camera, shaders["wireframe_quads"]);
        draw(character_visual_ST2, scene.camera, shaders["wireframe_quads"]);
    }
}


void scene_model::set_gui()
{
    ImGui::SliderFloat("Timer",  &timer.t, timer.t_min, timer.t_max, "%.2f s");
    ImGui::SliderFloat("Time scale", &timer.scale, 0.05f, 2.0f, "%.2f s");

    ImGui::Text("Display Mesh:");
    ImGui::Checkbox("Mesh", &gui_param.display_mesh); ImGui::SameLine();
    ImGui::Checkbox("Wireframe", &gui_param.display_wireframe); ImGui::SameLine();
    ImGui::Checkbox("Texture", &gui_param.display_texture);

    ImGui::Text("Display Skeleton:");
    ImGui::Checkbox("Bones", &gui_param.display_skeleton_bones);  ImGui::SameLine();
    ImGui::Checkbox("Joints", &gui_param.display_skeleton_joints);  ImGui::SameLine();
    ImGui::Checkbox("Frames", &gui_param.display_skeleton_frames);

    ImGui::Checkbox("Rest pose", &gui_param.display_rest_pose);

    // Start and stop animation
    bool const stop  = ImGui::Button("Stop"); ImGui::SameLine();
    bool const start = ImGui::Button("Start"); ImGui::SameLine();

    if(stop) timer.stop();
    if(start) timer.start();
}



#endif
