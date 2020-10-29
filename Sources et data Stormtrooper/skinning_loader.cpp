#include "skinning_loader.hpp"
#ifdef ST2

#include <fstream>
#include <sstream>

using namespace vcl;



void compute_influence(buffer<buffer<skinning_influence>>& influence, vcl::buffer <joint_geometry>& joints, vcl::buffer<joint_connectivity> connectivity, mesh& mon_mesh) {
    influence.resize(0);
    for (int i = 0; i < mon_mesh.position.size(); i++) {
        const vec3 position_vertex = mon_mesh.position[i];
        float d1 = 99999.;
        float d2 = 99999.;
        float d3 = 99999.;
        int j1 = 999;
        int j2 = 999;
        int j3 = 999;
        int y = joints.size();
        for (int j = 1; j < joints.size(); j++) {
            vec3 distancevec = ((position_vertex - joints[j].p));
            float squaredist = ((distancevec.x * distancevec.x) + (distancevec.y * distancevec.y) + (distancevec.z * distancevec.z));

            if (squaredist < d1) {
                d3 = d2;
                d2 = d1;
                j3 = j2;
                j2 = j1;
                d1 = squaredist;
                j1 = j;
            }
            else if (squaredist < d2) {
                d3 = d2;
                d2 = squaredist;
                j3 = j2;
                j2 = j;
            }
            else if (squaredist < d3) {
                d3 = squaredist;
                j3 = j;
            }
        }


        // Maintenant il faut corriger les erreurs du calcul automatique.
        // C'est un peu le test clé pour un bon skinning... 
        
        // Première tentative qui ne marchait pas bien :        
        //if (connectivity[j1].parent != j2 && connectivity[j2].parent != j1 && j1 != j2) {
         //   if ((j1 <= 14 & j2 <= 14))
         //       j2 = j1;
        //}


        // Je rends les mains rigides et ne prend qu'un seul joins par main
        if (j1 > 22 && j1 < 38) j1 = 22;
        if (j2 > 22 && j2 < 38) j2 = 22;
        if (j3 > 22 && j3 < 38) j3 = 22;
        if (j1 > 41) j1 = 41;
        if (j2 > 41) j2 = 41;
        if (j3 > 41) j3 = 41;
        
        // Correction des erreurs de jambe : cela ça marche correctement
        // Stratégie : j1 décide sur quelle jambe on est et j2 et j3 doivent suivre
        if ((j1 >= 3) && (j1 <= 6) && (j2 >= 8) && (j2 <= 10)) { // J1 sur jambe droite et J2 sur jambe gauche
            j2 = j2 - 4;    // Changement de jambe !
            //std::cout << "1. Ligne " << i << std::endl;
            vec3 distancevec = ((position_vertex - joints[j2].p));
            d2 = ((distancevec.x * distancevec.x) + (distancevec.y * distancevec.y) + (distancevec.z * distancevec.z));
        }
        if ((j1 >= 3) && (j1 <= 6) && (j3 >= 8) && (j3 <= 10)) { // J1 sur jambe droite et J3 sur jambe gauche
            j3 = j3 - 4;    // Changement de jambe !
            //std::cout << "1. Ligne " << i << std::endl;
            vec3 distancevec = ((position_vertex - joints[j2].p));
            d3 = ((distancevec.x * distancevec.x) + (distancevec.y * distancevec.y) + (distancevec.z * distancevec.z));
        }


        if ((j1 >= 7) && (j1 <= 10) && (j2 >= 4) && (j2 <= 6)) { // J1 sur jambe droite et J2 sur jambe gauche
            j2 = j2 + 4;    // Changement de jambe !
            //std::cout << "2. Ligne " << i << std::endl;
            vec3 distancevec = ((position_vertex - joints[j2].p));
            d2 = ((distancevec.x * distancevec.x) + (distancevec.y * distancevec.y) + (distancevec.z * distancevec.z));
        }
        if ((j1 >= 7) && (j1 <= 10) && (j3 >= 4) && (j3 <= 6)) { // J1 sur jambe droite et J3 sur jambe gauche
            j3 = j3 + 4;    // Changement de jambe !
            //std::cout << "2. Ligne " << i << std::endl;
            vec3 distancevec = ((position_vertex - joints[j2].p));
            d2 = ((distancevec.x * distancevec.x) + (distancevec.y * distancevec.y) + (distancevec.z * distancevec.z));
        }

        // J'avais plein de problème en ne prenant que deux vertex, j'essaie avec les 3 plus proches
        buffer< skinning_influence> infl_vertex;
        skinning_influence vertex_i1,vertex_i2, vertex_i3;  

        float e1 = 1/(sqrt(d1)+1.e-6);
        float e2 = 1/(sqrt(d2) + 1.e-6);
        float e3 = 1 / (sqrt(d3) + 1.e-6);
        vertex_i1.joint = j1;
        vertex_i1.weight = e1/(e1+e2+e3);
        infl_vertex.push_back(vertex_i1);

        vertex_i2.joint = j2;
        vertex_i2.weight = e2/(e1+e2+e3);
        infl_vertex.push_back(vertex_i2);

        vertex_i3.joint = j3;
        vertex_i3.weight = e3 / (e1 + e2 + e3);
        infl_vertex.push_back(vertex_i3);

        influence.push_back(infl_vertex);
    }

}


void load_character_data(skeleton_structure& skeleton, skinning_structure& skinning, mesh_drawable& shape_visual, timer_interval& timer, GLuint shader)
{
    timer = timer_interval();

    const float scaling = 0.005f;

    // Load mesh
    buffer<buffer<int> > vertex_correspondance;
    mesh character = mesh_load_file_obj("scenes/sources/skinning/assets/marine/mesh.obj", vertex_correspondance);

    skeleton.connectivity = read_skeleton_connectivity("scenes/sources/skinning/assets/marine/skeleton_connectivity");
    skeleton.rest_pose = read_skeleton_geometry("scenes/sources/skinning/assets/marine/skeleton_geometry_local", scaling);
    buffer<buffer<skinning_influence>> influence = read_skinning_influence("scenes/sources/skinning/assets/marine/skinning_data");
    skinning.influence = map_correspondance(influence, vertex_correspondance);

    skeleton.anim           = read_skeleton_animation("scenes/sources/skinning/assets/marine/skeleton_animation_run", scaling);  // t_max = 0.733f;

    GLuint texture_id = create_texture_gpu(image_load_png("scenes/sources/skinning/assets/marine/texture.png"));

    for (vec3& p : character.position) p *= scaling; // scale vertices of the mesh
    shape_visual.clear();
    shape_visual = mesh_drawable(character);
    shape_visual.shader = shader;
    shape_visual.uniform.shading.specular = 0.1f;
    shape_visual.uniform.shading.specular_exponent = 8;

    character.fill_empty_fields();
    skinning.rest_pose = character.position;
    skinning.rest_pose_normal = character.normal;
    skinning.deformed = character;

    shape_visual.texture_id = texture_id;

}


// Load mesh 2 : mon Stoormtrooper
void load_character_data_ST2(skeleton_structure& skeleton, skinning_structure& skinning, mesh_drawable& shape_visual, timer_interval& timer, GLuint shader)
{
    timer = timer_interval();
    const float scaling = 0.005f;

    mesh character_ST2 = mesh_load_file_obj("../../../DATA/ST2/Storstrooper3.obj");
    skeleton.connectivity = read_skeleton_connectivity("scenes/sources/skinning/assets/marine/skeleton_connectivity");
    skeleton.rest_pose = read_skeleton_geometry("../../../DATA/ST2/ST2_geometry_local_v5.txt", 1);
    buffer<buffer<skinning_influence>> influence2;
    compute_influence(influence2, skeleton.rest_pose, skeleton.connectivity, character_ST2);
    skinning.influence = influence2;

    // Il faut bouger la rest_position après le compute_influence !
    for (int i = 0; i < skeleton.rest_pose.data.size(); i++) {
        skeleton.rest_pose.data[i].p /= 8;
        skeleton.rest_pose.data[i].p.z *= -1;
        skeleton.rest_pose.data[i].p.x += .4;
        skeleton.rest_pose.data[i].p.y += .2;
    }
    // Même paramètres que pour la rest_position pour le mesh
    for (vec3& p : character_ST2.position) {
        p /= 8;
        p.z *= -1;
        p.x += .4;
        p.y += .2;
    }

    // Il faut aussi corriger un problème : pour calculer la "vraie" rest_pos, scene_model::frame_draw
    // va faire skeleton_rest_pose = local_to_global(skeleton.rest_pose, skeleton.connectivity)
    // Or, j'ai calé rest_pose_ST2 puis qu'il colle à rest_pose après ce local_to_global. 
    // Je dois donc inverser cette fonction sur rest_pos_ST2 ici avant d'aller plus loin
    const size_t N = skeleton.connectivity.size();
    auto& rp = skeleton.rest_pose;
    std::vector<joint_geometry> calcul;
    calcul.resize(N);
    calcul[0] = rp[0];
    for (size_t k = 1; k < N; ++k)
    {
        const int parent = skeleton.connectivity[k].parent;
        calcul[k].r = inverse(rp[parent].r) * rp[k].r;
        calcul[k].p = inverse(rp[parent].r).apply(rp[k].p - rp[parent].p);
    }
    skeleton.rest_pose = calcul;

    // Inutile de recharger l'animation : on se sert de celle de l'autre personnage

    GLuint texture_id = create_texture_gpu(image_load_png("../../../DATA/ST2/stormtrooper_helmet_diffuse.png"));

    shape_visual.clear();
    shape_visual = mesh_drawable(character_ST2);
    shape_visual.shader = shader;
    shape_visual.uniform.shading.specular = 0.1f;
    shape_visual.uniform.shading.specular_exponent = 8;

    character_ST2.fill_empty_fields();
    skinning.rest_pose = character_ST2.position;
    skinning.rest_pose_normal = character_ST2.normal;
    skinning.deformed  = character_ST2;
    shape_visual.texture_id = texture_id;
}

buffer<buffer<skinning_influence> > read_skinning_influence(const std::string& filename)
{
    buffer<buffer<skinning_influence> > influence;

    std::ifstream fid(filename);

    // first line = number of influence per pertex (fixed for all vertices)
    size_t N_bone_influence=0;
    fid >> N_bone_influence;

    assert_vcl(fid.good(), "Cannot read file "+filename);
    assert_vcl_no_msg(N_bone_influence>0 && N_bone_influence<=6);

    // Read influence associated to each vertex
    std::vector<skinning_influence> skinning_vertex;
    skinning_vertex.resize(N_bone_influence);

    while(fid.good())
    {
        // read list of [bone index] [skinning weights]
        for(size_t k=0; k<N_bone_influence && fid.good(); ++k)
            fid >> skinning_vertex[k].joint >> skinning_vertex[k].weight;

        if(fid.good())
            influence.push_back(skinning_vertex);
    }

    fid.close();


    // normalize skinning weights (sum up to one)
    for(size_t kv=0; kv<influence.size(); ++kv)
    {
        float w = 0.0f;
        // compute weight sum
        for(size_t k=0; k<N_bone_influence; ++k)
            w += influence[kv][k].weight;
        // normalize
        if(w>1e-5f)
            for(size_t k=0; k<N_bone_influence; ++k)
                influence[kv][k].weight /= w;

    }

    return influence;
}

buffer<buffer<joint_geometry_time> > read_skeleton_animation(const std::string& filename, float scaling)
{
    buffer<buffer<joint_geometry_time> > skeleton;

    std::ifstream fid(filename);
    while(fid.good())
    {
        size_t N_key=0;
        fid >> N_key;

        if(fid.good())
        {
            buffer<joint_geometry_time> animated_joint;
            animated_joint.resize(N_key);

            for(size_t k_key=0; k_key<N_key; ++k_key)
            {
                float key_time;
                vec3 p;
                vec4 q;

                fid >> key_time;
                fid >> p.x >> p.y >> p.z;
                fid >> q.x >> q.y >> q.z >> q.w;

                q = normalize(q);

                animated_joint[k_key] = {key_time, {p*scaling,q} };
            }

            skeleton.push_back(animated_joint);
        }
    }

    fid.close();

    return skeleton;
}

buffer<joint_connectivity> read_skeleton_connectivity(const std::string& filename)
{
    buffer<joint_connectivity> skeleton;

    std::ifstream fid(filename);

    while(fid.good())
    {
        std::string line;
        std::getline(fid, line);

        if(fid.good())
        {
            std::stringstream sstream(line);

            int k;
            int parent;
            std::string name;

            sstream >> k >> parent >> name;

            skeleton.push_back({parent,name});
        }
    }

    fid.close();

    return skeleton;
}

buffer<joint_geometry> read_skeleton_geometry(const std::string& filename, float scaling)
{
    buffer<joint_geometry> skeleton;

    std::ifstream fid(filename);
    while(fid.good())
    {
        std::string line;
        std::getline(fid, line);

        if(fid.good())
        {
            std::stringstream sstream(line);

            vec3 p;
            quaternion q;

            sstream >> p.x >> p.y >> p.z;
            sstream >> q.x >> q.y >> q.z >> q.w;

            //q = normalize(q);

            skeleton.push_back({p*scaling,q});
        }
    }

    fid.close();

    return skeleton;
}


#endif
