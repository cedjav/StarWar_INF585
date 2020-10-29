#include "skinning_loader.hpp"
#ifdef XWING

#include <fstream>
#include <sstream>

using namespace vcl;

void load_xwing(mesh_drawable& shape_visual, GLuint shader) {

    float scaling = 0.005;

    buffer<buffer<int> > vertex_correspondance;
    mesh mon_mesh = mesh_load_file_obj("../../../DATA/ARC170/arc170.obj", vertex_correspondance);
    GLuint texture_id = create_texture_gpu(image_load_png("../../../DATA/ARC170/Arc170_blinn1.png"));

    for (vec3& p : mon_mesh.position)
        p *= scaling; // scale vertices of the mesh

    shape_visual.clear();
    shape_visual = mesh_drawable(mon_mesh);
    shape_visual.shader = shader;
    shape_visual.uniform.shading.specular = 0.1f;
    shape_visual.uniform.shading.specular_exponent = 8;
    shape_visual.texture_id = texture_id;
}

#endif
