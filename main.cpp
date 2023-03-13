#include <cinolib/gl/glcanvas.h>

#include <cinolib/meshes/meshes.h>
#include <cinolib/gl/volume_mesh_controls.h>

#include <cinolib/drawable_octree.h>
#include <cinolib/drawable_segment_soup.h>
#include <cinolib/drawable_vector_field.h>
#include <cinolib/drawable_arrow.h>

#include "sphere_util.h"

using namespace cinolib;

/*      Reference
 *      Tetrahedron:
 *
 *          v3
 *         /| \
 *       /  |   \
 *     v0---|----v2
 *      \   |   /
 *        \ | /
 *          v1
*/

//model macros
#define ANT "../data/ant.mesh"
#define ARMADILLO "../data/armadillo.mesh"
#define BUNNY "../data/bunny.mesh"
#define CHINESE_DRAGON "../data/chinese_dragon.mesh"
#define DANCER_CHILDREN "../data/dancer_children.mesh"
#define DAVID "../data/david.mesh"
#define DRAGON "../data/dragon.mesh"
#define GARGOYLE "../data/gargoyle.mesh"
#define KITTEN "../data/kitten.mesh"
#define KITTY "../data/kitty.mesh"
#define MOUSE "../data/mouse.mesh"
#define PIG "../data/pig.mesh"

int main(int argc, char *argv[])
{
    //UI
    GLcanvas gui;

    //model
    DrawableTetmesh<> m(KITTY);
    gui.push(&m);
    gui.push(new VolumeMeshControls<DrawableTetmesh<>>(&m, &gui));
    //model srf
    Trimesh<> srf;
    export_surface(m, srf);

    //octree build
    Octree o;
    o.build_from_mesh_polys(srf);

    //distance field & sphere center/scale
    uint center_vid = 0;
    double center_dist = -inf_double;
    for(uint vid=0; vid<m.num_verts(); vid++) {
        //default
        m.vert_data(vid).uvw[0] = 0;
        //if not in the surface check the closest srf point and the distance to it
        if(!m.vert_is_on_srf(vid)) {
            m.vert_data(vid).uvw[0] = o.closest_point(m.vert(vid)).dist(m.vert(vid));
            //check if its the max distance
            if(m.vert_data(vid).uvw[0] > center_dist) {
                center_vid = vid;
                center_dist = m.vert_data(vid).uvw[0];
            }
        }
    }

    //sphere build
    std::cout << std::endl << std::endl;
    DrawableTetmesh<> sphere = get_sphere(m.vert(center_vid), center_dist, 50);
    std::cout << TXT_BOLDGREEN << "Centro: " << TXT_RESET << center_vid << " (" << m.vert(center_vid) << ")" << std::endl;
    std::cout << TXT_BOLDGREEN << "Raggio sfera: " << TXT_RESET << center_dist << std::endl;
    gui.push(&sphere);
    gui.push(new VolumeMeshControls<DrawableTetmesh<>>(&sphere,&gui));

    //vector field for visualizing sphere normals
    std::vector<vec3d> srf_verts;
    std::vector<vec3d> srf_normals;
    for(uint vid : sphere.get_surface_verts()) {
        srf_verts.push_back(sphere.vert(vid));
        srf_normals.push_back(sphere.vert_data(vid).normal * o.closest_point(sphere.vert(vid)).dist(sphere.vert(vid)));
    }
    //DrawableVectorField vf(srf_normals,srf_verts);
    //vf.set_arrow_size(0.1);
    //gui.push(&vf);

    std::vector<DrawableArrow> da;
    for(uint vid = 0; vid < srf_verts.size(); vid++)
        da.emplace_back(srf_verts[vid], srf_verts[vid] + srf_normals[vid] * o.closest_point(sphere.vert(vid)).dist(sphere.vert(vid)) * 10);

    for(auto & id : da)
        gui.push(&id);

    return gui.launch();
}