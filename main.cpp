#include <cinolib/gl/glcanvas.h>
#include <cinolib/gl/camera.h>

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

#define PRINT_SPHERERADIUS 0
#define PRINT_VECTORFIELD 0
#define PRINT_ARROW 0

int main(int argc, char *argv[]) {

    //UI
    GLcanvas gui;
    gui.show_side_bar = true;
    gui.side_bar_alpha = 0.5;
    // 1800 1035 -0.002324 -0.004054 -0.00237851 0.642835 0.435 -0.775047 0.631089 -0.0320702 0.000680942 -0.12701 -0.105865 0.986236 0.00162142 0.619007 0.768453 0.162205 0.00493968 0 0 0 1 1 0 0 -0 0 1 0 -0 0 0 1 -2.57134 0 0 0 1 1.02813 0 0 -0 0 1.78806 0 -0 0 0 -0.777805 -2 0 0 0 1

    //model vol
    DrawableTetmesh<> m(KITTY);
    m.show_mesh_points();
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
    for (uint vid = 0; vid < m.num_verts(); vid++) {
        //default
        m.vert_data(vid).uvw[0] = 0;
        //if not in the surface check the closest srf point and the distance to it
        if (!m.vert_is_on_srf(vid)) {
            m.vert_data(vid).uvw[0] = o.closest_point(m.vert(vid)).dist(m.vert(vid));
            //check if its the max distance
            if (m.vert_data(vid).uvw[0] > center_dist) {
                center_vid = vid;
                center_dist = m.vert_data(vid).uvw[0];
            }
        }
    }

    //sphere build
    std::cout << std::endl << std::endl;
    DrawableTetmesh<> sphere = get_sphere(m.vert(center_vid), center_dist);
    gui.push(&sphere, false);
    gui.push(new VolumeMeshControls<DrawableTetmesh<>>(&sphere, &gui));

    std::cout << TXT_BOLDGREEN << "Raggio sfera: " << TXT_RESET << sphere.bbox().delta_x() << std::endl;
    std::cout << TXT_BOLDGREEN << "Centro: " << TXT_RESET << center_vid << " (" << sphere.centroid() << ")" << std::endl;

#if PRINT_SPHERERADIUS == 1
    DrawableSegmentSoup radius;
    radius.push_seg(m.vert(center_vid), o.closest_point(m.vert(center_vid)));
    gui.push(&radius);
#endif

    //dir and vel of the movement
    std::vector<vec3d> srf_verts;
    std::vector<vec3d> srf_normals;
    for (uint vid: sphere.get_surface_verts()) {
        srf_verts.push_back(sphere.vert(vid));
        srf_normals.push_back(sphere.vert_data(vid).normal * o.closest_point(sphere.vert(vid)).dist(sphere.vert(vid)));
    }

//print as vectorfield
#if PRINT_VECTORFIELD == 1
    DrawableVectorField vf(srf_normals,srf_verts);
    vf.set_arrow_size(0.1);
    gui.push(&vf);
#endif

//print as vector of arrows
#if PRINT_ARROW == 1
    std::vector<DrawableArrow> da;
    for (uint vid = 0; vid < srf_verts.size(); vid++)
        da.emplace_back(srf_verts[vid], srf_verts[vid] + srf_normals[vid]);

    for (auto &id: da)
        gui.push(&id, false);
#endif


    return gui.launch();
}