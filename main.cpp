#include <cinolib/gl/glcanvas.h>
#include <cinolib/meshes/meshes.h>
#include <cinolib/gl/volume_mesh_controls.h>
#include <cinolib/drawable_segment_soup.h>

#include <cinolib/geodesics.h>
#include <cinolib/split_separating_simplices.h>

#include "sphere_util.h"

using namespace cinolib;

void get_df(DrawableTetmesh<> &m);
uint min_df_point(DrawableTetmesh<> &m);
uint max_df_point(DrawableTetmesh<> &m);
void remap_df(DrawableTetmesh<> &m, double shift);

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
    GLcanvas gui;
    DrawableTetmesh<> m(ARMADILLO);
    std::cout << std::endl;

    //surf_dist field
    get_df(m);

    std::cout << std::endl << TXT_BOLDWHITE << "Pre remap:" << TXT_RESET << std::endl;
    //max surf_dist field point
    uint center = min_df_point(m);
    //min surf_dist field point
    uint close = max_df_point(m);
    //remap and invert the surf_dist field
    remap_df(m, m.vert_data(close).uvw[0]);

    //check after remap
    std::cout << std::endl << TXT_BOLDWHITE << "Post remap:" << TXT_RESET << std::endl;
    std::cout << TXT_BOLDBLUE << "Vertice selezionato MIN: " << TXT_RESET << center << "\t";
    std::cout << TXT_BOLDBLUE << "Valore del campo MIN: " << TXT_RESET << m.vert_data(center).uvw[0] << std::endl;
    std::cout << TXT_BOLDBLUE << "Vertice selezionato MAX: " << TXT_RESET << close << "\t";
    std::cout << TXT_BOLDBLUE << "Valore del campo MAX: " << TXT_RESET << m.vert_data(close).uvw[0] << std::endl;

    /*
    uint adj_center = m.adj_v2v(center)[0];
    double scale_factor = (m.vert_data(center).uvw[0] - );
    */

    //find closest surface point and distance
    uint surf_vert;
    double surf_dist = inf_double;
    for(uint vid : m.get_surface_verts()) {
        double aux_distance = m.vert(center).dist(m.vert(vid));
        if(m.vert(center).dist(m.vert(vid)) < surf_dist) {
            surf_dist = aux_distance;
            surf_vert = vid;
        }
    }

    //draw line between center of the sphere e che closest surface point
    DrawableSegmentSoup r;
    r.push_seg(m.vert(center), m.vert(surf_vert), Color::RED());
    gui.push(&r);

    //sphere
    std::cout << std::endl << std::endl;
    DrawableTetmesh<> sphere = get_sphere(m.vert(center), surf_dist, 50);
    std::cout << std::endl << std::endl;

    //canvas
    gui.push(&m);
    gui.push(new VolumeMeshControls<DrawableTetmesh<>>(&m,&gui));
    gui.push(&sphere);
    gui.push(new VolumeMeshControls<DrawableTetmesh<>>(&sphere,&gui));

    return gui.launch();
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

/** Map the distance field to the vertices of the mesh passed as argument
 * @param m a tet mesh **/
void get_df(DrawableTetmesh<> &m){
    split_separating_simplices(m);
    std::vector<uint> bc = m.get_surface_verts();
    ScalarField d = compute_geodesics(m,bc,COTANGENT, 1.0, true);
    d.copy_to_mesh(m); //carica il campo scalare sui vertici (.vert_data(id).uvw)
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

/// Find the vertex with the minimum distance field value in the mesh passed as argument
/// @param m a tet mesh
/// @return the id of the vertex with the minimum distance from the surface
uint max_df_point(DrawableTetmesh<> &m){

    //trova il punto più vicino dalla superficie
    uint point = 0;
    for(int vid = 1; vid < m.num_verts(); vid++)
        if(m.vert_data(vid).uvw[0] > m.vert_data(point).uvw[0])
            point = vid;

    m.vert_data(point).color = Color::BLUE();
    m.show_in_vert_color();


    std::cout << TXT_BOLDMAGENTA << "Vertice selezionato MIN: " << TXT_RESET << point << "\t";
    std::cout << TXT_BOLDMAGENTA << "Valore del campo MIN: " << TXT_RESET << m.vert_data(point).uvw[0] << std::endl;

    return point;
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

/// Find the vertex with the maximum distance field value in the mesh passed as argument
/// @param m a tet mesh
/// @return the id of the vertex with the maximum distance from the surface
uint min_df_point(DrawableTetmesh<> &m) {

    //trova il punto più lontano dalla superficie
    uint point = 0;
    for(int vid = 1; vid < m.num_verts(); vid++)
        if(m.vert_data(vid).uvw[0] < m.vert_data(point).uvw[0])
            point = vid;

    m.vert_data(point).color = Color::RED();
    m.show_in_vert_color();

    std::cout << TXT_BOLDBLUE << "Vertice selezionato MIN: " << TXT_RESET << point << "\t";
    std::cout << TXT_BOLDBLUE << "Valore del campo MIN: " << TXT_RESET << m.vert_data(point).uvw[0] << std::endl;

    return point;
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

void remap_df(DrawableTetmesh<> &m, double shift) {

    for(uint vid = 0; vid < m.num_verts(); vid++) {
        m.vert_data(vid).uvw[0] *= -1.0;
        m.vert_data(vid).uvw[0] += shift;
    }

}