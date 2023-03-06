#include <cinolib/gl/glcanvas.h>
#include <cinolib/meshes/meshes.h>
#include <cinolib/gl/volume_mesh_controls.h>
#include <cinolib/geodesics.h>
#include <cinolib/split_separating_simplices.h>

#include "sphere_util.h"

using namespace cinolib;

void get_df(DrawableTetmesh<> &m);
uint max_df_point(DrawableTetmesh<> &m);

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

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

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

int main(int argc, char *argv[])
{
    //mesh
    DrawableTetmesh<> m("/Users/federicomeloni/Documents/GitHub/MeshRepository/Volume/L12014_polycubes/kitty.mesh");

    //distance field
    get_df(m);
    //max distance field point
    uint center = max_df_point(m);
    //magnitude of the vector
    double scale_factor = sqrt(pow(m.vert_data(center).uvw.x(),2)
            + pow(m.vert_data(center).uvw.y(),2)
            + pow(m.vert_data(center).uvw.z(),2));



    //sphere
    DrawableTetmesh<> sphere = get_sphere(m.vert(center), scale_factor, 18);

    GLcanvas gui;
    gui.push(&m);
    gui.push(new VolumeMeshControls<DrawableTetmesh<>>(&m,&gui));
    gui.push(&sphere);
    gui.push(new VolumeMeshControls<DrawableTetmesh<>>(&sphere,&gui));

    return gui.launch();
}

/** Map the distance field to the vertices of the mesh passed as argument
 * @param m a tet mesh **/
void get_df(cinolib::DrawableTetmesh<> &m){

    split_separating_simplices(m);
    std::vector<uint> bc = m.get_surface_verts();
    ScalarField d = compute_geodesics(m,bc,COTANGENT, 1.0, true);
    d.copy_to_mesh(m); //carica il campo scalare sui vertici (.vert_data(id).uvw)

}

/** Find the vertex with the maximum distance field value in the mesh passed as argument
 * @param m a tet mesh
 * @return the id of the vertex with the maximum distance from the surface **/
uint max_df_point(cinolib::DrawableTetmesh<> &m){

    //trova il punto più lontano dalla superficie
    uint center = 0;
    for(int vid = 1; vid < m.num_verts(); vid++)
        if(m.vert_data(vid).uvw < m.vert_data(center).uvw)
            center = vid;

    m.vert_data(center).color = Color::RED();

    return center;
}
