#include <cinolib/gl/glcanvas.h>

#include <cinolib/meshes/meshes.h>
#include <cinolib/gl/volume_mesh_controls.h>

#include <cinolib/find_intersections.h>

#include "sphere_util.h"
#include "ui_tools.h"

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

uint query_map(std::map<ipair, uint> &vert_map, Tetmesh<> &m, uint pid, uint off0, uint off1);

int main(int argc, char *argv[]) {

    //UI
    GLcanvas gui;
    gui.show_side_bar = true;
    gui.side_bar_alpha = 0.5;

    /*
    1800 1035 -0.002324 -0.004054 -0.00237851 0.642835 0.435 -0.775047 0.631089 -0.0320702 0.000680942 -0.12701 -0.105865 0.986236 0.00162142 0.619007 0.768453 0.162205 0.00493968 0 0 0 1 1 0 0 -0 0 1 0 -0 0 0 1 -2.57134 0 0 0 1 1.02813 0 0 -0 0 1.78806 0 -0 0 0 -0.777805 -2 0 0 0 1
    */

    std::cout << std::endl << TXT_BOLDMAGENTA << "Model loading" << TXT_RESET << std::endl;

    //model vol
    DrawableTetmesh<> m(KITTY);
    m.show_mesh_points();
    gui.push(&m);
    gui.push(new VolumeMeshControls<DrawableTetmesh<>>(&m, &gui));

    //model srf
    Trimesh<> srf;
    export_surface(m, srf);

    //octree build
    Octree oct;
    oct.build_from_mesh_polys(srf);

    //distance field & sp center/scale
    uint center_vid = 0;
    double center_dist = -inf_double;
    for (uint vid = 0; vid < m.num_verts(); vid++) {
        //default
        m.vert_data(vid).uvw[0] = 0;
        //if not in the surface check the closest srf point and the distance to it
        if (!m.vert_is_on_srf(vid)) {
            m.vert_data(vid).uvw[0] = oct.closest_point(m.vert(vid)).dist(m.vert(vid));
            //check if its max distance
            if (m.vert_data(vid).uvw[0] > center_dist) {
                center_vid = vid;
                center_dist = m.vert_data(vid).uvw[0];
            }
        }
    }

    //sp build
    std::cout << std::endl << TXT_BOLDMAGENTA << "Sphere loading" << TXT_RESET << std::endl;
    DrawableTetmesh<> sp = get_sphere(m.vert(center_vid), center_dist);
    sp.poly_set_color(Color::PASTEL_YELLOW());
    gui.push(&sp, false);
    gui.push(new VolumeMeshControls<DrawableTetmesh<>>(&sp, &gui));

    std::cout << TXT_BOLDGREEN << "Raggio sfera: " << TXT_RESET << sp.bbox().delta_x() << std::endl;
    std::cout << TXT_BOLDGREEN << "Centro: " << TXT_RESET << center_vid << " (" << sp.centroid() << ")" << std::endl;
    std::cout << std::endl << std::endl;

// :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
// :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
// :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

    //TODO [X] avanzamento 25% freccia con tasto
    //TODO [X] definire distanza inattività del fronte
    //TODO [X] fronte vertici inattivi
    //TODO [X] fronte facce inattive
    //TODO [X] split su faccia con tasto
    //TODO [X] visualizzare info come vertici e facce inattive

    //TODO [ ] propagazione su facce interne con la sostituzione del poliedro

    //visualization
    bool show_target = true;
    bool inactive_visual = false;
    bool dir_visual = false;
    std::vector<DrawableArrow> dir_arrows;
    //vert movement & face split
    double mov_speed = 0.25 ;
    double eps_percent = 0.1;
    double eps_inactive = srf.edge_min_length() * eps_percent;
    std::set<uint> inactive_verts;
    std::set<uint> inactive_faces;

    //comandi tastiera
    gui.callback_key_pressed = [&](int key, int modifier) {

        //arrow visualization
        if(key == GLFW_KEY_V) {
            dir_visual = !dir_visual;

            if(dir_visual) {
                std::cout << TXT_BOLDMAGENTA << "Movement direction: ON" << TXT_RESET << std::endl;
                showArrows(sp, oct, dir_arrows, gui);
            } else {
                std::cout << TXT_BOLDMAGENTA << "Movement direction: OFF" << TXT_RESET << std::endl;
                deleteArrows(dir_arrows, gui);
            }
        }

        //inactive visualization
        if(key == GLFW_KEY_B) {
            std::cout << TXT_BOLDMAGENTA << "Visual inactive vert and polys" << TXT_RESET << std::endl;
            inactive_visual = !inactive_visual;

            if(inactive_visual)
                showInactive(sp, inactive_verts, inactive_faces);
            else
                hideInactive(sp, inactive_verts, inactive_faces);

            sp.updateGL();
        }

        //target visualization
        if(key == GLFW_KEY_L) {
            show_target = !show_target;
            m.show_mesh(show_target);
        }

        //vert movement
        if(key == GLFW_KEY_N) {
            std::cout << TXT_BOLDMAGENTA << "Vert advancement" << TXT_RESET << std::endl;

            //every vert on the surf is moved
            for(uint vid : sp.get_surface_verts()) {

                // if the vert is active
                if(DOES_NOT_CONTAIN(inactive_verts, vid)) {

                    // vert moved
                    sp.vert(vid) += sp.vert_data(vid).normal * oct.closest_point(sp.vert(vid)).dist(sp.vert(vid)) * mov_speed;


                    //if the distance from the target is under the eps the vert becomes inactive
                    if (oct.closest_point(sp.vert(vid)).dist(sp.vert(vid)) * mov_speed < eps_inactive) {
                        std::cout << TXT_BOLDBLUE << "Vert inactive: " << vid << std::endl;
                        inactive_verts.insert(vid);
                    }

                }

            }

            if(dir_visual)
                updateArrows(sp, oct, dir_arrows, gui);
            if(inactive_visual)
                showInactive(sp, inactive_verts, inactive_faces);
            sp.updateGL();
        }

        //poly split
        if(key == GLFW_KEY_M) {
            std::cout << TXT_BOLDMAGENTA << "Surface poly split" << TXT_RESET << std::endl;

            std::set<uint> poly_to_split;
            std::set<uint> poly_expansion;
            std::map<ipair, uint> vert_map;
            std::set<uint> poly_to_remove;

            //from every surface get the surface poly
            for(uint fid : sp.get_surface_faces()) {
                //if the face is active take the poly
                if(DOES_NOT_CONTAIN(inactive_faces, fid)) {
                    poly_to_split.insert(sp.adj_f2p(fid)[0]);
                    //for every edge of the poly
                    for(uint eid : sp.adj_p2e(sp.adj_f2p(fid)[0])) {
                        //save og edge as ordered pair
                        ipair og_edge = std::make_pair(sp.edge_vert_id(eid, 0), sp.edge_vert_id(eid, 1));
                        if (og_edge.first > og_edge.second)
                            std::swap(og_edge.first, og_edge.second);
                        //ad new vert in the middl of the edge and save it in the map with the og edge as key
                        uint new_vert = sp.vert_add((sp.vert(og_edge.first) + sp.vert(og_edge.second)) / 2);
                        vert_map.insert(std::make_pair(og_edge, new_vert));
                    }
                }
            }

            //for every poly to split
            for(uint pid : poly_to_split) {

                //for every poly edge
                for(uint eid : sp.adj_p2e(pid))
                    //check every adjacent poly
                    for(uint adj_pid : sp.adj_e2p(eid))
                        //if the poly is not on the surface
                        if (DOES_NOT_CONTAIN(poly_to_split, adj_pid))
                            //add the poly to the expansion set
                            poly_expansion.insert(adj_pid);

                //new polys following the template
                std::vector<uint> poly_verts(4);

                /// === upper tet ======================================================================================
                poly_verts[0] = query_map(vert_map, sp, pid, 0, 3);     /*mid 0-3*/
                poly_verts[1] = query_map(vert_map, sp, pid, 1, 3);     /*mid 1-3*/
                poly_verts[2] = query_map(vert_map, sp, pid, 2, 3);     /*mid 2-3*/
                poly_verts[3] = sp.poly_vert_id(pid, 3);                          /*vert 3*/
                sp.poly_add(poly_verts);                                           /*poly add*/

                /// === left tet =======================================================================================
                poly_verts[0] = sp.poly_vert_id(pid, 0);                          /*vert 0*/
                poly_verts[0] = query_map(vert_map, sp, pid, 0, 1);     /*mid 0-1*/
                poly_verts[0] = query_map(vert_map, sp, pid, 0, 2);     /*mid 0-2*/
                poly_verts[0] = query_map(vert_map, sp, pid, 0, 3);     /*mid 0-3*/
                sp.poly_add(poly_verts);                                           /*poly add*/

                /// === right tet ======================================================================================
                poly_verts[0] = query_map(vert_map, sp, pid, 0, 2);     /*mid 0-2*/
                poly_verts[1] = query_map(vert_map, sp, pid, 1, 2);     /*mid 1-2*/
                poly_verts[2] = sp.poly_vert_id(pid, 2);                          /*vert 2*/
                poly_verts[3] = query_map(vert_map, sp, pid, 2, 3);     /*mid 2-3*/
                sp.poly_add(poly_verts);                                           /*poly add*/

                /// === front tet ======================================================================================
                poly_verts[0] = query_map(vert_map, sp, pid, 0, 1);     /*mid 0-1*/
                poly_verts[1] = sp.poly_vert_id(pid, 1);                          /*vert 1*/
                poly_verts[2] = query_map(vert_map, sp, pid, 1, 2);     /*mid 1-2*/
                poly_verts[3] = query_map(vert_map, sp, pid, 1, 3);     /*mid 1-3*/
                sp.poly_add(poly_verts);                                           /*poly add*/

                /// === lower face tet =================================================================================
                poly_verts[0] = query_map(vert_map, sp, pid, 0, 1);     /*mid 0-1*/
                poly_verts[1] = query_map(vert_map, sp, pid, 0, 2);     /*mid 0-2*/
                poly_verts[2] = query_map(vert_map, sp, pid, 1, 2);     /*mid 1-2*/
                poly_verts[3] = query_map(vert_map, sp, pid, 1, 3);     /*mid 1-3*/
                sp.poly_add(poly_verts);                                           /*poly add*/

                /// === left face tet ==================================================================================
                poly_verts[0] = query_map(vert_map, sp, pid, 0, 1);     /*mid 0-1*/
                poly_verts[1] = query_map(vert_map, sp, pid, 1, 3);     /*mid 1-3*/
                poly_verts[2] = query_map(vert_map, sp, pid, 0, 3);     /*mid 0-3*/
                poly_verts[3] = query_map(vert_map, sp, pid, 0, 2);     /*mid 0-2*/
                sp.poly_add(poly_verts);                                           /*poly add*/

                /// === right face tet =================================================================================
                poly_verts[0] = query_map(vert_map, sp, pid, 1, 2);     /*mid 1-2*/
                poly_verts[1] = query_map(vert_map, sp, pid, 2, 3);     /*mid 2-3*/
                poly_verts[2] = query_map(vert_map, sp, pid, 1, 3);     /*mid 1-3*/
                poly_verts[3] = query_map(vert_map, sp, pid, 0, 2);     /*mid 0-2*/
                sp.poly_add(poly_verts);                                           /*poly add*/

                /// === back face tet ==================================================================================
                poly_verts[0] = query_map(vert_map, sp, pid, 0, 3);     /*mid 0-3*/
                poly_verts[1] = query_map(vert_map, sp, pid, 2, 3);     /*mid 2-3*/
                poly_verts[2] = query_map(vert_map, sp, pid, 0, 2);     /*mid 0-2*/
                poly_verts[3] = query_map(vert_map, sp, pid, 1, 3);     /*mid 1-3*/
                sp.poly_add(poly_verts);                                           /*poly add*/

            }

            // expand to the adj to the surf polys
            for(uint pid : poly_expansion) {
                
            }


            if(dir_visual)
                updateArrows(sp, oct, dir_arrows, gui);
            if(inactive_visual)
                showInactive(sp, inactive_verts, inactive_faces);
            sp.updateGL();
        }


        return false;
    };

    return gui.launch();
}

uint query_map(std::map<ipair, uint> &vert_map, Tetmesh<> &m, uint pid, uint off0, uint off1) {
    ipair query_pair;
    if(m.poly_vert_id(pid, off0) < m.poly_vert_id(pid, off1))
        query_pair = std::make_pair(m.poly_vert_id(pid, off0) , m.poly_vert_id(pid, off1));
    else
        query_pair = std::make_pair(m.poly_vert_id(pid, off1) , m.poly_vert_id(pid, off0));

    return vert_map.at(query_pair);
}