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

bool flip2to2(Tetmesh<> &m, uint eid);
bool flip4to4(Tetmesh<> &m, uint eid, uint vid0, uint vid1);

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

            std::set<uint> edges_to_split;
            std::map<uint, ipair> vert_map;

            //from every surface get the surface poly
            for(uint fid : sp.get_surface_faces()) {
                uint pid = sp.adj_f2p(fid)[0];

                //if the face is active
                if(DOES_NOT_CONTAIN(inactive_faces, fid)) {
                    edges_to_split.insert(sp.adj_p2e(pid).begin(), sp.adj_p2e(pid).end());
                }
            }

            //split the edges
            for(uint eid : edges_to_split) {
                ipair og_edge = std::make_pair(sp.edge_vert_id(eid, 0), sp.edge_vert_id(eid, 1));
                uint new_vert = sp.edge_split(eid);
                vert_map.insert(std::make_pair(new_vert, og_edge));
            }

            //for every new vert
            for(auto map_elem : vert_map) {
                for(auto adj_vid : sp.adj_v2v(map_elem.first)) {
                    //if the adj vert isn't new && isn't from the original edge
                    if(vert_map.find(adj_vid) != vert_map.end() && adj_vid != map_elem.second.first && adj_vid != map_elem.second.second) {
                        sp.edge_flip(sp.edge_id(map_elem.first, adj_vid));
                        std::cout << TXT_BOLDBLUE << "Edge flip: " << sp.edge_id(map_elem.first, adj_vid) << std::endl;
                    }
                }
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

bool flip2to2(Tetmesh<> &m, uint eid) {
    //edge must be on the surface and with only two polys adj
    if(!m.edge_is_on_srf(eid)) return false;
    if(m.adj_e2p(eid).size() != 2) return false;

    uint pid_of = m.adj_e2p(eid).front(); //need the opp faces
    uint pid_ov = m.adj_e2p(eid).back(); //need the opp vert of the non shared face
    uint opp;

    //for every adj face to the edge
    for(uint fid : m.adj_e2f(eid))
        //if its not the face shared between the polys
        if(!m.poly_shared_face(pid_of, pid_ov) && m.poly_contains_face(fid, pid_ov))
            opp = m.face_vert_opposite_to(fid, eid);

    // construct all new elements
    uint tets[2][4];
    uint tet_id = 0;
    for(uint fid : m.poly_faces_opposite_to(pid_of,eid)) {
        tets[tet_id][0] = m.face_vert_id(fid, 0),
        tets[tet_id][1] = m.face_vert_id(fid, 1),
        tets[tet_id][2] = m.face_vert_id(fid, 2),
        tets[tet_id][3] = opp;
        if(m.poly_face_is_CCW(pid_of,fid)) std::swap(tets[tet_id][0], tets[tet_id][1]);
        tet_id++;
    }
    assert(tet_id == 2);

    // add new elements to the mesh
    for(tet_id=0; tet_id < 2; tet_id++) {
        uint new_pid = m.poly_add({tets[tet_id][0],
                                       tets[tet_id][1],
                                       tets[tet_id][2],
                                       tets[tet_id][3]});
        m.update_p_quality(new_pid);
    }

    m.edge_remove(eid);
    return true;
}

bool flip4to4(Tetmesh<> &m, uint eid, uint vid0, uint vid1) {
    std::vector<uint> cluster = m.adj_e2p(eid);
    if(cluster.size() == 4) return false;

    uint tet_id = 0;
    uint tets[4][4];
    //for every face adj to vid0
    for(uint fid : m.adj_v2f(vid0)) {
        //if the face doesnt contain the edge to flip
        if (!m.face_contains_edge(fid, eid)) {
            //check for every tet in the cluster
            for (uint pid: cluster) {
                //if the face is from one of the tets of the cluster
                if (m.poly_contains_face(pid, fid)) {
                    //every tet if formed from the face + vid1
                    tets[tet_id][0] = m.face_vert_id(fid, 0),
                    tets[tet_id][1] = m.face_vert_id(fid, 1),
                    tets[tet_id][2] = m.face_vert_id(fid, 2),
                    tets[tet_id][3] = vid1;
                    //winding check
                    if(m.poly_face_is_CCW(pid,fid)) std::swap(tets[tet_id][0], tets[tet_id][1]);
                    tet_id++;
                }
            }
        }
    }
    assert(tet_id==4);

    // add new elements to the mesh
    for(tet_id=0; tet_id < 4; tet_id++) {
        uint new_pid = m.poly_add({tets[tet_id][0],
                                   tets[tet_id][1],
                                   tets[tet_id][2],
                                   tets[tet_id][3]});
        m.update_p_quality(new_pid);
    }

    m.edge_remove(eid);
    return true;
}