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

typedef struct {
    uint opp_vid;
    ipair og_edge;
} edge_to_flip;

void split_n_flip(DrawableTetmesh<> &m);
bool flip2to2(DrawableTetmesh<> &m, uint eid);
bool flip4to4(DrawableTetmesh<> &m, uint eid, uint vid0, uint vid1);

int main( /* int argc, char *argv[] */ ) {

    //UI
    GLcanvas gui;
    gui.show_side_bar = true;
    gui.side_bar_alpha = 0.5;

    /*
    1800 1035 -0.002324 -0.004054 -0.00237851 0.642835 0.435 -0.775047 0.631089 -0.0320702 0.000680942 -0.12701 -0.105865 0.986236 0.00162142 0.619007 0.768453 0.162205 0.00493968 0 0 0 1 1 0 0 -0 0 1 0 -0 0 0 1 -2.57134 0 0 0 1 1.02813 0 0 -0 0 1.78806 0 -0 0 0 -0.777805 -2 0 0 0 1
    */

    std::cout << std::endl << TXT_BOLDMAGENTA << "Model loading" << TXT_RESET << std::endl;

    //model vol
    DrawableTetmesh<> vol(KITTY);
    vol.show_mesh_points();
    gui.push(&vol);
    gui.push(new VolumeMeshControls<DrawableTetmesh<>>(&vol, &gui));

    //model srf
    Trimesh<> srf;
    export_surface(vol, srf);

    //octree build
    Octree oct;
    oct.build_from_mesh_polys(srf);

    //distance field & m center/scale
    uint center_vid = 0;
    double center_dist = -inf_double;
    for (uint vid = 0; vid < vol.num_verts(); vid++) {
        //default
        vol.vert_data(vid).uvw[0] = 0;
        //if not in the surface check the closest srf point and the distance to it
        if (!vol.vert_is_on_srf(vid)) {
            vol.vert_data(vid).uvw[0] = oct.closest_point(vol.vert(vid)).dist(vol.vert(vid));
            //check if its max distance
            if (vol.vert_data(vid).uvw[0] > center_dist) {
                center_vid = vid;
                center_dist = vol.vert_data(vid).uvw[0];
            }
        }
    }

    //m build
    std::cout << std::endl << TXT_BOLDMAGENTA << "Sphere loading" << TXT_RESET << std::endl;
    DrawableTetmesh<> m = get_sphere(vol.vert(center_vid), center_dist);
    m.poly_set_color(Color::PASTEL_YELLOW());
    gui.push(&m, false);
    gui.push(new VolumeMeshControls<DrawableTetmesh<>>(&m, &gui));

    std::cout << TXT_BOLDGREEN << "Raggio sfera: " << TXT_RESET << m.bbox().delta_x() << std::endl;
    std::cout << TXT_BOLDGREEN << "Centro: " << TXT_RESET << center_vid << " (" << m.centroid() << ")" << std::endl;
    std::cout << std::endl << std::endl;

// :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
// :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
// :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

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
                showArrows(m, oct, dir_arrows, gui);
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
                showInactive(m, inactive_verts, inactive_faces);
            else
                hideInactive(m, inactive_verts, inactive_faces);

            m.updateGL();
        }

        //target visualization
        if(key == GLFW_KEY_L) {
            show_target = !show_target;
            vol.show_mesh(show_target);
        }

        //vert movement
        if(key == GLFW_KEY_N) {
            std::cout << TXT_BOLDMAGENTA << "Vert advancement" << TXT_RESET << std::endl;

            //every vert on the surf is moved
            for(uint vid : m.get_surface_verts()) {

                // if the vert is active
                if(DOES_NOT_CONTAIN(inactive_verts, vid)) {

                    // vert moved
                    m.vert(vid) += m.vert_data(vid).normal * oct.closest_point(m.vert(vid)).dist(m.vert(vid)) * mov_speed;


                    //if the distance from the target is under the eps the vert becomes inactive
                    if (oct.closest_point(m.vert(vid)).dist(m.vert(vid)) * mov_speed < eps_inactive) {
                        std::cout << TXT_BOLDBLUE << "Vert inactive: " << vid << std::endl;
                        inactive_verts.insert(vid);
                    }

                }

            }

            if(dir_visual)
                updateArrows(m, oct, dir_arrows, gui);
            if(inactive_visual)
                showInactive(m, inactive_verts, inactive_faces);
            m.updateGL();
        }

        //poly split
        if(key == GLFW_KEY_M)
            split_n_flip(m);


        return false;
    };

    return gui.launch();
}

void split_n_flip(DrawableTetmesh<> &m) {
    std::cout << TXT_BOLDMAGENTA << "Surface poly split" << TXT_RESET << std::endl;

    uint offset = m.num_verts();
    std::cout << TXT_BOLDWHITE << "Num verts pre split: " << offset << TXT_RESET << std::endl;
    std::set<uint> edges_to_split;

    //from every surface get the surface poly
    for(uint fid : m.get_surface_faces()) {
        uint pid = m.adj_f2p(fid)[0];

        edges_to_split.insert(m.adj_p2e(pid).begin(), m.adj_p2e(pid).end());
    }

    //split parameters
    uint new_vid;
    ipair og_edge;
    std::map<ipair, uint> v_map;
    std::queue<edge_to_flip> edges_to_flip;
    //split all the edges in the set
    for(uint eid : edges_to_split) {
        og_edge = unique_pair(m.edge_vert_id(eid, 0), m.edge_vert_id(eid, 1));

        //check if the edges created will need to be flipped
        for(uint fid : m.adj_e2f(eid))
            if(m.face_vert_opposite_to(fid, eid) < offset) {
                edge_to_flip etf;
                etf.opp_vid = m.face_vert_opposite_to(fid, eid);
                etf.og_edge = og_edge;
                edges_to_flip.push(etf);
            }

        //split and map update
        new_vid = m.edge_split(eid);
        v_map.emplace(og_edge,new_vid);
    }

    //some info
    uint num_verts = m.num_verts();
    std::cout << TXT_BOLDWHITE << "Num verts after split: " << num_verts << TXT_RESET << std::endl;

    //flip every edge we need to
    std::map<ipair, uint> try_again;
    while(!edges_to_flip.empty()) {

        //retrieve the info
        edge_to_flip etf = edges_to_flip.front();
        uint vid = v_map.at(etf.og_edge);
        uint eid = m.edge_id(vid, etf.opp_vid);

        //if the edge isnt on the srf we do the flip 4-4
        if(!m.edge_is_on_srf(eid)) {

            //flip parameters
            uint vid0, vid1;
            ipair query_pair;

            //query for the parameter vids for the flip
            query_pair = unique_pair(etf.og_edge.first, etf.opp_vid);
            vid0 = v_map.count(query_pair) ? v_map.at(query_pair) : 0;
            query_pair = unique_pair(etf.og_edge.second, etf.opp_vid);
            vid1 = v_map.count(query_pair) ? v_map.at(query_pair) : 0;

            //if the vids are both found we can do the flip
            if(vid0 * vid1 != 0) {
                //flip
                std::cout << TXT_BOLDBLUE << "Flip 4-4 : edge " << m.edge_vert_id(eid, 0) << " - " << m.edge_vert_id(eid, 1) << std::endl;
                bool result = flip4to4(m, eid, vid0, vid1);
                std::cout << (result ? TXT_BOLDGREEN : TXT_BOLDRED) << "Result " << result << TXT_RESET << std::endl << std::endl;

                //if the flip is good and is a 'try again' the edge is removed
                if(result && try_again.count(unique_pair(vid, etf.opp_vid)) != 0) try_again.erase(unique_pair(vid, etf.opp_vid));
                //if we cant do the flip now we can do it in the future
                if(!result) {
                    if (try_again.count(unique_pair(vid, etf.opp_vid)) == 1) {
                        if (m.edge_valence(eid) != try_again.at(unique_pair(vid, etf.opp_vid))) {
                            edges_to_flip.push(etf);
                            try_again[unique_pair(vid, etf.opp_vid)] = m.edge_valence(eid);
                        } else {
                            std::cout << TXT_BOLDRED << "already tried to flip this and he had the same valence, we are in a mexican standoff" << TXT_RESET << std::endl;
                            break;
                        }
                    } else {
                        edges_to_flip.push(etf);
                        try_again.insert({unique_pair(vid, etf.opp_vid), m.edge_valence(eid)});
                    }
                }
            }

        //if not we do the flip 2-2
        } else {

            //flip
            std::cout << TXT_BOLDBLUE << "Flip 2-2 : edge " << m.edge_vert_id(eid, 0) << " - " << m.edge_vert_id(eid, 1) << std::endl;
            bool result = flip2to2(m, eid);
            std::cout << (result ? TXT_BOLDGREEN : TXT_BOLDRED) << "Result " << result << TXT_RESET << std::endl << std::endl;
            //if we cant do the flip now we can to id in the future
            if(!result) edges_to_flip.push(etf);

        }

        edges_to_flip.pop();
    }

    m.updateGL();
}

bool flip2to2(DrawableTetmesh<> &m, uint eid) {

    //edge must be on the surface and with only two polys adj
    if(!m.edge_is_on_srf(eid)) {
        std::cout << TXT_BOLDRED << "edge not on the srf" << TXT_RESET << std::endl;
        return false;
    }
    if(m.adj_e2p(eid).size() != 2) {
        std::cout << TXT_BOLDRED << "cluster size is not 2 -> edge " << m.edge_vert_id(eid, 0) << " - " << m.edge_vert_id(eid, 1) << " / size: " << m.adj_e2p(eid).size() << TXT_RESET << std::endl;
        //for(uint pid : m.adj_e2p(eid))
        //    std::cout << m.poly_vert_id(pid, 0) << " - " << m.poly_vert_id(pid, 1) << " - " << m.poly_vert_id(pid, 2) << " - " << m.poly_vert_id(pid, 3) << std::endl;
        return false;
    }

    uint pid_of = m.adj_e2p(eid)[0]; //need the opp faces
    uint pid_ov = m.adj_e2p(eid)[1]; //need the opp vert of the non shared face
    uint opp;

    //for every adj face to the edge
    for(uint fid : m.poly_e2f(pid_ov, eid))
        if(m.face_is_on_srf(fid))
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
        //std::cout << TXT_BOLDYELLOW << "Adding new tet w/ verts: " << tets[tet_id][0] << " - " << tets[tet_id][1] << " - " << tets[tet_id][2] << " - " << tets[tet_id][3] << TXT_RESET << std::endl;
        uint new_pid = m.poly_add({tets[tet_id][0],
                                   tets[tet_id][1],
                                   tets[tet_id][2],
                                   tets[tet_id][3]});
        m.update_p_quality(new_pid);
    }

    m.edge_remove(eid);
    return true;
}

bool flip4to4(DrawableTetmesh<> &m, uint eid, uint vid0, uint vid1) {
    std::vector<uint> cluster = m.adj_e2p(eid);
    if(cluster.size() != 4) {
        std::cout << TXT_BOLDRED << "cluster size is not 4 -> edge " << m.edge_vert_id(eid, 0) << " - " << m.edge_vert_id(eid, 1) << " / size: " << m.adj_e2p(eid).size() << TXT_RESET << std::endl;
        //for(uint pid : m.adj_e2p(eid))
        //    std::cout << m.poly_vert_id(pid, 0) << " - " << m.poly_vert_id(pid, 1) << " - " << m.poly_vert_id(pid, 2) << " - " << m.poly_vert_id(pid, 3) << std::endl;
        return false;
    }
    if(vid0*vid1 == 0) {
        std::cout << TXT_BOLDRED << "edge on the srf" << TXT_RESET << std::endl;
        return false;
    }

    uint tet_id = 0;
    uint tets[4][4];
    //for every face adj to vid0
    for(uint fid : m.adj_v2f(vid0)) {
        //if the face doesnt contain the edge to flip
        if (!m.face_contains_edge(fid, eid)) {
            //check for every tet in the cluster
            for (uint pid : cluster) {
                //if the face is from one of the tets of the cluster
                if (m.poly_contains_face(pid, fid)) {
                    //every tet if formed from the face + vid1
                    tets[tet_id][0] = m.face_vert_id(fid, 0);
                    tets[tet_id][1] = m.face_vert_id(fid, 1);
                    tets[tet_id][2] = m.face_vert_id(fid, 2);
                    tets[tet_id][3] = vid1;
                    //winding check
                    if(m.poly_face_is_CCW(pid,fid)) std::swap(tets[tet_id][0], tets[tet_id][1]);
                    tet_id++;
                }
            }
        }
    }
    assert(tet_id == 4);

    // add new elements to the mesh
    for(tet_id=0; tet_id < 4; tet_id++) {
        // std::cout << TXT_BOLDYELLOW << "Adding new tet w/ verts: " << tets[tet_id][0] << " - " << tets[tet_id][1] << " - " << tets[tet_id][2] << " - " << tets[tet_id][3] << TXT_RESET << std::endl;
        uint new_pid = m.poly_add({tets[tet_id][0],
                                   tets[tet_id][1],
                                   tets[tet_id][2],
                                   tets[tet_id][3]});
        m.update_p_quality(new_pid);
    }

    m.edge_remove(eid);
    return true;
}