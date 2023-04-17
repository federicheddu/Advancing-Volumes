#include <cinolib/gl/glcanvas.h>
#include <cinolib/meshes/meshes.h>
#include <cinolib/gl/volume_mesh_controls.h>
#include <cinolib/bfs.h>
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

//debug prints
#define DEBUG_PRINT 0

//struct to store the target model and parameters
typedef struct Target {
    //structures
    DrawableTetmesh<> vol;
    DrawableTrimesh<> srf;
    Octree oct;
    //parameters
    double mov_speed = 0.25;
    double eps_percent = 0.1;
    double eps_inactive;
} Target;

//struct to query the edges to flip after the split
typedef struct edge_to_flip {
    uint opp_vid = 0;
    ipair og_edge = {0, 0};
} edge_to_flip;

//init
Target get_target();
void get_center(Target &target, uint &center_vid, double &center_dist);
//topological operations
void split_n_flip(DrawableTetmesh<> &m);
bool flip2to2(DrawableTetmesh<> &m, uint eid);
bool flip4to4(DrawableTetmesh<> &m, uint eid, uint vid0, uint vid1);
//movement operations
void expand(DrawableTetmesh<> &m, Target &target, std::vector<bool> &active_mask, std::vector<std::vector<uint>> &active_fronts);
void update_fronts(DrawableTetmesh<> &m, std::vector<bool> &active_mask, std::vector<std::vector<uint>> &active_fronts);

int main( /* int argc, char *argv[] */ ) {

    //UI
    GLcanvas gui;
    gui.show_side_bar = true;
    gui.side_bar_alpha = 0.5;

    //load & push the target model
    Target target = get_target();
    gui.push(&target.vol);
    gui.push(new VolumeMeshControls<DrawableTetmesh<>>(&target.vol, &gui));

    //distance field & m center/scale
    uint center_vid = 0;
    double center_dist = -inf_double;
    get_center(target, center_vid, center_dist);

    //m build
    std::cout << std::endl << TXT_BOLDMAGENTA << "Sphere loading" << TXT_RESET << std::endl;
    DrawableTetmesh<> m = get_sphere(target.vol.vert(center_vid), center_dist / 2);
    m.poly_set_color(Color::PASTEL_YELLOW());
    gui.push(&m, false);
    gui.push(new VolumeMeshControls<DrawableTetmesh<>>(&m, &gui));

    std::cout << std::endl << std::endl;

// :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
// :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
// :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

    //visualization
    bool show_target = true;
    bool inactive_visual = false;
    bool dir_visual = false;
    std::vector<DrawableArrow> dir_arrows;
    //vert movement parameters
    std::vector<bool> active_mask(m.num_verts());
    std::vector<std::vector<uint>> active_fronts;
    active_fronts.emplace_back(m.get_surface_verts());

    //comandi tastiera
    gui.callback_key_pressed = [&](int key, int modifier) {

        //arrow visualization
        if (key == GLFW_KEY_V)
            dir_visual = !dir_visual;

        //Target visualization
        if (key == GLFW_KEY_L)
            target.vol.show_mesh(show_target = !show_target);

        //vert movement
        if (key == GLFW_KEY_N)
            expand(m, target, active_mask, active_fronts);

        //poly split
        if (key == GLFW_KEY_M)
            split_n_flip(m);

        //visualization of arrows
        dir_visual ? updateArrows(m, target.oct, dir_arrows, active_fronts, gui) : deleteArrows(dir_arrows, gui);

        return false;
    };

    return gui.launch();
}

Target get_target() {
    Target target;

    std::cout << std::endl << TXT_BOLDMAGENTA << "Target loading" << TXT_RESET << std::endl;

    //model vol
    target.vol = DrawableTetmesh<>(BUNNY);
    target.vol.show_mesh_points();

    //model srf
    export_surface(target.vol, target.srf);

    //octree build
    target.oct.build_from_mesh_polys(target.srf);

    //inactive threshold
    target.eps_inactive = target.srf.edge_min_length() * target.eps_percent;

    return target;
}

void get_center(Target &target, uint &center_vid, double &center_dist) {

    for (uint vid = 0; vid < target.vol.num_verts(); vid++) {
        //default
        target.vol.vert_data(vid).uvw[0] = 0;
        //if not in the surface check the closest srf point and the distance to it
        if (!target.vol.vert_is_on_srf(vid)) {
            target.vol.vert_data(vid).uvw[0] = target.oct.closest_point(target.vol.vert(vid)).dist(target.vol.vert(vid));
            //check if its max distance
            if (target.vol.vert_data(vid).uvw[0] > center_dist) {
                center_vid = vid;
                center_dist = target.vol.vert_data(vid).uvw[0];
            }
        }
    }

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

    //* flip every edge we need to
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
                //if we cant do the flip now we can do it in the future (hopefully)
                if(!result) {
                    if (try_again.count(unique_pair(vid, etf.opp_vid)) == 1) {
                        if (m.edge_valence(eid) != try_again.at(unique_pair(vid, etf.opp_vid))) {
                            edges_to_flip.push(etf);
                            try_again[unique_pair(vid, etf.opp_vid)] = m.edge_valence(eid);
                        } else {
                            std::cout << TXT_BOLDRED << "already tried to flip this and he had the same valence, we are in a mexican standoff" << TXT_RESET << std::endl;
                            for(auto p : m.adj_e2p(eid))
                                m.poly_data(p).color = Color::RED();
                            //break;
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

            //if the flip is good and is a 'try again' the edge is removed
            if(result && try_again.count(unique_pair(vid, etf.opp_vid)) != 0) try_again.erase(unique_pair(vid, etf.opp_vid));
            //if we cant do the flip now we can do it in the future (hopefully)
            if(!result) {
                if (try_again.count(unique_pair(vid, etf.opp_vid)) == 1) {
                    if (m.edge_valence(eid) != try_again.at(unique_pair(vid, etf.opp_vid))) {
                        edges_to_flip.push(etf);
                        try_again[unique_pair(vid, etf.opp_vid)] = m.edge_valence(eid);
                    } else {
                        std::cout << TXT_BOLDRED << "already tried to flip this and he had the same valence, we are in a mexican standoff" << TXT_RESET << std::endl << std::endl;
                        for(auto p : m.adj_e2p(eid))
                            m.poly_data(p).color = Color::RED();
                        //break;
                    }
                } else {
                    edges_to_flip.push(etf);
                    try_again.insert({unique_pair(vid, etf.opp_vid), m.edge_valence(eid)});
                }
            }

        }

        edges_to_flip.pop();
    }
    /**/

    m.update_normals();
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

void expand(DrawableTetmesh<> &m, Target &target, std::vector<bool> &active_mask, std::vector<std::vector<uint>> &active_fronts) {

    //start
    std::cout << TXT_CYAN << "Expanding the model... ";

    //for every active front
    for(std::vector<uint> &front : active_fronts) {
        //for every vid in the front
        for (uint vid : front) {
            //move the vert
            m.vert(vid) += m.vert_data(vid).normal * target.oct.closest_point(m.vert(vid)).dist(m.vert(vid)) * target.mov_speed;
            //update the mask
            if(target.oct.closest_point(m.vert(vid)).dist(m.vert(vid)) < target.eps_inactive)
                active_mask[vid] = true;
        }
    }

    //job done
    std::cout << "DONE" << TXT_RESET << std::endl;

    //update che fronts
    update_fronts(m, active_mask, active_fronts);

    //update the model
    m.update_normals();
    m.updateGL();
}

void update_fronts(DrawableTetmesh<> &m, std::vector<bool> &active_mask, std::vector<std::vector<uint>> &active_fronts) {

    std::cout << TXT_CYAN << "Updating the fronts... ";

    //parameters
    uint active_counter = 0;
    std::vector<std::vector<uint>> new_fronts;
    std::unordered_set<uint> dect_front;

    //for every front active front
    for(std::vector<uint> &front : active_fronts) {

        uint query_front = 0;                       //index to query the og front and check for new fronts
        std::unordered_set<uint> visited_front;     //need to compare with the og

        //find an idx in the front outside the visited ones
        // -> if query_front is already visited or is inactive (and we are not out-of-bounds) increase query_front
        while(query_front < front.size() && (CONTAINS(visited_front, front.at(query_front)) || active_mask.at(front.at(query_front)))) query_front++;

        //check for every new front inside the old one (could be no one)
        while(query_front < front.size()) {

            //get the front
            bfs_srf_only(m, front.at(query_front), active_mask, dect_front);
            visited_front.insert(dect_front.begin(), dect_front.end());

            //add the front to the new vec
            std::vector<uint> new_front(dect_front.begin(), dect_front.end());
            new_fronts.emplace_back(new_front);
            active_counter += new_front.size();

            //find the next idx
            while(query_front < front.size() && (CONTAINS(visited_front, front.at(query_front)) || active_mask.at(front.at(query_front)))) query_front++;

        }
    }

    //give back the new active fronts
    active_fronts = new_fronts;

    std::cout << "DONE - active verts left: " << active_counter << TXT_RESET << std::endl;
}