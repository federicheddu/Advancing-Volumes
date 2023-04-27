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

//struct to store the target model and parameters
typedef struct data {
    //structures
    DrawableTetmesh<> vol;
    DrawableTrimesh<> srf;
    Octree oct;
    //model (sphere)
    DrawableTetmesh<> m;
    //parameters
    double mov_speed = 0.25;
    double eps_percent = 0.1;
    double eps_inactive = 0.01;
    //fronts_active
    std::vector<bool> active_mask;
    std::vector<uint> fronts_active;
    std::vector<uint> fronts_bounds;
} Data;

//struct to query the edges to flip after the split
typedef struct edge_to_flip {
    uint opp_vid = 0;
    ipair og_edge = {0, 0};
} edge_to_flip;

//setup of the env
Data setup(const char *path);
//topological operations
void split_n_flip(Data &d);
bool flip2to2(DrawableTetmesh<> &m, uint eid);
bool flip4to4(DrawableTetmesh<> &m, uint eid, uint vid0, uint vid1);
//movement operations
void expand(Data &d);
void update_fronts(Data &d);



int main( /* int argc, char *argv[] */ ) {

    //UI
    GLcanvas gui;
    gui.show_side_bar = true;
    gui.side_bar_alpha = 0.5;

    //load the data
    char path[] = BUNNY;
    Data data = setup(path);

    //gui push
    gui.push(&data.m, false);
    gui.push(new VolumeMeshControls<DrawableTetmesh<>>(&data.m, &gui));
    gui.push(&data.vol, true);
    gui.push(new VolumeMeshControls<DrawableTetmesh<>>(&data.vol, &gui));

    //visualization
    bool show_target = true;
    bool show_fronts = false;
    bool show_arrows = false;
    std::vector<DrawableArrow> dir_arrows;
    //vert movement parameters
    data.active_mask.resize(data.m.num_verts());
    data.fronts_active = data.m.get_surface_verts();
    data.fronts_bounds.emplace_back(data.m.num_srf_verts());
    //undo object
    Data undo_data = data;

    //keyboard commands
    gui.callback_key_pressed = [&](int key, int modifier) {

        bool handled = true;

        switch (key) {
            //arrow visualization
            case GLFW_KEY_V: {
                show_arrows = !show_arrows;
                if (show_arrows) updateArrows(data.m, data.oct, dir_arrows, data.fronts_active, gui);
                else deleteArrows(dir_arrows, gui);
                break;
            }

            //fronts visualization
            case GLFW_KEY_B: {
                show_fronts = !show_fronts;
                if (show_fronts) showFronts(data.m, data.active_mask);
                else deleteFronts(data.m);
                break;
            }

            //target visualization
            case GLFW_KEY_L: {
                data.vol.show_mesh(show_target = !show_target);
                break;
            }

            //vert movement
            case GLFW_KEY_N: {
                std::cout << std::endl <<TXT_BOLDMAGENTA << "Model expansion" << TXT_RESET << std::endl;

                //update undo
                undo_data = data;
                //move the verts
                expand(data);
                //UI
                show_arrows ? updateArrows(data.m, data.oct, dir_arrows, data.fronts_active, gui) : deleteArrows(dir_arrows, gui);
                show_fronts ? showFronts(data.m, data.active_mask) : deleteFronts(data.m);
                break;
            }

            //poly split
            case GLFW_KEY_M: {
                std::cout << std::endl << TXT_BOLDMAGENTA << "Poly split" << TXT_RESET << std::endl;

                //update undo
                undo_data = data;
                //split and flip polys in the surface
                split_n_flip(data);
                //UI
                show_arrows ? updateArrows(data.m, data.oct, dir_arrows, data.fronts_active, gui) : deleteArrows(dir_arrows, gui);
                show_fronts ? showFronts(data.m, data.active_mask) : deleteFronts(data.m);
                break;
            }

            //undo
            case GLFW_KEY_K: {
                std::cout << std::endl << TXT_BOLDYELLOW << "UNDO" << TXT_RESET << std::endl;

                //pop of all
                gui.pop(&data.m);

                //data recover
                data.m = undo_data.m;
                data.fronts_active = undo_data.fronts_active;
                data.fronts_bounds = undo_data.fronts_bounds;
                data.active_mask = undo_data.active_mask;

                //re-push on gui
                gui.push(&data.m, false);
                data.m.updateGL();

                //UI
                show_arrows ? updateArrows(data.m, data.oct, dir_arrows, data.fronts_active, gui) : deleteArrows(dir_arrows, gui);
                show_fronts ? showFronts(data.m, data.active_mask) : deleteFronts(data.m);
                break;
            }

            default:
                handled = false;
        }

        return handled;

    };

    return gui.launch();
}



//setup the env with model, target, oct etc...
Data setup(const char *path) {
    Data data;

    std::cout << std::endl << TXT_BOLDMAGENTA << "Data loading... " << std::endl;

    //model vol
    data.vol = DrawableTetmesh<>(path);
    data.vol.show_mesh_points();

    //model srf
    export_surface(data.vol, data.srf);

    //octree build
    data.oct.build_from_mesh_polys(data.srf);

    //inactive threshold
    data.eps_inactive = data.srf.edge_min_length() * data.eps_percent;

    //get info for placing the sphere
    uint center_vid = 0;
    double center_dist = -inf_double;
    //for every vid in the vol
    for (uint vid = 0; vid < data.vol.num_verts(); vid++) {
        //default
        data.vol.vert_data(vid).uvw[0] = 0;
        //if not in the surface check the closest srf point and the distance to it
        if (!data.vol.vert_is_on_srf(vid)) {
            data.vol.vert_data(vid).uvw[0] = data.oct.closest_point(data.vol.vert(vid)).dist(data.vol.vert(vid));
            //check if its max distance
            if (data.vol.vert_data(vid).uvw[0] > center_dist) {
                center_vid = vid;
                center_dist = data.vol.vert_data(vid).uvw[0];
            }
        }
    }

    //sphere build
    data.m = get_sphere(data.vol.vert(center_vid), center_dist / 2);
    data.m.poly_set_color(Color::PASTEL_YELLOW());

    std::cout << "DONE" << TXT_RESET << std::endl;

    return data;
}

//split all the edges in the active front and flip who needs to be flipped
void split_n_flip(Data &d) {

    //start of split
    std::cout << TXT_CYAN << "Splitting the polys... ";

    //data that we need for later
    uint offset = d.m.num_verts();

    //search of the edges to split inside the front
    std::set<uint> edges_to_split;
    //for every vid in active fronts_active
    for(uint vid : d.fronts_active)
        //for every adj face
        for(uint fid : d.m.adj_v2f(vid))
            //if is a surface face from the active front (check if the adj vids of the face are also in the front)
            if(d.m.face_is_on_srf(fid)
            && CONTAINS_VEC(d.fronts_active, d.m.face_v2v(fid, vid)[0])
            && CONTAINS_VEC(d.fronts_active, d.m.face_v2v(fid, vid)[1]))
                //add every edge of the active poly to the split list
                edges_to_split.insert(d.m.adj_p2e(d.m.adj_f2p(fid)[0]).begin(), d.m.adj_p2e(d.m.adj_f2p(fid)[0]).end());

    //split parameters
    uint new_vid;
    ipair og_edge;
    std::map<ipair, uint> v_map;
    std::queue<edge_to_flip> edges_to_flip;
    //split all the edges in the set
    for(uint eid : edges_to_split) {
        og_edge = unique_pair(d.m.edge_vert_id(eid, 0), d.m.edge_vert_id(eid, 1));

        //check if the edges created will need to be flipped
        for(uint fid : d.m.adj_e2f(eid))
            if(d.m.face_vert_opposite_to(fid, eid) < offset) {
                edge_to_flip etf;
                etf.opp_vid = d.m.face_vert_opposite_to(fid, eid);
                etf.og_edge = og_edge;
                edges_to_flip.push(etf);
            }

        //split and map update
        new_vid = d.m.edge_split(eid);
        v_map.emplace(og_edge,new_vid);
    }

    //end of split
    std::cout << "DONE" << TXT_RESET << std::endl;

    //start of flipping
    std::cout << TXT_CYAN << "Flipping the edges... ";

    //new verts size & cluster update
    uint num_verts = d.m.num_verts();
    d.active_mask.resize(num_verts);

    //* flip every edge we need to
    std::map<ipair, uint> try_again;
    while(!edges_to_flip.empty()) {

        //retrieve the info
        edge_to_flip etf = edges_to_flip.front();
        uint vid = v_map.at(etf.og_edge);
        uint eid = d.m.edge_id(vid, etf.opp_vid);

        //if the edge isnt on the srf we do the flip 4-4
        if(!d.m.edge_is_on_srf(eid)) {

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
                bool result = flip4to4(d.m, eid, vid0, vid1);

                //if the flip is good and is a 'try again' the edge is removed
                if(result && try_again.count(unique_pair(vid, etf.opp_vid)) != 0)
                    try_again.erase(unique_pair(vid, etf.opp_vid));

                //if we cant do the flip now we can do it in the future (hopefully)
                if(!result) {
                    if (try_again.count(unique_pair(vid, etf.opp_vid)) == 1) {
                        if (d.m.edge_valence(eid) != try_again.at(unique_pair(vid, etf.opp_vid))) {
                            edges_to_flip.push(etf);
                            try_again[unique_pair(vid, etf.opp_vid)] = d.m.edge_valence(eid);
                        }
                    } else {
                        edges_to_flip.push(etf);
                        try_again.insert({unique_pair(vid, etf.opp_vid), d.m.edge_valence(eid)});
                    }
                }

            }

        //if not we do the flip 2-2 (we need to check if the edge is from an active face)
        } else if(!d.active_mask[etf.opp_vid]) {

            //flip
            bool result = flip2to2(d.m, eid);

            //if the flip is good and is a 'try again' the edge is removed
            if(result && try_again.count(unique_pair(vid, etf.opp_vid)) != 0) try_again.erase(unique_pair(vid, etf.opp_vid));
            //if we cant do the flip now we can do it in the future (hopefully)
            if(!result) {
                if (try_again.count(unique_pair(vid, etf.opp_vid)) == 1) {
                    if (d.m.edge_valence(eid) != try_again.at(unique_pair(vid, etf.opp_vid))) {
                        edges_to_flip.push(etf);
                        try_again[unique_pair(vid, etf.opp_vid)] = d.m.edge_valence(eid);
                    }
                } else {
                    edges_to_flip.push(etf);
                    try_again.insert({unique_pair(vid, etf.opp_vid), d.m.edge_valence(eid)});
                }
            }

        }

        edges_to_flip.pop();
    }
    /**/

    //end of flipping
    std::cout << "DONE" << TXT_RESET << std::endl;

    //update the fronts
    update_fronts(d);

    d.m.update_normals();
    d.m.updateGL();
}

//edge flip 2-2 (must: surf & srf faces coplanar)
bool flip2to2(DrawableTetmesh<> &m, uint eid) {

    //edge must be on the surface and with only two polys adj
    if(!m.edge_is_on_srf(eid)) {
        //std::cout << TXT_BOLDRED << "edge not on the srf" << TXT_RESET << std::endl;
        return false;
    }
    if(m.adj_e2p(eid).size() != 2) {
        //std::cout << TXT_BOLDRED << "cluster size is not 2 -> edge " << m.edge_vert_id(eid, 0) << " - " << m.edge_vert_id(eid, 1) << " / size: " << m.adj_e2p(eid).size() << TXT_RESET << std::endl;
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
        uint new_pid = m.poly_add({tets[tet_id][0],
                                   tets[tet_id][1],
                                   tets[tet_id][2],
                                   tets[tet_id][3]});
        m.update_p_quality(new_pid);
    }

    m.edge_remove(eid);
    return true;
}

//edge flip 4-4
bool flip4to4(DrawableTetmesh<> &m, uint eid, uint vid0, uint vid1) {
    std::vector<uint> cluster = m.adj_e2p(eid);
    if(cluster.size() != 4) {
        //std::cout << TXT_BOLDRED << "cluster size is not 4 -> edge " << m.edge_vert_id(eid, 0) << " - " << m.edge_vert_id(eid, 1) << " / size: " << m.adj_e2p(eid).size() << TXT_RESET << std::endl;
        return false;
    }
    if(vid0*vid1 == 0) {
        //std::cout << TXT_BOLDRED << "edge on the srf" << TXT_RESET << std::endl;
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
        uint new_pid = m.poly_add({tets[tet_id][0],
                                   tets[tet_id][1],
                                   tets[tet_id][2],
                                   tets[tet_id][3]});
        m.update_p_quality(new_pid);
    }

    m.edge_remove(eid);
    return true;
}

//move the verts toward the target
void expand(Data &d) {

    //start
    std::cout << TXT_CYAN << "Expanding the model... ";

    //for every active front
    for(uint vid : d.fronts_active) {
        //get info
        vec3d og_pos = d.m.vert(vid);
        double dist = d.oct.closest_point(d.m.vert(vid)).dist(d.m.vert(vid));
        //move the vert
        d.m.vert(vid) += d.m.vert_data(vid).normal * dist * d.mov_speed;

        //check intersection
        uint iter_counter = 0;
        std::unordered_set<uint> intersect_ids;
        bool check_intersection;

        //check edge intersection
        do {
            //param default
            check_intersection = false;

            //check the umbrella
            for (uint fid: d.m.vert_adj_srf_faces(vid)) {
                vec3d verts[] = {d.m.face_verts(fid)[0], d.m.face_verts(fid)[1], d.m.face_verts(fid)[2]};
                check_intersection = d.oct.intersects_triangle(verts, false, intersect_ids) || check_intersection;
            }

            //go back by half distance
            d.m.vert(vid) -= d.m.vert_data(vid).normal * (dist/2) * d.mov_speed;

            //counter increase
            iter_counter++;

        } while(check_intersection && iter_counter < 5);

        //if edge still outside get back the vid
        if(check_intersection)
            d.m.vert(vid) = og_pos;

        //update the mask
        if(!check_intersection && d.oct.closest_point(d.m.vert(vid)).dist(d.m.vert(vid)) < d.eps_inactive)
            d.active_mask[vid] = true;
    }

    //job done
    std::cout << "DONE" << TXT_RESET << std::endl;

    //update che fronts_active
    update_fronts(d);

    //update the model
    d.m.update_normals();
    d.m.updateGL();
}

/*
//move the verts toward the target
void parallel_expand(Data &d) {

    //start
    std::cout << TXT_CYAN << "Expanding the model... ";

    PARALLEL_FOR(d.fronts_bounds.begin(), d.fronts_bounds )

    //for every active front
    for(uint vid : d.fronts_active) {
        //move the vert
        d.m.vert(vid) += d.m.vert_data(vid).normal * d.oct.closest_point(d.m.vert(vid)).dist(d.m.vert(vid)) * d.mov_speed;
        //update the mask
        if(d.oct.closest_point(d.m.vert(vid)).dist(d.m.vert(vid)) < d.eps_inactive)
            d.active_mask[vid] = true;
    }

    //job done
    std::cout << "DONE" << TXT_RESET << std::endl;

    //update che fronts_active
    update_fronts(d);

    //update the model
    d.m.update_normals();
    d.m.updateGL();
}
*/

//update the front (must be called after every variation of the model)
void update_fronts(Data &d) {

    std::cout << TXT_CYAN << "Updating the fronts... ";

    //parameters
    uint active_counter = 0;
    std::vector<uint> new_fronts;
    std::vector<uint> new_bounds;
    std::unordered_set<uint> dect_front;

    //worst case scenario
    new_fronts.reserve(d.fronts_active.size() + d.m.get_surface_edges().size());

    //index to query the og front and check for new fronts_active
    uint query_front = 0;

    //find an idx in the front outside the visited ones
    // -> if query_front is already visited or is inactive (and we are not out-of-bounds) increase query_front
    while (query_front < d.fronts_active.size() && (CONTAINS_VEC(new_fronts, d.fronts_active[query_front]) || d.active_mask[d.fronts_active[query_front]]))
        query_front++;

    //check for every new front inside the old one (could be no one)
    while (query_front < d.fronts_active.size()) {

        //get the front
        bfs_srf_only(d.m, d.fronts_active[query_front], d.active_mask, dect_front);

        //add the front to the new vec
        new_fronts.insert(new_fronts.end(), dect_front.begin(), dect_front.end());
        new_bounds.emplace_back(new_fronts.size());

        //find the next idx
        while (query_front < d.fronts_active.size() && (CONTAINS_VEC(new_fronts, d.fronts_active[query_front]) || d.active_mask[d.fronts_active[query_front]]))
            query_front++;
    }

    //resize of the front vector
    new_fronts.shrink_to_fit();

    //give back the new active front and bounds
    d.fronts_active = new_fronts;
    d.fronts_bounds = new_bounds;

    std::cout << "DONE - F: " << d.fronts_bounds.size() << " V: " << d.fronts_active.size() << TXT_RESET << std::endl;
}