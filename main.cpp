#include <cinolib/gl/glcanvas.h>
#include <cinolib/meshes/meshes.h>
#include <cinolib/gl/volume_mesh_controls.h>
#include <cinolib/gl/surface_mesh_controls.h>
#include <cinolib/find_intersections.h>
#include <cinolib/smoother.h>
#include <cinolib/predicates.h>
#include <cinolib/ARAP.h>

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
    DrawableTrimesh<> m_srf;
    std::unordered_map<uint,uint> m2srf_vmap;
    std::unordered_map<uint,uint> srf2m_vmap;
    //parameters
    double mov_speed = 0.25;
    double eps_percent = 0.1;
    double eps_inactive = 0.01;
    double edge_threshold = 0.01;
    //fronts_active
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
std::set<uint> search_split(Data &d, bool selective);
void split(Data &d, std::set<uint> &edges_to_split, std::map<ipair, uint> &v_map, std::queue<edge_to_flip> &edges_to_flip);
std::set<uint> search_split_int(Data &d);
void split_int(Data &d, std::set<uint> &edges_to_split);
void flip(Data &d, std::map<ipair, uint> &v_map, std::queue<edge_to_flip> &edges_to_flip);
void split_n_flip(Data &d, bool selective = false);
bool flip2to2(DrawableTetmesh<> &m, uint eid);
bool flip4to4(DrawableTetmesh<> &m, uint eid, uint vid0, uint vid1);
//movement operations
void expand(Data &d, bool refine = false);
void smooth(Data &d, int n_iter = 2);
void custom_mesh_smoother(AbstractPolygonMesh<> &m, const AbstractPolygonMesh<> &target, const SmootherOptions &opt = SmootherOptions());
//check operations
bool check_intersection(Data &d, uint vid);
bool check_volume(Data &d, uint vid);
//front management
void update_fronts(Data &d);
void front_from_seed(Data &d, uint seed, std::unordered_set<uint> &front);


int main( /* int argc, char *argv[] */ ) {

    //UI
    GLcanvas gui(1080, 720);
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
    bool show_target_matte = false;
    bool show_fronts = false;
    bool show_arrows = false;
    bool show_vol_color = false;
    std::vector<DrawableArrow> dir_arrows;
    //vert movement parameters
    data.fronts_active = data.m.get_surface_verts();
    data.fronts_bounds.emplace_back(data.m.num_srf_verts());
    //undo object
    Data reset_data = data;
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
                if (show_fronts) showFronts(data.m);
                else deleteFronts(data.m);
                data.m.updateGL();
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
                show_fronts ? showFronts(data.m) : deleteFronts(data.m);

                //update model
                data.m.update_normals();
                data.m.updateGL();
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
                show_fronts ? showFronts(data.m) : deleteFronts(data.m);

                //update model
                data.m.update_normals();
                data.m.updateGL();
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

                //UI
                show_arrows ? updateArrows(data.m, data.oct, dir_arrows, data.fronts_active, gui) : deleteArrows(dir_arrows, gui);
                show_fronts ? showFronts(data.m) : deleteFronts(data.m);

                //re-push on gui
                gui.push(&data.m, false);
                data.m.updateGL();
                break;
            }

            //show full target
            case GLFW_KEY_J: {
                show_target_matte = !show_target_matte;
                if(show_target_matte) data.vol.show_mesh_flat();
                else data.vol.show_mesh_points();
                break;
            }

            //vert movement with edge split by length
            case GLFW_KEY_G: {
                std::cout << std::endl <<TXT_BOLDMAGENTA << "Model expansion" << TXT_RESET << std::endl;

                //update undo
                undo_data = data;

                //move the verts

                expand(data, true);
                //UI
                show_arrows ? updateArrows(data.m, data.oct, dir_arrows, data.fronts_active, gui) : deleteArrows(dir_arrows, gui);
                show_fronts ? showFronts(data.m) : deleteFronts(data.m);

                //update model
                data.m.update_normals();
                data.m.updateGL();

                break;
            }

            //poly split
            case GLFW_KEY_H: {
                std::cout << std::endl << TXT_BOLDMAGENTA << "Poly split" << TXT_RESET << std::endl;

                //update undo
                undo_data = data;

                //split and flip polys in the surface
                split_n_flip(data, true);

                //UI
                show_arrows ? updateArrows(data.m, data.oct, dir_arrows, data.fronts_active, gui) : deleteArrows(dir_arrows, gui);
                show_fronts ? showFronts(data.m) : deleteFronts(data.m);

                //update model
                data.m.update_normals();
                data.m.updateGL();
                break;
            }

            //reset
            case GLFW_KEY_P: {
                std::cout << std::endl << TXT_BOLDYELLOW << "RESET" << TXT_RESET << std::endl;

                //pop of all
                gui.pop(&data.m);

                //data recover
                data.m = reset_data.m;
                data.fronts_active = reset_data.fronts_active;
                data.fronts_bounds = reset_data.fronts_bounds;

                //UI
                show_arrows ? updateArrows(data.m, data.oct, dir_arrows, data.fronts_active, gui) : deleteArrows(dir_arrows, gui);
                show_fronts ? showFronts(data.m) : deleteFronts(data.m);

                //re-push on gui
                gui.push(&data.m, false);
                data.m.updateGL();
                break;
            }

            //smoothing
            case GLFW_KEY_O: {

                //update undo
                undo_data = data;

                //smooth the surface
                smooth(data, 5);

                //UI
                show_arrows ? updateArrows(data.m, data.oct, dir_arrows, data.fronts_active, gui) : deleteArrows(dir_arrows, gui);
                show_fronts ? showFronts(data.m) : deleteFronts(data.m);
                //update model
                data.m.update_normals();
                data.m.updateGL();
                break;
            }

            //expand and smooth
            case GLFW_KEY_I: {

                //update undo
                undo_data = data;

                //expand and smooth
                expand(data, true);
                smooth(data, 5);

                //UI
                show_arrows ? updateArrows(data.m, data.oct, dir_arrows, data.fronts_active, gui) : deleteArrows(dir_arrows, gui);
                show_fronts ? showFronts(data.m) : deleteFronts(data.m);
                //update model
                data.m.update_normals();
                data.m.updateGL();
                break;
            }

            //show orient3D sign
            case GLFW_KEY_Y: {
                show_vol_color = !show_vol_color;
                if(show_vol_color) show_volume(data.m);
                else clear_colors(data.m);

                data.m.updateGL();
            }

            default:
                handled = false;
        }

        return handled;

    };

    return gui.launch();
}



//set up the env with model, target, oct etc...
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

    //edge length threshold
    data.edge_threshold = data.srf.edge_max_length() * 2;

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

    for(uint vid : data.m.get_surface_verts())
        data.m.vert_data(vid).label = false;

    std::cout << "DONE" << TXT_RESET << std::endl;

    return data;
}

//search the edges that need to be split
std::set<uint> search_split(Data &d, bool selective) {

    //search of the edges to split inside the front
    std::set<uint> edges_to_split;

    //if we have to refine only were the edges are too long
    if(selective)
        //for every vid in active fronts
        for(uint vid : d.fronts_active)
            //check the edge (surface) umbrella
            for(uint eid : d.m.vert_adj_srf_edges(vid))
                //check if the threshold is passed
                if(d.m.edge_length(eid) > d.edge_threshold)
                    //for every adj srf poly (face then poly) get the edges
                    for(uint fid : d.m.edge_adj_srf_faces(eid))
                        edges_to_split.insert(d.m.adj_p2e(d.m.adj_f2p(fid)[0]).begin(), d.m.adj_p2e(d.m.adj_f2p(fid)[0]).end());


    //if we have to refine all the active front
    if(!selective)
        //for every vid in active fronts
        for(uint vid : d.fronts_active)
            //for every adj face
            for(uint fid : d.m.adj_v2f(vid))
                //if is a surface face from the active front (check if the adj vids of the face are also in the front)
                if(d.m.face_is_on_srf(fid)
                   && CONTAINS_VEC(d.fronts_active, d.m.face_v2v(fid, vid)[0])
                   && CONTAINS_VEC(d.fronts_active, d.m.face_v2v(fid, vid)[1]))
                    //add every edge of the active poly to the split list
                    edges_to_split.insert(d.m.adj_p2e(d.m.adj_f2p(fid)[0]).begin(), d.m.adj_p2e(d.m.adj_f2p(fid)[0]).end());

    return edges_to_split;
}

//split the edges passed as parameters, give back the map og_edge/new_vert and a list of edges that need to be flipped
void split(Data &d, std::set<uint> &edges_to_split, std::map<ipair, uint> &v_map, std::queue<edge_to_flip> &edges_to_flip) {

    //data that we need for later
    uint offset = d.m.num_verts();

    //split parameters
    uint new_vid;
    ipair og_edge;
    //split all the edges in the set
    for(uint eid : edges_to_split) {
        og_edge = unique_pair(d.m.edge_vert_id(eid, 0), d.m.edge_vert_id(eid, 1));

        //check if the edges created will need to be flipped
        for(uint fid : d.m.adj_e2f(eid))
            if(d.m.face_vert_opposite_to(fid, eid) < offset
               && CONTAINS(edges_to_split, d.m.face_edge_id(fid, 0))
               && CONTAINS(edges_to_split, d.m.face_edge_id(fid, 1))
               && CONTAINS(edges_to_split, d.m.face_edge_id(fid, 2))) {
                edge_to_flip etf;
                etf.opp_vid = d.m.face_vert_opposite_to(fid, eid);
                etf.og_edge = og_edge;
                edges_to_flip.push(etf);
            }

        //split and map update
        new_vid = d.m.edge_split(eid);
        v_map.emplace(og_edge,new_vid);

        //set the activity
        d.m.vert_data(new_vid).label = false;
    }

}

//refine internal edges if they are too long
std::set<uint> search_split_int(Data &d) {

    //edge that need to be split
    std::set<uint> edges_to_split;

    //search for the internal edges with srf endpoint that exceed the threshold lenght
    for(uint vid : d.m.get_surface_verts())
        for(uint eid : d.m.adj_v2e(vid))
            if(!d.m.edge_is_on_srf(eid))
                if(d.m.edge_length(eid) > d.edge_threshold * 2)
                    edges_to_split.insert(eid);

    return edges_to_split;
}

void split_int(Data &d, std::set<uint> &edges_to_split) {

    //split all the edges
    for(uint eid : edges_to_split)
        d.m.edge_split(eid);

}

//flip every edge passed as parameter in the list
void flip(Data &d, std::map<ipair, uint> &v_map, std::queue<edge_to_flip> &edges_to_flip) {

    std::map<ipair, uint> try_again;
    while(!edges_to_flip.empty()) {

        //retrieve the info
        edge_to_flip etf = edges_to_flip.front();
        uint vid = v_map.at(etf.og_edge);
        uint eid = d.m.edge_id(vid, etf.opp_vid);

        //if the edge isn't on the srf we do the flip 4-4
        if(!d.m.edge_is_on_srf(eid) ) {

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
        } else if(!d.m.vert_data(etf.opp_vid).label) {

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
}

//split all the edges in the active front and flip who needs to be flipped
void split_n_flip(Data &d, bool selective) {

    //start of split
    std::cout << TXT_CYAN << "Splitting the polys... ";

    //data that we need for later
    uint offset = d.m.num_verts();

    //search of the edges to split inside the front
    std::set<uint> edges_to_split = search_split(d, selective);
    //split the edges while keep tracking of the edge/map relationship and the edges that will need to be flipped
    std::map<ipair, uint> v_map;
    std::queue<edge_to_flip> edges_to_flip;
    split(d, edges_to_split, v_map, edges_to_flip);

    //end of split
    std::cout << "DONE" << TXT_RESET << std::endl;

    //start of flipping
    std::cout << TXT_CYAN << "Flipping the edges... ";

    //* flip every edge we need to
    flip(d, v_map, edges_to_flip);

    //end of flipping
    std::cout << "DONE" << TXT_RESET << std::endl;

    //update the fronts
    update_fronts(d);
}

//edge flip 2-2 (must: surf & srf faces coplanar)
bool flip2to2(DrawableTetmesh<> &m, uint eid) {

    //edge must be on the surface and with only two polys adj
    if(!m.edge_is_on_srf(eid))
        return false;
    if(m.adj_e2p(eid).size() != 2)
        return false;

    bool result = true;
    uint pid_of = m.adj_e2p(eid)[0]; //need the opp faces
    uint pid_ov = m.adj_e2p(eid)[1]; //need the opp vert of the non-shared face
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

    //check on the volume
    double or1 = orient3d(m.vert(tets[0][0]), m.vert(tets[0][1]), m.vert(tets[0][2]), m.vert(tets[0][3]));
    double or2 = orient3d(m.vert(tets[1][0]), m.vert(tets[1][1]), m.vert(tets[1][2]), m.vert(tets[1][3]));
    result = or1 * or2 > 0;

    // add new elements to the mesh (if the volume is ok)
    if(result) {
        for (tet_id = 0; tet_id < 2; tet_id++) {
            uint new_pid = m.poly_add({tets[tet_id][0],
                                       tets[tet_id][1],
                                       tets[tet_id][2],
                                       tets[tet_id][3]});
            m.update_p_quality(new_pid);
        }
        m.edge_remove(eid);
    }

    return result;
}

//edge flip 4-4
bool flip4to4(DrawableTetmesh<> &m, uint eid, uint vid0, uint vid1) {
    std::vector<uint> cluster = m.adj_e2p(eid);
    if(cluster.size() != 4)
        return false;
    if(vid0*vid1 == 0)
        return false;

    bool result = true;

    uint tet_id = 0;
    uint tets[4][4];
    //for every face adj to vid0
    for(uint fid : m.adj_v2f(vid0)) {
        //if the face doesn't contain the edge to flip
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

    //volume check
    double orient = orient3d(m.vert(tets[0][0]), m.vert(tets[0][1]), m.vert(tets[0][2]), m.vert(tets[0][3]));
    for(int idx = 1; idx < 4 && result; idx++)
        result = orient3d(m.vert(tets[idx][0]), m.vert(tets[idx][1]), m.vert(tets[idx][2]), m.vert(tets[idx][3])) * orient  > 0;

    // add new elements to the mesh
    if(result) {
        for (tet_id = 0; tet_id < 4; tet_id++) {
            uint new_pid = m.poly_add({tets[tet_id][0],
                                       tets[tet_id][1],
                                       tets[tet_id][2],
                                       tets[tet_id][3]});
            m.update_p_quality(new_pid);
        }
        m.edge_remove(eid);
    }
    return result;
}

//move the verts toward the target
void expand(Data &d, bool refine) {

    //start
    std::cout << TXT_CYAN << "Expanding the model... ";

    //for every active front
    for(uint vid : d.fronts_active) {

        assert(d.m.vert_is_on_srf(vid));

        //get info
        vec3d og_pos = d.m.vert(vid);
        double dist = d.oct.closest_point(d.m.vert(vid)).dist(d.m.vert(vid));
        //move the vert
        d.m.vert(vid) += d.m.vert_data(vid).normal * dist * d.mov_speed;

        //check intersection
        uint iter_counter = 0;
        std::unordered_set<uint> intersect_ids;

        //check vert-edge-face intersection with the target
        while(check_intersection(d, vid) && iter_counter < 5) {
            d.m.vert(vid) -= d.m.vert_data(vid).normal * dist * d.mov_speed / 2;
            dist = d.oct.closest_point(d.m.vert(vid)).dist(d.m.vert(vid));
            iter_counter++;
        }
        //if edge still outside get back the vid
        if(check_intersection(d, vid))
            d.m.vert(vid) = og_pos;

        //positive volume check
//        iter_counter = 0;
//        while(check_volume(d, vid) && iter_counter < 5) {
//            d.m.vert(vid) -= (d.m.vert(vid) - og_pos) / 2;
//            iter_counter++;
//        }


        //update the mask
        if(d.oct.closest_point(d.m.vert(vid)).dist(d.m.vert(vid)) < d.eps_inactive)
            d.m.vert_data(vid).label = true;

    }

    //job done
    std::cout << "DONE" << TXT_RESET << std::endl;

    //refine where the edges are too long or just update the front
    if(refine)
        split_n_flip(d, true);
    else
        update_fronts(d);

    //refile internal edges
    if(refine) {
        std::set<uint> edges_to_split = search_split_int(d);
        split_int(d, edges_to_split);
    }
}

//smooth ONLY the surface of the model
void smooth(Data &d, int n_iter) {

    std::unordered_map<uint, uint> map_v2s, map_s2v;
    export_surface(d.m, d.m_srf, map_v2s, map_s2v);

    Octree octree;
    octree.build_from_mesh_polys(d.m_srf);

    //for the number of iterations selected
    for(int iter = 0; iter < n_iter; iter++) {

        //for every active front
        for (uint vid : d.m.get_surface_verts()) {

            if(d.m.vert_data(vid).label)
                continue;

            //calc the barycenter
            vec3d bary(0, 0, 0);
            for (uint adj : d.m_srf.adj_v2v(map_v2s.at(vid))) bary += d.m_srf.vert(adj);
            bary /= static_cast<double>(d.m_srf.adj_v2v(map_v2s.at(vid)).size());

            //move the vert
            d.m.vert(vid) = octree.closest_point(bary);

            //if there are any intersection the vert is locked
            d.m.vert_data(vid).label = check_intersection(d, vid);
        }

    }

    for (uint vid = 0; vid < d.m.num_verts(); vid++) {

        if(d.m.vert_is_on_srf(vid))
            continue;

        //calc the barycenter
        vec3d bary(0, 0, 0);
        for(uint adj : d.m.adj_v2v(vid)) bary += d.m.vert(adj);
        bary /= static_cast<double>(d.m.adj_v2v(vid).size());

    }

    //struct ARAP_data ad;
    //ARAP(d.m, ad);

    update_fronts(d);
}

bool check_intersection(Data &d, uint vid) {

    //params
    std::unordered_set<uint> intersect_ids;
    bool flag = false;

    //check the umbrella
    for (uint fid : d.m.vert_adj_srf_faces(vid)) {
        vec3d verts[] = {d.m.face_verts(fid)[0], d.m.face_verts(fid)[1], d.m.face_verts(fid)[2]};
        flag = d.oct.intersects_triangle(verts, false, intersect_ids) || flag;
    }

    return flag;
}

bool check_volume(Data &d, uint vid) {

    bool check = false;

    for(uint pid : d.m.adj_v2p(vid)) {
        vec3d pa = d.m.poly_vert(pid, 0);
        vec3d pb = d.m.poly_vert(pid, 1);
        vec3d pc = d.m.poly_vert(pid, 2);
        vec3d pd = d.m.poly_vert(pid, 3);
        check = orient3d(pa, pb, pc, pd) >= 0 || check;
    }

    return check;
}

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
    while (query_front < d.fronts_active.size() && (CONTAINS_VEC(new_fronts, d.fronts_active[query_front]) || d.m.vert_data(d.fronts_active[query_front]).label))
        query_front++;

    //check for every new front inside the old one (could be no one)
    while (query_front < d.fronts_active.size()) {

        //get the front
        front_from_seed(d, d.fronts_active[query_front], dect_front);

        //add the front to the new vec
        new_fronts.insert(new_fronts.end(), dect_front.begin(), dect_front.end());
        new_bounds.emplace_back(new_fronts.size());

        //find the next idx
        while (query_front < d.fronts_active.size() && (CONTAINS_VEC(new_fronts, d.fronts_active[query_front]) || d.m.vert_data(d.fronts_active[query_front]).label))
            query_front++;
    }

    //resize of the front vector
    new_fronts.shrink_to_fit();

    //give back the new active front and bounds
    d.fronts_active = new_fronts;
    d.fronts_bounds = new_bounds;

    std::cout << "DONE - F: " << d.fronts_bounds.size() << " V: " << d.fronts_active.size() << TXT_RESET << std::endl;
}

//get the front from the seed
void front_from_seed(Data &d, uint seed, std::unordered_set<uint> &front) {
    front.clear();
    front.insert(seed);

    std::queue<uint> q;
    q.push(seed);

    while(!q.empty()) {
        uint vid = q.front();
        q.pop();

        for(uint adj_vid : d.m.vert_adj_srf_verts(vid)) {
            if (!d.m.vert_data(adj_vid).label && DOES_NOT_CONTAIN(front, adj_vid)) {
                front.insert(adj_vid);
                q.push(adj_vid);
            }
        }
    }

}