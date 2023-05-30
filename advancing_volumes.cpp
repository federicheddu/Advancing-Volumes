#include "advancing_volumes.h"

//set up the env with model, target, oct etc...
Data setup(const char *path, bool load) {

    Data data;

    //std::cout << std::endl << TXT_BOLDMAGENTA << "Data loading... " << std::endl;

    if(load) {

        std::string model_path = path;
        std::string target_path = "../data/" + model_path.substr(11, model_path.size()-11);

        data.m = DrawableTetmesh<>(model_path.c_str());
        data.vol = DrawableTetmesh<>(target_path.c_str());
        data.vol.show_mesh_points();

    } else {

        //model vol
        data.vol = DrawableTetmesh<>(path);
        data.vol.show_mesh_points();

    }

    //model srf
    export_surface(data.vol, data.srf);

    //octree build
    data.oct.build_from_mesh_polys(data.srf);

    //edge length threshold
    data.edge_threshold = data.srf.bbox().diag() * 0.02;

    //inactive threshold
    data.eps_inactive = data.edge_threshold * data.eps_percent;

    if(!load) {
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
    }

    if(load) {
        //reload fronts
        for (uint vid : data.m.get_surface_verts()) {

            if(dist_calc(data, vid, true) < data.eps_inactive) {
                data.m.vert_data(vid).label = true;
                data.fronts_active.emplace_back(vid);
            } else
                data.m.vert_data(vid).label = false;
            //subdivide the front
            update_fronts(data);
        }
    } else {
        //init fronts
        for (uint vid: data.m.get_surface_verts()) {
            data.m.vert_data(vid).label = false;
            data.m.vert_data(vid).uvw.x() = 0;
        }
    }
    //std::cout << "DONE" << TXT_RESET << std::endl;

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
            for(uint fid : d.m.vert_adj_srf_faces(vid))
                //if is a surface face from the active front (check if the adj vids of the face are also in the front)
                /*if(CONTAINS_VEC(d.fronts_active, d.m.face_v2v(fid, vid)[0])
                  && CONTAINS_VEC(d.fronts_active, d.m.face_v2v(fid, vid)[1]))*/
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
        d.m.vert_data(new_vid).uvw.x() = 0;
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
                if(d.m.edge_length(eid) > d.edge_threshold)
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
    //std::cout << TXT_CYAN << "Splitting the polys... ";

    //data that we need for later
    uint offset = d.m.num_verts();

    //search of the edges to split inside the front
    std::set<uint> edges_to_split = search_split(d, selective);
    //split the edges while keep tracking of the edge/map relationship and the edges that will need to be flipped
    std::map<ipair, uint> v_map;
    std::queue<edge_to_flip> edges_to_flip;
    split(d, edges_to_split, v_map, edges_to_flip);

    //end of split
    //std::cout << "DONE" << TXT_RESET << std::endl;

    //start of flipping
    //std::cout << TXT_CYAN << "Flipping the edges... ";

    //* flip every edge we need to
    flip(d, v_map, edges_to_flip);

    //end of flipping
    //std::cout << "DONE" << TXT_RESET << std::endl;

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
void expand(Data &d, bool refine, ExpansionMode exp_mode) {

    //start
    //std::cout << TXT_CYAN << "Expanding the model... ";

    //get all the distances from the target model
    std::vector<double> front_dist(d.fronts_active.size());
    PARALLEL_FOR(0, d.fronts_active.size(), 1000, [&](int idx)
    {
        uint vid = d.fronts_active.at(idx);
        double dist;
        switch (exp_mode) {
            case CLOSEST_POINT: {
                dist = dist_calc(d, vid, false, true);
                break;
            }
            case RAYCAST: {
                dist = dist_calc(d, vid, true, true);
                break;
            }
            case LOCAL: {
                dist = dist_calc(d, vid, false);
                if(dist < d.eps_inactive*2) dist = dist_calc(d, vid, true, true);
                else dist = dist_calc(d, vid, false, true);
                break;
            }
        }
        front_dist.at(idx) = dist;
    });

    //move every vert in the active front
    for(uint idx = 0; idx < d.fronts_active.size(); idx++) {

        //get the vid
        uint vid = d.fronts_active.at(idx);
        assert(d.m.vert_is_on_srf(vid));

        //og pos to get back if something goes wrong
        vec3d og_pos = d.m.vert(vid);

        //get the distance calculated before
        double dist = front_dist.at(idx);

        //move the vert
        double movement = dist * d.mov_speed;
        d.m.vert(vid) += d.m.vert_data(vid).normal * movement;

        //put the vert in a safe place between [og_pos, actual_pos]
        go_back_safe(d, vid, og_pos);

        //update the mask
        if(dist_calc(d, vid, true) < d.eps_inactive)
            d.m.vert_data(vid).label = true;

        //if the vertex remains stationary for 5 iterations it is deactivated
        if(d.m.vert(vid) == og_pos) {
            d.m.vert_data(vid).uvw.x()++;
            if(d.m.vert_data(vid).uvw.x() == 5)
                d.m.vert_data(vid).label = true;
        } else
            d.m.vert_data(vid).uvw.x() = 0;

    }

    //job done
    //std::cout << "DONE" << TXT_RESET << std::endl;

    smooth(d, 5);

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

//laplacian smooth of the model
void smooth(Data &d, int n_iter) {

    //start
    //std::cout << TXT_CYAN << "Smoothing the model...";

    export_surface(d.m, d.m_srf);
    //mesh_smoother(d.m_srf, d.m_srf);
    Octree octree;
    octree.build_from_mesh_polys(d.m_srf);

    //for the number of iterations selected
    for(int iter = 0; iter < n_iter; iter++) {

        for(int vid = d.m.num_verts()-1; vid >= 0; vid--) {

            if(d.m.vert_is_on_srf(vid) && d.m.vert_data(vid).label)
                continue;

            //parameters
            vec3d og_pos = d.m.vert(vid); //seed for tangent plane (see project_on_tangent_plane)
            vec3d og_norm = d.m.vert_data(vid).normal; //seed for tangent plane (see project_on_tangent_plane)
            vec3d bary(0, 0, 0);

//            for (uint adj: d.m.adj_v2v(vid)) bary += d.m.vert(adj);
//            bary /= static_cast<double>(d.m.adj_v2v(vid).size());
//            d.m.vert(vid) = bary;
//            if(d.m.vert_is_on_srf(vid)) d.m.vert(vid) = project_onto_tangent_plane(bary, og_pos, og_norm);

            if(d.m.vert_is_on_srf(vid)) {
                for (uint adj: d.m.vert_adj_srf_verts(vid)) bary += d.m.vert(adj);
                bary /= static_cast<double>(d.m.vert_adj_srf_verts(vid).size());
                //d.m.vert(vid) = project_onto_tangent_plane(bary, og_pos, og_norm);
                //d.m.vert(vid) = octree.closest_point(bary);
                d.m.vert(vid) = bary;
                d.m.update_v_normal(vid);
                double dist; uint aux;
                octree.intersects_ray(d.m.vert(vid), d.m.vert_data(vid).normal, dist, aux);
                d.m.vert(vid) += d.m.vert_data(vid).normal * dist;
            } else {
                for (uint adj: d.m.adj_v2v(vid)) bary += d.m.vert(adj);
                bary /= static_cast<double>(d.m.adj_v2v(vid).size());
                d.m.vert(vid) = bary;
            }

            go_back_safe(d, vid, og_pos);

        }
    }

    //job done
    //std::cout << "DONE" << TXT_RESET << std::endl;
}

//check if there are any intersection
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

//check if sign of orient3D is ok
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

    //std::cout << TXT_CYAN << "Updating the fronts... ";

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

        //add the front to the new vec (if the vert is not alone in the front)
        if(dect_front.size() > 1) {
            new_fronts.insert(new_fronts.end(), dect_front.begin(), dect_front.end());
            new_bounds.emplace_back(new_fronts.size());
        } else
            d.m.vert_data(*begin(dect_front)).label = true;

        //find the next idx
        while (query_front < d.fronts_active.size() && (CONTAINS_VEC(new_fronts, d.fronts_active[query_front]) || d.m.vert_data(d.fronts_active[query_front]).label))
            query_front++;
    }

    //resize of the front vector
    new_fronts.shrink_to_fit();

    //give back the new active front and bounds
    d.fronts_active = new_fronts;
    d.fronts_bounds = new_bounds;

    //std::cout << "DONE - F: " << d.fronts_bounds.size() << " V: " << d.fronts_active.size() << TXT_RESET << std::endl;
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

//calc the distance from the target with a raycast hit
double dist_calc(Data &d, uint vid, bool raycast, bool flat) {

    //param
    uint id; //useless
    double dist;

    //raycast hit (normal direction)
    if(raycast)
        d.oct.intersects_ray(d.m.vert(vid), d.m.vert_data(vid).normal, dist, id);
    else
        dist = d.oct.closest_point(d.m.vert(vid)).dist(d.m.vert(vid));

    double adj_dist = 0, dist_sum = dist;
    if(flat) {
        for(uint adj_vid : d.m.vert_adj_srf_verts(vid)) {
            if (raycast)
                d.oct.intersects_ray(d.m.vert(adj_vid), d.m.vert_data(adj_vid).normal, adj_dist, id);
            else
                adj_dist = d.oct.closest_point(d.m.vert(adj_vid)).dist(d.m.vert(adj_vid));

            dist_sum += adj_dist;
        }
        dist_sum /= d.m.vert_adj_srf_verts(vid).size() + 1;
        dist = dist_sum;
    }

    return dist;
}

//check if the movement of the vert is safe, if not it goes back by bisection
void go_back_safe(Data &d, uint vid, const vec3d &og_pos) {

    int iter_counter;
    int max_iter = 50;

    //check for intersections (only if on surface)
    if(d.m.vert_is_on_srf(vid)) {
        iter_counter = 0;
        while (check_intersection(d, vid) && iter_counter < max_iter) {
            d.m.vert(vid) -= (d.m.vert(vid) - og_pos) / 2;
            iter_counter++;
        }
        //if edge still outside get back the vid
        if (iter_counter == max_iter && check_intersection(d, vid))
            d.m.vert(vid) = og_pos;
    }

    //if it is not already putted back to the og pos
    //check the volume with orient3D
    if(!(d.m.vert(vid) == og_pos)) {
        iter_counter = 0;
        while(check_volume(d, vid) && iter_counter < max_iter) {
            d.m.vert(vid) -= (d.m.vert(vid) - og_pos) / 2;
            iter_counter++;
        }
        if(iter_counter == max_iter && check_volume(d, vid))
            d.m.vert(vid) = og_pos;
    }

}

vec3d project_onto_tangent_plane(vec3d &point, vec3d &plane_og, vec3d &plane_norm) {

    vec3d u = point - plane_og;
    double dist = u.dot(plane_norm);
    vec3d proj = point - plane_norm * dist;

    return proj;

}

void final_projection(Data &d) {

    vec3d og_pos;
    double dist; uint aux;
    for(int iter = 0; iter < 5; iter++){
        for (uint vid: d.m.get_surface_verts()) {
            og_pos = d.m.vert(vid);

            dist = dist_calc(d, true);
            d.oct.intersects_ray(d.m.vert(vid), d.m.vert_data(vid).normal, dist, aux);
            d.m.vert(vid) += d.m.vert_data(vid).normal * dist;

            go_back_safe(d, vid, og_pos);
        }
        smooth(d);
    }

}