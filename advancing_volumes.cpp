#include "advancing_volumes.h"
#undef NDEBUG

//main function
void advancing_volume(Data &d) {

    d.step++;
    cout << BMAG << "Advancing Volume ITERATION " << d.step << RST << endl;

    if(d.running) expand(d);
    if(d.running) smooth(d, MODEL);
    if(d.running) do { refine(d); } while(d.multiple_refinement && refine_again(d));
    if(d.running) smooth(d, MODEL);
    update_front(d);

    cout << BMAG << "Advancing Volume ITERATION - Actives: " << d.front.size() << "/" << d.m.num_srf_verts() << RST << endl << endl;

}

/* ================================================================================================================== */

//expand the model
void expand(Data &d) {

    if(d.verbose) cout << BCYN << "Expanding model..." << RST;
    if(d.render) d.gui->pop_all_markers();

    //compute DDD: direction, distance and displacement -> saved into .vert_data(vid).normal
    compute_directions(d);
    compute_distances(d);
    compute_displacements(d);

    //priority queue for the front
    auto cmp = [&](uint a, uint b) { return d.m.vert_data(a).uvw[DIST] > d.m.vert_data(b).uvw[DIST]; };
    std::sort(d.front.begin(), d.front.end(), cmp);

    //move the verts
    for(uint vid : d.front) {
        move(d, vid, d.m.vert_data(vid).normal);
        if(!d.running) return;
    }

    //restore normals, update fronts, check for self intersections
    d.m.update_normals();
    check_self_intersection(d);
    if(!d.running) return;

    if(d.verbose && d.running) cout << BGRN << "DONE" << RST << endl;
}

//move the vert in the direction of the displacement
void move(Data &d, uint vid, vec3d &disp) {

    bool moved = true;

    //backup
    vec3d safe_pos = d.m.vert(vid);

    /* TODO: topological unlock */

    //update the vert position
    d.m.vert(vid) += disp;

    line_search(d, vid, safe_pos);
    if(!d.running) return;

    //update the active flag
    d.m.vert_data(vid).flags[ACTIVE] = dist_calc(d, vid, true) > d.inactivity_dist;

}

//compute the direction of movement for each vert (laplacian smoothing of the normals)
void compute_directions(Data &d) {

    vec3d direction;

    //direction smoothing
    for(int it = 0; it < d.smooth_dir_iters; it++) {
        for(uint vid : d.front) {
            //default movement
            direction = d.m.vert_data(vid).normal;
            //avg
            for(uint adj : d.m.vert_adj_srf_verts(vid))
                direction += d.m.vert_data(adj).normal;
            direction = direction / double(d.m.vert_adj_srf_verts(vid).size() + 1);
            direction.normalize();
            //update after smoothing
            d.m.vert_data(vid).normal = direction;
        }
    }

}

//compute the distance of travel for each vert
void compute_distances(Data &d) {

    double dist;
    for(uint vid : d.front) {
        dist = dist_calc(d, vid, d.only_raycast);
        if(!d.only_raycast && dist < d.switch_raycast_threshold)
            dist = dist_calc(d, vid, true);
        d.m.vert_data(vid).uvw[DIST] = dist;
    }

}

//compute the displacement of the vert (direction * distance)
void compute_displacements(Data &d) {

    for(uint vid : d.front)
        d.m.vert_data(vid).normal *= d.m.vert_data(vid).uvw[DIST] * 0.99;

}

//push forward dents
void avoid_dents(Data &d) {

}

//guarantees a safe position at the top after the movement
void line_search(Data &d, uint vid, vec3d &safe_pos) {

    vec3d final;
    int bck_cnt = 0, frd_cnt = 0;

    //check only if surface
    if(!d.m.vert_is_on_srf(vid)) return;

    //backup the current position
    final = d.m.vert(vid);

    //backward bisection
    while(bck_cnt < d.line_search_max && (check_intersection(d, vid) || is_vert_flipped(d, vid))) {
        //backward
        d.m.vert(vid) = (d.m.vert(vid) + safe_pos) / 2;
        //counter
        bck_cnt++;
    }

    //forward bisection
    while(frd_cnt < d.line_search_max && !check_intersection(d, vid) && !is_vert_flipped(d, vid)) {
        //update safe position
        safe_pos = d.m.vert(vid);
        //forward
        d.m.vert(vid) = (d.m.vert(vid) + final) / 2;
        //counter
        frd_cnt++;
    }

    //go to the last safe position
    if(check_intersection(d, vid) || is_vert_flipped(d, vid))
        d.m.vert(vid) = safe_pos;

}

void line_search_refined(Data &d, uint vid, vec3d &safe_pos) {

}

//check if the vert umbrella is intersecting the target
bool check_intersection(Data &d, uint vid) {

    //params
    bool flag = false;
    vec3d verts[3];
    std::unordered_set<uint> intersect_ids; //warning killer

    //check the umbrella
    for (uint fid : d.m.vert_adj_srf_faces(vid)) {
        verts[0] = d.m.face_verts(fid)[0];
        verts[1] = d.m.face_verts(fid)[1];
        verts[2] = d.m.face_verts(fid)[2];
        flag = d.oct->intersects_triangle(verts, false, intersect_ids);
        if(flag) break; //early stop
    }

    return flag;
}

//check intersection with himself
void check_self_intersection(Data &d) {

    if(d.verbose) cout << BCYN << "Checking self intersection..." << RST;
    std::set<ipair> intersections;
    std::unordered_map<uint, uint> m2srf, srf2m;

    export_surface(d.m, d.ms, m2srf, srf2m);
    find_intersections(d.ms, intersections);
    d.running = intersections.empty();

    uint face;
    std::vector<uint> sf(2), sv;
    for(auto inter : intersections) {
        sf[0] = inter.first;
        sf[1] = inter.second;

        for(uint fid : sf) {
            sv = d.ms.poly_verts_id(sf[0]);
            face = d.m.face_id({srf2m[sv[0]], srf2m[sv[1]], srf2m[sv[2]]});
            for (uint pid: d.m.adj_f2p(face))
                d.m.poly_data(pid).color = Color::RED();
        }

    }

    //feedback
    if(!d.running) cout << BRED << "FOUND" << RST << endl;
    if(!d.running && !d.render) my_assert(d, false, "SELF INTERSECTION", __FILE__, __LINE__);

}

/* ================================================================================================================== */


/* ================================================================================================================== */

void refine(Data &d, bool all) {

    if(d.verbose) cout << BCYN << "Refining model..." << RST;
    split(d, all);

    if(d.verbose) cout << BCYN << "Adjusting the topology..." << RST;
    flip(d);
    try_flips(d);

    if(d.verbose && d.running) cout << BGRN << "DONE" << RST << endl;

    //free memory
    d.ets.clear();
    d.v_map.clear();
    my_assert(d, d.etf.empty(), "The queue of edges to flip is not empty after the flips", __FILE__, __LINE__);
}

bool refine_again(Data &d) {

    std::vector<uint> srf_edges = d.m.get_surface_edges();
    double lenght = 0;
    for(uint eid : srf_edges)
        lenght += d.m.edge_length(eid);
    lenght /= double(srf_edges.size());

    return lenght > d.target_edge_length;

}

void edges_to_split(Data &d, bool all) {

    if(all) { //if you need to split everything
        for(uint eid = 0; eid < d.m.num_edges(); eid++) {
            d.ets.insert(eid);
        }
    } else { //if you need to split only the too long
        for (uint vid: d.front) { //for every vid in the front
            for (uint eid: d.m.adj_v2e(vid)) { //get the edges adj
                if (d.m.edge_length(eid) > d.target_edge_length) {
                    if (d.m.edge_is_on_srf(eid)) { //if the vid is on srf get every edge adj
                        for (uint fid: d.m.edge_adj_srf_faces(eid)) {
                            d.ets.insert(d.m.adj_p2e(d.m.adj_f2p(fid)[0]).begin(),d.m.adj_p2e(d.m.adj_f2p(fid)[0]).end());
                        }
                    } else { //if its internal get only that edge
                        d.ets.insert(eid);
                    }
                }
            }
        }
    }
}

void split(Data &d, bool all) {

    //search for edges to split
    edges_to_split(d, all);

    //initial number of verts
    uint nv = d.m.num_verts();

    //parameters
    uint new_vid, map_vid;
    ipair og_edge;
    edge_to_flip etf;

    //split all the edges in the set
    for(uint eid : d.ets) {
        //backup
        og_edge = unique_pair(d.m.edge_vert_id(eid, 0), d.m.edge_vert_id(eid, 1));

        //check if the edges created will need to be flipped
        for(uint fid : d.m.adj_e2f(eid)) {
            if (d.m.face_vert_opposite_to(fid, eid) < nv
                && CONTAINS(d.ets, d.m.face_edge_id(fid, 0))
                && CONTAINS(d.ets, d.m.face_edge_id(fid, 1))
                && CONTAINS(d.ets, d.m.face_edge_id(fid, 2))) {
                etf.og_edge = og_edge;
                etf.opp_vid = d.m.face_vert_opposite_to(fid, eid);
                d.etf.push(etf);
            }
        }

        //split
        new_vid = d.m.edge_split(eid);
        if(d.map && d.step > 0) { //update sphere mapping
            map_vid = d.mm.edge_split(eid);
            my_assert(d, new_vid == map_vid, "The new vid is not the same in the model and in the map", __FILE__, __LINE__);
        }

        //mark how the vert is created
        d.m.vert_data(new_vid).flags[SPLIT] = d.step > 0;
        d.m.vert_data(new_vid).flags[TOPOLOGICAL] = false;

        //update the edge_to_flip
        d.v_map.emplace(og_edge, new_vid);

        //set the activity
        d.m.vert_data(new_vid).flags[ACTIVE] = dist_calc(d, new_vid, true) > d.inactivity_dist;
        if(d.m.vert_data(new_vid).flags[ACTIVE]) d.front.push_back(new_vid);
    }
}

void flip(Data &d) {

    //flip parameters
    bool result;
    uint vid, eid;
    size_t count;
    ipair eid_pair;
    edge_to_flip etf;
    std::map<ipair, uint> try_again;

    while(!d.etf.empty()) {

        etf = d.etf.front();
        vid = d.v_map[etf.og_edge];
        eid = d.m.edge_id(vid, etf.opp_vid);
        eid_pair = unique_pair(vid, etf.opp_vid);

        //if the edge isn't on the srf we do the split 4-4
        if(!d.m.edge_is_on_srf(eid)) {

            uint vid0, vid1;
            ipair query_pair;

            //query for the parameter vids for the flip
            //need to find the two vids that will become the new edge
            query_pair = unique_pair(etf.og_edge.first, etf.opp_vid);
            vid0 = d.v_map.count(query_pair) ? d.v_map.at(query_pair) : 0;
            query_pair = unique_pair(etf.og_edge.second, etf.opp_vid);
            vid1 = d.v_map.count(query_pair) ? d.v_map.at(query_pair) : 0;

            //flip
            result = flip4to4(d.m, eid, vid0, vid1);
            //update the map
            if(d.map && d.step > 0 && result) {
                result = flip4to4(d.mm, d.mm.edge_id(eid_pair.first, eid_pair.second), vid0, vid1);
                if(!result) result = flip4to4(d.m, d.m.edge_id(vid0, vid1), eid_pair.first, eid_pair.second);
                my_assert(d, result, "Wrong restoring of the model in flip4to4", __FILE__, __LINE__);
            }

        } else if(d.m.vert_data(etf.opp_vid).flags[ACTIVE]) {
            //flip (no query needed, I can have only one setup)
            uint vid0, vid1;
            result = flip2to2(d.m, eid, vid0, vid1);
            //update the map
            if(d.map && d.step > 0 && result) {
                result = flip2to2(d.mm, d.mm.edge_id(eid_pair.first, eid_pair.second));
                if(!result) result = flip2to2(d.m, d.m.edge_id(vid0, vid1));
                my_assert(d, result, "Wrong restoring of the model in flip2to2", __FILE__, __LINE__);
            }
        }

        //if the flip is good and is a 'try_again' the edge is removed
        if(result && try_again.count(eid_pair) != 0) try_again.erase(eid_pair);

        //if we cant do the flip now we can do it in the future (hopefully)
        //we re-push in the queue if it's the first fail OR if the situation is better than the last time
        if(!result) {
            count = try_again.count(eid_pair);
            if(count == 0 || (count == 1 && d.m.edge_valence(eid) != try_again[eid_pair])) {
                d.etf.push(etf);
                try_again[eid_pair] = d.m.edge_valence(eid);
            }
        }
        //pop the queue
        d.etf.pop();
    }

}

void try_flips(Data &d) {

    uint vid0, vid1, new_eid;
    std::vector<uint> adj_f;
    double lenght, lenght_opp;
    bool result;

    for(uint eid : d.m.get_surface_edges()) {

        adj_f = d.m.edge_adj_srf_faces(eid);
        my_assert(d, adj_f.size() == 2, "Edge on srf has more than 2 adjacent srf faces", __FILE__, __LINE__);

        vid0 = d.m.face_vert_opposite_to(adj_f[0], eid);
        vid1 = d.m.face_vert_opposite_to(adj_f[1], eid);

        lenght = d.m.edge_length(eid);
        lenght_opp = d.m.vert(vid0).dist(d.m.vert(vid1));

        if(lenght_opp < lenght && d.m.adj_e2p(eid).size() == 2) {
            //flip to get a shorter edge
            std::vector<uint> evid = d.m.edge_vert_ids(eid);
            result = flip2to2(d.m, eid);
            //update the map
            if(d.map && d.step > 0 && result) {
                result = flip2to2(d.mm, d.mm.edge_id(evid[0], evid[1]));
                if(!result) result = flip2to2(d.m, d.m.edge_id(vid0, vid1));
                my_assert(d, result, "Bad restore in try_flips");
            }

            /* UNCOMMENT TO MARK IN RED THE EDGE FLIPPED
            if(result) {
                new_eid = d.m.edge_id(vid0, vid1);
                d.m.edge_data(new_eid).flags[MARKED] = true;
            }
            */

        }

    }
}

bool flip4to4(Tetmesh<> &m, uint eid, uint vid0, uint vid1) {

    std::vector<uint> cluster = m.adj_e2p(eid);
    //sanity checks
    if(cluster.size() != 4) return false;
    if(vid0*vid1 == 0) return false;

    //default
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

bool flip2to2(Tetmesh<> &m, uint eid) {
    uint vid0, vid1;
    return flip2to2(m, eid, vid0, vid1);
}

bool flip2to2(Tetmesh<> &m, uint eid, uint &vid0, uint &vid1) {

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
            vid0 = m.face_vert_opposite_to(fid, eid);
    for(uint fid : m.poly_e2f(pid_of, eid))
        if(m.face_is_on_srf(fid))
            vid1 = m.face_vert_opposite_to(fid, eid);
    opp = vid0;

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

/* ================================================================================================================== */

void smooth(Data &d, int mesh) {

    if(d.verbose && mesh == MODEL) cout << BCYN << "Smoothing the model..." << RST;
    if(d.verbose && mesh == MAP) cout << BCYN << "Smoothing the map..." << RST;
    if(mesh != MODEL && mesh != MAP) assert(false && "Bad enum in smoothing function");

    //parameters
    DrawableTetmesh<> &m = mesh == MODEL ? d.m : d.mm;
    DrawableTrimesh<> &ms = mesh == MODEL ? d.ms : d.mms;
    uint aux;
    double dist;
    vec3d bary, og, norm;
    std::vector<uint> adjs;
    bool srf, can_smooth;

    //get srf
    Octree octree;
    export_surface(m, ms);
    octree.build_from_mesh_polys(ms);

    //internal smoothing
    for(int iter = 0; iter < d.smooth_mesh_iters; iter++) {
        //for every vert
        for(uint vid = 0; vid < m.num_verts(); vid++) {
            //check if the vert is on srf
            srf = m.vert_is_on_srf(vid);
            //skip vids not active
            if(srf && mesh == MODEL && !m.vert_data(vid).flags[ACTIVE]) continue;
            //get adjs (only srf for srf verts)
            if(srf) adjs = m.vert_adj_srf_verts(vid);
            else adjs = m.adj_v2v(vid);
            //get initial barycenter and normal
            og = bary = m.vert(vid);
            norm = m.vert_data(vid).normal;
            //sum
            for(uint adj : adjs) bary += m.vert(adj);
            //avg
            bary = bary / double(adjs.size() + 1);
            //reproject if srf
            if(srf && mesh == MODEL) {
                octree.intersects_ray(bary, norm, dist, aux);
                bary = bary + (norm * dist);
                //bary = octree.closest_point(bary);
            }
            //check if legal move
            for(uint pid : m.adj_v2p(vid)) {
                can_smooth = !does_movement_flip(d, vid, pid, bary, mesh);
                if(!can_smooth) break;
            }
            //move
            if(can_smooth) {
                m.vert(vid) = bary;
                if(mesh == MODEL) line_search(d, vid, og);
            }
        }
    }

    //update all normals
    m.update_normals();
    if(mesh == MAP) m.scale(1/m.bbox().delta_x());

    if(d.verbose) cout << BGRN << "DONE" << rendl;
}

void smooth_pierre(Data &d, int mesh) {

    if(d.verbose && mesh == MODEL) cout << BCYN << "Smoothing [Pierre] the model..." << RST;
    if(d.verbose && mesh == MAP) cout << BCYN << "Smoothing [Pierre] the map..." << RST;
    if(mesh != MODEL && mesh != MAP) assert(false && "Bad enum in smoothing function");

    //parameters
    DrawableTetmesh<> &m = mesh == MODEL ? d.m : d.mm;
    DrawableTrimesh<> &ms = mesh == MODEL ? d.ms : d.mms;
    uint aux;
    bool srf, can_smooth;
    double vol, tvol, dist;
    vec3d center, og, norm;
    std::vector<uint> adjs;
    std::vector<vec3d> verts;

    //get srf
    Octree octree;
    export_surface(m, ms);
    octree.build_from_mesh_polys(ms);

    //smoothing iteration
    for(int iter = 0; iter < d.smooth_mesh_iters; iter++) {
        //for every vert
        for(uint vid = 0; vid < m.num_verts(); vid++) {
            //check if vert is on srf
            srf = m.vert_is_on_srf(vid);
            //skip if inactive
            if(srf && mesh == MODEL && !m.vert_data(vid).flags[ACTIVE]) continue;
            //get adjs (only srf for srf verts)
            if(srf) adjs = m.vert_adj_srf_verts(vid);
            else adjs = m.adj_v2p(vid);
            //get initial barycenter and normal
            og = m.vert(vid);
            norm = m.vert_data(vid).normal;

            //avg
            tvol= 0;
            center = vec3d(0, 0, 0);
            for(uint pid : adjs) {
                vol = m.poly_volume(pid);
                center += tetrahedron_circumcenter(m.poly_vert(pid, 0),
                                                   m.poly_vert(pid, 1),
                                                   m.poly_vert(pid, 2),
                                                   m.poly_vert(pid, 3)) * vol;
                tvol += vol;
            }
            center = center / tvol;
            //re-project if space was lose
            if(srf) {
                octree.intersects_ray(center, norm, dist, aux);
                center = center + (norm * dist);
            }
            //check if legal move
            for(uint pid : m.adj_v2p(vid)) {
                can_smooth = !does_movement_flip(d, vid, pid, center, mesh);
                if(!can_smooth) break;
            }
            //move
            if(can_smooth) {
                m.vert(vid) = center;
                if(mesh == MODEL) line_search(d, vid, og);
            }
        }
    }

    //restore the normals
    m.update_normals();
    if(mesh == MAP) m.scale(1/m.bbox().delta_x());

    if(d.verbose) cout << BGRN << "DONE" << rendl;
}

/* ================================================================================================================== */

//delete inactive verts from the front
void update_front(Data &d) {

    if(d.verbose) cout << BCYN << "Updating the front..." << RST;

    //remove inactive verts from the front
    d.front.erase(std::remove_if(d.front.begin(), d.front.end(), [&](uint vid) { return !d.m.vert_data(vid).flags[ACTIVE]; }), d.front.end());

    if(d.verbose) cout << BGRN << "DONE" << rendl;
}

/* ================================================================================================================== */

//my assert with log
void my_assert(Data &d, bool condition, std::string log, std::string file, int line) {

    if(!condition) {

        //params
        std::string name = d.name;
        int step = d.step;
        int active = d.front.size();
        int verts = d.m.num_verts();
        int srf_verts = d.m.get_surface_verts().size();
        int polys = d.m.num_polys();
        double vol = d.m.mesh_volume();
        int target_verts = d.ts.num_verts();
        double target_vol = d.tv.mesh_volume();
        double percent = vol / target_vol;
        double prev_vol = d.prev->m.mesh_volume();
        double prev_percent = prev_vol / target_vol;

        if(!d.path_log.empty()) {
            //open
            FILE *f = fopen(d.path_log.c_str(), "a");
            //mesh name
            fprintf(f, "\n%s;", name.c_str());
            //step
            fprintf(f, "%d;", step);
            //active fronts
            fprintf(f, "%d;", active);
            //num of verts
            fprintf(f, "%d;", verts);
            //num of srf verts
            fprintf(f, "%d;", srf_verts);
            //num of polys
            fprintf(f, "%d;", polys);
            //mesh volume
            fprintf(f, "%f;", vol);
            //target surface verts
            fprintf(f, "%d;", target_verts);
            //target volume
            fprintf(f, "%f;", target_vol);
            // % of the volume
            fprintf(f, "%f;", percent);
            // vol at prev step
            fprintf(f, "%f;", prev_vol);
            // % at prev step
            fprintf(f, "%f;", prev_percent);
            //msg
            fprintf(f, "%s;", log.c_str());
            //end
            fprintf(f, "%d ", line);
            fprintf(f, "%s", file.c_str());
            //close
            fclose(f);
        }

        cout << BRED << endl << endl;
        cout << "Mesh: " << name << "; " << endl;
        cout << "Step: " << step << "; " << endl;
        cout << "Actives: " << active << "; " << endl;
        cout << "Verts: " << verts << "; " << endl;
        cout << "Srf Verts: " << srf_verts << "; " << endl;
        cout << "Polys: " << polys << "; " << endl;
        cout << "Volume: " << vol << "; " << endl;
        cout << "Target verts: " << target_verts << "; " << endl;
        cout << "Target volume: " << target_vol << "; " << endl;
        cout << "Percent: " << percent << "; " << endl;
        cout << "Log: " << log << "; " << endl;
        cout << "File: " << file << "; " << endl;
        cout << "Line: " << line << "; " << endl;
        cout << rendl;
        //cout << name << ";" << step << ";" << active << ";" << verts << ";" << srf_verts << ";" << polys << ";" << vol << ";" << target_verts << ";" << target_vol << ";" << percent << ";" << log << ";" << file << ";" << line << ";" << endl;
        cout << endl;

        d.running = false;
        if(!d.render) {
            if(percent > 0.9) {
                save(d);
                save(*d.prev);
            }
            exit(98);
        }
    }
}