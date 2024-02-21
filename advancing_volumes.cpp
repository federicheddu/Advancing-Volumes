#include "advancing_volumes.h"
#undef NDEBUG

//main function
void advancing_volume(Data &d) {

    d.step++;
    d.unlock2_count = 0;
    d.unlock3_count = 0;
    cout << BMAG << "Advancing Volume ITERATION " << d.step << RST << endl;

    if(d.running) expand(d);
    if(d.running) refine(d);
    //if(d.running) smooth(d);
    update_front(d);

    cout << BMAG << "Advancing Volume ITERATION - Actives: " << d.front.size() << "/" << d.m.num_srf_verts() << RST << endl;
    cout << BMAG << "Advancing Volume ITERATION - Unlock2: " << d.unlock2_count << RST << endl;
    cout << BMAG << "Advancing Volume ITERATION - Unlock3: " << d.unlock3_count << RST << endl;

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
    CGAL_Q rt_disp[3];
    to_rational(disp, rt_disp);
    move(d, vid, rt_disp);
}

//move the vert in the direction of the displacement
void move(Data &d, uint vid, CGAL_Q *rt_disp) {

    bool moved = true;

    //backup
    CGAL_Q rt_safe_pos[3];
    copy(&d.rationals[3*vid], rt_safe_pos);

    //new position of the vert
    CGAL_Q rt_moved[3] = {d.rationals[3*vid+0] + rt_disp[0],
                          d.rationals[3*vid+1] + rt_disp[1],
                          d.rationals[3*vid+2] + rt_disp[2]};

    //refine (if needed) to move where I want
    topological_unlock(d, vid, rt_moved, rt_disp);
    if(!d.running) return;

    //update the vert
    d.m.vert(vid) = to_double(rt_moved);
    copy(rt_moved, &d.rationals[3*vid]);

    line_search(d, vid, rt_safe_pos);
    if(!d.running) return;

    /* TODO: snap rounding */

    //update the active flag
    if(dist_calc(d, vid, true) < d.inactivity_dist) d.m.vert_data(vid).flags[ACTIVE] = false;

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
            //dist = dist_calc(d, vid, true);
            dist = d.target_edge_length;
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
void line_search(Data &d, uint vid, CGAL_Q *safe_pos) {

    int bck_cnt = 0, frd_cnt = 0;
    CGAL_Q mid[3];
    CGAL_Q final[3];

    //check only if surface
    if(!d.m.vert_is_on_srf(vid)) return;

    //backup the current position
    copy(&d.rationals[3*vid], final);

    //backward bisection
    while(bck_cnt < d.line_search_max && check_intersection(d, vid)) {
        //backward
        midpoint(&d.rationals[3*vid], safe_pos, mid);
        copy(mid, &d.rationals[3*vid]);
        d.m.vert(vid) = to_double(mid);
        //counter
        bck_cnt++;
    }

    //forward bisection
    while(frd_cnt < d.line_search_max && !check_intersection(d, vid)) {
        //update safe position
        copy(&d.rationals[3*vid], safe_pos);
        //forward
        midpoint(&d.rationals[3*vid], final, mid);
        copy(mid, &d.rationals[3*vid]);
        d.m.vert(vid) = to_double(mid);
        //counter
        frd_cnt++;
    }

    //go to the last safe position
    if(check_intersection(d, vid)) {
        copy(safe_pos, &d.rationals[3 * vid]);
        d.m.vert(vid) = to_double(safe_pos);
    }

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

    export_surface(d.m, d.ms);
    find_intersections(d.ms, intersections);
    d.running = intersections.empty();

    //feedback
    if(!d.running) cout << BRED << "FOUND" << RST << endl;

}

/* ================================================================================================================== */

void topological_unlock(Data &d, uint vid, CGAL_Q *rt_moved, CGAL_Q *rt_disp) {

    bool blocking = false;
    std::vector<uint> adj = d.m.adj_v2p(vid);
    uint n_adj = adj.size();
    uint pid;
    uint limit = n_adj * n_adj * n_adj;

    //check for every tet adj to the vid if its blocking
    CGAL_Q unlock_pos[3];
    for(uint i = 0; i < n_adj; i++) {

        //get the pid and check if any tet will block the movement
        pid = adj[i];
        blocking = will_poly_flip(d, vid, pid, rt_moved);

        if(blocking) {
            //push forward the unlock_pos to unlock
            unlock_pos[0] = rt_moved[0] + rt_disp[0] / 2;
            unlock_pos[1] = rt_moved[1] + rt_disp[1] / 2;
            unlock_pos[2] = rt_moved[2] + rt_disp[2] / 2;
            unlock(d, vid, pid, unlock_pos);
            //restart the for
            i = 0;
            adj = d.m.adj_v2p(vid);
            n_adj = adj.size();
            assert(adj.size() < limit);
        }

        //visual debug
        if(!d.running && d.render) {
            vec3d fp_moved = to_double(rt_moved);
            d.gui->push_marker(d.m.vert(vid), "og", Color::BLUE(), 2, 4);
            d.gui->push_marker(fp_moved, "new", Color::BLUE(), 2, 4);
            d.running = false;
            return;
        }

    }

}

void unlock(Data &d, const uint vid, const uint pid, CGAL_Q *target) {

    CGAL_Q orient;
    std::vector<uint> vids, see_target;

    //get how many faces are blocking the movement
    for(uint fid : d.m.adj_p2f(pid))
        if(!will_face_flip(d, pid, fid, target)) see_target.push_back(fid);

    switch (see_target.size()) {
        case 3:
            unlock_see3(d, vid, pid, target, see_target);
            break;
        case 2:
            unlock_see2(d, vid, pid, target, see_target);
            break;
        case 1:
            unlock_see1(d, vid, pid, target, see_target);
            break;
        default:
            cout << BRED << "This is not meant to happen" << rendl;
            break;

    }
}

void unlock_see3(Data &d, uint vid, uint pid, CGAL_Q *target, std::vector<uint> &see_target) {

    //stats
    d.unlock3_count++;

    //sanity checks
    if (see_target.size() != 3) cout << "see_target.size(): " << see_target.size() << endl;
    assert(see_target.size()==3);
    std::vector<uint> p_vids = d.m.poly_verts_id(pid);
    assert(orient3d(&d.rationals[p_vids[0]*3], &d.rationals[p_vids[1]*3], &d.rationals[p_vids[2]*3], &d.rationals[p_vids[3]*3]) * d.orient_sign > 0);

    int f_hid = -1;
    for(uint fid : d.m.adj_p2f(pid)) {
        if(will_face_flip(d, pid, fid, target)) {
            assert(f_hid==-1);
            f_hid = fid;
        }
    }

    //error check -- this should never happen
    if(f_hid==-1) {
        cout << "f_hid: " << f_hid << endl;
        cout << "see_target: ";
        for(auto fid : see_target) cout << fid << " ";
        cout << endl;
        cout << "pid: " << pid << endl;
        d.m.poly_data(pid).color = Color::RED();
        d.m.updateGL();
        d.running = false;
        return;
    }

    //get the intersection point
    CGAL_Q P[3];
    std::vector<uint> f_vids = d.m.face_verts_id(f_hid);
    plane_line_intersection(&d.rationals[3*f_vids[0]],&d.rationals[3*f_vids[1]],&d.rationals[3*f_vids[2]],&d.rationals[3*d.m.poly_vert_opposite_to(pid,f_hid)],target, P);

    //split the face & update the coords
    uint new_vid = d.m.face_split(f_hid);
    d.rationals.push_back(P[0]);
    d.rationals.push_back(P[1]);
    d.rationals.push_back(P[2]);
    d.m.vert(new_vid) = to_double(P);

    //set activity flag
    if(d.m.vert_is_on_srf(new_vid)) {
        d.m.vert_data(new_vid).flags[ACTIVE] = dist_calc(d, new_vid, true) > d.inactivity_dist;
        if(d.m.vert_data(new_vid).flags[ACTIVE]) d.front.push_back(new_vid);
    }

    //mark how vert was created
    d.m.vert_data(new_vid).flags[SPLIT] = false;
    d.m.vert_data(new_vid).flags[TOPOLOGICAL] = true;

    for(uint ppid : d.m.adj_v2p(vid)) {
        std::vector<uint> vids = d.m.poly_verts_id(ppid);
        assert(orient3d(&d.rationals[vids[0]*3], &d.rationals[vids[1]*3], &d.rationals[vids[2]*3], &d.rationals[vids[3]*3]) * d.orient_sign > 0);
    }
    for(uint ppid : d.m.adj_v2p(new_vid)) {
        std::vector<uint> vids = d.m.poly_verts_id(ppid);
        assert(orient3d(&d.rationals[vids[0]*3], &d.rationals[vids[1]*3], &d.rationals[vids[2]*3], &d.rationals[vids[3]*3]) * d.orient_sign > 0);
    }
}

void unlock_see2(Data &d, uint vid, uint pid, CGAL_Q *target, std::vector<uint> &see_target) {

    //stats
    d.unlock2_count++;
    //sanity checks
    std::vector<uint> p_vids = d.m.poly_verts_id(pid);
    assert(orient3d(&d.rationals[p_vids[0]*3], &d.rationals[p_vids[1]*3], &d.rationals[p_vids[2]*3], &d.rationals[p_vids[3]*3]) * d.orient_sign > 0);
    uint f0 = see_target[0];
    uint f1 = see_target[1];
    assert(f0!=f1);
    assert(d.m.poly_contains_face(pid,f0));
    assert(d.m.poly_contains_face(pid,f1));
    assert(d.m.face_contains_vert(f0,vid) || d.m.face_contains_vert(f1,vid));

    uint eid = d.m.face_shared_edge(f0,f1);
    uint opp = d.m.poly_edge_opposite_to(pid,eid);
    uint v0  = d.m.edge_vert_id(eid,0);
    uint v1  = d.m.edge_vert_id(eid,1);
    uint v2  = d.m.edge_vert_id(opp,0);
    uint v3  = d.m.edge_vert_id(opp,1);

    CGAL_Q P[3];
    plane_line_intersection(&d.rationals[3*v0],&d.rationals[3*v1],target,&d.rationals[3*v2],&d.rationals[3*v3],P);
    // makes sure P is in between v2 and v3
    assert(orient3d(&d.rationals[3*v0],&d.rationals[3*v1],P,&d.rationals[3*v2])*
           orient3d(&d.rationals[3*v0],&d.rationals[3*v1],P,&d.rationals[3*v3])<0);

    uint new_vid = d.m.edge_split(opp);
    d.rationals.push_back(P[0]);
    d.rationals.push_back(P[1]);
    d.rationals.push_back(P[2]);
    d.m.vert(new_vid) = to_double(P);

    //set activity flag
    if(d.m.vert_is_on_srf(new_vid)) {
        d.m.vert_data(new_vid).flags[ACTIVE] = dist_calc(d, new_vid, true) > d.inactivity_dist;
        if(d.m.vert_data(new_vid).flags[ACTIVE]) d.front.push_back(new_vid);
    }

    //mark how vert was created
    d.m.vert_data(new_vid).flags[SPLIT] = false;
    d.m.vert_data(new_vid).flags[TOPOLOGICAL] = true;

    //sanity checks
    for(uint ppid : d.m.adj_v2p(vid)) {
        std::vector<uint> vids = d.m.poly_verts_id(ppid);
        assert(orient3d(&d.rationals[vids[0]*3], &d.rationals[vids[1]*3], &d.rationals[vids[2]*3], &d.rationals[vids[3]*3]) * d.orient_sign > 0);
    }
    for(uint ppid : d.m.adj_v2p(new_vid)) {
        std::vector<uint> vids = d.m.poly_verts_id(ppid);
        assert(orient3d(&d.rationals[vids[0]*3], &d.rationals[vids[1]*3], &d.rationals[vids[2]*3], &d.rationals[vids[3]*3]) * d.orient_sign > 0);
    }
}

void unlock_see1(Data &d, uint vid, uint pid, CGAL_Q *target, std::vector<uint> &see_target) {
    CGAL_Q orient = orient3d(&d.rationals[d.m.face_vert_id(see_target[0],0)*3],
                             &d.rationals[d.m.face_vert_id(see_target[0],1)*3],
                             &d.rationals[d.m.face_vert_id(see_target[0],2)*3],
                             target);

    cout << BRED << endl;
    cout << "you should not be here" << endl;
    cout << "unlock_by_edge_split: " << see_target.size() << endl;
    cout << "vid: " << vid << " - fid seen: " << see_target[0] << endl;
    cout << "orient: ";
    cout << orient.exact().numerator() << "/" << orient.exact().denominator() << endl;
    cout << "pid blocking: " << pid << endl;
    cout << "pid adj face: " << RST;

    d.m.poly_data(d.m.adj_f2p(see_target[0])[0]).color = Color::CYAN();
    d.m.poly_data(d.m.adj_f2p(see_target[0])[1]).color = Color::GREEN();

    cout << endl << endl;

    d.m.updateGL();
    d.running = false;
}

/* ================================================================================================================== */

void refine(Data &d) {

    if(d.verbose) cout << BCYN << "Refining model..." << RST;
    split(d);

    if(d.verbose) cout << BCYN << "Adjusting the topology..." << RST;
    flip(d);

    if(d.verbose && d.running) cout << BGRN << "DONE" << RST << endl;

    //free memory
    d.ets.clear();
    d.v_map.clear();
    assert(d.etf.empty());
}

void edges_to_split(Data &d) {

    for(uint vid : d.front)
        for(uint eid : d.m.vert_adj_srf_edges(vid))
            if(d.m.edge_length(eid) > d.target_edge_length)
                for(uint fid : d.m.edge_adj_srf_faces(eid))
                    d.ets.insert(d.m.adj_p2e(d.m.adj_f2p(fid)[0]).begin(), d.m.adj_p2e(d.m.adj_f2p(fid)[0]).end());

}

void split(Data &d) {

    //search for edges to split
    d.ets.clear();
    edges_to_split(d);

    //initial number of verts
    uint nv = d.m.num_verts();

    //parameters
    uint new_vid;
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

        //get the midpoint where split
        CGAL_Q rmid[3];
        uint vid_left = d.m.edge_vert_id(eid, 0);
        uint vid_right = d.m.edge_vert_id(eid, 1);
        midpoint(&d.rationals[3*vid_left], &d.rationals[3*vid_right], rmid);

        //split
        vec3d fmid = to_double(rmid);
        new_vid = d.m.edge_split(eid, fmid);

        //rationals coords
        d.rationals.push_back(rmid[0]);
        d.rationals.push_back(rmid[1]);
        d.rationals.push_back(rmid[2]);

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

        } else if(d.m.vert_data(etf.opp_vid).flags[ACTIVE]) {
            //flip (no query needed, i can have only one setup)
            result = flip2to2(d.m, eid);
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

/* ================================================================================================================== */

void smooth(Data &d) {

    if(d.verbose) cout << BCYN << "Smoothing the model..." << rendl;

    //parameters
    CGAL_Q rt_bary[3];
    std::vector<uint> adjs;
    size_t adjs_size;
    //surface

    //for the number of iteration selected
    for(int iter = 0; iter < d.smooth_mesh_iters; iter++) {

        Octree oct_srf;
        export_surface(d.m, d.ms);
        mesh_smoother(d.ms, d.ms);
        oct_srf.build_from_mesh_polys(d.ms);

        for(uint vid : d.m.get_surface_verts()) {

            vec3d pos = oct_srf.closest_point(d.m.vert(vid));
            CGAL_Q rt_pos[3];
            to_rational(pos, rt_pos);

            //diplacement
            rt_pos[0] = d.rationals[3*vid+0] - rt_pos[0];
            rt_pos[1] = d.rationals[3*vid+1] - rt_pos[1];
            rt_pos[2] = d.rationals[3*vid+2] - rt_pos[2];

            //porco dio
            move(d, vid, rt_pos);
            if(!d.running) return;

        }

    }

    if(d.verbose) cout << BGRN << "DONE" << rendl;
}

/* ================================================================================================================== */

//delete inactive verts from the front
void update_front(Data &d) {

    //remove inactive verts from the front
    d.front.erase(std::remove_if(d.front.begin(), d.front.end(), [&](uint vid) { return !d.m.vert_data(vid).flags[ACTIVE]; }), d.front.end());

}