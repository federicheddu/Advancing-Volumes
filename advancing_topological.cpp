#include "advancing_topological.h"

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

        if(d.rationals) {
            //get rational split point
            uint vid_left = d.m.edge_vert_id(eid, 0);
            uint vid_right = d.m.edge_vert_id(eid, 1);
            CGAL_Q rmp[3]; //rational midpoint
            midpoint(&d.exact_coords[3 * vid_left], &d.exact_coords[3 * vid_right], rmp);

            //split
            if(d.render || d.m.edge_is_on_srf(eid)) { //update floating point only if rendering or its on srf (need for octree)
                vec3d fmp = vec3d(CGAL::to_double(rmp[0]),
                                  CGAL::to_double(rmp[1]),
                                  CGAL::to_double(rmp[2]));
                new_vid = d.m.edge_split(eid, fmp);
            } else {
                new_vid = d.m.edge_split(eid);
            }

            //update rational coords
            d.exact_coords.push_back(rmp[0]);
            d.exact_coords.push_back(rmp[1]);
            d.exact_coords.push_back(rmp[2]);
        } else {
            //split
            new_vid = d.m.edge_split(eid);
        }

        //update the map
        v_map.emplace(og_edge,new_vid);

        //set the activity
        d.m.vert_data(new_vid).label = false;
        d.m.vert_data(new_vid).uvw[MOV] = 0;
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

    uint vid_left, vid_right;
    vec3d fmp;
    CGAL_Q rmp[3]; //rational midpoint

    //split all the edges
    for(uint eid : edges_to_split) {
        //get edge vid
        vid_left = d.m.edge_vert_id(eid, 0);
        vid_right = d.m.edge_vert_id(eid, 1);
        //rational midpoint
        midpoint(&d.exact_coords[3*vid_left], &d.exact_coords[3*vid_right], rmp);
        //if rendering is on update the floating point
        if(d.render) fmp = vec3d(CGAL::to_double(rmp[0]), CGAL::to_double(rmp[1]), CGAL::to_double(rmp[2]));
        d.exact_coords.push_back(rmp[0]);
        d.exact_coords.push_back(rmp[1]);
        d.exact_coords.push_back(rmp[2]);
        //if rendering is on use the floating point
        d.render ? d.m.edge_split(eid, fmp) : d.m.edge_split(eid);
    }

}

//flip every edge passed as parameter in the list
void flip(Data &d, std::map<ipair, uint> &v_map, std::queue<edge_to_flip> &edges_to_flip) {

    //std::cout << "Flipping" << std::endl;

    std::map<ipair, uint> try_again;
    while(!edges_to_flip.empty()) {

        //retrieve the info
        edge_to_flip etf = edges_to_flip.front();
        uint vid = v_map.at(etf.og_edge);
        uint eid = d.m.edge_id(vid, etf.opp_vid);

        //if the edge isn't on the srf we do the flip 4-4
        if(!d.m.edge_is_on_srf(eid) ) {
            //std::cout << "4-4" << std::endl;

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
            //std::cout << "2-2" << std::endl;

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

    //search of the edges to split inside the front
    std::set<uint> edges_to_split = search_split(d, selective);
    //split the edges while keep tracking of the edge/map relationship and the edges that will need to be flipped
    std::map<ipair, uint> v_map;
    std::queue<edge_to_flip> edges_to_flip;
    split(d, edges_to_split, v_map, edges_to_flip);

    // flip every edge we need to
    flip(d, v_map, edges_to_flip);

    //split_separating_simplices(d.m);

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

// ==================== TOPOLOGICAL UNLOCK ====================

void topological_unlock(Data &d, uint vid, CGAL_Q *moved, CGAL_Q *move) {

    bool blocking; //if true restart the for + unlock by edge split

    for(uint i = 0; i < d.m.adj_v2p(vid).size(); blocking ? i = 0 : i++) {
        uint pid = d.m.adj_v2p(vid)[i];
        blocking = tet_is_blocking(d, vid, pid, moved);
        if(blocking) {
            CGAL_Q unlock_pos[3] = {moved[0] + move[0] / 2,
                                    moved[1] + move[1] / 2,
                                    moved[2] + move[2] / 2};
            unlock_by_edge_split(d, pid, vid, unlock_pos);
        }
        if(!d.running) {
            assert(d.render); //if not rendering end the program
            vec3d flt_moved = vec3d(CGAL::to_double(moved[0]), CGAL::to_double(moved[1]), CGAL::to_double(moved[2]));
            d.gui->push_marker(d.m.vert(vid), "og", Color::BLUE(), 2, 4);
            d.gui->push_marker(flt_moved, "new", Color::BLUE(), 2, 4);
            d.running = false;
            return;
        }
    }

}

bool tet_is_blocking(Data &data, const uint vid, const uint pid, CGAL_Q *target)
{
    assert(data.m.poly_contains_vert(pid,vid));

    uint f_opp = data.m.poly_face_opposite_to(pid,vid);
    uint v0 = data.m.face_vert_id(f_opp,0);
    uint v1 = data.m.face_vert_id(f_opp,1);
    uint v2 = data.m.face_vert_id(f_opp,2);
    if(data.m.poly_face_is_CCW(pid,f_opp)) std::swap(v0,v1);

    // it the positive half space of the edge opposite to front_vert
    // does not contain the new_pos, the triangle is blocking
    if(orient3d(&data.exact_coords[3*v0],
                &data.exact_coords[3*v1],
                &data.exact_coords[3*v2],
                target)>=0) return true;
    return false;
}

void unlock_by_edge_split(Data &d, const uint pid, const uint vid, CGAL_Q *target) {
    
    assert(d.m.poly_contains_vert(pid,vid));

    // get how many faces are blocking the movement
    std::vector<uint> see_target;
    for(uint fid : d.m.adj_p2f(pid))
    {
        auto vids = d.m.face_verts_id(fid);
        if(d.m.poly_face_is_CW(pid,fid)) std::swap(vids[0],vids[1]);
        if(orient3d(&d.exact_coords[3*vids[0]],
                    &d.exact_coords[3*vids[1]],
                    &d.exact_coords[3*vids[2]],
                    target)<0) see_target.push_back(fid);
    }

    if(see_target.size() == 3)
        unlock_see3(d, vid, pid, target, see_target);
    else if(see_target.size() == 2)
        unlock_see2(d, vid, pid, target, see_target);
    else
        unlock_see1(d, vid, pid, target, see_target);

}

void unlock_see3(Data &d, uint vid, uint pid, CGAL_Q *exact_target, std::vector<uint> &see_target) {

    if(d.ultra_verbose) std::cout << std::endl << "Unlock by edge split [vid: " << vid << ", case 3]" << std::endl;
    if (see_target.size() != 3) std::cout << "see_target.size(): " << see_target.size() << std::endl;
    assert(see_target.size()==3);

    int f_hid = -1;
    for(uint fid : d.m.adj_p2f(pid)) {
        if(does_movement_flip(d, pid, fid, exact_target)) {
            assert(f_hid==-1);
            f_hid = fid;
        }
    }

    //error check -- this should never happen
    if(f_hid==-1) {
        assert(d.render); //if !d.render the program ends
        std::cout << "f_hid: " << f_hid << std::endl;
        std::cout << "see_target: ";
        for(auto fid : see_target) std::cout << fid << " ";
        std::cout << std::endl;
        std::cout << "pid: " << pid << std::endl;
        d.m.poly_data(pid).color = Color::RED();
        d.m.updateGL();
        d.running = false;
        return;
    }

    CGAL_Q P[3];
    plane_line_intersection(&d.exact_coords[3*d.m.face_vert_id(f_hid,0)],
                            &d.exact_coords[3*d.m.face_vert_id(f_hid,1)],
                            &d.exact_coords[3*d.m.face_vert_id(f_hid,2)],
                            &d.exact_coords[3*d.m.poly_vert_opposite_to(pid,f_hid)],
                            exact_target, P);

    uint num_polys = d.m.num_polys(); // need to know this before the split -- only for ultraverbose

    uint vfresh = d.m.face_split(f_hid);
    d.exact_coords.push_back(P[0]);
    d.exact_coords.push_back(P[1]);
    d.exact_coords.push_back(P[2]);
    if(d.render || d.m.vert_is_on_srf(vid))
        d.m.vert(vfresh) = vec3d(CGAL::to_double(P[0]),CGAL::to_double(P[1]),CGAL::to_double(P[2]));
    if(d.enable_snap_rounding)
        snap_rounding(d, vfresh);

    // ultra verbose -- give pid of the new polys
    if(d.ultra_verbose) {
        std::cout << "new poly: ";
        for (uint pid: d.m.adj_v2p(vfresh))
            if (pid >= num_polys) std::cout << pid << " ";
        std::cout << std::endl;
    }
    // debug -- color the new polys
    if(d.debug_colors) {
        for (uint pid: d.m.adj_v2p(vfresh))
            if (pid >= num_polys)
                d.m.poly_data(pid).color = Color::MAGENTA();
    }

    for(uint pid : d.m.adj_v2p(vid)) {
        assert(orient3d(&d.exact_coords[d.m.poly_vert_id(pid, 0)*3],
                        &d.exact_coords[d.m.poly_vert_id(pid, 1)*3],
                        &d.exact_coords[d.m.poly_vert_id(pid, 2)*3],
                        &d.exact_coords[d.m.poly_vert_id(pid, 3)*3]) < 0);
    }
    for(uint pid : d.m.adj_v2p(vfresh)) {
        assert(orient3d(&d.exact_coords[d.m.poly_vert_id(pid, 0)*3],
                        &d.exact_coords[d.m.poly_vert_id(pid, 1)*3],
                       &d.exact_coords[d.m.poly_vert_id(pid, 2)*3],
                       &d.exact_coords[d.m.poly_vert_id(pid, 3)*3]) < 0);
    }
}

void unlock_see2(Data &d, uint vid, uint pid, CGAL_Q *exact_target, std::vector<uint> &see_target) {

    if(d.ultra_verbose) std::cout << std::endl << "Unlock by edge split [vid: " << vid << ", case 2]" << std::endl;

    uint f0 = see_target[0];
    uint f1 = see_target[1];
    assert(f0!=f1);
    assert(d.m.poly_contains_face(pid,f0));
    assert(d.m.poly_contains_face(pid,f1));
    // I think this is the only configuration possible
    assert(d.m.face_contains_vert(f0,vid) ||
           d.m.face_contains_vert(f1,vid));

    uint eid = d.m.face_shared_edge(f0,f1);
    uint opp = d.m.poly_edge_opposite_to(pid,eid);
    uint v0  = d.m.edge_vert_id(eid,0);
    uint v1  = d.m.edge_vert_id(eid,1);
    uint v2  = d.m.edge_vert_id(opp,0);
    uint v3  = d.m.edge_vert_id(opp,1);

    CGAL_Q P[3];
    plane_line_intersection(&d.exact_coords[3*v0],
                            &d.exact_coords[3*v1],
                            exact_target,
                            &d.exact_coords[3*v2],
                            &d.exact_coords[3*v3],P);
    // makes sure P is in between v2 and v3
    assert(orient3d(&d.exact_coords[3*v0],&d.exact_coords[3*v1],P,&d.exact_coords[3*v2])*
           orient3d(&d.exact_coords[3*v0],&d.exact_coords[3*v1],P,&d.exact_coords[3*v3])<0);

    uint num_polys = d.m.num_polys(); // need to know this before the split -- only for ultraverbose & debug

    uint new_vid = d.m.edge_split(opp);
    assert(new_vid==d.exact_coords.size()/3);
    d.exact_coords.push_back(P[0]);
    d.exact_coords.push_back(P[1]);
    d.exact_coords.push_back(P[2]);
    if(d.render || d.m.vert_is_on_srf(new_vid))
        d.m.vert(new_vid) = vec3d(CGAL::to_double(P[0]),CGAL::to_double(P[1]),CGAL::to_double(P[2]));

    // ultra verbose -- give pid of the new polys
    if(d.ultra_verbose) {
        std::cout << "new poly: ";
        for (uint pid : d.m.adj_v2p(new_vid))
            if (pid >= num_polys) std::cout << pid << " ";
        std::cout << std::endl;
    }
    // debug -- color the new polys
    if(d.debug_colors) {
        for (uint pid : d.m.adj_v2p(new_vid))
            if (pid >= num_polys)
                d.m.poly_data(pid).color = Color::MAGENTA();
    }

    for(uint pid : d.m.adj_v2p(vid)) {
        assert(orient3d(&d.exact_coords[d.m.poly_vert_id(pid, 0)*3],
                        &d.exact_coords[d.m.poly_vert_id(pid, 1)*3],
                        &d.exact_coords[d.m.poly_vert_id(pid, 2)*3],
                        &d.exact_coords[d.m.poly_vert_id(pid, 3)*3]) < 0);
    }
    for(uint pid : d.m.adj_v2p(new_vid)) {
        assert(orient3d(&d.exact_coords[d.m.poly_vert_id(pid, 0)*3],
                        &d.exact_coords[d.m.poly_vert_id(pid, 1)*3],
                        &d.exact_coords[d.m.poly_vert_id(pid, 2)*3],
                        &d.exact_coords[d.m.poly_vert_id(pid, 3)*3]) < 0);
    }
}

void unlock_see1(Data &d, uint vid, uint pid, CGAL_Q *exact_target, std::vector<uint> &see_target) {

    std::cout << TXT_BOLDRED << std::endl;
    std::cout << "you should not be here" << std::endl;
    std::cout << "unlock_by_edge_split: " << see_target.size() << std::endl;

    std::cout << "vid: " << vid << " - fid seen: " << see_target[0] << std::endl;

    std::cout << "pid blocking: " << pid << std::endl;

    std::cout << "pid adj face: " << TXT_RESET;
    std::cout << TXT_BOLDCYAN << d.m.adj_f2p(see_target[0])[0] << " " << TXT_RESET;
    d.m.poly_data(d.m.adj_f2p(see_target[0])[0]).color = Color::CYAN();
    std::cout << TXT_BOLDGREEN << d.m.adj_f2p(see_target[0])[1] << TXT_RESET;
    d.m.poly_data(d.m.adj_f2p(see_target[0])[1]).color = Color::GREEN();

    std::cout << std::endl << std::endl;

    std::vector<uint> pos, neg, nul;
    for(uint fid : d.m.adj_p2f(pid))
    {
        auto vids = d.m.face_verts_id(fid);
        if(d.m.poly_face_is_CW(pid,fid)) std::swap(vids[0],vids[1]);
        if(orient3d(&d.exact_coords[3*vids[0]],
                    &d.exact_coords[3*vids[1]],
                    &d.exact_coords[3*vids[2]],
                    exact_target) == 0) nul.push_back(fid);
        if(orient3d(&d.exact_coords[3*vids[0]],
                    &d.exact_coords[3*vids[1]],
                    &d.exact_coords[3*vids[2]],
                    exact_target) > 0) pos.push_back(fid);
        if(orient3d(&d.exact_coords[3*vids[0]],
                    &d.exact_coords[3*vids[1]],
                    &d.exact_coords[3*vids[2]],
                    exact_target) < 0) neg.push_back(fid);
    }

    if(!nul.empty()) {
        std::cout << "Ci sono pid collassati" << std::endl;
        for(auto i : see_target)
            std::cout << "Fid:" << i << " - pid: " << d.m.adj_f2p(i)[0] << " - " << d.m.adj_f2p(i)[1] << std::endl;
    }
    std::cout << std::endl << std::endl;

    std::cout << "Ci sono " << pos.size() << " positivi" << std::endl;
    std::cout << "Ci sono " << neg.size() << " negativi" << std::endl;
    std::cout << "Ci sono " << nul.size() << " nulli" << std::endl;

    std::cout << std::endl << std::endl;

    for(int pid = 0; pid < d.m.num_polys(); pid++) {
        if(orient3d(&d.exact_coords[d.m.poly_vert_id(pid, 0)*3],
                    &d.exact_coords[d.m.poly_vert_id(pid, 1)*3],
                    &d.exact_coords[d.m.poly_vert_id(pid, 2)*3],
                    &d.exact_coords[d.m.poly_vert_id(pid, 3)*3]) == 0)
            std::cout << TXT_BOLDRED << "Ci sono pid collassati (" << pid << ")" << std::endl;
    }

    assert(d.render); //if !d.render the program ends
    d.m.updateGL();
    d.running = false;
}