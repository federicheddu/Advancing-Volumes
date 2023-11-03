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

    //data that we need for later
    uint offset = d.m.num_verts();

    //search of the edges to split inside the front
    std::set<uint> edges_to_split = search_split(d, selective);
    //split the edges while keep tracking of the edge/map relationship and the edges that will need to be flipped
    std::map<ipair, uint> v_map;
    std::queue<edge_to_flip> edges_to_flip;
    split(d, edges_to_split, v_map, edges_to_flip);

    // flip every edge we need to
    flip(d, v_map, edges_to_flip);

    split_separating_simplices(d.m);

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