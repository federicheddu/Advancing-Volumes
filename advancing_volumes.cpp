#include "advancing_volumes.h"

void advancing_volume(Data &d) {

    d.step++;
    cout << BMAG << "Advancing Volume ITERATION " << d.step << RESET << endl;

    if(d.running) expand(d);
    if(d.running) refine(d);
    if(d.running) smooth(d);

    cout << BMAG << "Advancing Volume ITERATION " << BGRN << "DONE" << RESET << endl;

}

void expand(Data &d) {

}

void compute_directions(Data &d) {

    vec3d move;
    std::vector<uint> adjs;

    //direction smoothing
    for(int it = 0; it < d.smooth_dir_iters; it++) {
        for(uint vid : d.front) {
            //default movement
            move = d.m.vert_data(vid).normal;
            //avg
            adjs = d.m.vert_adj_srf_verts(vid);
            for(uint adj : adjs)
                move += d.m.vert_data(adj).normal;
            move /= double(adjs.size()+1);
            move.normalize();
            //update after smoothing
            d.m.vert_data(vid).normal = move;
        }
    }

}

void compute_distances(Data &d) {

    double dist;
    for(uint vid : d.front) {
        dist = dist_calc(d, vid, d.only_raycast);
        if(!d.only_raycast && dist < d.inactivity_dist < d.switch_raycast_threshold)
            dist = dist_calc(d, vid, true);
        d.m.vert_data(vid).uvw[DIST] = dist;
    }

}

void compute_displacement(Data &d) {
    for(uint vid : d.front)
        d.m.vert_data(vid).normal = d.m.vert_data(vid).normal * d.m.vert_data(vid).uvw[DIST] * 0.99;
}

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
    std::unordered_set<uint> intersect_ids;
    bool flag = false;

    //check the umbrella
    for (uint fid : d.m.vert_adj_srf_faces(vid)) {
        vec3d verts[] = {d.m.face_verts(fid)[0], d.m.face_verts(fid)[1], d.m.face_verts(fid)[2]};
        flag = d.oct->intersects_triangle(verts, false, intersect_ids);
        if(flag) break; //early stop
    }

    return flag;
}