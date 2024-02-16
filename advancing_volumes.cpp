#include "advancing_volumes.h"

void advancing_volume(Data &d) {

    d.step++;
    cout << BMAG << "Advancing Volume ITERATION " << d.step << RESET << endl;

    if(d.running) expand(d);
    //if(d.running) refine(d);
    //if(d.running) smooth(d);

    cout << BMAG << "Advancing Volume ITERATION " << BGRN << "DONE" << RESET << endl;

}

void expand(Data &d) {

    if(d.verbose) cout << BCYN << "Expanding model..." << RESET;
    if(d.render) d.gui->pop_all_markers();

    //compute DDD: direction, distance and displacement -> saved into .vert_data(vid).normal
    //compute_directions(d);
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
    /* TODO: update fronts here */
    check_self_intersection(d);
    if(!d.running) return;

    if(d.verbose) cout << BGRN << "DONE" << RESET << endl;
}

void move(Data &d, uint vid, vec3d &disp) {
    CGAL_Q rt_disp[3];
    to_rational(disp, rt_disp);
    move(d, vid, rt_disp);
}

void move(Data &d, uint vid, CGAL_Q *rt_disp) {

    bool moved = true;

    //backup
    CGAL_Q rt_safe_pos[3];
    copy(&d.rationals[3*vid], rt_safe_pos);

    //new position of the vert
    CGAL_Q rt_moved[3] = {d.rationals[3*vid+0] + rt_disp[0],
                          d.rationals[3*vid+1] + rt_disp[1],
                          d.rationals[3*vid+2] + rt_disp[2]};

    /* TODO: topological unlock */

    //update the vert
    d.m.vert(vid) = to_double(rt_moved);
    copy(rt_moved, &d.rationals[3*vid]);

    line_search(d, vid, rt_safe_pos);
    if(!d.running) return;

    /* TODO: snap rounding */

    //update the active flag
    if(dist_calc(d, vid, true) < d.inactivity_dist) d.m.vert_data(vid).flags[ACTIVE] = false;

}

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
            direction /= (d.m.vert_adj_srf_verts(vid).size() + 1);
            direction.normalize();
            //update after smoothing
            d.m.vert_data(vid).normal = direction;
        }
    }

}

void compute_distances(Data &d) {

    double dist;
    for(uint vid : d.front) {
        dist = dist_calc(d, vid, d.only_raycast);
        if(!d.only_raycast && dist < d.switch_raycast_threshold)
            dist = dist_calc(d, vid, true);
        d.m.vert_data(vid).uvw[DIST] = dist;
    }

}

void compute_displacements(Data &d) {

    for(uint vid : d.front)
        d.m.vert_data(vid).normal *= d.m.vert_data(vid).uvw[DIST] * 0.99;

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

void check_self_intersection(Data &d) {

    if(d.verbose) cout << BCYN << "Checking self intersection..." << RESET;
    std::set<ipair> intersections;

    export_surface(d.m, d.ms);
    find_intersections(d.ms, intersections);
    d.running = intersections.empty();

    //feedback
    if(!d.running) cout << BRED << "FOUND" << RESET << endl;

}