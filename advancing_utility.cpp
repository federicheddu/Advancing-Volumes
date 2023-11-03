#include "advancing_utility.h"

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

    //laplacian smoothing of the distance -> if flat is true, the distance is the average of the distance of the adj verts
    double adj_dist = 0, dist_sum = dist;
    if(flat) {
        for(uint adj_vid : d.m.vert_adj_srf_verts(vid)) {
            if (raycast)
                d.oct.intersects_ray(d.m.vert(adj_vid), d.m.vert_data(adj_vid).normal, adj_dist, id);
            else
                adj_dist = d.oct.closest_point(d.m.vert(adj_vid)).dist(d.m.vert(adj_vid));

            dist_sum += adj_dist;
        }
        dist = dist_sum / (double)(d.m.vert_adj_srf_verts(vid).size() + 1);
    }

    return dist;
}

//check if the movement of the vert is safe, if not it goes back by bisection
bool go_back_safe(Data &d, uint vid, const vec3d &og_pos) {

    int iter_counter;
    int max_iter = 7;
    bool check = true;

    //check for intersections (only if on surface)
    if(d.m.vert_is_on_srf(vid)) {
        iter_counter = 0;
        while (check_intersection(d, vid) && iter_counter < max_iter) {
            d.m.vert(vid) -= (d.m.vert(vid) - og_pos) / 2;
            iter_counter++;
        }
        //if edge still outside get back the vid
        if (iter_counter == max_iter && check_intersection(d, vid)) {
            d.m.vert(vid) = og_pos;
            //std::cout << TXT_BOLDRED << "The vert " << vid << " failed the line search for intersection" << TXT_RESET << std::endl;
            check = false;
        }
    }

    //if it is not already putted back to the og pos
    //check the volume with orient3D
    if(!(d.m.vert(vid) == og_pos)) {
        iter_counter = 0;
        while(check_volume(d, vid) && iter_counter < max_iter) {
            d.m.vert(vid) -= (d.m.vert(vid) - og_pos) / 2;
            iter_counter++;
        }
        if(iter_counter == max_iter && check_volume(d, vid)) {
            d.m.vert(vid) = og_pos;
            //std::cout << TXT_BOLDRED << "The vert " << vid << " failed the line search for volume sign" << TXT_RESET << std::endl;
            check = false;
        }
    }

    return check;
}

//check if there are any intersection with the target mesh
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