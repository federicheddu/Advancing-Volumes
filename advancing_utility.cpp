#include "advancing_utility.h"

//calc the distance from the target with a raycast hit
double dist_calc(Data &d, uint vid, bool raycast, bool flat) {

    //param
    uint id; //useless
    double dist;

    //raycast hit (normal direction)
    if(raycast)
        d.oct->intersects_ray(d.m.vert(vid), d.m.vert_data(vid).normal, dist, id);
    else
        dist = d.oct->closest_point(d.m.vert(vid)).dist(d.m.vert(vid));

    //laplacian smoothing of the distance -> if flat is true, the distance is the average of the distance of the adj verts
    double adj_dist = 0, dist_sum = dist;
    if(flat) {
        for(uint adj_vid : d.m.vert_adj_srf_verts(vid)) {
            if (raycast)
                d.oct->intersects_ray(d.m.vert(adj_vid), d.m.vert_data(adj_vid).normal, adj_dist, id);
            else
                adj_dist = d.oct->closest_point(d.m.vert(adj_vid)).dist(d.m.vert(adj_vid));

            dist_sum += adj_dist;
        }
        dist = dist_sum / (double)(d.m.vert_adj_srf_verts(vid).size() + 1);
    }

    return dist;
}

double get_dist(Data &d, uint vid) {
    double dist;

    for(int idx = 0; idx < d.fronts_active.size(); idx++) {
        //get the vid
        vid = d.fronts_active.at(idx);
        //get the distance (if the vert is near the target, use the raycast to get the distance)
        dist = dist_calc(d, vid, false);
        if (dist < d.eps_inactive * 2)
            dist = dist_calc(d, vid, true, true);
        else
            dist = dist_calc(d, vid, false, true);
    }

    return dist;
}

//check if the movement of the vert is safe, if not it goes back by bisection
bool go_back_safe(Data &d, uint vid, CGAL_Q *rt_pos) {

    int iter_counter = 0;
    int max_iter = 7;
    bool same_side = true;
    bool check = true;
        CGAL_Q tmp[3];

    //check for intersections (only if on surface)
    if(d.m.vert_is_on_srf(vid)) {
        iter_counter = 0;
        while (check_intersection(d, vid) && iter_counter < max_iter) {
            midpoint(&d.exact_coords[vid * 3], rt_pos, tmp);
            copy(tmp, &d.exact_coords[vid * 3]);
            d.m.vert(vid) = vec3d(CGAL::to_double(d.exact_coords[vid*3+0]),
                                  CGAL::to_double(d.exact_coords[vid*3+1]),
                                  CGAL::to_double(d.exact_coords[vid*3+2]));
            iter_counter++;
        }
        //if edge still outside get back the vid
        if (iter_counter == max_iter && check_intersection(d, vid)) {
            copy(rt_pos, &d.exact_coords[vid * 3]);
            d.m.vert(vid) = vec3d(CGAL::to_double(rt_pos[0]),
                                  CGAL::to_double(rt_pos[1]),
                                  CGAL::to_double(rt_pos[2]));
            //std::cout << TXT_BOLDRED << "The vert " << vid << " failed the line search for intersection" << TXT_RESET << std::endl;
            check = false;
        }
    }

    /** this should not be necessary anymore

    //if it is not already putted back to the og pos
    //check the volume with orient3D
    if(!(d.m.vert(vid) == og_pos)) {
        iter_counter = 0;
        while(vert_flipped(d, vid, og_pos) && iter_counter < max_iter) {
            d.m.vert(vid) -= (d.m.vert(vid) - og_pos) / 2;
            iter_counter++;
        }
        if(iter_counter == max_iter && vert_flipped(d, vid, og_pos)) {
            d.m.vert(vid) = og_pos;
            //std::cout << TXT_BOLDRED << "The vert " << vid << " failed the line search for volume sign" << TXT_RESET << std::endl;
            check = false;
        }
    }

    **/

    return check;
}

//check if the poly is flipped (orient3D < 0)
bool poly_flipped(Data &d, uint pid) {

    bool flipped = false;

    vec3d pa = d.m.poly_vert(pid, 0);
    vec3d pb = d.m.poly_vert(pid, 1);
    vec3d pc = d.m.poly_vert(pid, 2);
    vec3d pd = d.m.poly_vert(pid, 3);

    flipped = orient3d(pa, pb, pc, pd) > 0;

    return flipped;
}

//check if there are any intersection with the target mesh
bool check_intersection(Data &d, uint vid) {

    //params
    std::unordered_set<uint> intersect_ids;
    bool flag = false;

    //check the umbrella
    for (uint fid : d.m.vert_adj_srf_faces(vid)) {
        vec3d verts[] = {d.m.face_verts(fid)[0], d.m.face_verts(fid)[1], d.m.face_verts(fid)[2]};
        flag = d.oct->intersects_triangle(verts, false, intersect_ids) || flag;
    }

    return flag;
}

//check if sign of orient3D is ok
bool vert_flipped(Data &d, uint vid, vec3d &og_pos) {

    bool sides[2];
    bool flipped;

    for (uint pid: d.m.adj_v2p(vid)) {
        sides[0] = vert_side(d, d.m.poly_face_opposite_to(pid, vid), og_pos);
        sides[1] = vert_side(d, d.m.poly_face_opposite_to(pid, vid), d.m.vert(vid));
        flipped = sides[0] != sides[1];
        if(flipped) break;
    }

    return flipped;
}

//check if sign of orient3D is ok (with rationals)
bool vert_flipped(Data &d, uint vid, CGAL_Q *og_pos) {
    bool sides[2];
    bool flipped;

    for(uint pid : d.m.adj_v2p(vid)) {
        sides[0] = vert_side(d, d.m.poly_face_opposite_to(pid, vid), og_pos);
        sides[1] = vert_side(d, d.m.poly_face_opposite_to(pid, vid), &d.exact_coords[vid]);
        flipped = sides[0] != sides[1];
        if(flipped) break;
    }

    return flipped;
}

//check if the vert is on the right side of the face
bool vert_side(Data &d, uint fid, vec3d &vert) {

    bool side;

    vec3d verts[] = {d.m.face_verts(fid)[0], d.m.face_verts(fid)[1], d.m.face_verts(fid)[2]};
    side = orient3d(verts[0], verts[1], verts[2], vert) > 0;

    return side;

}

bool vert_side(Data &d, uint fid, CGAL_Q *vert) {
    bool side;

    uint verts_id[] = {d.m.face_verts_id(fid)[0], d.m.face_verts_id(fid)[1], d.m.face_verts_id(fid)[2]};
    CGAL_Q verts[3][3] = {{d.exact_coords[verts_id[0]*3+0], d.exact_coords[verts_id[0]*3+1], d.exact_coords[verts_id[0]*3+2]},
                          {d.exact_coords[verts_id[1]*3+0], d.exact_coords[verts_id[1]*3+1], d.exact_coords[verts_id[1]*3+2]},
                          {d.exact_coords[verts_id[2]*3+0], d.exact_coords[verts_id[2]*3+1], d.exact_coords[verts_id[2]*3+2]}};
    side = orient3d(verts[0], verts[1], verts[2], vert) > 0;

    return side;
}

bool does_movement_flip(Data &d, uint pid, uint fid, vec3d &target) {
    bool flipped;

    vec3d verts[] = {d.m.face_verts(fid)[0], d.m.face_verts(fid)[1], d.m.face_verts(fid)[2]};
    if (d.m.poly_face_is_CW(pid, fid)) std::swap(verts[0], verts[1]);
    flipped = orient3d(verts[0], verts[1], verts[2], target) * d.orient_sign <= 0; //if the signs are discordant we have a flip

    return flipped;
}

bool does_movement_flip(Data &d, uint pid, uint fid, CGAL_Q *target) {
    bool flipped;

    uint verts_id[] = {d.m.face_verts_id(fid)[0], d.m.face_verts_id(fid)[1], d.m.face_verts_id(fid)[2]};
    if(d.m.poly_face_is_CW(pid, fid)) std::swap(verts_id[0], verts_id[1]);
    CGAL_Q verts[3][3] = {{d.exact_coords[verts_id[0]*3+0], d.exact_coords[verts_id[0]*3+1], d.exact_coords[verts_id[0]*3+2]},
                          {d.exact_coords[verts_id[1]*3+0], d.exact_coords[verts_id[1]*3+1], d.exact_coords[verts_id[1]*3+2]},
                          {d.exact_coords[verts_id[2]*3+0], d.exact_coords[verts_id[2]*3+1], d.exact_coords[verts_id[2]*3+2]}};
    flipped = orient3d(verts[0], verts[1], verts[2], target) * d.orient_sign <= 0; //if the signs are discordant we have a flip

    return flipped;
}

void vert_normal(Data &d, uint vid, CGAL_Q *normal) {

    //init
    normal[0] = 0;
    normal[1] = 0;
    normal[2] = 0;

    //get the face normals
    for(uint fid : d.m.vert_adj_srf_faces(vid)) {
        CGAL_Q aux[3];
        uint verts_id[] = {d.m.face_verts_id(fid)[0], d.m.face_verts_id(fid)[1], d.m.face_verts_id(fid)[2]};
        triangle_normal(&d.exact_coords[verts_id[0]*3], &d.exact_coords[verts_id[1]*3], &d.exact_coords[verts_id[2]*3], aux);
        normal[0] += aux[0];
        normal[1] += aux[1];
        normal[2] += aux[2];
    }

    //avg the normals
    int size = d.m.vert_adj_srf_faces(vid).size();
    normal[0] = normal[0] / size;
    normal[1] = normal[1] / size;
    normal[2] = normal[2] / size;

}

bool check_self_intersection(Data &d) {

    if(d.verbose)
        std::cout << TXT_BOLDCYAN << "Checking self-intersection..." << TXT_RESET;

    bool intersection;
    std::set<ipair> intersections;

    //checks for early stop (fail)
    export_surface(d.m, d.m_srf);
    intersections.clear();
    find_intersections(d.m_srf, intersections);
    intersection = !intersections.empty();

    //feedback print
    if(intersection)
        std::cout << std::endl << TXT_BOLDRED << "FAILED: " << TXT_RESET << TXT_RED << "AUTO-INTERSECTION" << TXT_RESET << std::endl;
    else if(d.verbose)
        std::cout << TXT_BOLDGREEN << "PASSED" << TXT_RESET << std::endl;

    d.running = !intersection;

    return intersection;
}

bool snap_rounding(Data &d, const uint vid)
{
    if(!d.enable_snap_rounding) return true;

    // keep a safe copy of the exact coordinates
    CGAL_Q tmp[3];

    // round them to the closest double
    tmp[0] = CGAL::to_double(d.exact_coords[3*vid+0]);
    tmp[1] = CGAL::to_double(d.exact_coords[3*vid+1]);
    tmp[2] = CGAL::to_double(d.exact_coords[3*vid+2]);

    // check for flips
    bool flips = vert_flipped(d, vid, tmp);

    // no flips => i can snap
    if(!flips) {
        copy(tmp, &d.exact_coords[3 * vid]);
        d.m.vert(vid) = vec3d(CGAL::to_double(d.exact_coords[vid * 3 + 0]),
                              CGAL::to_double(d.exact_coords[vid * 3 + 1]),
                              CGAL::to_double(d.exact_coords[vid * 3 + 2]));
    }

    return flips;
}


//get the path of the target model
std::string get_target_path(std::string &base) {
    std::string path = get_file_name(base, false);

    size_t pos = path.find_last_of('_');
    path = (pos>=path.size()) ? path : path.substr(0, pos);

    path = "../data/" + path + ".mesh";

    return path;
}

void folder_check(std::string &path) {
}