#include "advancing_utility.h"

//srf min edge length
double get_min_edge_length(Data &d) {
    double min_edge = std::numeric_limits<double>::max();

    for(uint eid : d.m.get_surface_edges()) {
        double edge = d.m.edge_length(eid);
        if(edge < min_edge) min_edge = edge;
    }

    return min_edge;
}

//srf max edge length
double get_max_edge_length(Data &d) {
    double max_edge = 0;

    for(uint eid : d.m.get_surface_edges()) {
        double edge = d.m.edge_length(eid);
        if(edge > max_edge) max_edge = edge;
    }

    return max_edge;
}

//srf avg edge length
double get_avg_edge_length(Data &d) {
    double avg_edge = 0;
    double count = 0;

    for(uint eid : d.m.get_surface_edges()) {
        avg_edge += d.m.edge_length(eid);
        count++;
    }

    return avg_edge / count;
}



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

    return dist;
}

double get_dist(Data &d, uint vid) {
    double dist;

    for(int idx = 0; idx < d.fronts_active.size(); idx++) {
        //get the vid
        vid = d.fronts_active.at(idx);
        //get the distance (if the vert is near the target, use the raycast to get the distance)
        dist = dist_calc(d, vid, false);
        if (dist < d.inactivity_dist * 2)
            dist = dist_calc(d, vid, true, true);
        else
            dist = dist_calc(d, vid, false, true);
    }

    return dist;
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
        flag = d.oct->intersects_triangle(verts, false, intersect_ids);
        if(flag) break; //early stop
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


vec3d to_double(CGAL_Q *r) {
    return vec3d(CGAL::to_double(r[0]), CGAL::to_double(r[1]), CGAL::to_double(r[2]));
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
        if(d.render || d.m.vert_is_on_srf(vid)) {
            d.m.vert(vid) = vec3d(CGAL::to_double(d.exact_coords[vid * 3 + 0]),
                                  CGAL::to_double(d.exact_coords[vid * 3 + 1]),
                                  CGAL::to_double(d.exact_coords[vid * 3 + 2]));
        }
    }

    return flips;
}



//get the path of the target model
bool file_check(std::string &path) {
    bool check;
    FILE *f = nullptr;

    f = fopen(path.c_str(), "r");
    check = f != nullptr;
    fclose(f);

    return check;
}

std::string get_target_path(std::string &base) {
    std::string path = get_file_name(base, false);

    size_t pos = path.find_last_of('_');
    path = (pos>=path.size()) ? path : path.substr(0, pos);

    path = "../data/" + path + ".mesh";

    return path;
}

std::string get_rationals_path(std::string &base) {
    std::string path = base;

    size_t pos = path.find_last_of('.');
    path = (pos>=path.size()) ? path : path.substr(0, pos+1);
    path += "txt";

    return path;
}

void folder_check(std::string &path) {
}


// print the error in the log file and stop the execution with an assert
void errorcheck(Data &d, bool check, std::string msg) {
    if(!check) {
        std::string name = get_file_name(d.load_target, false);
        std::cout << TXT_BOLDRED << "ERROR " << name << ": " << TXT_RESET << TXT_RED << msg << TXT_RESET << std::endl;
        filelog(d, msg);
        assert(false);
    }
}

// print the log file
void filelog(Data &d, std::string msg) {

    //params
    std::string path = "../results/log.txt";
    std::string name = get_file_name(d.load_target, false);
    int step = d.step;
    int active = d.fronts_active.size();
    int verts = d.m.num_verts();
    int srf_verts = d.m.get_surface_verts().size();
    int polys = d.m.num_polys();
    double vol = d.m.mesh_volume();
    int target_verts = d.vol.get_surface_verts().size();
    double target_vol = d.vol.mesh_volume();
    double percent = d.m.mesh_volume() / d.vol.mesh_volume();

    //open
    FILE *f = fopen(path.c_str(), "a");
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
    //msg
    fprintf(f, "%s;", msg.c_str());
    //close
    fclose(f);
}