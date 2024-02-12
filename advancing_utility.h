#ifndef ADVANCING_VOLUMES_ADVANCING_UTILITY_H
#define ADVANCING_VOLUMES_ADVANCING_UTILITY_H

#include "advancing_data.h"
#include <cinolib/find_intersections.h>

using namespace cinolib;

//srf edge lengths
double get_min_edge_length(Data &d, bool only_active);
double get_max_edge_length(Data &d, bool only_active);
double get_avg_edge_length(Data &d, bool only_active);

//distance calc
double dist_calc(Data &d, uint vid, bool raycast);
//check operations on pids
bool poly_flipped(Data &d, uint pid);
//check operations on vids
bool check_intersection(Data &d, uint vid);

//check if the vert movement is safe
bool vert_side(Data &d, uint fid, vec3d &vert);
bool vert_side(Data &d, uint fid, CGAL_Q *vert);
bool vert_flipped(Data &d, uint vid, vec3d &og_pos);
bool vert_flipped(Data &d, uint vid, CGAL_Q *og_pos);
bool does_movement_flip(Data &d, uint pid, uint fid, vec3d &target);
bool does_movement_flip(Data &d, uint pid, uint fid, CGAL_Q *target);

//utility for rationals
void vert_normal(Data &d, uint vid, CGAL_Q *normal);
bool check_self_intersection(Data &d);
vec3d to_double(CGAL_Q *r);
bool snap_rounding(Data &data, uint vid);

//file utils
bool file_check(std::string &path);
std::string get_target_path(std::string &base);
std::string get_rationals_path(std::string &base);
void folder_check(std::string &path);

void errorcheck(Data &d, bool check, std::string msg);
void filelog(Data &d, std::string msg);

#endif //ADVANCING_VOLUMES_ADVANCING_UTILITY_H
