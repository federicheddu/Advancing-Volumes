#ifndef ADVANCING_VOLUMES_ADVANCING_UTILITY_H
#define ADVANCING_VOLUMES_ADVANCING_UTILITY_H

#include "advancing_data.h"
#include <cinolib/find_intersections.h>

using namespace cinolib;

//distance calc
double dist_calc(Data &d, uint vid, bool raycast = false, bool flat = false);
double get_dist(Data &d, uint vid);
//line search
bool go_back_safe(Data &d, uint vid, CGAL_Q *rt_pos);
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
bool snap_rounding(Data &data, const uint vid);

#endif //ADVANCING_VOLUMES_ADVANCING_UTILITY_H
