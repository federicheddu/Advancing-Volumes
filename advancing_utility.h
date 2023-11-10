#ifndef ADVANCING_VOLUMES_ADVANCING_UTILITY_H
#define ADVANCING_VOLUMES_ADVANCING_UTILITY_H

#include "advancing_data.h"

using namespace cinolib;

//distance calc
double dist_calc(Data &d, uint vid, bool raycast = false, bool flat = false);
//line search
bool go_back_safe(Data &d, uint vid, vec3d &og_pos);
//check operations on pids
bool poly_flipped(Data &d, uint pid);
//check operations on vids
bool check_intersection(Data &d, uint vid);
bool vert_flipped(Data &d, uint vid, vec3d &og_pos);
bool vert_side(Data &d, uint fid, vec3d &vert);

#endif //ADVANCING_VOLUMES_ADVANCING_UTILITY_H
