#ifndef ADVANCING_VOLUMES_ADVANCING_UTILITY_H
#define ADVANCING_VOLUMES_ADVANCING_UTILITY_H

#include "advancing_data.h"

using namespace cinolib;

//distance calc
double dist_calc(Data &d, uint vid, bool raycast = false, bool flat = false);
//line search
bool go_back_safe(Data &d, uint vid, const vec3d &og_pos);
//check operations
bool check_intersection(Data &d, uint vid);
bool check_volume(Data &d, uint vid);

#endif //ADVANCING_VOLUMES_ADVANCING_UTILITY_H
