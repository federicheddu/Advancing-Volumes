#ifndef ADVANCING_VOLUMES_ADVANCING_VOLUMES_H
#define ADVANCING_VOLUMES_ADVANCING_VOLUMES_H

#include <cinolib/find_intersections.h>
#include <cinolib/smoother.h>
#include <cinolib/quality_tet.h>
#include <chrono>

#include "advancing_data.h"
#include "advancing_front.h"
#include "advancing_utility.h"
#include "advancing_topological.h"
#include "advancing_io.h"
#include "visualization.h"
#include "text_format.h"

using namespace cinolib;

//main function
void advancing_volume(Data &data);

//vert operations
void expand(Data &d);
void move(Data &d, uint vid, std::vector<vec3d> &movements);
void refine(Data &d, bool internal = true);
void final_projection(Data &d);
std::vector<vec3d> get_movements(Data &d, int iter = 100);
bool line_search(Data &d, uint vid, CGAL_Q *rt_og_pos);

#endif //ADVANCING_VOLUMES_ADVANCING_VOLUMES_H
