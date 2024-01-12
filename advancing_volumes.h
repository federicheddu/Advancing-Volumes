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
#include "visualization.h"
#include "text_format.h"

using namespace cinolib;

//main function
void advancing_volume(Data &data);

//vert operations
void expand(Data &d);
void refine(Data &d, bool internal = true);
void final_projection(Data &d);
std::map<uint, vec3d> get_movements(Data &d, int iter = 5);

//setup function
void parse_input(int argc, char *argv[]);
Data setup(const char *path, Octree *oct, bool load = false);
Data init_data(Octree *oct, const char *mode);
Data load_data(Octree *oct, const char *model, const char *target = nullptr);
void save_data(Data &d);
void init_model(Data &d);
void set_param(Data &d);

//rationals functions
void set_exact_coords(Data &d);
void add_last_rationals(Data &d);
void save_rationals(Data &d, std::string &path);
void load_rationals(Data &d, std::string &path);

#endif //ADVANCING_VOLUMES_ADVANCING_VOLUMES_H
