#ifndef ADVANCING_VOLUMES_ADVANCING_VOLUMES_H
#define ADVANCING_VOLUMES_ADVANCING_VOLUMES_H

#include <cinolib/find_intersections.h>
#include <cinolib/smoother.h>
#include <cinolib/quality_tet.h>

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

//setup function
Data setup(const char *path, bool load = false);
Data init_data(const char *model);
Data load_data(const char *model, const char *target = nullptr);
void init_model(Data &d);
void set_param(Data &d);

#endif //ADVANCING_VOLUMES_ADVANCING_VOLUMES_H
