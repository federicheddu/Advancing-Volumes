#ifndef ADVANCING_VOLUMES_ADVANCING_FRONT_H
#define ADVANCING_VOLUMES_ADVANCING_FRONT_H

#include "advancing_data.h"
#include "advancing_utility.h"

//front management
void init_fronts(Data &d);
void load_fronts(Data &d);
void update_fronts(Data &d);
void front_from_seed(Data &d, uint seed, std::unordered_set<uint> &front);
void get_front_dist(Data &d, bool only_ray = false);

#endif //ADVANCING_VOLUMES_ADVANCING_FRONT_H
