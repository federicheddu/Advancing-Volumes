#ifndef ADVANCING_VOLUMES_ADVANCING_IO_H
#define ADVANCING_VOLUMES_ADVANCING_IO_H

#include "advancing_data.h"
#include "advancing_topological.h"
#include "advancing_front.h"

//setup function
void setup(Data &data, int argc, char *argv[], Octree *oct);
void parse_input(Data &d, int argc, char *argv[]);
void init_data(Data &data, Octree *oct);
void load_data(Data &data, Octree *oct);
void save_data(Data &d);
void init_model(Data &d);
void set_param(Data &d);

//get from data
std::string dataset_string();

//rationals functions
void set_exact_coords(Data &d);
void add_last_rationals(Data &d);
void save_rationals(Data &d, std::string &path);
void load_rationals(Data &d);

#endif //ADVANCING_VOLUMES_ADVANCING_IO_H
