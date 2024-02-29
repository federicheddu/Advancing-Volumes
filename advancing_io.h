#ifndef ADVANCING_VOLUMES_ADVANCING_IO_H
#define ADVANCING_VOLUMES_ADVANCING_IO_H

#include "advancing_volumes.h"

//general setup
void setup(Data &d, Octree *oct);
//parse input
void parse(Data &d, int argc, char *argv[]);
int which_arg(std::string &arg);
//save-load
void save(Data &d);

#endif //ADVANCING_VOLUMES_ADVANCING_IO_H
