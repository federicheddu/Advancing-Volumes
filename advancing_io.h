#ifndef ADVANCING_VOLUMES_ADVANCING_IO_H
#define ADVANCING_VOLUMES_ADVANCING_IO_H

#include "advancing_volumes.h"

void setup(Data &d, Octree *oct);
void parse(Data &d, int argc, char *argv[]);
int which_arg(std::string &arg);

#endif //ADVANCING_VOLUMES_ADVANCING_IO_H
