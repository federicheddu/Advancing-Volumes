#ifndef ADVANCING_VOLUMES_ADVANCING_VOLUMES_H
#define ADVANCING_VOLUMES_ADVANCING_VOLUMES_H

#include "advancing_io.h"

void advancing_volume(Data &d);

//main steps
void expand(Data &d);
void refine(Data &d);
void smooth(Data &d);

//expand funcitons
void compute_direction(Data &d);
void compute_distances(Data &d);
void compute_displacement(Data &d);
void line_search(Data &d, uint vid, CGAL_Q *rt_og_pos);
void check_self_intersection();

#endif //ADVANCING_VOLUMES_ADVANCING_VOLUMES_H