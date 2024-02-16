#ifndef ADVANCING_VOLUMES_ADVANCING_VOLUMES_H
#define ADVANCING_VOLUMES_ADVANCING_VOLUMES_H

#include "advancing_io.h"
#include <cinolib/find_intersections.h>

void advancing_volume(Data &d);

//main steps
void expand(Data &d);
void refine(Data &d);
void smooth(Data &d);

//expand functions
void move(Data &d, uint vid, vec3d &disp);
void move(Data &d, uint vid, CGAL_Q *rt_disp);
void compute_directions(Data &d);
void compute_distances(Data &d);
void compute_displacements(Data &d);
void line_search(Data &d, uint vid, CGAL_Q *safe_pos);
bool check_intersection(Data &d, uint vid);
void check_self_intersection(Data &d);

//front functions
void update_front(Data &d);

#endif //ADVANCING_VOLUMES_ADVANCING_VOLUMES_H