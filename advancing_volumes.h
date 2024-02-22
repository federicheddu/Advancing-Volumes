#ifndef ADVANCING_VOLUMES_ADVANCING_VOLUMES_H
#define ADVANCING_VOLUMES_ADVANCING_VOLUMES_H
#undef NDEBUG
#include "advancing_io.h"
#include <cinolib/find_intersections.h>

void advancing_volume(Data &d);

//main steps
void expand(Data &d);
void refine(Data &d);
void smooth(Data &d);

//expand functions
void move(Data &d, uint vid, vec3d &disp);
void compute_directions(Data &d);
void compute_distances(Data &d);
void compute_displacements(Data &d);
void line_search(Data &d, uint vid, vec3d &safe_pos);
bool check_intersection(Data &d, uint vid);
void check_self_intersection(Data &d);

//refine functions
void split(Data &d);
void edges_to_split(Data &d);
void flip(Data &d);
void try_flips(Data &d);
bool flip4to4(Tetmesh<> &m, uint eid, uint vid0, uint vid1);
bool flip2to2(Tetmesh<> &m, uint eid);

//front functions
void update_front(Data &d);

#endif //ADVANCING_VOLUMES_ADVANCING_VOLUMES_H