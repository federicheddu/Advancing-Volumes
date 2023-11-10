#ifndef ADVANCING_VOLUMES_ADVANCING_TOPOLOGICAL_H
#define ADVANCING_VOLUMES_ADVANCING_TOPOLOGICAL_H

#include <cinolib/split_separating_simplices.h>
#include <cinolib/predicates.h>
#include "advancing_data.h"


//struct to query the edges to flip after the split
typedef struct edge_to_flip {
    uint opp_vid = 0;
    ipair og_edge = {0, 0};
} edge_to_flip;

//topological operations
std::set<uint> search_split(Data &d, bool selective);
void split(Data &d, std::set<uint> &edges_to_split, std::map<ipair, uint> &v_map, std::queue<edge_to_flip> &edges_to_flip);
std::set<uint> search_split_int(Data &d);
void split_int(Data &d, std::set<uint> &edges_to_split);
void flip(Data &d, std::map<ipair, uint> &v_map, std::queue<edge_to_flip> &edges_to_flip);
void split_n_flip(Data &d, bool selective = false);
bool flip2to2(DrawableTetmesh<> &m, uint eid);
bool flip4to4(DrawableTetmesh<> &m, uint eid, uint vid0, uint vid1);
bool tet_is_blocking(Data &data, const uint id, const uint pid, const vec3d& target);
void unlock_by_edge_split(Data &d, const uint pid, const uint vid, const vec3d& target);

#endif //ADVANCING_VOLUMES_ADVANCING_TOPOLOGICAL_H
