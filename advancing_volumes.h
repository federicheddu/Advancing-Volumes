#ifndef ADVANCING_VOLUMES_ADVANCING_VOLUMES_H
#define ADVANCING_VOLUMES_ADVANCING_VOLUMES_H

#include <cinolib/gl/glcanvas.h>
#include <cinolib/meshes/meshes.h>
#include <cinolib/gl/volume_mesh_controls.h>
#include <cinolib/gl/surface_mesh_controls.h>
#include <cinolib/find_intersections.h>
#include <cinolib/smoother.h>
#include <cinolib/predicates.h>
#include <cinolib/ARAP.h>
#include <cinolib/split_separating_simplices.h>
#include <cinolib/quality_tet.h>
#include <cinolib/io/read_NODE_ELE.h>
#include <cinolib/io/write_NODE_ELE.h>

#include "sphere.h"
#include "visualization.h"
#include "text_format.h"

using namespace cinolib;

typedef struct data {
    //structures
    DrawableTetmesh<> vol;
    DrawableTrimesh<> srf;
    Octree oct;
    //model (sphere)
    DrawableTetmesh<> m;
    DrawableTrimesh<> m_srf;
    std::unordered_map<uint,uint> m2srf_vmap;
    std::unordered_map<uint,uint> srf2m_vmap;
    //parameters
    double mov_speed = 0.1;         // % of the distance used for the movement
    double eps_percent = 0.1;       // % of the min edge length used for the inactive threshold
    double eps_inactive = 0.01;     // dist fot the inactivity (in setup is set to eps * srf max edge length)
    double edge_threshold = 0.01;   // edge length threshold for refinement (in setup is set to 2 * srf max edge length)
    //fronts_active
    std::vector<uint> fronts_active;
    std::vector<uint> fronts_bounds;
    std::set<uint> stuck_in_place;
    //gui
    GLcanvas *gui;
} Data;

//struct to query the edges to flip after the split
typedef struct edge_to_flip {
    uint opp_vid = 0;
    ipair og_edge = {0, 0};
} edge_to_flip;

//main function
void advancing_volume(Data &data);
//setup of the env
Data setup(const char *path, bool load = false);
//topological operations
std::set<uint> search_split(Data &d, bool selective);
void split(Data &d, std::set<uint> &edges_to_split, std::map<ipair, uint> &v_map, std::queue<edge_to_flip> &edges_to_flip);
std::set<uint> search_split_int(Data &d);
void split_int(Data &d, std::set<uint> &edges_to_split);
void flip(Data &d, std::map<ipair, uint> &v_map, std::queue<edge_to_flip> &edges_to_flip);
void split_n_flip(Data &d, bool selective = false);
bool flip2to2(DrawableTetmesh<> &m, uint eid);
bool flip4to4(DrawableTetmesh<> &m, uint eid, uint vid0, uint vid1);
//vert operations
void expand(Data &d);
void refine(Data &d, bool internal = true);
void smooth(Data &d, int n_iter = 10);
void smooth_jacobian(Data &d, int n_iter = 10);
void smooth_stellar(Data &d);
double dist_calc(Data &d, uint vid, bool raycast = false, bool flat = false);
bool go_back_safe(Data &d, uint vid, const vec3d &og_pos);
void final_projection(Data &d);
vec3d project_onto_tangent_plane(vec3d &point, vec3d &plane_og, vec3d &plane_norm);
//check operations
bool check_intersection(Data &d, uint vid);
bool check_volume(Data &d, uint vid);
//front management
void update_fronts(Data &d);
void front_from_seed(Data &d, uint seed, std::unordered_set<uint> &front);

#endif //ADVANCING_VOLUMES_ADVANCING_VOLUMES_H
