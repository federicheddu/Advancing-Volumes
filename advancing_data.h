#ifndef ADVANCING_VOLUMES_ADVANCING_DATA_H
#define ADVANCING_VOLUMES_ADVANCING_DATA_H

#include <cinolib/meshes/meshes.h>
#include <cinolib/gl/glcanvas.h>
#include <cinolib/gl/surface_mesh_controls.h>
#include <cinolib/gl/volume_mesh_controls.h>
#include <cinolib/octree.h>

#include "sphere.h"

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

#endif //ADVANCING_VOLUMES_ADVANCING_DATA_H