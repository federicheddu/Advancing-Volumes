#ifndef ADVANCING_VOLUMES_ADVANCING_DATA_H
#define ADVANCING_VOLUMES_ADVANCING_DATA_H

#include <cinolib/meshes/meshes.h>
#include <cinolib/gl/glcanvas.h>
#include <cinolib/gl/surface_mesh_controls.h>
#include <cinolib/gl/volume_mesh_controls.h>
#include <cinolib/octree.h>
#include "rationals.h"
#include "sphere.h"

using namespace cinolib;

enum {
    DIST,
    MOV,
    UNUSED_AV
};

typedef struct data {

    //loading path
    std::string load_target;
    std::string load_model;
    std::string load_exact;

    //execution
    bool running = true;
    bool batch = false;
    int step = 0;
    int save_every = 1;

    //structures
    DrawableTetmesh<> vol;
    DrawableTrimesh<> srf;
    Octree *oct;

    //model (sphere)
    DrawableTetmesh<> m;
    DrawableTrimesh<> m_srf;

    //rationals
    bool rationals = true;
    std::vector<CGAL_Q> exact_coords;

    //parameters
    double sphere_rad = 0.9;
    double mov_speed = 0.8;         // % of the distance used for the movement
    double eps_percent = 0.1;       // % of the min edge length used for the inactive threshold
    double eps_inactive = 0.01;     // dist fot the inactivity (in setup is set to eps * srf max edge length)
    double edge_threshold = 0.01;   // edge length threshold for refinement (in setup is set to 2 * srf max edge length)

    //utility
    bool check_intersections = true;    // check if the mesh is self intersecting
    bool enable_snap_rounding = false;   // snap the rational coords to the closest double
    bool verbose = true;                // print sub-steps
    bool ultra_verbose = true;         // print everything (debug)
    bool debug_colors = false;           // color the mesh for debug
    int orient_sign = -1;               // sign for the orientation of the mesh (1 or -1)

    //fronts_active
    std::vector<uint> fronts_active;
    std::vector<uint> fronts_bounds;
    std::set<uint> stuck_in_place;

    //gui
    bool render;
    GLcanvas *gui;

} Data;

#endif //ADVANCING_VOLUMES_ADVANCING_DATA_H
