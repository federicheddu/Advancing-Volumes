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

    /** params **/
    //line search
    bool line_search = true;
    bool forward_line_search = true;
    //movement
    bool only_raycast = false;
    bool priority_queue = true;
    bool check_intersections = true;    // check if the mesh is self intersecting
    //refinement
    bool multiple_refinement = true;
    bool internal_refinement = true;
    //smoothing
    bool smoothing = true;
    int smooth_iters = 5;
    //text and debug
    bool verbose = true;                // print sub-steps
    bool ultra_verbose = false;         // print everything (debug)
    bool debug_colors = false;           // color the mesh for debug
    //other
    bool enable_snap_rounding = false;   // snap the rational coords to the closest double

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
    double eps_percent = 0.1;       // % of the min edge length used for the inactive threshold
    double inactivity_dist = 0.01;     // dist fot the inactivity (in setup is set to eps * srf max edge length)
    double target_edge_length = 0.01;   // edge length threshold for refinement (in setup is set to 2 * srf max edge length)

    //utility
    int orient_sign = -1;               // sign for the orientation of the mesh (1 or -1)

    //fronts_active
    std::vector<uint> fronts_active;
    std::vector<uint> fronts_bounds;

    //gui
    bool render;
    GLcanvas *gui;

} Data;

#endif //ADVANCING_VOLUMES_ADVANCING_DATA_H
