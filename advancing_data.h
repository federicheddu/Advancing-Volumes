#ifndef ADVANCING_VOLUMES_ADVANCING_DATA_H
#define ADVANCING_VOLUMES_ADVANCING_DATA_H

#undef NDEBUG

#include <cinolib/meshes/meshes.h>
#include <cinolib/gl/glcanvas.h>
#include <cinolib/gl/surface_mesh_controls.h>
#include <cinolib/gl/volume_mesh_controls.h>
#include <cinolib/octree.h>
#include <cinolib/smoother.h>

#include "advancing_rationals.h"
#include "ANSI_color_codes.h"

using namespace cinolib;

//double uvw overwrite
enum {
    DIST,
    MOV,
    UNUSED_AV
};

//bool flags overwrite
enum {
    ACTIVE = UNUSED_0,          //vert is active or not
    SPLIT = UNUSED_1,           //vert origined from refinement
    TOPOLOGICAL = UNUSED_2      //vert origined from topological unlock
};

//struct to query the edges to flip after the split
typedef struct edge_to_flip {
    uint opp_vid = 0;
    ipair og_edge = {0, 0};
} edge_to_flip;

typedef struct data {

    //movement
    int smooth_dir_iters = 200;         //iters of direction smoothing
    int line_search_max = 10;           //max iters of line search
    bool only_raycast = false;          //move only with raycast (no closest point)
    //refinement
    bool multiple_refinement = true;
    bool internal_refinement = true;
    //smoothing
    int smooth_mesh_iters = 5;
    //text and debug
    bool verbose = true;                // print sub-steps
    //other
    bool enable_snap_rounding = false;  // snap the rational coords to the closest double

    //strings
    std::string name;
    std::string str_model;
    std::string str_target;
    std::string str_rationals;

    //execution
    int step = 0;
    bool running = true;
    bool step_by_step = false;
    int save_every = 1;

    //structures
    DrawableTetmesh<> m;    //model
    DrawableTrimesh<> ms;   //model surface
    DrawableTetmesh<> tv;   //target volume
    DrawableTrimesh<> ts;   //target surface
    Octree *oct;            //octree from ts

    //verts info
    std::vector<uint> front;
    std::vector<CGAL_Q> rationals;

    //refinement
    std::set<uint> ets;             //edges that need to be split
    std::map<ipair, uint> v_map;    //maps the og edge (ipair) to the new vert
    std::queue<edge_to_flip> etf;   //edges that need to be flipped

    //parameters
    double sphere_rad = 0.9;
    double target_edge_length = 2.0;        // multiplied in setup with the avg length of srf edges of the target
    double switch_raycast_threshold = 1;    // multiplied in setup with the avg length of srf edges of the target
    double inactivity_dist = 0.1;           // multiplied in setup with the avg length of srf edges of the target

    //utility
    int orient_sign = -1;               // sign for the orientation of the mesh (1 or -1)

    //gui
    bool render = false;
    GLcanvas *gui = nullptr;

    //counters
    int unlock2_count = 0;
    int unlock3_count = 0;

} Data;

//initial model
DrawableTetmesh<> hardcode_model();
void init_model(Data &d);
//utility
double dist_calc(Data &d, uint vid, bool raycast);
bool will_poly_flip(Data &d, const uint vid, const uint pid, CGAL_Q *target);
bool will_face_flip(Data &d, const uint pid, const uint fid, CGAL_Q *target);
//data conversion
vec3d to_double(CGAL_Q *src);
void to_rational(vec3d &src, CGAL_Q *dest);

#endif //ADVANCING_VOLUMES_ADVANCING_DATA_H
