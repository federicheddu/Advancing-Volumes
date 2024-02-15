#include "advancing_io.h"

void setup(Data &d, Octree *oct) {

    //name
    if(!d.str_model.empty()) d.name = get_file_name(d.str_model, false);
    else d.name = get_file_name(d.str_target, false);

    //target volume
    d.tv = DrawableTetmesh<>(d.str_target.c_str());
    //target surface
    export_surface(d.tv, d.ts);
    d.ts.show_mesh_points();
    d.ts.mesh_data().filename = "Target";
    d.ts.updateGL();
    //octree
    oct->build_from_mesh_polys(d.ts);
    d.oct = oct;
    //params
    d.target_edge_length *= d.ts.edge_avg_length();
    d.inactivity_dist *= d.ts.edge_avg_length();

    //model
    init_model(d);

    //fronts init (all active)
    for(uint vid : d.m.get_surface_verts()) {
        d.front.emplace_back(vid);
        d.m.vert_data(vid).flags[ACTIVE] = true;
    }

    /* TODO: initial refinement */

    //rationals
    if(d.str_rationals.empty()) { //no rationals to load - brand new model
        d.rationals.resize(d.m.num_verts() * 3);
        for (uint vid = 0; vid < d.m.num_verts(); vid++)
            to_rational(d.m.vert(vid), &d.rationals[3 * vid]);
    } else { /*rationals to load - checkpoint to load*/ }

}