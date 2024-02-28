#include "advancing_io.h"
#undef NDEBUG

void setup(Data &d, Octree *oct) {

    bool loading = !d.path_model.empty();

    //name of the model
    if(loading) d.name = get_file_name(d.path_model, false);
    else d.name = get_file_name(d.path_target, false);

    //target volume
    d.tv = DrawableTetmesh<>(d.path_target.c_str());
    //target surface
    export_surface(d.tv, d.ts);
    d.ts.mesh_data().filename = "target_" + d.name;
    d.ts.show_mesh_points();
    d.ts.updateGL();
    //octree
    oct->build_from_mesh_polys(d.ts);
    d.oct = oct;
    //params
    d.target_edge_length        *= d.ts.edge_avg_length();
    d.switch_raycast_threshold  *= d.ts.edge_avg_length();
    d.inactivity_dist           *= d.ts.edge_avg_length();

    //model
    if(loading) d.m = DrawableTetmesh<>(d.path_model.c_str());
    else init_model(d);
    d.m.mesh_data().filename = "model_" + d.name;

    //fronts init (all active)
    for(uint vid : d.m.get_surface_verts()) {
        d.front.emplace_back(vid);
        d.m.vert_data(vid).flags[ACTIVE] = true;
    }

    //initial refinement of the model
    while(d.start_refinement && refine_again(d)) refine(d, true);

    //sphere mapping
    if(d.map) {
        if(loading) d.mm = DrawableTetmesh<>(d.path_map.c_str());
        else d.mm = d.m;
        d.mm.mesh_data().filename = "map_" + d.name;
    }

    //get file results folder path and log file path
    d.path_res = __FILE__;
    d.path_res += "results/";
    d.path_log = d.path_res + "log.txt";
}

void parse(Data &d, int argc, char *argv[]) {

    int arg;
    std::string arg_str;
    bool loading = false;

    for(int i=1; i<argc; i++) {

        //get the type of parameter
        arg_str = argv[i];
        arg = which_arg(arg_str);
        //get the parameter index
        i++;

        switch (arg) {
            case ARG_TARGET:
                d.path_target = argv[i];
                break;
            case ARG_MODEL:
                d.path_model = argv[i];
                loading = true;
                break;
            case ARG_MAP:
                d.path_map = argv[i];
                break;
            default:
                my_assert(d, false, "Parameters not valid", __FILE__, __LINE__);
                break;
        }

        //make sure there is a target to load
        assert(d.path_target.empty() && "No target");
        //only way to deactivate map is loading and no path_map
        d.map = !(loading && d.path_map.empty());
    }

}

void save(Data &d) {

}

int which_arg(std::string &arg) {

    if(arg == "-t") return ARG_TARGET;
    if(arg == "-m") return ARG_MODEL;
    if(arg == "-s") return ARG_MAP;
    return ARG_ERR;

}