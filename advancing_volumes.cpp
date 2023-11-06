#include "advancing_volumes.h"

//main function
void advancing_volume(Data &data) {

    //model expansion
    expand(data);
    std::cout << TXT_BOLDRED << "Verts stuck during expansion: " << data.stuck_in_place.size() << TXT_RESET << std::endl;
    //refinement
    refine(data);
    //front update
    update_fronts(data);

    //update model and UI
    data.m.update_normals();
}

//move the verts toward the target
void expand(Data &d) {

    std::cout << TXT_BOLDCYAN << "Expanding model..." << TXT_RESET;

    d.gui->pop_all_markers();
    d.stuck_in_place.clear();

    //get all the distances from the target model
    std::vector<double> front_dist(d.fronts_active.size());
    PARALLEL_FOR(0, d.fronts_active.size(), 1000, [&](int idx)
    {
        //get the vid
        uint vid = d.fronts_active.at(idx);
        //get the distance (if the vert is near the target, use the raycast to get the distance)
        double dist = dist_calc(d, vid, false);
        if(dist < d.eps_inactive*2)
            dist = dist_calc(d, vid, true, true);
        else
            dist = dist_calc(d, vid, false, true);
        //save the distance
        front_dist.at(idx) = dist;
    });

    //move every vert in the active front
    for(uint idx = 0; idx < d.fronts_active.size(); idx++) {

        //get the vid
        uint vid = d.fronts_active.at(idx);
        assert(d.m.vert_is_on_srf(vid));

        //og pos to get back if something goes wrong
        vec3d og_pos = d.m.vert(vid);

        //get the distance calculated before
        double dist = front_dist.at(idx);

        //move the vert
        double movement = dist * d.mov_speed;
        d.m.vert(vid) += d.m.vert_data(vid).normal * movement;

        //put the vert in a safe place between [og_pos, actual_pos]
        bool check = go_back_safe(d, vid, og_pos);
        if (!check) d.stuck_in_place.insert(vid);

        //update the mask
        if(dist_calc(d, vid, true) < d.eps_inactive)
            d.m.vert_data(vid).label = true;

        //if the vertex remains stationary for 5 iterations it is deactivated
        if(d.m.vert(vid) == og_pos) {
            d.m.vert_data(vid).uvw.x()++;
            if(d.m.vert_data(vid).uvw.x() == 5)
                d.m.vert_data(vid).label = true;
        } else
            d.m.vert_data(vid).uvw.x() = 0;

    }

    std::cout << TXT_GREEN << "DONE" << TXT_RESET << std::endl;

}

void refine(Data &d, bool internal) {

    std::cout << TXT_CYAN << "Refining the model..." << TXT_RESET;

    //refine external edges
    split_n_flip(d, true);

    if(internal) {
        //refine internal edges
        std::set<uint> edges_to_split = search_split_int(d);
        split_int(d, edges_to_split);
    }

    std::cout << TXT_GREEN << "DONE" << TXT_RESET << std::endl;

}

void final_projection(Data &d) {

    vec3d og_pos;
    double dist; uint aux;
    for(int iter = 0; iter < 5; iter++){
        for (uint vid: d.m.get_surface_verts()) {
            og_pos = d.m.vert(vid);

            dist = dist_calc(d, vid, true);
            d.oct.intersects_ray(d.m.vert(vid), d.m.vert_data(vid).normal, dist, aux);
            d.m.vert(vid) += d.m.vert_data(vid).normal * dist;

            go_back_safe(d, vid, og_pos);
        }
    }

}

/** ================================================================================================================ **/

//set up the env with model, target, oct etc...
Data setup(const char *path, bool load) {
    return load ? load_data(path) : init_data(path);
}

Data init_data(const char *model) {
    Data data;

    //model vol
    data.vol = DrawableTetmesh<>(model);
    data.vol.show_mesh_points();

    set_param(data);

    init_model(data);

    //fronts build
    init_fronts(data);

    return data;
}

Data load_data(const char *model, const char *target) {
    Data data;

    std::string model_path = model;
    std::string target_path = target == nullptr ? "../data/" + model_path.substr(11, model_path.size()-11) : target;;

    data.m = DrawableTetmesh<>(model_path.c_str());
    data.vol = DrawableTetmesh<>(target_path.c_str());
    data.vol.show_mesh_points();

    //set other parameters of data struct
    set_param(data);

    //compute the fronts
    load_fronts(data);

    return data;
}

void init_model(Data &d) {

    //get info for placing the sphere
    uint center_vid = 0;
    double center_dist = -inf_double;
    //for every vid in the vol
    for (uint vid = 0; vid < d.vol.num_verts(); vid++) {
        //default
        d.vol.vert_data(vid).uvw[0] = 0;
        //if not in the surface check the closest srf point and the distance to it
        if (!d.vol.vert_is_on_srf(vid)) {
            d.vol.vert_data(vid).uvw[0] = d.oct.closest_point(d.vol.vert(vid)).dist(d.vol.vert(vid));
            //check if its max distance
            if (d.vol.vert_data(vid).uvw[0] > center_dist) {
                center_vid = vid;
                center_dist = d.vol.vert_data(vid).uvw[0];
            }
        }
    }

    //sphere build
    d.m = get_sphere(d.vol.vert(center_vid), center_dist / 2);
}

void set_param(Data &d) {

    //model srf
    export_surface(d.vol, d.srf);

    //octree build
    d.oct.build_from_mesh_polys(d.srf);

    //edge length threshold
    d.edge_threshold = d.srf.bbox().diag() * 0.02;

    //inactive threshold
    d.eps_inactive = d.edge_threshold * d.eps_percent;

}