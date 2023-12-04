#include "advancing_volumes.h"

//main function
void advancing_volume(Data &data) {

    data.step++;
    std::cout << TXT_BOLDMAGENTA << "Advancing volume ITERATION " << data.step << TXT_RESET << std::endl;

    //clear colors if debug
    if(data.debug_colors) clearColors(data.m);

    //model expansion
    if(data.running) expand(data);
    if(data.check_intersections) check_self_intersection(data);
    //refinement
    if(data.running) refine(data);
    add_last_rationals(data);
    //front update
    update_fronts(data);

    //update model and UI
    data.m.update_normals();

    std::cout << TXT_BOLDMAGENTA << "Advancing volume ITERATION " << data.step << TXT_RESET;
    if(data.running) std::cout << TXT_BOLDGREEN << " DONE" << TXT_RESET << std::endl;
    else std::cout << TXT_BOLDRED << " STOPPED" << TXT_RESET << std::endl;
    std::cout << std::endl;
}

//move the verts toward the target
void expand(Data &d) {

    if(d.verbose)
        std::cout << TXT_BOLDCYAN << "Expanding model..." << TXT_RESET;

    d.gui->pop_all_markers();
    d.stuck_in_place.clear();

    //get all the distances from the target model
    get_front_dist(d); //default parallel - add false to get linear

    //move every vert in the active front
    for(uint idx = 0; idx < d.fronts_active.size(); idx++) {

        //get the vid
        uint vid = d.fronts_active.at(idx);
        assert(d.m.vert_is_on_srf(vid));

        //backup position and distance from the target
        vec3d og_pos = d.m.vert(vid);
        double dist = d.m.vert_data(vid).uvw[DIST];

        //move the vert
        double movement = dist * d.mov_speed;
        vec3d moved = d.m.vert_data(vid).normal * movement;

        // TODO: redo this part
        //for(uint avid : d.m.vert_adj_srf_verts(vid)) {
        //    //get the distance (if the vert is near the target, use the raycast to get the distance)
        //    dist = dist_calc(d, avid, false);
        //    if (dist < d.eps_inactive * 2)
        //        dist = dist_calc(d, avid, true, true);
        //    else
        //        dist = dist_calc(d, avid, false, true);
        //    moved += d.m.vert_data(avid).normal * dist * d.mov_speed;
        //}
        //moved = moved / (d.m.vert_adj_srf_verts(vid).size() + 1);
        moved = og_pos + moved;

        topological_unlock(d, vid, moved);
        if (!d.running) return;

        //update the vert (+ rationals)
        d.m.vert(vid) = moved;
        if(d.rationals) {
            d.exact_coords[vid * 3 + 0] = moved.x();
            d.exact_coords[vid * 3 + 1] = moved.y();
            d.exact_coords[vid * 3 + 2] = moved.z();
        }

        //put the vert in a safe place between [og_pos, actual_pos]
        bool check = go_back_safe(d, vid, og_pos);
        if (!check) d.stuck_in_place.insert(vid);

        //update the mask
        if(dist_calc(d, vid, true) < d.eps_inactive)
            d.m.vert_data(vid).label = true;

        //if the vertex remains stationary for 5 iterations it is deactivated
        if(d.m.vert(vid) == og_pos) {
            d.m.vert_data(vid).uvw[MOV]++;
            if(d.m.vert_data(vid).uvw[MOV] == 5)
                d.m.vert_data(vid).label = true;
        } else
            d.m.vert_data(vid).uvw[MOV] = 0;

    }

    if(d.verbose)
        std::cout << TXT_GREEN << "DONE" << TXT_RESET << std::endl;

}

void refine(Data &d, bool internal) {

    if(d.verbose)
        std::cout << TXT_BOLDCYAN << "Refining the model..." << TXT_RESET;

    //refine external edges
    split_n_flip(d, true);

    if(internal) {
        //refine internal edges
        std::set<uint> edges_to_split = search_split_int(d);
        split_int(d, edges_to_split);
    }

    if(d.verbose)
        std::cout << TXT_GREEN << "DONE" << TXT_RESET << std::endl;

}

void final_projection(Data &d) {

    vec3d og_pos;
    double dist; uint aux;
    for(int iter = 0; iter < 5; iter++){
        for (uint vid: d.m.get_surface_verts()) {
            og_pos = d.m.vert(vid);

            dist = dist_calc(d, vid, true);
            d.oct->intersects_ray(d.m.vert(vid), d.m.vert_data(vid).normal, dist, aux);
            d.m.vert(vid) += d.m.vert_data(vid).normal * dist;

            go_back_safe(d, vid, og_pos);
        }
    }

}

/** ================================================================================================================ **/

//set up the env with model, target, oct etc...
Data setup(const char *path, Octree *oct, bool load) {
    return load ? load_data(path) : init_data(path, oct);
}

Data init_data(const char *model, Octree *oct) {
    Data data;

    //model vol
    data.vol = DrawableTetmesh<>(model);
    data.vol.show_mesh_points();

    set_param(data);

    //octree
    oct->build_from_mesh_polys(data.srf);
    data.oct = oct;

    init_model(data);

    //fronts build
    set_exact_coords(data);
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
    set_exact_coords(data);
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
        d.vol.vert_data(vid).uvw[DIST] = 0;
        //if not in the surface check the closest srf point and the distance to it
        if (!d.vol.vert_is_on_srf(vid)) {
            d.vol.vert_data(vid).uvw[DIST] = d.oct->closest_point(d.vol.vert(vid)).dist(d.vol.vert(vid));
            //check if its max distance
            if (d.vol.vert_data(vid).uvw[DIST] > center_dist) {
                center_vid = vid;
                center_dist = d.vol.vert_data(vid).uvw[DIST];
            }
        }
    }

    //sphere build
    d.m = get_sphere(d.vol.vert(center_vid), center_dist / 2);
}

void set_param(Data &d) {

    //model srf
    export_surface(d.vol, d.srf);

    //edge length threshold
    d.edge_threshold = d.srf.bbox().diag() * 0.02;

    //inactive threshold
    d.eps_inactive = d.edge_threshold * d.eps_percent;

}

void set_exact_coords(Data &data) {

    if(!rationals_are_working()) std::exit(2);
    data.exact_coords.resize(data.m.num_verts()*3);

    for(uint vid=0; vid<data.m.num_verts(); vid++) {
        data.exact_coords[3*vid+0] = data.m.vert(vid).x();
        data.exact_coords[3*vid+1] = data.m.vert(vid).y();
        data.exact_coords[3*vid+2] = data.m.vert(vid).z();
    }

}

void add_last_rationals(Data &d) {

    if(!rationals_are_working()) std::exit(2);

    if(d.m.num_verts() > d.exact_coords.size() / 3) {
        for(int vid = (int)d.exact_coords.size() / 3; vid < d.m.num_verts(); vid++) {
            d.exact_coords.emplace_back(d.m.vert(vid).x());
            d.exact_coords.emplace_back(d.m.vert(vid).y());
            d.exact_coords.emplace_back(d.m.vert(vid).z());
        }
    }

}
