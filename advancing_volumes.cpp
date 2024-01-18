#include "advancing_volumes.h"

//main function
void advancing_volume(Data &data) {

    data.step++;
    std::cout << TXT_BOLDMAGENTA << "Advancing volume ITERATION " << data.step << TXT_RESET << std::endl;

    //clear colors if debug
    if(data.debug_colors) clearColors(data.m);

    //model expansion
    if(data.running) expand(data);
    if(data.running && data.check_intersections) check_self_intersection(data);
    //refinement
    if(data.running) refine(data);
    add_last_rationals(data);
    //front update
    update_fronts(data);

    //update model and UI
    data.m.update_normals();

    //save the data
    if((data.step != 0 && data.step % data.save_every == 0) || !data.running)
        save_data(data);

    std::cout << TXT_BOLDMAGENTA << "Advancing volume ITERATION " << data.step << TXT_RESET;
    if(data.running) std::cout << TXT_BOLDGREEN << " DONE" << TXT_RESET << std::endl;
    else std::cout << TXT_BOLDRED << " STOPPED" << TXT_RESET << std::endl;
    std::cout << std::endl;
}

//move the verts toward the target
void expand(Data &d) {

    uint vid;
    if(d.verbose) std::cout << TXT_BOLDCYAN << "Expanding model..." << TXT_RESET;
    if(d.render) d.gui->pop_all_markers();
    d.stuck_in_place.clear();

    //get all the distances from the target model
    get_front_dist(d); //default parallel - add false to get linear
    std::map<uint, vec3d> movements = get_movements(d);

    //move every vert in the active front
    for(uint idx = 0; idx < d.fronts_active.size(); idx++) {

        //get the vid
        vid = d.fronts_active.at(idx);
        assert(d.m.vert_is_on_srf(vid));

        move(d, vid, movements);

        if(!d.running) return;
    }

    if(d.verbose)
        std::cout << TXT_GREEN << "DONE" << TXT_RESET << std::endl;

}

bool move(Data &d, uint vid, std::map<uint, vec3d> &movements) {

    bool moved = true;

    //backup positions
    vec3d og_pos = d.m.vert(vid);
    CGAL_Q rt_og_pos[3];
    copy(&d.exact_coords[vid*3], rt_og_pos);

    //displacement of the vert
    vec3d move = movements[vid];
    //vec3d move = movements[vid] * d.m.vert_data(vid).uvw[DIST] * d.mov_speed;

    CGAL_Q rt_move[3] = {move.x(), move.y(), move.z()};
    CGAL_Q rt_moved[3] = {d.exact_coords[vid * 3 + 0] + rt_move[0],
                          d.exact_coords[vid * 3 + 1] + rt_move[1],
                          d.exact_coords[vid * 3 + 2] + rt_move[2]};

    //get sure we can move the vert where we want
    topological_unlock(d, vid, rt_moved, rt_move);
    if (!d.running) return false;

    //update the vert (+ rationals)
    d.m.vert(vid) = vec3d(CGAL::to_double(rt_moved[0]),
                          CGAL::to_double(rt_moved[1]),
                          CGAL::to_double(rt_moved[2]));
    d.exact_coords[vid * 3 + 0] = rt_moved[0];
    d.exact_coords[vid * 3 + 1] = rt_moved[1];
    d.exact_coords[vid * 3 + 2] = rt_moved[2];

    //put the vert in a safe place between [og_pos, actual_pos]
    bool check = go_back_safe(d, vid, rt_og_pos);
    if (!check) d.stuck_in_place.insert(vid);

    // snap the rational coords to the closest double
    if(d.enable_snap_rounding)
        snap_rounding(d, vid);

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

    return moved;
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
    CGAL_Q rt_og_pos[3];
    double dist; uint aux;
    for(int iter = 0; iter < 5; iter++){
        for (uint vid: d.m.get_surface_verts()) {
            og_pos = d.m.vert(vid);
            copy(&d.exact_coords[vid*3], rt_og_pos);

            dist = dist_calc(d, vid, true);
            d.oct->intersects_ray(d.m.vert(vid), d.m.vert_data(vid).normal, dist, aux);
            d.m.vert(vid) += d.m.vert_data(vid).normal * dist;

            go_back_safe(d, vid, rt_og_pos);
        }
    }

}

std::map<uint, vec3d> get_movements(Data &d, int iter) {

    std::map<uint, vec3d> movements;
    std::map<uint, vec3d> bckp_norms;

    // init movements
    for (uint vid: d.m.get_surface_verts())
        movements[vid] = d.m.vert_data(vid).normal;

    // smoothing iterations (default 100)
    for (int it = 0; it < iter; it++) {

        // compute new movements
        for (uint vid : d.m.get_surface_verts()) {
            //get the first
            bckp_norms[vid] = movements[vid];

            //sum
            for (uint adj: d.m.vert_adj_srf_verts(vid))
                bckp_norms[vid] += movements[adj];

            //avg & normalize
            bckp_norms[vid] = bckp_norms[vid] / (d.m.vert_adj_srf_verts(vid).size() + 1);
            bckp_norms[vid].normalize();
        }

        movements = bckp_norms;
    }

    //dist multiplication (0.9 is for not going to inter)
    for(uint vid : d.m.get_surface_verts())
        movements[vid] = movements[vid] * d.m.vert_data(vid).uvw[DIST] * 0.99;

    /**/

    return movements;
}

/** ================================================================================================================ **/