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

    std::cout << TXT_BOLDMAGENTA << "Advancing volume ITERATION " << data.step << " - Active: " << data.fronts_active.size() << TXT_RESET;
    if(data.running) std::cout << TXT_BOLDGREEN << " DONE" << TXT_RESET << std::endl;
    else std::cout << TXT_BOLDRED << " STOPPED" << TXT_RESET << std::endl;
    std::cout << std::endl;
}

//move the verts toward the target
void expand(Data &d) {

    uint vid;
    if(d.verbose) std::cout << TXT_BOLDCYAN << "Expanding model..." << TXT_RESET;
    if(d.render) d.gui->pop_all_markers();

    compute_movements(d);

    //get all the distances from the target model
    get_front_dist(d, false); //default parallel - add false to get linear

    //movement (direction * distance)
    for(uint vid : d.fronts_active)
        d.m.vert_data(vid).normal = d.m.vert_data(vid).normal * d.m.vert_data(vid).uvw[DIST] * 0.99;

    //move every vert in the active front
    for(uint idx = 0; idx < d.fronts_active.size(); idx++) {

        //get the vid
        //if(d.running % 2 == 0) vid = d.fronts_active.at(idx);
        //else vid = d.fronts_active.at(d.fronts_active.size()-idx-1);
        vid = d.fronts_active.at(idx);
        errorcheck(d, d.m.vert_is_on_srf(vid), "Vert " + std::to_string(vid) + " not on surface");

        //displace the vert
        move(d, vid);

        //if something went wrong stall everything
        if(!d.running) return;
    }

    //restore the normals
    d.m.update_normals();

    if(d.verbose)
        std::cout << TXT_GREEN << "DONE" << TXT_RESET << std::endl;

}

void move(Data &d, uint vid) {

    bool moved = true;

    //backup positions
    vec3d og_pos = d.m.vert(vid);
    CGAL_Q rt_og_pos[3];
    copy(&d.exact_coords[vid*3], rt_og_pos);

    //displacement of the vert
    vec3d move = d.m.vert_data(vid).normal;
    CGAL_Q rt_move[3] = {move.x(), move.y(), move.z()};

    //new position of the vert
    CGAL_Q rt_moved[3] = {d.exact_coords[vid * 3 + 0] + rt_move[0],
                          d.exact_coords[vid * 3 + 1] + rt_move[1],
                          d.exact_coords[vid * 3 + 2] + rt_move[2]};

    //get sure we can move the vert where we want
    topological_unlock(d, vid, rt_moved, rt_move);
    if(!d.running) return;

    //update the vert (+ rationals)
    d.m.vert(vid) = vec3d(CGAL::to_double(rt_moved[0]),
                          CGAL::to_double(rt_moved[1]),
                          CGAL::to_double(rt_moved[2]));
    d.exact_coords[vid * 3 + 0] = rt_moved[0];
    d.exact_coords[vid * 3 + 1] = rt_moved[1];
    d.exact_coords[vid * 3 + 2] = rt_moved[2];

    //put the vert in a safe place between [og_pos, actual_pos]
    moved = line_search(d, vid, rt_og_pos);
    if(!d.running) return;
    //update how many times in a row the vert is stuck
    if(!moved) d.m.vert_data(vid).uvw[MOV]++;
    else d.m.vert_data(vid).uvw[MOV] = 0;

    // snap the rational coords to the closest double
    if(d.enable_snap_rounding) snap_rounding(d, vid);

    //update the mask (note: if the vertex remains stationary for 5 iterations it is deactivated)
    if(d.m.vert_data(vid).uvw[MOV] == 5 || dist_calc(d, vid, true) < d.inactivity_dist)
        d.m.vert_data(vid).label = true;

}

void refine(Data &d, bool internal) {

    if(d.verbose)
        std::cout << TXT_BOLDCYAN << "Refining the model..." << TXT_RESET;

    //refine surface edges
    split_n_flip(d, true);

    //refine internal edges
    if(internal) {
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

            line_search(d, vid, rt_og_pos);
        }
    }

}

void compute_movements(Data &d, int iters) {

    bool can_smooth;
    vec3d move;
    vec3d fpmoved;
    CGAL_Q rtmoved[3];

    //direction smoothing
    for(int iter = 0; iter < iters; iter++) {
        //every srf vid
        for(uint vid : d.fronts_active) {
            //default
            can_smooth = true;
            //get normal
            move = d.m.vert_data(vid).normal;
            //adj normals
            for(uint adj : d.m.vert_adj_srf_verts(vid))
                move += d.m.vert_data(adj).normal;
            //avg and normalize
            move = move / (d.m.vert_adj_srf_verts(vid).size()+1);
            move.normalize();
            //check the move is ok
            //fpmoved = move * d.m.vert_data(vid).uvw[DIST] * 0.99;
            //rtmoved[0] = d.exact_coords[vid * 3 + 0] + fpmoved.x();
            //rtmoved[1] = d.exact_coords[vid * 3 + 1] + fpmoved.y();
            //rtmoved[2] = d.exact_coords[vid * 3 + 2] + fpmoved.z();
            //for(uint adj : d.m.adj_v2p(vid))
            //    can_smooth = can_smooth && !tet_is_blocking(d, vid, adj, rtmoved);
            ////if smoothing doesn't cause problems
            //if(can_smooth)
                d.m.vert_data(vid).normal = move;
        }
    }

    //movement (direction * distance)
    //for(uint vid : d.fronts_active)
    //    d.m.vert_data(vid).normal = d.m.vert_data(vid).normal * d.m.vert_data(vid).uvw[DIST] * 0.99;

}

//check if the movement of the vert is safe, if not it goes back by bisection
bool line_search(Data &d, uint vid, CGAL_Q *rt_og_pos) {

    int bck_cnt = 0, frd_cnt = 0;
    int max_iter = 7;
    bool moved_check = true;
    CGAL_Q mid[3];
    CGAL_Q final[3];

    //moved_check for intersections (only if on surface)
    if(d.m.vert_is_on_srf(vid)) {

        //backup the current position
        copy(&d.exact_coords[vid*3], final);

        //if the vert is outside the model, push it back by bisection
        bck_cnt = 0;
        while (bck_cnt < max_iter && check_intersection(d, vid)) {
            //bisection backward
            midpoint(&d.exact_coords[vid * 3], rt_og_pos, mid);
            copy(mid, &d.exact_coords[vid * 3]);
            d.m.vert(vid) = to_double(mid);
            //how many times the vert is pushed back
            bck_cnt++;
        }

        //try to push the vert forward by another half if it was pushed back
        while(frd_cnt < max_iter && !check_intersection(d, vid)) {
            //update the safe position
            copy(&d.exact_coords[vid * 3], rt_og_pos);
            //bisection forward
            midpoint(&d.exact_coords[vid * 3], final, mid);
            copy(mid, &d.exact_coords[vid * 3]);
            d.m.vert(vid) = to_double(mid);
            //how many times the vert is pushed forward
            frd_cnt++;
        }

        //if vert still outside get back the vid
        if (check_intersection(d, vid)) {
            copy(rt_og_pos, &d.exact_coords[vid * 3]);
            d.m.vert(vid) = to_double(rt_og_pos);
            //if the condition is true the vert has been moved from og_pos
            moved_check = frd_cnt > 0;
        }

    }

    return moved_check;
}

/** ================================================================================================================ **/