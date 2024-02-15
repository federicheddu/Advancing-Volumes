#include "advancing_volumes.h"

void advancing_volume(Data &d) {

    d.step++;
    cout << BMAG << "Advancing Volume ITERATION " << d.step << RESET << endl;

    if(d.running) expand(d);
    if(d.running) refine(d);
    if(d.running) smooth(d);

    cout << BMAG << "Advancing Volume ITERATION " << BGRN << "DONE" << RESET << endl;

}

void expand(Data &d) {

}

void compute_directions(Data &d) {

    vec3d move;
    std::vector<uint> adjs;

    //direction smoothing
    for(int it = 0; it < d.smooth_dir; it++) {
        for(uint vid : d.front) {
            //default movement
            move = d.m.vert_data(vid).normal;
            //avg
            adjs = d.m.vert_adj_srf_verts(vid);
            for(uint adj : adjs)
                move += d.m.vert_data(adj).normal;
            move /= double(adjs.size()+1);
            move.normalize();
            //update after smoothing
            d.m.vert_data(vid).normal = move;
        }
    }

}

void compute_distances(Data &d) {

    double dist;
    for(uint vid : d.front) {
        dist = dist_calc(d, vid, d.only_raycast);
        if(!d.only_raycast && dist < d.inactivity_dist < d.switch_raycast_threshold)
            dist = dist_calc(d, vid, true);
        d.m.vert_data(vid).uvw[DIST] = dist;
    }

}

void compute_displacement(Data &d) {
    for(uint vid : d.front)
        d.m.vert_data(vid).normal = d.m.vert_data(vid).normal * d.m.vert_data(vid).uvw[DIST] * 0.99;
}