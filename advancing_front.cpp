#include "advancing_front.h"

void init_fronts(Data &d) {
    for(uint vid : d.m.get_surface_verts()) {
        d.fronts_active.emplace_back(vid);
        d.m.vert_data(vid).label = false;
        d.m.vert_data(vid).uvw[MOV] = 0;
    }

    d.fronts_bounds.emplace_back(d.fronts_active.size());
    update_fronts(d);
}

void load_fronts(Data &d) {
    for (uint vid : d.m.get_surface_verts()) {

        if(dist_calc(d, vid, true) < d.inactivity_dist)
            d.m.vert_data(vid).label = true;
        else {
            d.m.vert_data(vid).label = false;
            d.fronts_active.emplace_back(vid);
        }
    }

    d.fronts_bounds.emplace_back(d.fronts_active.size());
    update_fronts(d);
}

//update the front (must be called after every variation of the model)
void update_fronts(Data &d) {

    //std::cout << TXT_CYAN << "Updating the fronts... ";

    //parameters
    uint active_counter = 0;
    std::vector<uint> new_fronts;
    std::vector<uint> new_bounds;
    std::unordered_set<uint> dect_front;

    //worst case scenario
    new_fronts.reserve(d.fronts_active.size() + d.m.get_surface_edges().size());

    //index to query the og front and check for new fronts_active
    uint query_front = 0;

    //find an idx in the front outside the visited ones
    // -> if query_front is already visited or is inactive (and we are not out-of-bounds) increase query_front
    while (query_front < d.fronts_active.size() && (CONTAINS_VEC(new_fronts, d.fronts_active[query_front]) || d.m.vert_data(d.fronts_active[query_front]).label))
        query_front++;

    //check for every new front inside the old one (could be no one)
    while (query_front < d.fronts_active.size()) {

        //get the front
        front_from_seed(d, d.fronts_active[query_front], dect_front);

        //add the front to the new vec (if the vert is not alone in the front)
        if(dect_front.size() > 1) {
            new_fronts.insert(new_fronts.end(), dect_front.begin(), dect_front.end());
            new_bounds.emplace_back(new_fronts.size());
        } else
            d.m.vert_data(*begin(dect_front)).label = true;

        //find the next idx
        while (query_front < d.fronts_active.size() && (CONTAINS_VEC(new_fronts, d.fronts_active[query_front]) || d.m.vert_data(d.fronts_active[query_front]).label))
            query_front++;
    }

    //resize of the front vector
    new_fronts.shrink_to_fit();

    //give back the new active front and bounds
    d.fronts_active = new_fronts;
    d.fronts_bounds = new_bounds;

    //std::cout << "DONE - F: " << d.fronts_bounds.size() << " V: " << d.fronts_active.size() << TXT_RESET << std::endl;
}

//get the front from the seed
void front_from_seed(Data &d, uint seed, std::unordered_set<uint> &front) {
    front.clear();
    front.insert(seed);

    std::queue<uint> q;
    q.push(seed);

    while(!q.empty()) {
        uint vid = q.front();
        q.pop();

        for(uint adj_vid : d.m.vert_adj_srf_verts(vid)) {
            if (!d.m.vert_data(adj_vid).label && DOES_NOT_CONTAIN(front, adj_vid)) {
                front.insert(adj_vid);
                q.push(adj_vid);
            }
        }
    }

}

void get_front_dist(Data &d, bool only_ray) {

        uint vid;
        double dist;

        for(int idx = 0; idx < d.fronts_active.size(); idx++) {
            //get the vid
            vid = d.fronts_active.at(idx);
            //get the distance (if the vert is near the target, use the raycast to get the distance)
            dist = dist_calc(d, vid, true);
            if(!only_ray) {
                if(dist < d.inactivity_dist * 2) dist = dist_calc(d, vid, true, true);
                else dist = dist_calc(d, vid, false, true);
            }
            //save the distance
            d.m.vert_data(vid).uvw[DIST] = dist;
        }

}