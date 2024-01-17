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

    std::cout << TXT_BOLDMAGENTA << "Advancing volume ITERATION " << data.step << TXT_RESET;
    if(data.running) std::cout << TXT_BOLDGREEN << " DONE" << TXT_RESET << std::endl;
    else std::cout << TXT_BOLDRED << " STOPPED" << TXT_RESET << std::endl;
    std::cout << std::endl;

    if(data.step != 0 && data.step % data.save_every == 0)
        save_data(data);
}

//move the verts toward the target
void expand(Data &d) {

    uint vid;
    std::queue<uint> try_later;

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

        if(!move(d, vid, movements)) {
            try_later.push(vid);
            if(d.ultra_verbose) std::cout << "Pushed " << vid << " to try_later" << std::endl;
        }
        if(!d.running) return;
    }

    //do all the try-laters
    while(!try_later.empty()) {
        vid = try_later.front();
        if(!move(d, vid, movements)) {
            try_later.push(vid);
            if(d.ultra_verbose) std::cout << "Pushed " << vid << " to try_later" << std::endl;
        }
        try_later.pop();
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
    CGAL_Q rt_moved[3] = {d.exact_coords[vid * 3 + 0] + move.x(),
                          d.exact_coords[vid * 3 + 1] + move.y(),
                          d.exact_coords[vid * 3 + 2] + move.z()};

    //get sure we can move the vert where we want
    moved = topological_unlock(d, vid, rt_moved, rt_move);
    if (!d.running || !moved) return false;

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

    std::map<uint, vec3d> normals;
    std::map<uint, vec3d> new_normals;

    // init normals
    for (uint vid: d.m.get_surface_verts()) {
        normals[vid] = d.m.vert_data(vid).normal * d.m.vert_data(vid).uvw[DIST] * d.mov_speed;
    }

    //* smoothing iterations
    for (int it = 0; it < iter; it++) {

        // compute new normals
        for (uint vid : d.m.get_surface_verts()) {
            new_normals[vid] = normals[vid];
            for (uint adj: d.m.vert_adj_srf_verts(vid))
                new_normals[vid] += normals[adj];
            new_normals[vid] = new_normals[vid] / (d.m.vert_adj_srf_verts(vid).size() + 1);
        }
        normals = new_normals;
    }

    /**/

    return normals;
}

/** ================================================================================================================ **/

//set up the env with model, target, oct etc...
void setup(Data &data, int argc, char *argv[], Octree *oct) {
    parse_input(data, argc, argv);

    if(data.load_model.empty())
        init_data(data, oct);
    else
        load_data(data, oct);

}

void parse_input(Data &d, int argc, char *argv[]) {

    FILE *f = nullptr;
    int load = 0;
    int choice = 0;
    bool check;

    d.save_every = -1;

    //case no input
    switch (argc) {
        //no input -> interactive
        case 1: {

            //menu
            do {
                std::cout << "[1] Load something [0] New model" << std::endl;
                std::cout << "Insert: ";
                std::cin >> load;

                if (load != 1 && load != 0)
                    std::cout << std::endl << "Input not valid... retry" << std::endl;
            } while (load != 1 && load != 0);

            //end line
            std::cout << std::endl;

            //new model
            if (load == 0) {

                do {
                    std::cout << "Target path: ";
                    std::cin >> d.load_target;

                    f = fopen(d.load_target.c_str(), "r");
                    if (f == nullptr)
                        std::cout << std::endl << "Path not valid... retry" << std::endl;
                } while (f == nullptr);
                fclose(f);
                f = nullptr;

                //load model
            } else {

                //input model to load
                do {
                    std::cout << "Model path: ";
                    std::cin >> d.load_model;

                    check = file_check(d.load_model);
                    if (!check) std::cout << std::endl << "Path not valid... retry" << std::endl;
                } while (!check);

                //target path tip
                d.load_target = get_target_path(d.load_model);
                std::cout << "Is \"" << d.load_target << "\"your target path?" << std::endl;
                std::cout << "[1] yes [0] no" << std::endl;
                std::cout << "Input: ";
                std::cin >> choice;
                check = file_check(d.load_target);

                //input target model
                if (choice == 0 || !check) {
                    if (!check) std::cout << "The path is not valid, you have to insert it manually." << std::endl;
                    do {
                        std::cout << "Target path: ";
                        std::cin >> d.load_target;

                        check = file_check(d.load_target);
                        if (!check) std::cout << std::endl << "Path not valid... retry" << std::endl;
                    } while (!check);
                }

                //exact coords tip
                d.load_exact = get_rationals_path(d.load_model);
                std::cout << "Is \"" << d.load_exact << "\" your target path?";
                std::cout << "[1] yes [0] no" << std::endl;
                std::cout << "Input: ";
                std::cin >> choice;
                check = file_check(d.load_exact);

                //input rational coords path
                if (choice == 0 || !check) {
                    if (!check) std::cout << "The path is not valid, you have to insert it manually." << std::endl;
                    do {
                        std::cout << "Rationals path: ";
                        std::cin >> d.load_exact;

                        check = file_check(d.load_exact);
                        if (!check) std::cout << std::endl << "Path not valid... retry" << std::endl;
                    } while (!check);
                }
            }
            break;
        }
        //only target -> new model
        case 2: {
            d.load_target = argv[1];
            check = file_check(d.load_target);
            if(!check) {
                std::cout << TXT_BOLDRED << "Target path not valid" << TXT_RESET << std::endl;
                exit(1);
            }
            break;
        }
        //only target & number (save_every) -> default no rendering & batch true
        case 3: {
            //aux for error check
            char *endptr;

            //get args
            d.load_target = argv[1];
            check = file_check(d.load_target);
            d.save_every = (int) strtol(argv[2], &endptr, 10);
            check = check && endptr != argv[2];

            //default settings
            d.render = false;
            d.batch = true;
            d.ultra_verbose = false;

            //exit
            if(!check) {
                std::cout << TXT_BOLDRED << "Input not valid" << TXT_RESET << std::endl;
                exit(1);
            }

            //no need to go on (ik is bad programming)
            return;
        }
        //all parameters -> load model
        case 4: {
            //get the paths
            d.load_target = argv[1];
            d.load_model = argv[2];
            d.load_exact = argv[3];

            //check on all paths
            check = file_check(d.load_target);
            check = check && file_check(d.load_model);
            check = check && file_check(d.load_exact);

            //exit
            if (!check) {
                std::cout << TXT_BOLDRED << "Paths not valid" << TXT_RESET << std::endl;
                exit(1);
            }
            break;
        }
        //invalid args
        default: {
            std::cout << TXT_BOLDRED << "Args not valid" << TXT_RESET << std::endl;
            exit(1);
            break;
        }
    }

    //save every N steps
    std::cout << "Save every N step: " << std::endl;
    std::cout << "[1] 1 step [2] 10 steps [3] end [0] custom" << std::endl;
    std::cout << "Input: ";
    std::cin >> choice;
    switch (choice) {
        case 1:
            d.save_every = 1;
            break;
        case 2:
            d.save_every = 10;
            break;
        case 3:
            d.save_every = 0;
            break;
        default:
            std::cout << "Insert custom number:";
            std::cin >> d.save_every;
    }

    //render or only terminal
    std::cout << "Do you want the rendering?" << std::endl;
    std::cout << "[1] render [0] no render" << std::endl;
    std::cout << "Input: ";
    std::cin >> choice;
    d.render = choice == 1;
}

void init_data(Data &data, Octree *oct) {

    //model vol
    data.vol = DrawableTetmesh<>(data.load_target.c_str());
    data.vol.show_mesh_points();

    set_param(data);

    //octree
    oct->build_from_mesh_polys(data.srf);
    data.oct = oct;

    //get the sphere
    init_model(data);

    //fronts build
    init_fronts(data);

    // refine the sphere
    std::set<uint> edges_to_split;
    std::map<ipair, uint> v_map;
    std::queue<edge_to_flip> edges_to_flip;
    data.rationals = false;
    std::cout << "Edge threshold: " << data.edge_threshold << std::endl;
    while(data.m.edge_min_length() > data.edge_threshold / 2) {
        std::cout << "Avg edge length: " << data.m.edge_min_length() << std::endl;

        edges_to_split.clear();
        v_map.clear();
        edges_to_flip = std::queue<edge_to_flip>();

        for (int eid = 0; eid < data.m.num_edges(); eid++)
            edges_to_split.insert(eid);
        split(data, edges_to_split, v_map, edges_to_flip);
        flip(data, v_map, edges_to_flip);
    }
    std::cout << "Avg edge length: " << data.m.edge_min_length() << std::endl;
    data.rationals = true;
    data.m.updateGL();

    set_exact_coords(data);

}

void load_data(Data &data, Octree *oct) {

    data.m = DrawableTetmesh<>(data.load_model.c_str());
    data.vol = DrawableTetmesh<>(data.load_target.c_str());
    data.vol.show_mesh_points();

    //set other parameters of data struct
    set_param(data);

    //octree
    oct->build_from_mesh_polys(data.srf);
    data.oct = oct;

    //load rationals
    std::string &r_path = data.m.mesh_data().filename;
    size_t pos = r_path.find_last_of('.');
    r_path = (pos>=r_path.size()) ? r_path : r_path.substr(0, pos+1);
    r_path += "txt";
    load_rationals(data, r_path);

    assert(data.exact_coords.size() / 3 == data.m.num_verts());

    //load fronts
    load_fronts(data);

}

void save_data(Data &d) {

    std::cout << std::endl << "SAVING..." << std::endl;
    std::string name = get_file_name(d.vol.mesh_data().filename, false);

    std::string folder_path = "../" + name + "/";
    std::cout << "Folder path: " << folder_path << std::endl;

    std::string mesh_path = folder_path + name + "_" + std::to_string(d.step) + ".mesh";
    std::cout << "Tetmesh: " << mesh_path << std::endl;

    std::string rat_path = folder_path + name + "_" + std::to_string(d.step) + ".txt";
    std::cout << "Rationals: " << rat_path << std::endl;

    /*
    std::string srf_path = folder_path + name + ".obj";
    std::cout << "Trimesh: " << srf_path << std::endl;
    */

    d.m.save(mesh_path.c_str());
    save_rationals(d, rat_path);
    //d.srf.save(srf_path.c_str());

    std::cout << "DONE" << std::endl;
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
    d.m = get_sphere(d.vol.vert(center_vid), center_dist * d.sphere_rad);
}

void set_param(Data &d) {

    //model srf
    export_surface(d.vol, d.srf);

    //edge length threshold
    d.edge_threshold = d.srf.bbox().diag() * 0.02;

    //inactive threshold
    d.eps_inactive = d.edge_threshold * d.eps_percent;

}

/** ================================================================================================================ **/

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

void save_rationals(Data &d, std::string &path) {
    FILE *file = nullptr;
    file = fopen(path.c_str(), "w");
    if (file == nullptr) exit(1);

    for(auto &rt : d.exact_coords) {
        mpq_out_str(file, 10, rt.exact().mpq());
        fprintf(file, "\n");
    }

    fclose(file);
}

void load_rationals(Data &d, std::string &path) {

    FILE *file = nullptr;
    file = fopen(path.c_str(), "r");
    if (file == nullptr) exit(1);

    int check;
    mpq_ptr aux = nullptr;
    aux = (mpq_ptr) malloc(sizeof(__mpq_struct));
    if(aux == nullptr) exit(1);
    do {
        check = mpq_inp_str(aux, file, 10);
        if(check != 0) d.exact_coords.emplace_back(aux);
    } while(check != 0);

    free(aux);
    fclose(file);

}