#include "advancing_io.h"


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
    load_rationals(data);

    assert(data.exact_coords.size() / 3 == data.m.num_verts());

    if(data.render) {
        for(uint vid = 0; vid < data.m.num_verts(); vid++)
            data.m.vert(vid) = vec3d(CGAL::to_double(data.exact_coords[3*vid+0]),
                                     CGAL::to_double(data.exact_coords[3*vid+2]),
                                     CGAL::to_double(data.exact_coords[3*vid+1]));
    }

    data.m.update_normals();

    //load fronts
    load_fronts(data);

    std::string name = get_file_name(data.load_model, false);
    size_t pos = name.find_last_of('_');
    data.step = stoi(name.substr(pos+1));
    data.m.mesh_data().filename = data.load_target;

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

void load_rationals(Data &d) {

    FILE *file = nullptr;
    file = fopen(d.load_exact.c_str(), "r");
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