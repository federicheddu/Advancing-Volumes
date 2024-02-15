#include "advancing_io.h"


//set up the env with model, target, oct etc...
void setup(Data &data, int argc, char *argv[], Octree *oct) {
    parse_input(data, argc, argv);

    if(data.load_model.empty())
        init_data(data, oct);
    else
        load_data(data, oct);

    for(uint vid : data.m.get_surface_verts()) {
        data.m.vert_data(vid).flags[SPLIT] = false;
        data.m.vert_data(vid).flags[TOPOLOGICAL] = false;
    }
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
                std::cout << "[1] Load from dataset [2] New model [0] Load" << std::endl;
                std::cout << "Insert: ";
                std::cin >> load;

                if (load != 1 && load != 0)
                    std::cout << std::endl << "Input not valid... retry" << std::endl;
            } while (load != 1 && load != 0);

            //end line
            std::cout << std::endl;

            switch (load) {
                case 1: {
                    d.load_target = dataset_string();
                    break;
                }
                case 2: {
                    do {
                        std::cout << "Target path: ";
                        std::cin >> d.load_target;

                        f = fopen(d.load_target.c_str(), "r");
                        if (f == nullptr)
                            std::cout << std::endl << "Path not valid... retry" << std::endl;
                    } while (f == nullptr);
                    fclose(f);
                    f = nullptr;
                }
                case 0: { //load model

                    //input model to load
                    do {
                        std::cout << "Model path: ";
                        std::cin >> d.load_model;

                        check = file_check(d.load_model);
                        if (!check) std::cout << std::endl << "Path not valid... retry" << std::endl;
                    } while (!check);

                    //target path tip
                    d.load_target = get_target_path(d.load_model);
                    std::cout << "Is \"" << d.load_target << "\" your target path?" << std::endl;
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
                    std::cout << "Is \"" << d.load_exact << "\" your rationals path?" << std::endl;
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
                    break;
                }
                default: {
                    std::cout << TXT_BOLDRED << "Input not valid" << TXT_RESET << std::endl;
                    exit(1);
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
    std::cout << "Edge threshold: " << data.target_edge_length << std::endl;
    while(data.m.edge_min_length() > data.target_edge_length / 2) {
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

    errorcheck(data, data.exact_coords.size() / 3 == data.m.num_verts(), "Non torna il numero di vertici nella load");

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

bool have_to_save(Data &d) {
    bool hts;
    int real_step = d.step_by_step ? d.step / NUM_STEPS : d.step;

    hts = d.save_every != 0;
    hts = hts && real_step % d.save_every == 0;
    hts = hts || !d.running;

    return hts;
}

void save_data(Data &d) {

    if(d.ultra_verbose)
        std::cout << TXT_BOLDYELLOW << "SAVING..." << TXT_RESET << std::endl;

    //get model name
    std::string name = get_file_name(d.vol.mesh_data().filename, false);

    //get folder path
    std::string folder_path = "../results/" + name + "/";
    system(("mkdir -p " + folder_path).c_str());
    if(d.ultra_verbose)
        std::cout << "Folder path: " << folder_path << std::endl;

    //get model path
    std::string mesh_path = folder_path + name + "_" + std::to_string(d.step) + ".mesh";
    if(d.ultra_verbose)
        std::cout << "Tetmesh: " << mesh_path << std::endl;

    //get rationals path
    std::string rat_path = folder_path + name + "_" + std::to_string(d.step) + ".txt";
    if(d.ultra_verbose)
        std::cout << "Rationals: " << rat_path << std::endl;

    //save
    d.m.save(mesh_path.c_str());
    save_rationals(d, rat_path);

    if(d.ultra_verbose)
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
    d.target_edge_length = d.srf.edge_avg_length() * 1.5;

    //inactive threshold
    d.inactivity_dist = d.target_edge_length * d.eps_percent;

}

std::string dataset_string() {

    int choice;
    std::vector<std::string> data_paths = { "../data/airplane.mesh",             //0
                                            "../data/angel1.mesh",               //1
                                            "../data/angel2.mesh",               //2
                                            "../data/angel3.mesh",               //3
                                            "../data/ant.mesh",                  //4
                                            "../data/armadillo.mesh",            //5
                                            "../data/armadillo_deformed.mesh",   //6
                                            "../data/bimba.mesh",                //7
                                            "../data/bird.mesh",                 //8
                                            "../data/blade.mesh",                //9
                                            "../data/bone.mesh",                 //10
                                            "../data/bumpy_sphere.mesh",         //11
                                            "../data/bunny.mesh",                //12
                                            "../data/buste.mesh",                //13
                                            "../data/camile_hand.mesh",          //14
                                            "../data/chamfer.mesh",              //15
                                            "../data/chinese_dragon.mesh",       //16
                                            "../data/cubespikes.mesh",           //17
                                            "../data/david.mesh",                //18
                                            "../data/devil.mesh",                //19
                                            "../data/dilo.mesh",                 //20
                                            "../data/dog.mesh",                  //21
                                            "../data/duck.mesh",                 //22
                                            "../data/fandisk.mesh",              //23
                                            "../data/femur.mesh",                //24
                                            "../data/foot.mesh",                 //25
                                            "../data/frog.mesh",                 //26
                                            "../data/gargoyle.mesh",             //27
                                            "../data/gear.mesh",                 //28
                                            "../data/hand.mesh",                 //29
                                            "../data/hand_olivier.mesh",         //30
                                            "../data/homer.mesh",                //31
                                            "../data/horse.mesh",                //32
                                            "../data/isidora_horse.mesh",        //33
                                            "../data/lego.mesh",                 //34
                                            "../data/lion.mesh",                 //35
                                            "../data/memento.mesh",              //36
                                            "../data/moai.mesh",                 //37
                                            "../data/mouse.mesh",                //38
                                            "../data/octa_flower.mesh",          //39
                                            "../data/pig.mesh",                  //40
                                            "../data/ramses.mesh",               //41
                                            "../data/sphere.mesh",               //42
                                            "../data/sphinx.mesh",               //43
                                            "../data/spider.mesh",               //44
                                            "../data/stag3.mesh",                //45
                                            "../data/table4.mesh",               //46
                                            "../data/table4_2.mesh",             //47
                                            "../data/table8.mesh"                //48
    };

    std::cout << "Choose a model from the list:" << std::endl;
    for(int i = 0; i < data_paths.size(); i++) {
        printf(TXT_BOLDWHITE "[%2d] " TXT_RESET "%-24s", i, get_file_name(data_paths[i], false).c_str());
        if(i%3 == 2 || i == data_paths.size()-1) std::cout << std::endl;
        else std::cout << "\t";
    }

    do {
        std::cout << "Input: ";
        std::cin >> choice;

        if(choice < 0 || choice >= data_paths.size())
            std::cout << "Input not valid... retry" << std::endl;
    } while(choice < 0 || choice >= data_paths.size());

    return data_paths[choice];
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