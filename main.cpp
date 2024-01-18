#include "advancing_volumes.h"
#undef NDEBUG

using namespace cinolib;

/*      Reference
 *      Tetrahedron:
 *
 *          v3
 *         /| \
 *       /  |   \
 *     v0---|----v2
 *      \   |   /
 *        \ | /
 *          v1
*/

int main(int argc, char *argv[]) {

    std::vector<std::string> data_paths = { "../data/cubespikes.mesh",           //0
                                            "../data/armadillo.mesh",            //1
                                            "../data/bunny.mesh",                //2
                                            "../data/lego.mesh",                 //3
                                            "../data/table8.mesh",               //4
                                            "../data/spider.mesh",               //5
                                            "../data/ant.mesh",                  //6
                                            "../data/chinese_dragon.mesh",       //7
                                            "../data/bimba.mesh",                //8
                                            "../data/table4.mesh",               //9
                                            "../data/stag3.mesh",                //10
                                            "../data/table4_2.mesh",             //11
                                            "../data/horse.mesh",                //12
                                            "../data/gear.mesh",                 //13
                                            "../data/chamfer.mesh",              //14
                                            "../data/hand_olivier.mesh",         //15
                                            "../data/sphere.mesh",               //16
                                            "../data/femur.mesh",                //17
                                            "../data/fandisk.mesh",              //18
                                            "../data/frog.mesh",                 //19
                                            "../data/hand.mesh",                 //20
                                            "../data/blade.mesh",                //21
                                            "../data/buste.mesh",                //22
                                            "../data/memento.mesh",              //23
                                            "../data/sphinx.mesh",               //24
                                            "../data/bone.mesh",                 //25
                                            "../data/foot.mesh",                 //26
                                            "../data/bird.mesh",                 //27
                                            "../data/moai.mesh",                 //28
                                            "../data/airplane.mesh",             //29
                                            "../data/mouse.mesh",                //30
                                            "../data/pig.mesh",                  //31
                                            "../data/isidora_horse.mesh",        //32
                                            "../data/david.mesh",                //33
                                            "../data/octa_flower.mesh",          //34
                                            "../data/duck.mesh",                 //35
                                            "../data/ramses.mesh",               //36
                                            "../data/lion.mesh",                 //37
                                            "../data/armadillo_deformed.mesh",   //38
                                            "../data/homer.mesh",                //39
                                            "../data/devil.mesh",                //40
                                            "../data/camile_hand.mesh",          //41
                                            "../data/bumpy_sphere.mesh",         //42
                                            "../data/gargoyle.mesh",             //43
                                            "../data/dilo.mesh",                 //44
                                            "../data/angel2.mesh",               //45
                                            "../data/angel3.mesh",               //46
                                            "../data/angel1.mesh",               //47
                                            "../data/dog.mesh"                   //48
    };

    //load the data
    Data data;
    Octree oct;
    setup(data, argc, argv, &oct);

    //UI
    GLcanvas gui(1080, 720);
    gui.side_bar_alpha = 0.8;
    gui.depth_cull_markers = false;
    data.gui = &gui;

    //gui push
    gui.push(&data.m, false);
    gui.push(new VolumeMeshControls<DrawableTetmesh<>>(&data.m, &gui));
    gui.push(&data.vol, true);
    gui.push(new VolumeMeshControls<DrawableTetmesh<>>(&data.vol, &gui));

    //visualization
    UI_Mode uiMode = BLANK;
    bool show_target = true;
    bool show_target_matte = false;
    bool show_stuck_verts = false;
    bool show_stuck_edges = false;
    bool show_only_adj = false;
    bool show_mov_diff = false;
    std::vector<DrawableArrow> dir_arrows;
    std::map<uint, vec3d> movements;
    DrawableSegmentSoup norms;
    DrawableSegmentSoup movs;
    movs.default_color = Color::BLUE();
    //vert movement parameters
    data.fronts_active = data.m.get_surface_verts();
    data.fronts_bounds.emplace_back(data.m.num_srf_verts());
    //undo object
    Data reset_data = data;
    Data undo_data = data;
    //selection
    int sel_vid = 0;
    int sel_pid = 0;

    //GUI button callbacks
    if(data.render) gui.callback_app_controls = [&]() {

        if(ImGui::Button("Expand and Refine")) {
            //undo backup
            undo_data = data;

            advancing_volume(data);

            //update UI
            UI_Manager(data.m, uiMode, data.oct, dir_arrows, data.fronts_active, gui);
            data.m.updateGL();
        }

        if(ImGui::Button("Expand and Refine x10"))  {
            undo_data = data;

            for(int i = 0; i < 10 && data.running; i++) {
                std::cout << TXT_BOLDMAGENTA << "Iter: " << i+1 << TXT_RESET << std::endl;
                advancing_volume(data);
            }

            //update model and UI
            UI_Manager(data.m, uiMode, data.oct, dir_arrows, data.fronts_active, gui);
            data.m.updateGL();
        }

        if(ImGui::Button("Projection")) {
            undo_data = data;

            final_projection(data);

            //update model and UI
            data.m.update_normals();
            UI_Manager(data.m, uiMode, data.oct, dir_arrows, data.fronts_active, gui);
            data.m.updateGL();
        }

        if(ImGui::Button("Go until convergence")) {

            data.ultra_verbose = false;

            while(!data.fronts_active.empty() && data.running)
                advancing_volume(data);

            UI_Manager(data.m, uiMode, data.oct, dir_arrows, data.fronts_active, gui);
            data.m.updateGL();
        }

        if(ImGui::Button("Old vs New movement")) {

            show_mov_diff = !show_mov_diff;

            if(show_mov_diff) {

                if(data.step == 0)
                    get_front_dist(data);

                norms.use_gl_lines = true;
                norms.default_color = Color::RED();
                movs.use_gl_lines = true;
                movs.default_color = Color::BLUE();

                movements = get_movements(data);

                for(int idx = 0; idx < data.fronts_active.size(); idx++) {

                    uint vid = data.fronts_active.at(idx);

                    //normal displacement
                    vec3d v = data.m.vert(vid);
                    vec3d n = data.m.vert_data(vid).normal;
                    double l = data.m.vert_data(vid).uvw[DIST];
                    vec3d d = n*l;
                    norms.push_seg(v, v+d);

                    //movement with avg
                    for(auto avid : data.m.vert_adj_srf_verts(vid)) {
                        n = data.m.vert_data(avid).normal;
                        l = data.m.vert_data(avid).uvw[DIST];
                        d += n*l;
                    }
                    d = d / (data.m.vert_adj_srf_verts(vid).size() + 1);
                    movs.push_seg(v, v+d);
                }

                gui.push(&norms, false);
                gui.push(&movs, false);

            } else { //dont show the displacement
                gui.pop(&norms);
                gui.pop(&movs);
                norms.clear();
                movs.clear();
            }

        }

        if(ImGui::Button("Debug panic")) {

            std::map<uint, vec3d> movements = get_movements(data, 1);

            if(data.step == 0)
                get_front_dist(data);

            for(uint vid : data.m.get_surface_verts()) {
                vec3d move = movements[vid];
                vec3d v = data.m.vert(vid);
                norms.push_seg(v, v+move);
            }

            gui.push(&norms, false);
        }

        if(ImGui::Button("Move single vid")) {

        }

        ImGui::Text("===========================");

        if(ImGui::Button("Jacobian info")) {

            double jc, sum, avg, min, max;
            jc = tet_scaled_jacobian(data.m.poly_vert(0, 0),
                                     data.m.poly_vert(0, 1),
                                     data.m.poly_vert(0, 2),
                                     data.m.poly_vert(0, 3));
            sum = min = max = jc;
            for(uint pid = 1; pid < data.m.num_polys(); pid++) {
                jc = tet_scaled_jacobian(data.m.poly_vert(pid, 0),
                                         data.m.poly_vert(pid, 1),
                                         data.m.poly_vert(pid, 2),
                                         data.m.poly_vert(pid, 3));
                sum += jc;
                if(jc < min) min = jc;
                if(jc > max) max = jc;
            }
            avg = sum / (double)data.m.num_polys();

            std::cout << TXT_BOLDYELLOW << "Jacobian INFO" << TXT_RESET << std::endl;
            std::cout << "Avg SJ: " << avg << std::endl;
            std::cout << "Min: " << min << std::endl;
            std::cout << "Max: " << max << std::endl << std::endl;

        }

        if(ImGui::Button("Show stuck verts")) {
            show_stuck_verts = !show_stuck_verts;

            if(show_stuck_verts) {
                std::cout << TXT_BOLDGREEN << "Showing stuck verts" << TXT_RESET << std::endl;
                show_stuck_v(data.m, data.stuck_in_place, gui);
            } else {
                std::cout << TXT_BOLDGREEN << "Hiding stuck verts" << TXT_RESET << std::endl;
                gui.pop_all_markers();
            }
            data.m.updateGL();
        }

        if(ImGui::Button("Show stuck edges")) {
            show_stuck_edges = !show_stuck_edges;

            if(show_stuck_edges)
                show_stuck_e(data.m, data.stuck_in_place, gui);
            else
                data.m.show_marked_edge(false);

            data.m.updateGL_marked();
            data.m.updateGL();
        }

        ImGui::Text("===========================");

        if(ImGui::InputInt("vid", &sel_vid), 0) {}

        if(ImGui::Button("Adj SJ")) {
            std::cout << TXT_BOLDGREEN << "Selected vert: " << sel_vid;
            std::cout << " - num adj: " << data.m.adj_v2p(sel_vid).size() << TXT_RESET << std::endl;

            for(auto pid : data.m.adj_v2p(sel_vid))
                std::cout << "pid: " << pid << " SJ: " << tet_scaled_jacobian(data.m.poly_vert(pid, 0), data.m.poly_vert(pid, 1), data.m.poly_vert(pid, 2), data.m.poly_vert(pid, 3)) << std::endl;
        }

        if(ImGui::Button("Show only adj")) {
            show_only_adj = !show_only_adj;

            if(show_only_adj) {
                for(int pid = 0; pid < data.m.num_polys(); pid++)
                    data.m.poly_data(pid).flags[HIDDEN] = true;
                for(uint pid : data.m.adj_v2p(sel_vid))
                    data.m.poly_data(pid).flags[HIDDEN] = false;
            } else {
                for(int pid = 0; pid < data.m.num_polys(); pid++)
                    data.m.poly_data(pid).flags[HIDDEN] = false;
            }

            data.m.updateGL();
        }

        ImGui::Text("===========================");

        if(ImGui::InputInt("pid", &sel_pid), 0) {}

        if(ImGui::Button("HL in red")) {
            data.m.poly_data(sel_pid).color = Color::PASTEL_MAGENTA();
            data.m.updateGL();
        }

        ImGui::Text("===========================");

        if(ImGui::Button("Undo")) {

            //counter back
            data.step--;
            std::cout << TXT_BOLDYELLOW << "UNDO to ITERATION " << data.step << TXT_RESET << std::endl << std::endl;

            //pop of all
            gui.pop(&data.m);

            //data recover
            data.m = undo_data.m;
            data.fronts_active = undo_data.fronts_active;
            data.fronts_bounds = undo_data.fronts_bounds;

            //UI
            UI_Manager(data.m, uiMode, data.oct, dir_arrows, data.fronts_active, gui);

            //re-push on gui
            gui.push(&data.m, false);
            data.m.updateGL();
        }

        if(ImGui::Button("Reset")) {
            //pop of all
            gui.pop(&data.m);

            //data recover
            data.m = reset_data.m;
            data.fronts_active = reset_data.fronts_active;
            data.fronts_bounds = reset_data.fronts_bounds;

            //UI
            UI_Manager(data.m, uiMode, data.oct, dir_arrows, data.fronts_active, gui);

            //re-push on gui
            gui.push(&data.m, false);
            data.m.updateGL();
        }

        if(ImGui::Button("Save")) {
            save_data(data);
        }

        ImGui::Text("===========================");

        if(ImGui::Button("Clear UI")) {
            uiMode = BLANK;
            UI_Manager(data.m, uiMode, data.oct, dir_arrows, data.fronts_active, gui);
            data.m.updateGL();
        }

        if(ImGui::Button("Highlight model")) {
            uiMode = HIGHLIGHT;
            UI_Manager(data.m, uiMode, data.oct, dir_arrows, data.fronts_active, gui);
            data.m.updateGL();
        }

        if(ImGui::Button("Show Fronts")) {
            uiMode = FRONTS;
            UI_Manager(data.m, uiMode, data.oct, dir_arrows, data.fronts_active, gui);
            data.m.updateGL();
        }

        if(ImGui::Button("Show Target")) {
            data.vol.show_mesh(show_target = !show_target);
        }

        if(ImGui::Button("Matte target")) {
            show_target_matte = !show_target_matte;
            if(show_target_matte) data.vol.show_mesh_flat();
            else data.vol.show_mesh_points();
        }

        if(ImGui::Button("Show vert direction")) {
            uiMode = DIRECTION;
            UI_Manager(data.m, uiMode, data.oct, dir_arrows, data.fronts_active, gui);
        }

        if(ImGui::Button("Show orient3D sign")) {
            uiMode = VOLUME;
            UI_Manager(data.m, uiMode, data.oct, dir_arrows, data.fronts_active, gui);
            data.m.updateGL();
        }

        if(ImGui::Button("Screen colors")) {
            for (uint &f : data.m.get_surface_faces()) {
                uint poly = data.m.adj_f2p(f)[0];
                data.m.poly_data(poly).color = Color(255.f/255.f, 198.f/255.f, 70.f/255.f);
            }
            data.m.updateGL();
            // 1800 1035 0.0157587 -0.00979251 0.223252 0.660849 0.5 -0.0462633 -0.674176 0.73712 -0.0149085 0.719672 -0.534246 -0.443457 0.0975828 0.692771 0.509969 0.509902 -0.1274 0 0 0 1 1 0 0 -0 0 1 0 -0 0 0 1 -2.6434 0 0 0 1 0.870092 0 0 -0 0 1.5132 0 -0 0 0 -0.756602 -2 0 0 0 1
        }

    };

    if(!data.render) {
        while(!data.fronts_active.empty() && data.running)
            advancing_volume(data);
    }

    return data.render ? gui.launch() : 0;

    /**/
}