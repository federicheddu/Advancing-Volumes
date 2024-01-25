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
    bool show_only_adj = false;
    bool show_mov_diff = false;
    std::vector<DrawableArrow> dir_arrows;
    std::vector<vec3d> movements;
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

        if(ImGui::Button("R:Normal/B:Movement")) {

            show_mov_diff = !show_mov_diff;

            if(show_mov_diff) {

                //vert and displacement
                vec3d v, d;

                data.m.update_normals();
                get_front_dist(data);

                //normals (RED)
                norms.use_gl_lines = true;
                norms.default_color = Color::RED();
                for(auto vid : data.fronts_active) {
                    v = data.m.vert(vid);
                    d = data.m.vert_data(vid).normal * data.m.vert_data(vid).uvw[DIST] * 0.99;
                    norms.push_seg(v, v+d);
                }

                //movement (BLUE)
                movs.use_gl_lines = true;
                movs.default_color = Color::BLUE();
                compute_movements(data);
                for(auto vid : data.fronts_active) {
                    v = data.m.vert(vid);
                    d = data.m.vert_data(vid).normal;
                    movs.push_seg(v, v+d);
                }

                //push on gui
                gui.push(&norms, false);
                gui.push(&movs, false);

                //restore normals
                data.m.update_normals();

            } else { //don't show the displacement
                gui.pop(&norms);
                gui.pop(&movs);
                norms.clear();
                movs.clear();
            }

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

        if(ImGui::Button("Give me distance")) {
            std::cout << "Vid: " << sel_vid << std::endl;
            std::cout << "Distance: " << data.m.vert_data(sel_vid).uvw[DIST] << std::endl;
        }

        if(ImGui::Button("Is active")) {
            std::cout << "Vid: " << sel_vid << std::endl;
            if(CONTAINS_VEC(data.fronts_active, sel_vid))
                std::cout << "Active" << std::endl;
            else
                std::cout << "Inactive" << std::endl;
        }

        if(ImGui::Button("Put marker")) {
            gui.push_marker(data.m.vert(sel_vid), std::to_string(sel_vid), Color::RED(), 2, 4);
        }

        if(ImGui::Button("Delete markers")) {
            gui.pop_all_markers();
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

    int under100 = 0;
    if(!data.render) {
        while(!data.fronts_active.empty() && data.running && data.step < 50 && under100 < 10) {
            advancing_volume(data);
            if(data.fronts_active.size() < 100) under100++;
        }
    }

    return data.render ? gui.launch() : 0;

    /**/
}