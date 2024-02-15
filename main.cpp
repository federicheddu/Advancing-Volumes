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
    data.gui = &gui;

    //gui push
    gui.push(&data.m, false);
    gui.push(new VolumeMeshControls<DrawableTetmesh<>>(&data.m, &gui));
    gui.push(&data.vol, true);
    gui.push(new VolumeMeshControls<DrawableTetmesh<>>(&data.vol, &gui));

    //visualization
    UI_Mode uiMode = BLANK;
    bool show_fronts = false;
    bool show_target = true;
    bool show_target_matte = false;
    bool show_only_adj = false;
    bool show_mov_diff = false;
    bool show_srf_smooth = false;
    bool show_vert_origin = false;
    bool show_edges_to_refine = false;
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

        //parameters checkbox for execution
        ImGui::SetNextItemOpen(false,ImGuiCond_Once);
        if(ImGui::TreeNode("Params")) {

            ImGui::Text("Line Search");
            ImGui::Checkbox("Line Search", &data.line_search);
            ImGui::Checkbox("Forward LS", &data.forward_line_search);

            ImGui::Text("Movement");
            ImGui::Checkbox("Only Raycast", &data.only_raycast);
            ImGui::Checkbox("Priority Queue", &data.priority_queue);
            ImGui::Checkbox("Check Intersection", &data.check_intersections);

            ImGui::Text("Refinement");
            ImGui::Checkbox("Internal Refinement", &data.internal_refinement);
            ImGui::Checkbox("Multiple Refinement", &data.multiple_refinement);

            ImGui::Text("Optimization");
            ImGui::Checkbox("Smoothing", &data.smoothing);
            ImGui::InputInt("Iters", &data.smooth_iters);
            ImGui::Checkbox("Smooth Internal", &data.smooth_internal);
            ImGui::Checkbox("Smooth Projection", &data.smooth_project);
            ImGui::Checkbox("Srf mesh smoother", &data.smooth_smooth);

            ImGui::Text("Text and debug options");
            ImGui::Checkbox("Verbose", &data.verbose);
            ImGui::Checkbox("Ultra Verbose", &data.ultra_verbose);
            ImGui::Checkbox("Debug Colors", &data.debug_colors);

            ImGui::Text("Other");
            ImGui::Checkbox("Snap Rounding", &data.enable_snap_rounding);

            ImGui::TreePop();
        }

        ImGui::SetNextItemOpen(false,ImGuiCond_Once);
        if(ImGui::TreeNode("Advancing Volume")) {

            //my algorithm
            if(ImGui::Button("Advancing Volume")) {
                //undo backup
                undo_data = data;
                //1-step algorithm
                advancing_volume(data);
                //update UI
                UI_Manager(data, uiMode, dir_arrows);
                data.m.updateGL();
            }

            if(ImGui::Button("ON/OFF S.b.S.")) {

                //need to turn off
                if(data.step_by_step && data.step % NUM_STEPS != 0) {
                    //undo backup
                    undo_data = data;
                    //end of "macro" iteration
                    while(data.step % NUM_STEPS != 0)
                        advancing_volume(data);
                }
                data.step_by_step = !data.step_by_step;

                std::cout << std::endl;
                std::cout << TXT_BOLDYELLOW << "STEP BY STEP MODE: ";
                std::cout << (data.step_by_step ? TXT_BOLDGREEN : TXT_BOLDRED);
                std::cout << (data.step_by_step ? "ON" : "OFF");
                std::cout << TXT_RESET << std::endl << std::endl;

                //update UI
                UI_Manager(data, uiMode, dir_arrows);
                data.m.updateGL();
            }

            //check where every vert will go
            if (ImGui::Button("R:Normal/B:Movement")) {

                show_mov_diff = !show_mov_diff;

                if (show_mov_diff) {

                    //vert and displacement
                    vec3d v, d;
                    data.m.update_normals();

                    //normals (RED)
                    norms.use_gl_lines = true;
                    norms.default_color = Color::RED();
                    compute_distances(data);
                    compute_displacements(data);
                    for (auto vid: data.fronts_active) {
                        v = data.m.vert(vid);
                        d = data.m.vert_data(vid).normal;
                        norms.push_seg(v, v + d);
                    }

                    //movement (BLUE)
                    movs.use_gl_lines = true;
                    movs.default_color = Color::BLUE();
                    compute_directions(data);
                    compute_distances(data);
                    compute_displacements(data);
                    for (auto vid: data.fronts_active) {
                        v = data.m.vert(vid);
                        d = data.m.vert_data(vid).normal;
                        movs.push_seg(v, v + d);
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

            ImGui::TreePop();
        }

        ImGui::SetNextItemOpen(false,ImGuiCond_Once);
        if(ImGui::TreeNode("Info")) {

            if(ImGui::Button("Jacobian info")) {

                double jc, sum, avg, min, max;
                jc = tet_scaled_jacobian(data.m.poly_vert(0, 0),
                                         data.m.poly_vert(0, 1),
                                         data.m.poly_vert(0, 2),
                                         data.m.poly_vert(0, 3));
                sum = min = max = jc;
                for (uint pid = 1; pid < data.m.num_polys(); pid++) {
                    jc = tet_scaled_jacobian(data.m.poly_vert(pid, 0),
                                             data.m.poly_vert(pid, 1),
                                             data.m.poly_vert(pid, 2),
                                             data.m.poly_vert(pid, 3));
                    sum += jc;
                    if (jc < min) min = jc;
                    if (jc > max) max = jc;
                }
                avg = sum / (double) data.m.num_polys();

                std::cout << TXT_BOLDYELLOW << "Jacobian INFO" << TXT_RESET << std::endl;
                std::cout << "Avg SJ: " << avg << std::endl;
                std::cout << "Min: " << min << std::endl;
                std::cout << "Max: " << max << std::endl << std::endl;

            }

            if(ImGui::Button("Edge Lenghts")) {
                std::cout << "TEL: " << data.target_edge_length << std::endl;
                std::cout << "Max active edge: " << get_max_edge_length(data, true) << std::endl;
                std::cout << "Min active edge: " << get_min_edge_length(data, true) << std::endl;
                std::cout << "Avg active edge: " << get_avg_edge_length(data, true) << std::endl;

            }

            if(ImGui::Button("Show edges to refine")) {
                show_edges_to_refine = !show_edges_to_refine;

                if(show_edges_to_refine) {
                    for(uint eid : data.m.get_surface_edges())
                        data.m.edge_data(eid).flags[MARKED] = false;

                    for(uint eid : data.m.get_surface_edges())
                        //if(one of the vids is active)
                        if(!data.m.vert_data(data.m.edge_vert_id(eid, 0)).label || !data.m.vert_data(data.m.edge_vert_id(eid, 1)).label)
                            if(data.target_edge_length < data.m.edge_length(eid))
                                data.m.edge_data(eid).flags[MARKED] = true;
                    data.m.show_marked_edge(true);
                    data.m.updateGL();
                } else {
                    for(uint eid : data.m.get_surface_edges())
                        data.m.edge_data(eid).flags[MARKED] = false;
                    data.m.show_marked_edge(false);
                    data.m.updateGL();
                }
            }

            ImGui::Text("===========================");

            if (ImGui::InputInt("vid", &sel_vid), 0) {}

            if (ImGui::Button("Adj SJ")) {
                std::cout << TXT_BOLDGREEN << "Selected vert: " << sel_vid;
                std::cout << " - num adj: " << data.m.adj_v2p(sel_vid).size() << TXT_RESET << std::endl;

                for (auto pid: data.m.adj_v2p(sel_vid))
                    std::cout << "pid: " << pid << " SJ: "
                              << tet_scaled_jacobian(data.m.poly_vert(pid, 0), data.m.poly_vert(pid, 1),
                                                     data.m.poly_vert(pid, 2), data.m.poly_vert(pid, 3)) << std::endl;
            }

            if (ImGui::Button("Show only adj")) {
                show_only_adj = !show_only_adj;

                if (show_only_adj) {
                    for (int pid = 0; pid < data.m.num_polys(); pid++)
                        data.m.poly_data(pid).flags[HIDDEN] = true;
                    for (uint pid: data.m.adj_v2p(sel_vid))
                        data.m.poly_data(pid).flags[HIDDEN] = false;
                } else {
                    for (int pid = 0; pid < data.m.num_polys(); pid++)
                        data.m.poly_data(pid).flags[HIDDEN] = false;
                }

                data.m.updateGL();
            }

            if (ImGui::Button("Give me distance")) {
                std::cout << "Vid: " << sel_vid << std::endl;
                std::cout << "Distance: " << data.m.vert_data(sel_vid).uvw[DIST] << std::endl;
                std::cout << "Inactivity at: " << data.inactivity_dist << std::endl;
            }

            if (ImGui::Button("Is active")) {
                bool is_active;

                std::cout << "Vid: " << sel_vid << std::endl;
                is_active = CONTAINS_VEC(data.fronts_active, sel_vid);
                std::cout << "Active list: " << (is_active ? "Active" : "Inactive") << std::endl;
                is_active = !data.m.vert_data(sel_vid).label;
                std::cout << "Active label: " << (is_active ? "Active" : "Inactive") << std::endl;
            }

            if (ImGui::Button("Put marker")) {
                gui.push_marker(data.m.vert(sel_vid), std::to_string(sel_vid), Color::RED(), 2, 4);
            }

            if (ImGui::Button("Delete markers")) {
                gui.pop_all_markers();
            }

            ImGui::Text("===========================");

            if (ImGui::InputInt("pid", &sel_pid), 0) {}

            if (ImGui::Button("HL in red")) {
                data.m.poly_data(sel_pid).color = Color::PASTEL_MAGENTA();
                data.m.updateGL();
            }

            ImGui::TreePop();
        }

        ImGui::SetNextItemOpen(false,ImGuiCond_Once);
        if(ImGui::TreeNode("Actions")) {

            if (ImGui::Button("Undo")) {

                //pop of all
                gui.pop(&data.m);

                //data recover
                data = undo_data;
                std::cout << TXT_BOLDYELLOW << "UNDO to ITERATION " << data.step << TXT_RESET << std::endl << std::endl;

                //UI
                UI_Manager(data, uiMode, dir_arrows);

                //re-push on gui
                gui.push(&data.m, false);
                data.m.updateGL();
            }

            if (ImGui::Button("Reset")) {
                //pop of all
                gui.pop(&data.m);

                //data recover
                data.m = reset_data.m;
                data.fronts_active = reset_data.fronts_active;
                data.fronts_bounds = reset_data.fronts_bounds;

                //UI
                UI_Manager(data, uiMode, dir_arrows);

                //re-push on gui
                gui.push(&data.m, false);
                data.m.updateGL();
            }

            if (ImGui::Button("Save")) {
                save_data(data);
            }

            ImGui::TreePop();
        }

        ImGui::SetNextItemOpen(false,ImGuiCond_Once);
        if(ImGui::TreeNode("Visual")) {

            if (ImGui::Button("Clear UI")) {
                uiMode = BLANK;
                UI_Manager(data, uiMode, dir_arrows);
                data.m.updateGL();
            }

            if (ImGui::Button("Highlight model")) {
                uiMode = HIGHLIGHT;
                UI_Manager(data, uiMode, dir_arrows);
                data.m.updateGL();
            }

            if (ImGui::Button("Show Fronts")) {
                show_fronts = !show_fronts;
                if (show_fronts) showFronts(data);
                else data.gui->pop_all_markers();
            }

            if (ImGui::Button("Show Target")) {
                data.vol.show_mesh(show_target = !show_target);
            }

            if (ImGui::Button("Matte target")) {
                show_target_matte = !show_target_matte;
                if (show_target_matte) data.vol.show_mesh_flat();
                else data.vol.show_mesh_points();
            }

            /*
            if(ImGui::Button("Show vert direction")) {
                uiMode = DIRECTION;
                UI_Manager(data, uiMode, dir_arrows);
            }
            */

            if (ImGui::Button("Show orient3D sign")) {
                uiMode = VOLUME;
                UI_Manager(data, uiMode, dir_arrows);
                data.m.updateGL();
            }

            if (ImGui::Button("Screen colors")) {
                for (uint &f: data.m.get_surface_faces()) {
                    uint poly = data.m.adj_f2p(f)[0];
                    data.m.poly_data(poly).color = Color(255.f / 255.f, 198.f / 255.f, 70.f / 255.f);
                }
                data.m.updateGL();
                // 1800 1035 0.0157587 -0.00979251 0.223252 0.660849 0.5 -0.0462633 -0.674176 0.73712 -0.0149085 0.719672 -0.534246 -0.443457 0.0975828 0.692771 0.509969 0.509902 -0.1274 0 0 0 1 1 0 0 -0 0 1 0 -0 0 0 1 -2.6434 0 0 0 1 0.870092 0 0 -0 0 1.5132 0 -0 0 0 -0.756602 -2 0 0 0 1
            }

            if(ImGui::Button("Show srf smoothed")) {
                show_srf_smooth = !show_srf_smooth;

                if(show_srf_smooth) {
                    data.m_srf.updateGL();
                    gui.push(&data.m_srf);
                } else {
                    gui.pop(&data.m_srf);
                }
            }

            if(ImGui::Button("Show vert origin")) {
                show_vert_origin = !show_vert_origin;

                if(show_vert_origin) {

                    for(uint vid : data.m.get_surface_verts()) {
                        if(data.m.vert_data(vid).flags[SPLIT]) {
                            gui.push_marker(data.m.vert(vid), "", Color::GREEN(), 2, 4);
                        } else if(data.m.vert_data(vid).flags[TOPOLOGICAL]) {
                            gui.push_marker(data.m.vert(vid), "", Color::RED(), 2, 4);
                        }
                    }

                } else {
                    gui.pop_all_markers();
                }

            }

            ImGui::TreePop();
        }

    };

    gui.callback_mouse_left_click = [&](int modifiers) -> bool
    {
        if(modifiers & GLFW_MOD_SHIFT)
        {
            vec3d p;
            vec2d click = gui.cursor_pos();
            if(gui.unproject(click, p)) // transform click in a 3d point
            {
                uint vid = data.m.pick_edge(p);
                std::cout << "ID " << vid << " - l: " << data.m.edge_length(vid) << std::endl;
            }
        }
        return false;
    };

    int under100 = 0;
    if(!data.render) {
        while(!data.fronts_active.empty() && data.running && data.step < 50 && under100 < 10) {
            advancing_volume(data);
            if(data.fronts_active.size() < 100) under100++;
        }
        filelog(data, "DONE");
    }

    return data.render ? gui.launch() : 0;

    /**/
}