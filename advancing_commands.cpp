#include "advancing_commands.h"

bool key_commands(Data &d, int key, int modifier) {

    bool handled = true;

    static DrawableSegmentSoup norms;
    static DrawableSegmentSoup movs;
    static bool show_target = true;
    static bool show_disp = false;
    static bool show_long = false;
    static bool show_flips = false;
    static int substep = 0;

    switch (key) {
        //algorithm
        case GLFW_KEY_SPACE: {
            if(!d.step_by_step) advancing_volume(d);
            else cout << endl << TYEL << "Pls end the step by step..." << rendl << endl;
            d.m.updateGL();
            if(d.map) d.mm.updateGL();
            break;
        }
        case GLFW_KEY_ENTER: {
            int active_prev = d.front.size(), counter = 0;
            double target_vol = d.tv.mesh_volume();

            while(!d.front.empty() && d.running) {
                //algorithm
                advancing_volume(d);

                //check for progress
                if(active_prev == d.front.size()) counter++; else counter = 0;
                active_prev = d.front.size();

                if(counter > 10) {
                    cout << TYEL << "NO PROGRESS" << rendl;
                    cout << TYEL << "Volume: " << d.m.mesh_volume() / target_vol << rendl;
                    break;
                }
            }
            d.m.updateGL();
            if(d.map) d.mm.updateGL();
            break;
        }
        case GLFW_KEY_BACKSLASH: {
            d.step_by_step = true;

            if(substep == 0) {
                d.step++;
                cout << BMAG << "Advancing Volume ITERATION " << d.step << RST << endl;
            }

            if(substep == 0 && d.running) expand(d);
            if(substep == 1 && d.running) smooth(d);
            if(substep == 2 && d.running) do { refine(d); } while(d.multiple_refinement && refine_again(d));
            if(substep == 3 && d.running) smooth(d);
            if(substep == 4) update_front(d);
            substep++;

            substep = substep % 5;
            d.step_by_step = substep != 0;
            d.m.updateGL();
            if(d.map) d.mm.updateGL();

            if(substep == 0)
                cout << BMAG << "Advancing Volume ITERATION - Actives: " << d.front.size() << "/" << d.m.num_srf_verts() << RST << endl << endl;

            break;
        }
        //show the target
        case GLFW_KEY_M: {
            cout << TYEL << "Show/Unshow Target" << rendl;
            show_target = !show_target;
            d.ts.show_mesh(show_target);
            break;
        }
        case GLFW_KEY_1: {
            expand(d);
            d.m.updateGL();
            if(d.map) d.mm.updateGL();
            break;
        }
        case GLFW_KEY_2: {
            refine(d);
            d.m.updateGL();
            if(d.map) d.mm.updateGL();
            break;
        }
        case GLFW_KEY_3: {
            smooth(d);
            d.m.updateGL();
            if(d.map) d.mm.updateGL();
            break;
        }
        case GLFW_KEY_4: {
            compute_distances(d);
            update_front(d);
            d.m.updateGL();
            if(d.map) d.mm.updateGL();
            break;
        }
        //show the displacement (BLUE) vs normal (RED)
        case GLFW_KEY_N: {
            cout << TYEL << "Show/Unshow Displacements" << rendl;
            show_disp = !show_disp;
            if(show_disp) {
                view_displacements(d, norms, movs);
            } else {
                d.gui->pop(&norms);
                d.gui->pop(&movs);
                norms.clear();
                movs.clear();
            }
            break;
        }
        //show edges that are too long
        case GLFW_KEY_B: {
            cout << TYEL << "Show/Unshow Edges Too Long" << rendl;
            show_long = !show_long;
            if(show_long)view_toolong(d);
            else unmark_edges(d);
            break;
        }
        //show possible flips
        case GLFW_KEY_K: {
            cout << TYEL << "Show/Unshow Possible Flips" << rendl;
            show_flips = !show_flips;
            if(show_flips) view_possible_flip(d);
            else unmark_edges(d);
            break;
        }
        //try flips
        case GLFW_KEY_L: {
            cout << TYEL << "Try flips" << rendl;
            try_flips(d);
            break;
        }
        //unmark edges
        case GLFW_KEY_COMMA: {
            cout << TYEL << "Unmark edges" << rendl;
            unmark_edges(d);
            break;
        }
        default: {
            handled = false;
            break;
        }
    }

    return handled;
}

void gui_commands(Data &d) {

    ImVec2 bsize(100,50);

    if(ImGui::Button("Check MAP", bsize))
        map_check(d);


}

bool click_commands(Data &d, int modifiers) {

    //get info
    if(modifiers & GLFW_MOD_CONTROL) {
        vec3d p;
        vec2d click = d.gui->cursor_pos();
        if(d.gui->unproject(click, p)) {
            uint vid = d.m.pick_vert(p);
            cout << vid << "is " << (d.m.vert_data(vid).flags[ACTIVE] ? "ACTIVE" : "INACTIVE") << endl;
        }
    }

    //split the edge
    if(modifiers & GLFW_MOD_ALT) {
        vec3d p;
        vec2d click = d.gui->cursor_pos();
        if(d.gui->unproject(click, p)) {
            uint eid = d.m.pick_edge(p);
            d.m.edge_data(eid).flags[MARKED] = true;
            d.m.updateGL();
            if(d.map) {
                d.mm.edge_data(eid).flags[MARKED] = true;
                d.mm.updateGL();
            }
        }
    }

    return false;
}

/* ================================================================================================================== */

void view_displacements(Data &d, DrawableSegmentSoup &norms, DrawableSegmentSoup &movs) {

    //vert and displacement
    vec3d vert, disp;
    d.m.update_normals();

    //normals (RED)
    norms.use_gl_lines = true;
    norms.default_color = Color::RED();
    compute_distances(d);
    compute_displacements(d);
    for (auto vid: d.front) {
        vert = d.m.vert(vid);
        disp = d.m.vert_data(vid).normal;
        norms.push_seg(vert, vert + disp);
    }

    //restore normals
    d.m.update_normals();

    //movement (BLUE)
    movs.use_gl_lines = true;
    movs.default_color = Color::BLUE();
    compute_directions(d);
    compute_distances(d);
    compute_displacements(d);
    for (auto vid: d.front) {
        vert = d.m.vert(vid);
        disp = d.m.vert_data(vid).normal;
        movs.push_seg(vert, vert + disp);
    }

    //push on gui
    d.gui->push(&norms, false);
    d.gui->push(&movs, false);

    //restore normals
    d.m.update_normals();

}

void view_toolong(Data &d) {

    for (auto eid: d.m.get_surface_edges()) {
        d.m.edge_data(eid).flags[MARKED] = false;
        if (d.m.edge_length(eid) > d.target_edge_length)
            d.m.edge_data(eid).flags[MARKED] = true;
    }
    d.m.updateGL();
}

void view_possible_flip(Data &d) {

    uint vid0, vid1;
    std::vector<uint> adj_f;
    double lenght, lenght_opp;

    for(uint eid : d.m.get_surface_edges()) {

        adj_f = d.m.edge_adj_srf_faces(eid);
        my_assert(d, adj_f.size() == 2, "Edge has more than 2 adjacent faces", __FILE__, __LINE__);

        vid0 = d.m.face_vert_opposite_to(adj_f[0], eid);
        vid1 = d.m.face_vert_opposite_to(adj_f[1], eid);

        lenght = d.m.edge_length(eid);
        lenght_opp = d.m.vert(vid0).dist(d.m.vert(vid1));

        if(lenght_opp < lenght && d.m.adj_e2p(eid).size() == 2)
            d.m.edge_data(eid).flags[MARKED] = true;

    }
    d.m.updateGL();

}

void unmark_edges(Data &d) {
    for (auto eid: d.m.get_surface_edges())
        d.m.edge_data(eid).flags[MARKED] = false;
    d.m.updateGL();
}

/* ================================================================================================================== */