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

    //algorithm data
    Data d, d_prev;
    Octree oct;
    std::string output;

    //input parameters -> no render
    if(argc == 3) {

        //set up the env
        parse(d, argc, argv);
        setup(d, &oct);
        d.prev = &d_prev;

        cout << endl << BYEL;
        cout << "Target: " << d.path_target << endl;
        cout << "Model: " << (!d.path_model.empty() ? d.path_model : "NONE") << endl;
        cout << "Log: " << (!d.path_map.empty() ? d.path_map : "NONE") << rendl;
        cout << BCYN << "Mapping: " << (d.map ? "ON" : "OFF") << rendl << endl;

        //to early top and save
        int active_prev = d.front.size(), counter = 0;
        double target_vol = d.tv.mesh_volume();

        //algorithm loop
        while(!d.front.empty() && d.running) {

            //for undo
            d_prev =  d;

            //algorithm
            advancing_volume(d);

            //check for progress
            if(active_prev == d.front.size()) counter++; else counter = 0;
            my_assert(d, counter < 10, "NO PROGRESS");
            active_prev = d.front.size();

        }

        //log output
        if(d.running) my_assert(d, false, "COMPLETE");

        return 0;
    }
    /**/

    /* === GUI === */

    //loading parameters
    if(argc > 1) {
        parse(d, argc, argv);
    } else {
        d.path_target = "/Users/federicheddu/Documents/VOLMAP/G1/bunny.mesh";
        d.path_model = "";
        d.path_map = "";
    }

    //setup
    setup(d, &oct);
    d.prev = &d_prev;
    d.m.updateGL();

    cout << endl << BYEL;
    cout << "Target: " << d.path_target << endl;
    cout << "Model: " << (!d.path_model.empty() ? d.path_model : "NONE") << endl;
    cout << "Log: " << (!d.path_map.empty() ? d.path_map : "NONE") << rendl;
    cout << BCYN << "Mapping: " << (d.map ? "ON" : "OFF") << rendl << endl;

    //render
    GLcanvas gui(1080, 720);
    GLcanvas gui_map(1080, 720);
    d.render = true;

    //push on gui
    gui.push(&d.m, false);
    gui.push(&d.ts, true);
    gui.push(new VolumeMeshControls<DrawableTetmesh<>>(&d.m, &gui));
    gui.push(new SurfaceMeshControls<DrawableTrimesh<>>(&d.ts, &gui));
    d.gui = &gui;

    //sphere mapping
    if(d.map) {
        d.mm.updateGL();
        gui_map.push(&d.mm);
        gui.push(new VolumeMeshControls<DrawableTetmesh<>>(&d.mm, &gui_map));
    }

    //app commands -> SPACE for one iteration
    gui.callback_app_controls = [&]() {gui_commands(d);};
    gui.callback_key_pressed = [&](int key, int modifier) { return key_commands(d, key, modifier);};
    gui.callback_mouse_left_click = [&](int modifier) -> bool {return click_commands(d, modifier);};

    if(d.map) return gui.launch({&gui_map});
    else return gui.launch();
}