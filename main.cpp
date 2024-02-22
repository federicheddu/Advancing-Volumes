#include "advancing_volumes.h"
#include "advancing_commands.h"

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
    Data d;
    Octree oct;

    //input parameters -> no render
    if(argc > 1) {
        while(!d.front.empty())
            advancing_volume(d);
        return 0;
    }

    //models to load
    d.str_model = "";
    d.str_target = "../data/duck.mesh";
    setup(d, &oct);

    //render
    GLcanvas gui(1080, 720);
    d.render = true;
    //push on gui
    gui.push(&d.m, false);
    gui.push(&d.ts, true);
    gui.push(new VolumeMeshControls<DrawableTetmesh<>>(&d.m, &gui));
    gui.push(new SurfaceMeshControls<DrawableTrimesh<>>(&d.ts, &gui));
    d.gui = &gui;

    //app commands -> SPACE for one iteration
    gui.callback_app_controls = [&]() {gui_commands(d);};
    gui.callback_key_pressed = [&](int key, int modifier) { return key_commands(d, key, modifier);};
    gui.callback_mouse_left_click = [&](int modifier) -> bool {return click_commands(d, modifier);};

    return gui.launch();
}