#include "advancing_volumes.h"

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

        return 0;
    }

    //models to load
    d.str_model = "";
    d.str_target = "../data/bunny.mesh";
    d.str_rationals = "";
    setup(d, &oct);

    //render
    GLcanvas gui(1080, 720);
    gui.push(&d.m, false);
    gui.push(&d.ts, true);
    gui.push(new VolumeMeshControls<DrawableTetmesh<>>(&d.m, &gui));
    gui.push(new SurfaceMeshControls<DrawableTrimesh<>>(&d.ts, &gui));
    d.gui = &gui;

    return gui.launch();
}