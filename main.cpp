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

    std::vector<vec3d> movements;
    DrawableSegmentSoup norms;
    DrawableSegmentSoup movs;

    gui.callback_key_pressed = [&](int key, int modifier) {\

        bool handled = true;

        switch (key) {
            case GLFW_KEY_SPACE:
                advancing_volume(d);
                d.m.updateGL();
                d.m.update_bbox();
                d.m.update_quality();
                break;
            case GLFW_KEY_M: {

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
                //compute_directions(d);
                compute_distances(d);
                compute_displacements(d);
                for (auto vid: d.front) {
                    vert = d.m.vert(vid);
                    disp = d.m.vert_data(vid).normal;
                    movs.push_seg(vert, vert + disp);
                }

                //push on gui
                gui.push(&norms, false);
                gui.push(&movs, false);

                //restore normals
                d.m.update_normals();
                break;
            }
            case GLFW_KEY_N:
                gui.pop(&norms);
                gui.pop(&movs);
                norms.clear();
                movs.clear();
                break;
            default:
                handled = false;
                break;
        }

        return handled;
    };

    return gui.launch();
}