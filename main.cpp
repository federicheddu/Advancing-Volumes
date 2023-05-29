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

//model macros
#define ANT "../data/ant.mesh"                          //perfect
#define ARMADILLO "../data/armadillo.mesh"              //very good
#define BUNNY "../data/bunny.mesh"                      //bottom compenetration, very low
#define CHINESE_DRAGON "../data/chinese_dragon.mesh"
#define DAVID "../data/david.mesh"                      //too complex model
#define DRAGON "../data/dragon.mesh"
#define GARGOYLE "../data/gargoyle.mesh"                //complex model
#define MOUSE "../data/mouse.mesh"                      //almost good
#define PIG "../data/pig.mesh"                          //ear compenetration

int main( /* int argc, char *argv[] */ ) {

    //UI
    GLcanvas gui(1080, 720);
    gui.side_bar_alpha = 0.5;

    //load the data
    char path[] = DAVID;
    Data data = setup(path);

    //gui push
    gui.push(&data.m, false);
    gui.push(new VolumeMeshControls<DrawableTetmesh<>>(&data.m, &gui));
    gui.push(&data.vol, true);
    gui.push(new VolumeMeshControls<DrawableTetmesh<>>(&data.vol, &gui));

    //visualization
    UI_Mode uiMode = BLANK;
    bool show_target = true;
    bool show_target_matte = false;
    std::vector<DrawableArrow> dir_arrows;
    //vert movement parameters
    double prev_vol = 0;
    bool raycast = false;
    data.fronts_active = data.m.get_surface_verts();
    data.fronts_bounds.emplace_back(data.m.num_srf_verts());
    //undo object
    Data reset_data = data;
    Data undo_data = data;

    //keyboard commands
    gui.callback_key_pressed = [&](int key, int modifier) {

        bool handled = true;

        switch (key) {

            /** VISUALIZATION KEYS **/
            //clear the mesh
            case GLFW_KEY_COMMA: {
                uiMode = BLANK;
                UI_Manager(data.m, uiMode, data.oct, dir_arrows, data.fronts_active, gui);
                data.m.updateGL();
                break;
            }
            //arrow (direction) visualization
            case GLFW_KEY_V: {
                uiMode = DIRECTION;
                UI_Manager(data.m, uiMode, data.oct, dir_arrows, data.fronts_active, gui);
                break;
            }
            //fronts visualization
            case GLFW_KEY_B: {
                uiMode = FRONTS;
                UI_Manager(data.m, uiMode, data.oct, dir_arrows, data.fronts_active, gui);
                data.m.updateGL();
                break;
            }
            //target visualization
            case GLFW_KEY_L: {
                data.vol.show_mesh(show_target = !show_target);
                break;
            }
            //show full target
            case GLFW_KEY_J: {
                show_target_matte = !show_target_matte;
                if(show_target_matte) data.vol.show_mesh_flat();
                else data.vol.show_mesh_points();
                break;
            }
            //show orient3D sign
            case GLFW_KEY_Y: {
                uiMode = VOLUME;
                UI_Manager(data.m, uiMode, data.oct, dir_arrows, data.fronts_active, gui);
                data.m.updateGL();

                break;
            }

            /** MOVEMENT KEYS **/
            //model expansion
            case GLFW_KEY_N: {
                std::cout << std::endl <<TXT_BOLDMAGENTA << "Model expansion" << TXT_RESET << std::endl;

                //update undo
                undo_data = data;

                //move the verts
                expand(data);

                //UI
                UI_Manager(data.m, uiMode, data.oct, dir_arrows, data.fronts_active, gui);

                //update model
                data.m.update_normals();
                data.m.updateGL();
                break;
            }
            //smoothing
            case GLFW_KEY_O: {

                //update undo
                undo_data = data;

                //smooth the surface
                smooth(data);

                //update model and UI
                data.m.update_normals();
                UI_Manager(data.m, uiMode, data.oct, dir_arrows, data.fronts_active, gui);
                data.m.updateGL();
                break;
            }

            /** TOPOLOGICAL KEYS **/
            //poly split
            case GLFW_KEY_M: {
                std::cout << std::endl << TXT_BOLDMAGENTA << "Poly split" << TXT_RESET << std::endl;

                //update undo
                undo_data = data;

                //split and flip polys in the surface
                split_n_flip(data);

                //update model
                data.m.update_normals();
                UI_Manager(data.m, uiMode, data.oct, dir_arrows, data.fronts_active, gui);
                data.m.updateGL();
                break;
            }
            //poly split
            case GLFW_KEY_H: {
                std::cout << std::endl << TXT_BOLDMAGENTA << "Poly split" << TXT_RESET << std::endl;

                //update undo
                undo_data = data;

                //split and flip polys in the surface
                split_n_flip(data, true);

                //UI
                UI_Manager(data.m, uiMode, data.oct, dir_arrows, data.fronts_active, gui);

                //update model
                data.m.update_normals();
                data.m.updateGL();
                break;
            }

            /** COMBO KEYS **/
            //vert movement with edge split by length
            case GLFW_KEY_G: {
                std::cout << std::endl <<TXT_BOLDMAGENTA << "Model expansion" << TXT_RESET << std::endl;

                //update undo
                undo_data = data;

                //move the verts
                expand(data, true);

                //UI
                UI_Manager(data.m, uiMode, data.oct, dir_arrows, data.fronts_active, gui);

                //update model
                data.m.update_normals();
                data.m.updateGL();

                break;
            }
            //expand and smooth
            case GLFW_KEY_I: {

                //update undo
                undo_data = data;

                //expand and smooth
                expand(data, true);
                smooth(data);

                //update model and UI
                data.m.update_normals();
                UI_Manager(data.m, uiMode, data.oct, dir_arrows, data.fronts_active, gui);
                data.m.updateGL();
                break;
            }
            //expand and smooth with volume decision for the expansion mode
            case GLFW_KEY_MINUS: {
                //update undo
                undo_data = data;

                prev_vol = data.m.mesh_volume();

                if(!raycast) {
                    std::cout << TXT_BOLDMAGENTA << "Closest point mode" << TXT_RESET << std::endl;
                    //expand and smooth (with the closest point)
                    expand(data, true, CLOSEST_POINT);
                    smooth(data);
                } else {
                    std::cout << TXT_BOLDMAGENTA << "Ray mode" << TXT_RESET << std::endl;
                    //expand and smooth (with raycast)
                    expand(data, true, RAYCAST);
                    smooth(data, 20);
                }

                raycast = prev_vol * 1.005 >= data.m.mesh_volume();
                prev_vol = data.m.mesh_volume();

                //update model and UI
                data.m.update_normals();
                UI_Manager(data.m, uiMode, data.oct, dir_arrows, data.fronts_active, gui);
                data.m.updateGL();
                break;
            }
            //expand and smooth in raycast mode
            case GLFW_KEY_BACKSPACE: {
                //update undo
                undo_data = data;

                std::cout << TXT_BOLDMAGENTA << "Ray mode" << TXT_RESET << std::endl;
                //expand and smooth (with raycast)
                expand(data, true, RAYCAST);
                smooth(data, 20);

                //update model and UI
                data.m.update_normals();
                UI_Manager(data.m, uiMode, data.oct, dir_arrows, data.fronts_active, gui);
                data.m.updateGL();
                break;
            }
            //expand and smooth with local decision for the expansion mode
            case GLFW_KEY_PERIOD: {

                undo_data = data;

                expand(data, true, LOCAL);
                smooth(data);

                //update model and UI
                data.m.update_normals();
                UI_Manager(data.m, uiMode, data.oct, dir_arrows, data.fronts_active, gui);
                data.m.updateGL();

                break;
            }
            //10 iteration of PERIOD KEY
            case GLFW_KEY_SPACE: {

                undo_data = data;

                for(int i = 0; i < 10; i++){
                    std::cout << TXT_BOLDMAGENTA << "Iter: " << i+1 << TXT_RESET << std::endl;
                    expand(data, true, LOCAL);
                    smooth(data);
                    data.m.update_normals();
                }
                std::cout << TXT_BOLDMAGENTA << "DONE" << TXT_RESET << std::endl;

                //update model and UI
                UI_Manager(data.m, uiMode, data.oct, dir_arrows, data.fronts_active, gui);
                data.m.updateGL();

                break;
            }
            case GLFW_KEY_F1: {
                undo_data = data;

                for(int i = 0; i < 100; i++){
                    std::cout << TXT_BOLDMAGENTA << "Iter: " << i+1 << TXT_RESET << std::endl;
                    expand(data, true, LOCAL);
                    smooth(data);
                    data.m.update_normals();
                }
                std::cout << TXT_BOLDMAGENTA << "DONE" << TXT_RESET << std::endl;

                //update model and UI
                UI_Manager(data.m, uiMode, data.oct, dir_arrows, data.fronts_active, gui);
                data.m.updateGL();

                break;
            }
            //final projection
            case GLFW_KEY_0: {
                undo_data = data;

                final_projection(data);

                //update model and UI
                data.m.update_normals();
                UI_Manager(data.m, uiMode, data.oct, dir_arrows, data.fronts_active, gui);
                data.m.updateGL();

                break;
            }

            /** UNDO KEYS**/
            //undo
            case GLFW_KEY_K: {
                std::cout << std::endl << TXT_BOLDYELLOW << "UNDO" << TXT_RESET << std::endl;

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
                break;
            }
            //reset
            case GLFW_KEY_P: {
                std::cout << std::endl << TXT_BOLDYELLOW << "RESET" << TXT_RESET << std::endl;

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
                break;
            }

            default:
                handled = false;
        }

        return handled;

    };

    return gui.launch();
}