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

int main( /* int argc, char *argv[] */ ) {

    //UI
    GLcanvas gui(1080, 720);
    gui.side_bar_alpha = 0.5;

    std::vector<std::string> path = {"../data/cubespikes.mesh",           //0
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
                                     "../data/dog.mesh"};                 //48
    //load the data
    Data data = setup(path[32].c_str());

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