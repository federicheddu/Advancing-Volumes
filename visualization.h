#ifndef ADVANCING_VOLUMES_VISUALIZATION_H
#define ADVANCING_VOLUMES_VISUALIZATION_H

#include <cinolib/meshes/meshes.h>
#include <cinolib/drawable_arrow.h>
#include <cinolib/drawable_octree.h>
#include <cinolib/gl/glcanvas.h>
#include <cinolib/parallel_for.h>
#include "advancing_data.h"

using namespace cinolib;


typedef enum {BLANK, DIRECTION, FRONTS, VOLUME, HIGHLIGHT} UI_Mode;

void UI_Manager(Data &d, UI_Mode uiMode, std::vector<DrawableArrow> &dir_arrows);

void showArrows(Tetmesh<> &m, Octree *oct, std::vector<DrawableArrow> &dir_arrows, std::vector<uint> &active_fronts, GLcanvas &gui);

void deleteArrows(std::vector<DrawableArrow> &dir_arrows, GLcanvas &gui);

void updateArrows(Tetmesh<> &m, Octree &oct, std::vector<DrawableArrow> &dir_arrows, std::vector<uint> &active_fronts, GLcanvas &gui);

void showFronts(Data &d);

void showVolume(DrawableTetmesh<> &m);

void clearColors(DrawableTetmesh<> &m);

void highlightModel(DrawableTetmesh<> &m);

void show_stuck_v(DrawableTetmesh<> &m, std::set<uint> &stuck, GLcanvas &gui);

void show_stuck_e(DrawableTetmesh<> &m, std::set<uint> &stuck, GLcanvas &gui);



#endif //ADVANCING_VOLUMES_VISUALIZATION_H
