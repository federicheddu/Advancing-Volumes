#include "ui_tools.h"

void showArrows(Tetmesh<> &m, Octree &oct, std::vector<DrawableArrow> &dir_arrows, std::vector<std::vector<uint>> &active_fronts, GLcanvas &gui) {

    for(auto &front : active_fronts)
        for (auto vid: front) {
            dir_arrows.emplace_back(m.vert(vid),
                                    m.vert(vid) + m.vert_data(vid).normal *
                                                  oct.closest_point(m.vert(vid)).dist(m.vert(vid)));
            dir_arrows.back().size = 0.005;
        }

    for (auto &da: dir_arrows)
        gui.push(&da, false);

}

void deleteArrows(std::vector<DrawableArrow> &dir_arrows, GLcanvas &gui) {

    for (auto &id: dir_arrows) {
        gui.pop(&id);
    }

}

void updateArrows(Tetmesh<> &m, Octree &oct, std::vector<DrawableArrow> &dir_arrows, std::vector<std::vector<uint>> &active_fronts, GLcanvas &gui) {
    deleteArrows(dir_arrows, gui);
    showArrows(m, oct, dir_arrows, active_fronts, gui);
}