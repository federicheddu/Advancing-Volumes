#include "ui_tools.h"

void showArrows(Tetmesh<> &m, Octree &oct, std::vector<DrawableArrow> &dir_arrows, std::vector<uint> &active_fronts, GLcanvas &gui) {

    for (auto vid : active_fronts) {
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
    dir_arrows.clear();

}

void updateArrows(Tetmesh<> &m, Octree &oct, std::vector<DrawableArrow> &dir_arrows, std::vector<uint> &active_fronts, GLcanvas &gui) {
    deleteArrows(dir_arrows, gui);
    showArrows(m, oct, dir_arrows, active_fronts, gui);
}

void showFronts(DrawableTetmesh<> &m) {

    deleteFronts(m);
    for(uint vid : m.get_surface_verts())
        if(!m.vert_data(vid).label)
            for(uint pid : m.adj_v2p(vid))
                m.poly_data(pid).color = Color::PASTEL_ORANGE();

}

void deleteFronts(DrawableTetmesh<> &m) {

    for (uint fid : m.get_surface_faces())
            m.poly_data(m.adj_f2p(fid)[0]).color = Color::PASTEL_YELLOW();


}