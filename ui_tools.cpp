#include "ui_tools.h"

void UI_Manager(DrawableTetmesh<> &m , UI_Mode uiMode, Octree &oct, std::vector<DrawableArrow> &dir_arrows, std::vector<uint> &active_fronts, GLcanvas &gui) {
    static UI_Mode prev = BLANK;

    if(prev == BLANK && uiMode == BLANK)
        return;

    switch (uiMode) {
        case BLANK: {
            clearColors(m);
            deleteArrows(dir_arrows, gui);
            break;
        }
        case DIRECTION: {
            showArrows(m, oct, dir_arrows, active_fronts, gui);
            break;
        }
        case FRONTS: {
            showFronts(m);
            break;
        }
        case VOLUME: {
            showVolume(m);
            break;
        }
    }

    prev = uiMode;
}

void showArrows(Tetmesh<> &m, Octree &oct, std::vector<DrawableArrow> &dir_arrows, std::vector<uint> &active_fronts, GLcanvas &gui) {

    deleteArrows(dir_arrows, gui);

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

void showFronts(DrawableTetmesh<> &m) {

    clearColors(m);

    PARALLEL_FOR(0, m.get_surface_verts().size(), 1000, [&](uint idx)
    {
        uint vid = m.get_surface_verts().at(idx);

        if(!m.vert_data(vid).label)
            for(uint pid : m.adj_v2p(vid))
                m.poly_data(pid).color = Color::PASTEL_ORANGE();
    });

}

void showVolume(DrawableTetmesh<> &m) {

    PARALLEL_FOR(0,m.num_polys(),1000,[&](uint pid)
    {
        double volume = orient3d(m.poly_vert(pid, 0), m.poly_vert(pid, 1), m.poly_vert(pid, 2), m.poly_vert(pid, 3));
        m.poly_data(pid).color = volume > 0 ? Color::RED() : Color::PASTEL_CYAN();
    });

}

void clearColors(DrawableTetmesh<> &m) {

    PARALLEL_FOR(0, m.num_polys(), 100, [&](uint pid)
    {
        m.poly_data(pid).color = Color::WHITE();
    });

}