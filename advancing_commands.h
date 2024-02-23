#ifndef ADVANCING_VOLUMES_ADVANCING_COMMANDS_H
#define ADVANCING_VOLUMES_ADVANCING_COMMANDS_H

#include "advancing_volumes.h"

bool key_commands(Data &d, int key, int modifier);
void gui_commands(Data &d);
bool click_commands(Data &d, int modifiers);


//algorithm

//view commands
void view_displacements(Data &d, DrawableSegmentSoup &norms, DrawableSegmentSoup &movs);
void view_toolong(Data &d);
void view_possible_flip(Data &d);
void unmark_edges(Data &d);

#endif //ADVANCING_VOLUMES_ADVANCING_COMMANDS_H
