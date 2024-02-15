#include "advancing_volumes.h"

void advancing_volume(Data &d) {

    d.step++;
    cout << BMAG << "Advancing Volume ITERATION " << d.step << RESET << endl;

    if(d.running) expand(d);
    if(d.running) refine(d);
    if(d.running) smooth(d);

    cout << BMAG << "Advancing Volume ITERATION " << BGRN << "DONE" << RESET << endl;

}