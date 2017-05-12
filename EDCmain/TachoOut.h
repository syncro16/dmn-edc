#include "defines.h"
#include "Arduino.h"

class TachoOut {
public:
    TachoOut();
    void init();
    void setRpm(int rpm);
    void updateAutomatically();
};

extern TachoOut tacho;