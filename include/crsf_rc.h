#ifndef CRSF_RC_H
#define CRSF_RC_H

#include <stdint.h>

void crsf_init();
void crsf_update();
int16_t crsf_get_channel(uint8_t idx);

#endif // CRSF_RC_H