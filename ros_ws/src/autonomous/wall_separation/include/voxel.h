#pragma once

#include <inttypes.h>

#define VOXEL_HISTORY_LENGTH 10

class Voxel
{
    public:
    uint32_t get_score();
    void add_score(uint32_t score);
    void start_new_episode();
    void increment_score();

    private:
    uint32_t scores[VOXEL_HISTORY_LENGTH];
};