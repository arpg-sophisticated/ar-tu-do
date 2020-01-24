#pragma once

#include <inttypes.h>

#define VOXEL_HISTORY_LENGTH 20

class Voxel
{
    public:
    double x;
    double y;
    uint32_t get_score()
    {
        // average over the last scores != 0.
        double score = 0;
        double count = 0;
        for (int i = 0; i < VOXEL_HISTORY_LENGTH; i++)
        {
            if (this->scores[i] != 0)
            {
                score += this->scores[i];
                count++;
            }
        }
        return (uint32_t)(score / count);
    }

    void start_new_episode()
    {
        // Move all array elements to the right, shifting out the last element.
        memmove(this->scores + 1 * sizeof(uint32_t), this->scores, VOXEL_HISTORY_LENGTH - 1);
        this->scores[0] = 0;
    }

    void increment_score()
    {
        this->scores[0]++;
    }

    private:
    uint32_t scores[VOXEL_HISTORY_LENGTH];
};