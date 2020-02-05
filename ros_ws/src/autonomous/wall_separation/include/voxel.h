#pragma once

#include <inttypes.h>
//#include <sensor_msgs/PointField.h>
#include <cstring>

#define VOXEL_HISTORY_LENGTH 20

class Voxel
{
    public:
    float x;
    float y;

    uint32_t get_score()
    {
        if (this->score_valid)
            return this->score;
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
        this->score_valid = true;
        return this->score = (uint32_t)(score / count);
    }

    void start_new_episode()
    {
        // Move all array elements to the right, shifting out the last element.
        memmove(this->scores + 1 * sizeof(uint32_t), this->scores, VOXEL_HISTORY_LENGTH - 1);
        this->scores[0] = 0;
        this->score_valid = false;
    }

    void increment_score()
    {
        this->scores[0]++;
    }

    void setScore(uint32_t pScore)
    {
        this->score = pScore;
    }

    private:
    bool score_valid;
    uint32_t score;
    uint32_t scores[VOXEL_HISTORY_LENGTH];
};
