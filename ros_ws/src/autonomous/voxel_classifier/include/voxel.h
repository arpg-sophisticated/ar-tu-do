#pragma once

#include <inttypes.h>
//#include <sensor_msgs/PointField.h>
#include <algorithm>
#include <cmath>
#include <cstring>

#define VOXEL_HISTORY_LENGTH 2

class Voxel
{
    public:
    float x;
    float y;
    float z = 0;
    int clusterID;

    double get_score()
    {
        if (this->score_valid)
            return this->score;
        // average over the last scores != 0.
        double score = 0;
        double count = 0;
        for (int i = 0; i < VOXEL_HISTORY_LENGTH; i++)
        {
            score += this->scores[i];
            count++;
        }
        this->score_valid = true;
        this->score = score / count;
        return this->score;
    }

    void start_new_episode()
    {
        // Move all array elements to the right, shifting out the last element.
        float tmpScores[VOXEL_HISTORY_LENGTH];
        for (size_t i = 0; i < VOXEL_HISTORY_LENGTH; i++)
            tmpScores[i] = this->scores[i];
        for (size_t i = 1; i < VOXEL_HISTORY_LENGTH; i++)
            this->scores[i] = tmpScores[i - 1];
        this->scores[0] = 0.0;
        this->score_valid = false;
    }

    void increment_score(double increment)
    {
        this->scores[0] = this->scores[0] + increment;
    }

    void setScore(uint32_t pScore)
    {
        this->score = pScore;
    }

    private:
    bool score_valid;
    double score;
    double scores[VOXEL_HISTORY_LENGTH];
};
