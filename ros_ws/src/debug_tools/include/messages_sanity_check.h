#pragma once

#include <ros/ros.h>

class MessageContinuityCheck
{
    private:
    long m_old_seq = -1;
    unsigned long m_skipped_messages = 0;
    unsigned long m_max_skipped_messages = 0;

    public:
    MessageContinuityCheck(){};

    void handleMessageSeq(std::string message_prefix, unsigned long seq);
};
