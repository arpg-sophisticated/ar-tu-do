#include "messages_sanity_check.h"

void MessageContinuityCheck::handleMessageSeq(std::string message_prefix, unsigned long seq)
{
    if (seq != m_old_seq + 1)
    {
        unsigned long skipped_messages_count = seq - m_old_seq - 1;
        m_skipped_messages += skipped_messages_count;
        if (skipped_messages_count > m_max_skipped_messages)
        {
            m_max_skipped_messages = skipped_messages_count;
        }
        // std::cout << message_prefix << "After seq=" << m_old_seq << " comes seq=" << seq
        //           << " all messages in between are skipped! max skipped: " << m_max_skipped_messages
        //           << " total: " << m_skipped_messages << "/" << seq << std::endl;
    }
    m_old_seq = seq;
}
