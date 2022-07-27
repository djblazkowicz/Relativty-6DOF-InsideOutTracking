#ifndef SPECTACULAR_AI_REALSENSE_SESSION_HPP
#define SPECTACULAR_AI_REALSENSE_SESSION_HPP

#include <memory>
#include <spectacularAI/types.hpp>

namespace spectacularAI {
struct VioOutput;

namespace rsPlugin {

/**
 * VIO session.
 */
struct Session {
    /** Check if new output is available */
    SPECTACULAR_AI_API virtual bool hasOutput() const = 0;
    /** Get output from the queue, if available. If not, returns {} */
    SPECTACULAR_AI_API virtual std::shared_ptr<const VioOutput> getOutput() = 0;
    /**
     * Wait until new output is available and then return it.
     * Do not use with useReaderThread=false.
     */
    SPECTACULAR_AI_API virtual std::shared_ptr<const VioOutput> waitForOutput() = 0;

    /**
     * Add an external trigger input. Causes additional output corresponding
     * to a certain timestamp to be generated.
     *
     * @param t timestamp, monotonic float seconds
     * @param tag additonal tag to indentify this particular trigger event.
     *  The default outputs corresponding to input camera frames have a tag 0.
     */
    SPECTACULAR_AI_API virtual void addTrigger(double t, int tag) = 0;

    SPECTACULAR_AI_API virtual ~Session();
};

}
}

#endif
