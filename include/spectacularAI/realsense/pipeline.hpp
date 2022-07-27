#ifndef SPECTACULAR_AI_REALSENSE_PIPELINE_HPP
#define SPECTACULAR_AI_REALSENSE_PIPELINE_HPP

#include <spectacularAI/types.hpp>
#include <spectacularAI/mapping.hpp>
#include <functional>

namespace rs2 {
class config;
class device;
class frame;
}

namespace spectacularAI {
namespace rsPlugin {
struct Configuration;
struct Session;

/**
 * The Pipeline object stores information about the device and
 * also sets the default configuration, e.g., enables and configures necessary
 * RealSense streams and sensors. The actual VIO reading is started with
 * startSession. It is possible to change the RealSense configuration between
 * calling configureX and startSession, but the VIO tracking is not guaranteed
 * to be compatible with such customized configurations.
 */
class Pipeline {
public:
    /** Create a pipeline with the default configuration. */
    SPECTACULAR_AI_API Pipeline();
    /** Create a pipeline with custom configuration */
    SPECTACULAR_AI_API Pipeline(const Configuration &config);

    /** Configure the RealSense device for VIO */
    SPECTACULAR_AI_API void configureDevice(rs2::device &device);

    /** Set up the RealSense config for VIO */
    SPECTACULAR_AI_API void configureStreams(rs2::config &config);

    /** Set a mapping callback */
    SPECTACULAR_AI_API void setMapperCallback(const std::function<void(spectacularAI::mapping::MapperOutputPtr)> &onMapperOutput);

    /**
     * Start a new VIO session on the background.
     * Internally creates an rs2::pipeline and starts it.
     */
    SPECTACULAR_AI_API std::unique_ptr<Session> startSession(rs2::config &config);

    /**
     * Start a new VIO session on the background.
     * Also invoke the given callback on each rs2::frame.
     * Performing heavy computations directly in the callback is not recommended.
     */
    SPECTACULAR_AI_API std::unique_ptr<Session> startSession(
        rs2::config &config,
        const std::function<void(const rs2::frame &frame)> &callback);

    SPECTACULAR_AI_API ~Pipeline();

private:
    struct impl;
    std::unique_ptr<impl> pimpl;
};
}
}

#endif
