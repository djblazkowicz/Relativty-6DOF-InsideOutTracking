#ifndef SPECTACULAR_AI_REALSENSE_PLUGIN_CONFIGURATION_HPP
#define SPECTACULAR_AI_REALSENSE_PLUGIN_CONFIGURATION_HPP

#include <string>
#include <map>

namespace spectacularAI {
namespace rsPlugin {

/**
 * Plugin and Spectacular AI VIO SDK configuration variables.
 */
struct Configuration {
    bool useStereo = true;
    bool useRgb = true;
    bool fastVio = false;
    bool alignedDepth = false; // When true, uses RealSense SDK to align depth image
    bool useSlam = false;
    std::string mapSavePath = "";
    std::string mapLoadPath = "";

    // If not empty, the session will be recorded to the given folder
    std::string recordingFolder = "";
    bool recordingOnly = false;

    /**
     * Internal SDK parameter overrides (key-value pairs).
     * Not safe for unsanitized user input.
     */
    std::map<std::string, std::string> internalParameters;
};
}
}

#endif
