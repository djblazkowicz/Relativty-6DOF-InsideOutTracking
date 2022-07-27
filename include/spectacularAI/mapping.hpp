#ifndef SPECTACULAR_AI_MAPPING_HPP
#define SPECTACULAR_AI_MAPPING_HPP

#include <map>
#include <vector>

#include "output.hpp"
#include "types.hpp"

namespace spectacularAI {
namespace mapping {

struct Frame {
    /**
     * Camera pose information
     */
    CameraPose cameraPose;

    /**
     * Image data from the camera, might not always exist
     */
    std::shared_ptr<const Bitmap> image;

    /**
     * Set when image type is depth. Depth image values are multiplied by this
     * number to to make their unit meters (e.g., if depth is integer mm,
     * this will be set to 1.0 / 1000)
     */
    double depthScale;
};

struct FrameSet {
    /**
     * Primary camera frame used for tracking
     */
    std::shared_ptr<Frame> primaryFrame;

    /**
     * Secondary camera frame used for tracking when
     * using stereo cameras
     */
    std::shared_ptr<Frame> secondaryFrame;

    /**
     * RGB color frame if such exists. When using RGB-D
     * this is the same as primaryFrame
     */
    std::shared_ptr<Frame> rgbFrame;

    /**
     * Depth frame
     */
    std::shared_ptr<Frame> depthFrame;

    /*
    Compute the depth map at the camera pose of the given frame (for example rgbFrame).
    If depthFrame is already aligned to the the given camera, returns a pointer to depthFrame.
    */
    SPECTACULAR_AI_API virtual
    std::shared_ptr<Frame> getAlignedDepthFrame(const std::shared_ptr<Frame> target) const = 0;

    /*
    * Returns an undistorted version of the given frame (i.e., with camera model = undistorted pinhole).
    * If the given frame is not distorted, returns a pointer to distortedFrame.
    */
    SPECTACULAR_AI_API virtual
    std::shared_ptr<Frame> getUndistortedFrame(const std::shared_ptr<Frame> distortedFrame) const = 0;

    SPECTACULAR_AI_API virtual ~FrameSet() {};
};

/**
 * Container for point clouds, where the points may have optional properties
 * such as colors and normals
 */
struct PointCloud {
    SPECTACULAR_AI_API virtual std::size_t size() const = 0;
    SPECTACULAR_AI_API virtual bool empty() const = 0;
    SPECTACULAR_AI_API virtual bool hasNormals() const = 0;
    SPECTACULAR_AI_API virtual bool hasColors() const = 0;
    SPECTACULAR_AI_API virtual Vector3f getPosition(std::size_t index) const = 0;
    SPECTACULAR_AI_API virtual Vector3f getNormal(std::size_t index) const = 0;
    SPECTACULAR_AI_API virtual std::array<std::uint8_t, 3> getRGB24(std::size_t index) const = 0;
    SPECTACULAR_AI_API virtual const Vector3f* getPositionData() = 0;
    SPECTACULAR_AI_API virtual const Vector3f* getNormalData() = 0;
    SPECTACULAR_AI_API virtual const std::uint8_t* getRGB24Data() = 0;
    SPECTACULAR_AI_API virtual ~PointCloud() {};
};

struct KeyFrame {
    /**
     * Unique ID for this keyframe. Monotonically increasing.
     */
    int64_t id;

    /**
     *  Set of frames available for this keyframe.
     */
    std::shared_ptr<FrameSet> frameSet;

    /**
     * (Optional) Point cloud for this keyframe.
     */
    std::shared_ptr<PointCloud> pointCloud;
};

struct Map {
    /**
     * ID to KeyFrame map. The shared_ptr objects are always non-empty.
     * May be changed to a similar associative container in the future,
     * do not rely on this being std::map.
     */
    std::map<int64_t, std::shared_ptr<const KeyFrame>> keyFrames;
};

struct MapperOutput {
    /**
     * Up to date map. Always non-empty
     */
    std::shared_ptr<const Map> map;

    /**
     * List of keyframes that were modified or deleted. Ordered from
     * oldest to newest
     */
    std::vector<int64_t> updatedKeyFrames;

    /**
     * True for the last update before program exit
     */
    bool finalMap;
};
using MapperOutputPtr = std::shared_ptr<const MapperOutput>;

}
}

#endif
