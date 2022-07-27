#ifndef SPECTACULAR_AI_OUTPUT_HPP
#define SPECTACULAR_AI_OUTPUT_HPP

#include "types.hpp"
#include <memory>
#include <vector>
#include <string>

namespace spectacularAI {
#ifndef SPECTACULAR_AI_DISABLE_GNSS_VIO
struct GnssVioOutput; // fwd decl
#endif

struct Camera {
    /**
     * Convert pixel coordinates to camera coordinates
     *
     * @param pixel
     * @param ray
     * @return true if conversion succeeded
     */
    SPECTACULAR_AI_API virtual bool pixelToRay(const PixelCoordinates &pixel, Vector3d &ray) const = 0;

    /**
     * Convert camera coordinates to pixel coordinates
     *
     * @param ray
     * @param pixel
     * @return true if conversion succeeded
     */
    SPECTACULAR_AI_API virtual bool rayToPixel(const Vector3d &ray, PixelCoordinates &pixel) const = 0;

    /**
     * Defined as the rectified intrinsic matrix if there's distortion.
     *
     * @return OpenCV convention
     *    fx 0 ppx
     *    0 fy ppy
     *    0  0 1
     */
    SPECTACULAR_AI_API virtual Matrix3d getIntrinsicMatrix() const = 0;

    /**
     * Project from camera coordinates to normalized device coordinates (NDC).
     *
     * @param nearClip
     * @param farClip
     */
    SPECTACULAR_AI_API virtual Matrix4d getProjectionMatrixOpenGL(double nearClip, double farClip) const = 0;

    SPECTACULAR_AI_API virtual ~Camera();

    SPECTACULAR_AI_API static std::unique_ptr<Camera> buildPinhole(const Matrix3d &intrinsicMatrix, int width, int height);

};

/** Pose of camera and conversions between world, camera, and pixel coordinates. */
struct CameraPose {
    /** Camera pose */
    Pose pose;

    /** Camera intrinsic transformations */
    std::shared_ptr<const Camera> camera;

    /** Matrix that converts homogeneous world coordinates to homogeneous camera coordinates */
    SPECTACULAR_AI_API Matrix4d getWorldToCameraMatrix() const;

    /** Matrix that converts homogeneous camera coordinates to homogeneous world coordinates */
    SPECTACULAR_AI_API Matrix4d getCameraToWorldMatrix() const;

    /** Position in world coordinates */
    SPECTACULAR_AI_API Vector3d getPosition() const;

    /**
     * Convert pixel coordinates to rays in world coordinates.
     *
     * @param pixel
     * @param origin current camera position in world coordinates
     * @param ray direction of ray from the origin
     * @return true if conversion succeeded
     */
    SPECTACULAR_AI_API bool pixelToWorld(const PixelCoordinates &pixel, Vector3d &origin, Vector3d &ray) const;

    /**
     * Convert world coordinates to pixel coordinates
     *
     * @param point
     * @param pixel
     * @return true if conversion succeeded
     */
    SPECTACULAR_AI_API bool worldToPixel(const Vector3d &point, PixelCoordinates &pixel) const;
};

/** Main output structure */
struct VioOutput {
    /**
     * Current tracking status
     */
    TrackingStatus status;

    /**
     * The current pose, with the timestamp in the clock used for input
     * sensor data and camera frames.
     */
    Pose pose;

    /**
     * Velocity vector (xyz) in m/s in the coordinate system used by pose.
     */
    Vector3d velocity;

    /**
     * Angular velocity vector in SI units (rad/s) and the coordinate system used by pose.
     */
    Vector3d angularVelocity;

    /**
     * Linear acceleration in SI units (m/s^2).
     */
    Vector3d acceleration;

    /**
     * Uncertainty of the current position as a 3x3 covariance matrix
     */
    Matrix3d positionCovariance;

    /**
     * Uncertainty of velocity as a 3x3 covariance matrix
     */
    Matrix3d velocityCovariance;

    /**
     * List of poses, where the first element corresponds to current pose
     * and the following (zero or more) values are the recent smoothed
     * historical positions
     */
    std::vector<Pose> poseTrail;

    /**
     * Point cloud (list of FeaturePoints) that correspond to
     * features currently seen by the camera (empty if not supported)
     */
    std::vector<FeaturePoint> pointCloud;

    /**
     * The input frame tag. This is the value given in addFrame... methods
     */
    int tag;

#ifndef SPECTACULAR_AI_DISABLE_GNSS_VIO
    /** GNSS-VIO output if available, otherwise {} (nullptr) */
    std::shared_ptr<const GnssVioOutput> globalPose;
#endif

    /**
     * Current pose in camera coordinates
     *
     * @param cameraId 0 for primary, 1 for secondary camera
     */
    SPECTACULAR_AI_API virtual CameraPose getCameraPose(int cameraId) const = 0;

    /**
     * Returns the output to a JSON string if supported. Otherwise returns
     * an empty string.
     */
    SPECTACULAR_AI_API virtual std::string asJson() const;

    SPECTACULAR_AI_API virtual ~VioOutput();
};

#ifndef SPECTACULAR_AI_DISABLE_GNSS_VIO
struct GnssVioOutput {
    /** Position in global coordinates in WGS84 (mean) */
    WgsCoordinates coordinates;

    /**
     * Orientation of the device as quaternion representing camera-to-ENU
     * transformation, i.e., a camera-to-world where the world coordinates are
     * defined in an East-North-Up coordinate system. Note that the origin of the
     * ENU coordinates system does not matter in this context.
     *
     * See the helper methods for obtaining the Euler angles.
     */
    Quaternion orientation;

    /** Velocity in ENU coordinates (m/s) */
    Vector3d velocity;

    /**
     * Uncertainty of the estimate as a covariance in an East-North Up (ENU)
     * coordiante system. See the helper methods for obtaining a single
     * vertical and horizontal number.
     */
    Matrix3d enuPositionCovariance;

    /**
     * Uncertainty of velocity as a 3x3 covariance matrix (in ENU coordinates)
     */
    Matrix3d velocityCovariance;

    /**
     * Get the global pose of a particular camera. The "world" coordinate system
     * of the camera pose is an East-North-Up system, whose origin is at the given
     * WGS84 coordinates.
     *
     * @param cameraId index of the camera whose pose is returned
     * @param enuOrigin the origin of the ENU coordinate system
     */
    SPECTACULAR_AI_API virtual CameraPose getEnuCameraPose(int cameraId, WgsCoordinates enuOrigin) const = 0;
    SPECTACULAR_AI_API virtual ~GnssVioOutput();
};
#endif

using VioOutputPtr = std::shared_ptr<const VioOutput>;
}

#endif
