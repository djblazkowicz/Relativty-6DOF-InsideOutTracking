#ifndef SPECTACULAR_AI_TYPES_HPP
#define SPECTACULAR_AI_TYPES_HPP

#include <array>
#include <cstdint>
#include <memory>

// forward declarations for Bitmap
namespace cv { class Mat; }

/** Identifies functions and methods that are a part of the API */
#ifdef _MSC_VER
    #define SPECTACULAR_AI_API __declspec(dllexport)
#else
    #define SPECTACULAR_AI_API __attribute__((visibility("default")))
#endif

namespace spectacularAI {

/** Coordinates of an image pixel, subpixel accuracy */
struct PixelCoordinates {
    float x, y;
};

/** Vector in R^3, can represent, e.g., velocity, position or angular velocity */
struct Vector3d {
    double x, y, z;
};

/** Vector in R^3 (single precision) */
struct Vector3f {
    float x, y, z;
};

/** Quaternion representation of a rotation. Hamilton convention.*/
struct Quaternion {
    double x, y, z, w;
};

/**
 * A 3x3 matrix, row major (accessed as m[row][col]).
 * Also note that when the matrix is symmetric (like covariance matrices),
 * there is no difference between row-major and column-major orderings.
 */
using Matrix3d = std::array<std::array<double, 3>, 3>;

/**
 * A 4x4 matrix, row major (accessed as m[row][col]).
 * Typically used with homogeneous coordinates.
 */
using Matrix4d = std::array<std::array<double, 4>, 4>;

/** Represents the pose (position & orientation) of a device at a given time */
struct Pose {
    /** Timestamp in seconds. Monotonically increasing */
    double time;

    /**
     * 3D position in a right-handed metric coordinate system
     * where the z-axis points up
     */
    Vector3d position;

    /** Orientation quaternion in the same coordinate system as position */
    Quaternion orientation;

    /** Matrix that converts homogeneous local coordinates to homogeneous world coordinates */
    SPECTACULAR_AI_API Matrix4d asMatrix() const;
    /** Create a pose from a timestamp and local-to-world matrix */
    SPECTACULAR_AI_API static Pose fromMatrix(double t, const Matrix4d &localToWorld);
};

#ifndef SPECTACULAR_AI_DISABLE_GNSS_VIO
/**
 * Global coordinates, WGS-84
 */
struct WgsCoordinates {
    /** Latitude in degrees */
    double latitude;
    /** Longitude in degrees */
    double longitude;
    /** Altitude in meters */
    double altitude;
};
#endif

/** An element of the point cloud */
struct FeaturePoint {
    /**
     * An integer ID to identify same points in different
     * revisions of the point clouds
     */
    int id;

    /** Global position of the feature point */
    Vector3d position;

    /** Implementation-defined status/type */
    int status = 0;
};

/** 6-DoF pose tracking status */
enum class TrackingStatus {
    /** Initial status when tracking starts and is still initializing */
    INIT = 0,
    /** Tracking is accurate (but not globally referenced) */
    TRACKING = 1,
    /**
     * Tracking has failed. Outputs are no longer produced until
     * the system recovers, which will be reported as another tracking state
     */
    LOST_TRACKING = 2
};

/** Specifies the pixel format of a bitmap */
enum class ColorFormat {
    NONE,
    GRAY,
    RGB,
    RGBA,
    RGBA_EXTERNAL_OES, // internal
    BGR,
    BGRA,
    GRAY16
};

/**
 * Simple wrapper for a bitmap in RAM (not GPU RAM).
 */
struct Bitmap {
    /** Create an bitmap with undefined contents */
    SPECTACULAR_AI_API
    static std::unique_ptr<Bitmap> create(int width, int height, ColorFormat colorFormat);

    /** Create bitmap wrapper for plain data (not copied) */
    SPECTACULAR_AI_API
    static std::unique_ptr<Bitmap> createReference(int width, int height, ColorFormat colorFormat, std::uint8_t *data);

    /** Create a bitmap wrapper from an OpenCV matrix */
    SPECTACULAR_AI_API
    static std::unique_ptr<Bitmap> createReference(cv::Mat &mat);

    /**
     * Create an OpenCV matrix that may refer to this image as a shallow copy.
     * use .clone() to make a deep copy if needed. If flipColors = true (default)
     * automatically converts from RGB to BGR if necessary.
     */
    SPECTACULAR_AI_API
    cv::Mat asOpenCV(bool flipColors = true);

    SPECTACULAR_AI_API virtual ~Bitmap();

    /** Image width in pixels */
    SPECTACULAR_AI_API virtual int getWidth() const = 0;
    /** Image height in pixels */
    SPECTACULAR_AI_API virtual int getHeight() const = 0;
    /** Pixel color format / channel configuration */
    SPECTACULAR_AI_API virtual ColorFormat getColorFormat() const = 0;

    /**
     * Data in row major order. Rows must be contiguous. 8-32 bits per pixel,
     * as defined by ColorFormat.
     */
    SPECTACULAR_AI_API virtual const std::uint8_t *getDataReadOnly() const = 0;
    SPECTACULAR_AI_API virtual std::uint8_t *getDataReadWrite() = 0;
};

}

#endif
