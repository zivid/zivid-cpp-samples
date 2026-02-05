/*
Expose a C API.
*/
#include "Zivid/Application.h"
#include "Zivid/Camera.h"
#include "Zivid/CoreCAPI/CoreCAPI.h"
#include "Zivid/Exception.h"
#include "Zivid/Frame.h"
#include "Zivid/Point.h"
#include "Zivid/PointCloud.h"
#include "Zivid/Settings.h"

#include <cmath>
#include <type_traits>
#include <utility>

extern "C"
{
    struct zivid_error
    {
        std::string message;
    };

    const char *zivid_error_message(zivid_error_t error)
    {
        return error->message.c_str();
    }

    void zivid_error_destruct(zivid_error_t error)
    {
        if(error)
        {
            delete error;
        }
    }
} // End of extern "C"

template<typename Fn>
bool translateExceptions(zivid_error_t *out_error, Fn &&fn)
{
    try
    {
        fn();
    }
    catch(const std::exception &e)
    {
        *out_error = new zivid_error{ Zivid::toString(e) };
        return false;
    }
    catch(...)
    {
        *out_error = new zivid_error{ "Unknown internal error" };
        return false;
    }
    return true;
}

// Unfortunately does not work yet because of lacking C++20 support
// static_assert(std::is_layout_compatible_v<Zivid::PointXYZ, Zivid::PointXYZ>);
static_assert(std::is_standard_layout_v<Zivid::PointXYZ>);
static_assert(std::is_standard_layout_v<zivid_point_xyz_t>);
static_assert(std::is_trivial_v<Zivid::PointXYZ>);
static_assert(std::is_trivial_v<zivid_point_xyz_t>);
static_assert(sizeof(Zivid::PointXYZ) == sizeof(zivid_point_xyz_t));

#define ZIVID_DEFINE_C_OPAQUE_TYPE(CPP_TYPE, C_TYPE)                                                                   \
    struct C_TYPE                                                                                                      \
    {                                                                                                                  \
        template<typename... Args>                                                                                     \
        C_TYPE(Args &&...args)                                                                                         \
            : actual{ std::forward<Args>(args)... }                                                                    \
        {}                                                                                                             \
                                                                                                                       \
        C_TYPE(const C_TYPE &) = delete;                                                                               \
        C_TYPE &operator=(const C_TYPE &) = delete;                                                                    \
        C_TYPE(C_TYPE &&) = delete;                                                                                    \
        C_TYPE &operator=(C_TYPE &&) = delete;                                                                         \
                                                                                                                       \
        CPP_TYPE actual;                                                                                               \
    }

#define ZIVID_DEFINE_DESTRUCT(C_TYPE)                                                                                  \
    void C_TYPE##_destruct(C_TYPE##_t object)                                                                          \
    {                                                                                                                  \
        if(object)                                                                                                     \
        {                                                                                                              \
            delete object;                                                                                             \
        }                                                                                                              \
    }

ZIVID_DEFINE_C_OPAQUE_TYPE(Zivid::Settings, zivid_settings);
ZIVID_DEFINE_C_OPAQUE_TYPE(Zivid::Application, zivid_application);
ZIVID_DEFINE_C_OPAQUE_TYPE(Zivid::Camera, zivid_camera);
ZIVID_DEFINE_C_OPAQUE_TYPE(Zivid::Frame, zivid_frame);
ZIVID_DEFINE_C_OPAQUE_TYPE(Zivid::PointCloud, zivid_point_cloud);

extern "C"
{
    zivid_settings_t zivid_settings_construct(const char *file_name, zivid_error_t *out_error)
    {
        zivid_settings_t new_settings = nullptr;
        if(file_name == nullptr)
        {
            *out_error = new zivid_error{ "File name is null" };
            return new_settings;
        }
        translateExceptions(out_error, [&]() {
            new_settings = std::make_unique<zivid_settings>(std::string{ file_name }).release();
        });
        return new_settings;
    }

    ZIVID_DEFINE_DESTRUCT(zivid_settings);

    zivid_application_t zivid_application_create(zivid_error_t *out_error)
    {
        zivid_application_t new_application = nullptr;
        translateExceptions(out_error, [&]() { new_application = std::make_unique<zivid_application>().release(); });
        return new_application;
    }

    ZIVID_DEFINE_DESTRUCT(zivid_application);

    ZIVID_DEFINE_DESTRUCT(zivid_camera);

    zivid_camera_t zivid_connect_camera(zivid_application_t application, zivid_error_t *out_error)
    {
        zivid_camera_t new_camera = nullptr;
        if(application == nullptr)
        {
            *out_error = new zivid_error{ "Application is null" };
            return new_camera;
        }
        translateExceptions(out_error, [&]() {
            new_camera = std::make_unique<zivid_camera>(application->actual.connectCamera()).release();
        });
        return new_camera;
    }

    void zivid_camera_disconnect(zivid_camera_t camera)
    {
        if(camera)
        {
            camera->actual.disconnect();
        }
    }

    ZIVID_DEFINE_DESTRUCT(zivid_frame);

    zivid_frame_t zivid_camera_capture(zivid_camera_t camera, zivid_settings_t settings, zivid_error_t *out_error)
    {
        if(camera == nullptr)
        {
            *out_error = new zivid_error{ "Camera is null" };
            return nullptr;
        }
        if(settings == nullptr)
        {
            *out_error = new zivid_error{ "Settings is null" };
            return nullptr;
        }

        zivid_frame_t frame = nullptr;
        translateExceptions(out_error, [&]() {
            frame = std::make_unique<zivid_frame>(camera->actual.capture(settings->actual)).release();
        });
        return frame;
    }

    void zivid_frame_save(zivid_frame_t frame, const char *file_name, zivid_error_t *out_error)
    {
        if(frame == nullptr)
        {
            *out_error = new zivid_error{ "Frame is null" };
            return;
        }
        if(file_name == nullptr)
        {
            *out_error = new zivid_error{ "File name is null" };
            return;
        }

        translateExceptions(out_error, [&]() { frame->actual.save(std::string{ file_name }); });
    }

    zivid_frame_t zivid_frame_load(const char *file_name, zivid_error_t *out_error)
    {
        zivid_frame_t frame = nullptr;
        translateExceptions(out_error, [&]() {
            frame = std::make_unique<zivid_frame>(Zivid::Frame(file_name)).release();
        });
        return frame;
    }

    ZIVID_DEFINE_DESTRUCT(zivid_point_cloud);

    zivid_point_cloud_t zivid_frame_get_point_cloud(zivid_frame_t frame, zivid_error_t *out_error)
    {
        if(frame == nullptr)
        {
            *out_error = new zivid_error{ "Frame is null" };
            return nullptr;
        }

        zivid_point_cloud_t new_point_cloud = nullptr;
        translateExceptions(out_error, [&]() {
            new_point_cloud = std::make_unique<zivid_point_cloud>(frame->actual.pointCloud()).release();
        });
        return new_point_cloud;
    }

    void zivid_array2d_point_xyz_destruct(zivid_array2d_point_xyz_t array)
    {
        if(array)
        {
            delete[] array->data;
            delete array;
        }
    }

    void zivid_array2d_point_z_destruct(zivid_array2d_point_z_t array)
    {
        if(array)
        {
            delete[] array->data;
            delete array;
        }
    }

    void zivid_array2d_color_rgba_srgb_destruct(zivid_array2d_color_rgba_srgb_t array)
    {
        if(array)
        {
            delete[] array->data;
            delete array;
        }
    }

    zivid_array2d_point_xyz_t zivid_point_cloud_copy_points_xyz(
        zivid_point_cloud_t point_cloud,
        zivid_error_t *out_error)
    {
        if(point_cloud == nullptr)
        {
            *out_error = new zivid_error{ "Point cloud is null" };
            return nullptr;
        }

        zivid_array2d_point_xyz_t new_array = nullptr;
        translateExceptions(out_error, [&]() {
            const auto &actual = point_cloud->actual;
            uint32_t width = actual.width();
            uint32_t height = actual.height();
            auto data = new Zivid::PointXYZ[width * height];
            actual.copyData(data);
            auto c_point_xyz_type = reinterpret_cast<zivid_point_xyz_t *>(data);
            new_array = new zivid_array2d_point_xyz{ c_point_xyz_type, width, height };
        });

        return new_array;
    }

    zivid_array2d_point_z_t zivid_point_cloud_copy_points_z(zivid_point_cloud_t point_cloud, zivid_error_t *out_error)
    {
        if(point_cloud == nullptr)
        {
            *out_error = new zivid_error{ "Point cloud is null" };
            return nullptr;
        }

        zivid_array2d_point_z_t new_array = nullptr;
        translateExceptions(out_error, [&]() {
            const auto &actual = point_cloud->actual;
            uint32_t width = actual.width();
            uint32_t height = actual.height();
            auto data = new Zivid::PointZ[width * height];
            actual.copyData(data);
            auto c_point_z_type = reinterpret_cast<zivid_point_z_t *>(data);
            new_array = new zivid_array2d_point_z{ c_point_z_type, width, height };
        });

        return new_array;
    }

    zivid_array2d_color_rgba_srgb_t zivid_point_cloud_copy_color_rgba_srgb(
        zivid_point_cloud_t point_cloud,
        zivid_error_t *out_error)
    {
        if(point_cloud == nullptr)
        {
            *out_error = new zivid_error{ "Point cloud is null" };
            return nullptr;
        }

        zivid_array2d_color_rgba_srgb_t colors_array = nullptr;
        translateExceptions(out_error, [&]() {
            const auto &actual = point_cloud->actual;
            uint32_t width = actual.width();
            uint32_t height = actual.height();
            auto data = new Zivid::ColorRGBA_SRGB[width * height];
            actual.copyData(data);
            auto c_color_rgba_srgb_type = reinterpret_cast<zivid_color_rgba_srgb_t *>(data);
            colors_array = new zivid_array2d_color_rgba_srgb{ c_color_rgba_srgb_type, width, height };
        });

        return colors_array;
    }

} // End of extern "C"
