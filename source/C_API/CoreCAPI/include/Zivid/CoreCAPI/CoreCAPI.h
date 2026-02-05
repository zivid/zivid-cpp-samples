#pragma once

#include "corecapi_export.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct zivid_error *zivid_error_t;
    CORECAPI_EXPORT const char *zivid_error_message(zivid_error_t error);
    CORECAPI_EXPORT void zivid_error_destruct(zivid_error_t error);

    typedef struct zivid_settings *zivid_settings_t;
    CORECAPI_EXPORT zivid_settings_t zivid_settings_construct(const char *file_name, zivid_error_t *out_error);
    CORECAPI_EXPORT void zivid_settings_destruct(zivid_settings_t settings);

    typedef struct zivid_application *zivid_application_t;
    CORECAPI_EXPORT zivid_application_t zivid_application_create(zivid_error_t *out_error);
    CORECAPI_EXPORT void zivid_application_destruct(zivid_application_t application);

    typedef struct zivid_camera *zivid_camera_t;
    CORECAPI_EXPORT void zivid_camera_destruct(zivid_camera_t camera);
    CORECAPI_EXPORT zivid_camera_t zivid_connect_camera(zivid_application_t application, zivid_error_t *out_error);
    CORECAPI_EXPORT void zivid_camera_disconnect(zivid_camera_t camera);

    typedef struct zivid_frame *zivid_frame_t;
    CORECAPI_EXPORT void zivid_frame_destruct(zivid_frame_t frame);
    CORECAPI_EXPORT zivid_frame_t
    zivid_camera_capture(zivid_camera_t camera, zivid_settings_t settings, zivid_error_t *out_error);
    CORECAPI_EXPORT void zivid_frame_save(zivid_frame_t frame, const char *file_name, zivid_error_t *out_error);
    CORECAPI_EXPORT zivid_frame_t zivid_frame_load(const char *file_name, zivid_error_t *out_error);

    typedef struct zivid_point_cloud *zivid_point_cloud_t;
    CORECAPI_EXPORT void zivid_point_cloud_destruct(zivid_point_cloud_t point_cloud);
    CORECAPI_EXPORT zivid_point_cloud_t zivid_frame_get_point_cloud(zivid_frame_t frame, zivid_error_t *out_error);

    // duplicated from Zivid/Point.h
    typedef struct zivid_point_xyz
    {
        float x;
        float y;
        float z;
    } zivid_point_xyz_t;

    typedef struct zivid_point_z
    {
        float z;
    } zivid_point_z_t;

    typedef struct zivid_color_rgba_srgb
    {
        uint8_t r;
        uint8_t g;
        uint8_t b;
        uint8_t a;
    } zivid_color_rgba_srgb_t;

    struct zivid_array2d_color_rgba_srgb
    {
        zivid_color_rgba_srgb_t *data;
        uint32_t width;
        uint32_t height;
    };
    typedef struct zivid_array2d_color_rgba_srgb *zivid_array2d_color_rgba_srgb_t;

    struct zivid_array2d_point_xyz
    {
        zivid_point_xyz_t *data;
        uint32_t width;
        uint32_t height;
    };
    typedef struct zivid_array2d_point_xyz *zivid_array2d_point_xyz_t;

    struct zivid_array2d_point_z
    {
        zivid_point_z_t *data;
        uint32_t width;
        uint32_t height;
    };
    typedef struct zivid_array2d_point_z *zivid_array2d_point_z_t;

    CORECAPI_EXPORT void zivid_array2d_point_z_destruct(zivid_array2d_point_z_t array);
    CORECAPI_EXPORT void zivid_array2d_point_xyz_destruct(zivid_array2d_point_xyz_t array);
    CORECAPI_EXPORT void zivid_array2d_color_rgba_srgb_destruct(zivid_array2d_color_rgba_srgb_t array);
    CORECAPI_EXPORT zivid_array2d_point_xyz_t
    zivid_point_cloud_copy_points_xyz(zivid_point_cloud_t point_cloud, zivid_error_t *out_error);
    CORECAPI_EXPORT zivid_array2d_point_z_t
    zivid_point_cloud_copy_points_z(zivid_point_cloud_t point_cloud, zivid_error_t *out_error);
    CORECAPI_EXPORT zivid_array2d_color_rgba_srgb_t
    zivid_point_cloud_copy_color_rgba_srgb(zivid_point_cloud_t point_cloud, zivid_error_t *out_error);

#ifdef __cplusplus
} // End of extern "C"
#endif
