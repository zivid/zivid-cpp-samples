#include "Zivid/CoreCAPI/CoreCAPI.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PATH_MAX_LENGTH 256

static void assert_error(zivid_error_t error)
{
    if(error != NULL)
    {
        printf("Error: %s\n", zivid_error_message(error));
        zivid_error_destruct(error);
        exit(-1);
    }
};

static void print_points(zivid_array2d_point_xyz_t points)
{
    printf("Points:\n");
    for(uint32_t i = 0; i < points->height; i++)
    {
        for(uint32_t j = 0; j < points->width; j++)
        {
            zivid_point_xyz_t point = points->data[i * points->width + j];
            printf("x = %f, y = %f, z = %f\n", point.x, point.y, point.z);
        }
    }
}

int main(int argc, char *argv[])
{
    printf("Running SingleCaptureCAPI\n");
    zivid_error_t error = NULL;
    zivid_application_t application = zivid_application_create(&error);
    assert_error(error);

    // Capture (2D+3D) using default settings from sample data
    char settings_path[PATH_MAX_LENGTH] = { ZIVID_SAMPLE_DATA_DIR };
    const char *suffix = "/Settings/Default.yml";
    size_t available_space = PATH_MAX_LENGTH - strlen(settings_path) - 1;

    if(strlen(suffix) > available_space)
    {
        (void)fprintf(stderr, "Error: Path exceeds maximum length\n");
        exit(-1);
    }

    strncat(settings_path, suffix, available_space);

    zivid_settings_t settings = zivid_settings_construct(settings_path, &error);
    assert_error(error);

    zivid_camera_t camera = zivid_connect_camera(application, &error);
    assert_error(error);

    zivid_frame_t frame = zivid_camera_capture(camera, settings, &error);
    assert_error(error);

    zivid_frame_save(frame, "output.zdf", &error);
    assert_error(error);
    printf("Saved frame to output.zdf\n");

    zivid_point_cloud_t point_cloud = zivid_frame_get_point_cloud(frame, &error);
    assert_error(error);

    zivid_array2d_point_xyz_t points = zivid_point_cloud_copy_points_xyz(point_cloud, &error);
    assert_error(error);

    printf(
        "Point cloud [%d x %d] has %d points\n",
        points->width,
        points->height,
        points->width * points->height * sizeof(zivid_point_xyz_t) / sizeof(float));

    // Depth map
    zivid_array2d_point_z_t depth_map = zivid_point_cloud_copy_points_z(point_cloud, &error);
    assert_error(error);
    printf(
        "Depth map [%d x %d] has %d points\n",
        depth_map->width,
        depth_map->height,
        depth_map->width * depth_map->height * sizeof(zivid_point_z_t) / sizeof(float));

    // Colors
    zivid_array2d_color_rgba_srgb_t colors = zivid_point_cloud_copy_color_rgba_srgb(point_cloud, &error);
    assert_error(error);
    printf(
        "Color map [%d x %d] has %d colors\n",
        colors->width,
        colors->height,
        colors->width * colors->height * sizeof(zivid_color_rgba_srgb_t) / sizeof(uint8_t));

    zivid_array2d_color_rgba_srgb_destruct(colors);
    zivid_array2d_point_z_destruct(depth_map);
    zivid_array2d_point_xyz_destruct(points);
    zivid_point_cloud_destruct(point_cloud);
    zivid_frame_destruct(frame);
    zivid_camera_destruct(camera);
    zivid_settings_destruct(settings);
    zivid_application_destruct(application);

    printf("Successfully ran SingleCaptureCAPI\n");
    return 0;
}
