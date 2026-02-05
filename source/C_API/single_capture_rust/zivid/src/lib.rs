//! Rust wrapper for the Zivid C API
//! This module provides a safe Rust interface to the Zivid SDK C API.

use std::ffi::{c_char, CStr, CString};
use std::ops::Index;
use std::path::Path;
use std::path::PathBuf;
use std::ptr;

// =============================================================================
// FFI Declarations
// =============================================================================

#[repr(C)]
struct ZividErrorC {
    _private: [u8; 0],
}

type ZividErrorPtr = *mut ZividErrorC;

#[repr(C)]
struct ZividSettings {
    _private: [u8; 0],
}

type ZividSettingsPtr = *mut ZividSettings;

#[repr(C)]
struct ZividApplication {
    _private: [u8; 0],
}

type ZividApplicationPtr = *mut ZividApplication;

#[repr(C)]
struct ZividCamera {
    _private: [u8; 0],
}

type ZividCameraPtr = *mut ZividCamera;

#[repr(C)]
struct ZividFrame {
    _private: [u8; 0],
}

type ZividFramePtr = *mut ZividFrame;

#[repr(C)]
struct ZividPointCloud {
    _private: [u8; 0],
}

type ZividPointCloudPtr = *mut ZividPointCloud;

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct PointXYZ {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct PointZ {
    pub z: f32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct ColorRGBA {
    pub r: u8,
    pub g: u8,
    pub b: u8,
    pub a: u8,
}

#[repr(C)]
struct Array2DPointXYZ {
    data: *mut PointXYZ,
    width: u32,
    height: u32,
}

type Array2DPointXYZPtr = *mut Array2DPointXYZ;

#[repr(C)]
struct Array2DPointZ {
    data: *mut PointZ,
    width: u32,
    height: u32,
}

type Array2DPointZPtr = *mut Array2DPointZ;

#[repr(C)]
struct Array2DColorRGBA {
    data: *mut ColorRGBA,
    width: u32,
    height: u32,
}

type Array2DColorRGBAPtr = *mut Array2DColorRGBA;

#[link(name = "CoreCAPI")]
extern "C" {
    fn zivid_error_message(error: ZividErrorPtr) -> *const c_char;
    fn zivid_error_destruct(error: ZividErrorPtr);

    fn zivid_settings_construct(
        file_name: *const c_char,
        out_error: *mut ZividErrorPtr,
    ) -> ZividSettingsPtr;
    fn zivid_settings_destruct(settings: ZividSettingsPtr);

    fn zivid_application_create(out_error: *mut ZividErrorPtr) -> ZividApplicationPtr;
    fn zivid_application_destruct(application: ZividApplicationPtr);

    fn zivid_connect_camera(
        application: ZividApplicationPtr,
        out_error: *mut ZividErrorPtr,
    ) -> ZividCameraPtr;
    fn zivid_camera_destruct(camera: ZividCameraPtr);
    fn zivid_camera_disconnect(camera: ZividCameraPtr);

    fn zivid_camera_capture(
        camera: ZividCameraPtr,
        settings: ZividSettingsPtr,
        out_error: *mut ZividErrorPtr,
    ) -> ZividFramePtr;
    fn zivid_frame_destruct(frame: ZividFramePtr);
    fn zivid_frame_save(
        frame: ZividFramePtr,
        file_name: *const c_char,
        out_error: *mut ZividErrorPtr,
    );
    fn zivid_frame_load(file_name: *const c_char, out_error: *mut ZividErrorPtr) -> ZividFramePtr;
    fn zivid_frame_get_point_cloud(
        frame: ZividFramePtr,
        out_error: *mut ZividErrorPtr,
    ) -> ZividPointCloudPtr;

    fn zivid_point_cloud_destruct(point_cloud: ZividPointCloudPtr);
    fn zivid_point_cloud_copy_points_xyz(
        point_cloud: ZividPointCloudPtr,
        out_error: *mut ZividErrorPtr,
    ) -> Array2DPointXYZPtr;
    fn zivid_point_cloud_copy_points_z(
        point_cloud: ZividPointCloudPtr,
        out_error: *mut ZividErrorPtr,
    ) -> Array2DPointZPtr;
    fn zivid_point_cloud_copy_color_rgba_srgb(
        point_cloud: ZividPointCloudPtr,
        out_error: *mut ZividErrorPtr,
    ) -> Array2DColorRGBAPtr;

    fn zivid_array2d_point_xyz_destruct(array: Array2DPointXYZPtr);
    fn zivid_array2d_point_z_destruct(array: Array2DPointZPtr);
    fn zivid_array2d_color_rgba_srgb_destruct(array: Array2DColorRGBAPtr);
}

// =============================================================================
// Error Handling
// =============================================================================

#[derive(Debug)]
pub struct ZividError {
    message: String,
}

impl ZividError {
    fn new(message: String) -> Self {
        Self { message }
    }
}

impl std::fmt::Display for ZividError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Zivid error: {}", self.message)
    }
}

impl std::error::Error for ZividError {}

pub type Result<T> = std::result::Result<T, ZividError>;

fn check_error(error_ptr: ZividErrorPtr) -> Result<()> {
    if error_ptr.is_null() {
        Ok(())
    } else {
        unsafe {
            let message = zivid_error_message(error_ptr);
            let msg_str = if message.is_null() {
                String::from("Unknown error")
            } else {
                CStr::from_ptr(message).to_string_lossy().into_owned()
            };
            zivid_error_destruct(error_ptr);
            Err(ZividError::new(msg_str))
        }
    }
}

// =============================================================================
// Safe Wrapper Types
// =============================================================================

pub struct Application {
    ptr: ZividApplicationPtr,
}

impl Application {
    pub fn new() -> Result<Self> {
        let mut error: ZividErrorPtr = ptr::null_mut();
        let ptr = unsafe { zivid_application_create(&mut error) };
        check_error(error)?;

        if ptr.is_null() {
            return Err(ZividError::new("Failed to create application".into()));
        }

        Ok(Self { ptr })
    }

    pub fn connect_camera(&self) -> Result<Camera> {
        let mut error: ZividErrorPtr = ptr::null_mut();
        let camera_ptr = unsafe { zivid_connect_camera(self.ptr, &mut error) };
        check_error(error)?;

        if camera_ptr.is_null() {
            return Err(ZividError::new("Failed to connect to camera".into()));
        }

        Ok(Camera { ptr: camera_ptr })
    }
}

impl Drop for Application {
    fn drop(&mut self) {
        if !self.ptr.is_null() {
            unsafe {
                zivid_application_destruct(self.ptr);
            }
        }
    }
}

pub struct Settings {
    ptr: ZividSettingsPtr,
}

impl Settings {
    pub fn from_file(path: impl AsRef<Path>) -> Result<Self> {
        let path_str = path.as_ref().to_string_lossy().into_owned();
        let c_path = CString::new(path_str).map_err(|e| ZividError::new(e.to_string()))?;

        let mut error: ZividErrorPtr = ptr::null_mut();
        let ptr = unsafe { zivid_settings_construct(c_path.as_ptr(), &mut error) };
        check_error(error)?;

        if ptr.is_null() {
            return Err(ZividError::new("Failed to construct settings".into()));
        }

        Ok(Self { ptr })
    }
}

impl Drop for Settings {
    fn drop(&mut self) {
        if !self.ptr.is_null() {
            unsafe {
                zivid_settings_destruct(self.ptr);
            }
        }
    }
}

pub struct Camera {
    ptr: ZividCameraPtr,
}

impl Camera {
    pub fn capture(&self, settings: &Settings) -> Result<Frame> {
        let mut error: ZividErrorPtr = ptr::null_mut();
        let frame_ptr = unsafe { zivid_camera_capture(self.ptr, settings.ptr, &mut error) };
        check_error(error)?;

        if frame_ptr.is_null() {
            return Err(ZividError::new("Failed to capture frame".into()));
        }

        Ok(Frame { ptr: frame_ptr })
    }

    pub fn disconnect(&self) {
        if !self.ptr.is_null() {
            unsafe {
                zivid_camera_disconnect(self.ptr);
            }
        }
    }
}

impl Drop for Camera {
    fn drop(&mut self) {
        if !self.ptr.is_null() {
            unsafe {
                zivid_camera_destruct(self.ptr);
            }
        }
    }
}

pub struct Frame {
    ptr: ZividFramePtr,
}

impl Frame {
    pub fn load(path: impl AsRef<Path>) -> Result<Self> {
        let path_str = path.as_ref().to_string_lossy().into_owned();
        let c_path = CString::new(path_str).map_err(|e| ZividError::new(e.to_string()))?;

        let mut error: ZividErrorPtr = ptr::null_mut();
        let ptr = unsafe { zivid_frame_load(c_path.as_ptr(), &mut error) };
        check_error(error)?;

        if ptr.is_null() {
            return Err(ZividError::new("Failed to load frame".into()));
        }

        Ok(Self { ptr })
    }

    pub fn save(&self, path: impl AsRef<Path>) -> Result<()> {
        let path_str = path.as_ref().to_string_lossy().into_owned();
        let c_path = CString::new(path_str).map_err(|e| ZividError::new(e.to_string()))?;

        let mut error: ZividErrorPtr = ptr::null_mut();
        unsafe {
            zivid_frame_save(self.ptr, c_path.as_ptr(), &mut error);
        }
        check_error(error)?;

        Ok(())
    }

    pub fn get_point_cloud(&self) -> Result<PointCloud> {
        let mut error: ZividErrorPtr = ptr::null_mut();
        let pc_ptr = unsafe { zivid_frame_get_point_cloud(self.ptr, &mut error) };
        check_error(error)?;

        if pc_ptr.is_null() {
            return Err(ZividError::new("Failed to get point cloud".into()));
        }

        Ok(PointCloud { ptr: pc_ptr })
    }
}

impl Drop for Frame {
    fn drop(&mut self) {
        if !self.ptr.is_null() {
            unsafe {
                zivid_frame_destruct(self.ptr);
            }
        }
    }
}

pub struct PointCloud {
    ptr: ZividPointCloudPtr,
}

impl PointCloud {
    pub fn copy_points_xyz(&self) -> Result<Array2D<PointXYZ>> {
        let mut error: ZividErrorPtr = ptr::null_mut();
        let array_ptr = unsafe { zivid_point_cloud_copy_points_xyz(self.ptr, &mut error) };
        check_error(error)?;

        if array_ptr.is_null() {
            return Err(ZividError::new("Failed to copy XYZ points".into()));
        }

        let (width, height, data) = unsafe {
            let arr = &*array_ptr;
            let len = (arr.width * arr.height) as usize;
            let data = if len > 0 && !arr.data.is_null() {
                std::slice::from_raw_parts(arr.data, len).to_vec()
            } else {
                Vec::new()
            };
            (arr.width, arr.height, data)
        };

        unsafe {
            zivid_array2d_point_xyz_destruct(array_ptr);
        }

        Ok(Array2D {
            data,
            width,
            height,
        })
    }

    pub fn copy_points_z(&self) -> Result<Array2D<PointZ>> {
        let mut error: ZividErrorPtr = ptr::null_mut();
        let array_ptr = unsafe { zivid_point_cloud_copy_points_z(self.ptr, &mut error) };
        check_error(error)?;

        if array_ptr.is_null() {
            return Err(ZividError::new("Failed to copy Z points".into()));
        }

        let (width, height, data) = unsafe {
            let arr = &*array_ptr;
            let len = (arr.width * arr.height) as usize;
            let data = if len > 0 && !arr.data.is_null() {
                std::slice::from_raw_parts(arr.data, len).to_vec()
            } else {
                Vec::new()
            };
            (arr.width, arr.height, data)
        };

        unsafe {
            zivid_array2d_point_z_destruct(array_ptr);
        }

        Ok(Array2D {
            data,
            width,
            height,
        })
    }

    pub fn copy_colors_rgba(&self) -> Result<Array2D<ColorRGBA>> {
        let mut error: ZividErrorPtr = ptr::null_mut();
        let array_ptr = unsafe { zivid_point_cloud_copy_color_rgba_srgb(self.ptr, &mut error) };
        check_error(error)?;

        if array_ptr.is_null() {
            return Err(ZividError::new("Failed to copy RGBA colors".into()));
        }

        let (width, height, data) = unsafe {
            let arr = &*array_ptr;
            let len = (arr.width * arr.height) as usize;
            let data = if len > 0 && !arr.data.is_null() {
                std::slice::from_raw_parts(arr.data, len).to_vec()
            } else {
                Vec::new()
            };
            (arr.width, arr.height, data)
        };

        unsafe {
            zivid_array2d_color_rgba_srgb_destruct(array_ptr);
        }

        Ok(Array2D {
            data,
            width,
            height,
        })
    }
}

impl Drop for PointCloud {
    fn drop(&mut self) {
        if !self.ptr.is_null() {
            unsafe {
                zivid_point_cloud_destruct(self.ptr);
            }
        }
    }
}

#[derive(Debug)]
pub struct Array2D<T> {
    data: Vec<T>,
    width: u32,
    height: u32,
}

impl<T> Array2D<T> {
    pub fn width(&self) -> u32 {
        self.width
    }

    pub fn height(&self) -> u32 {
        self.height
    }

    pub fn len(&self) -> usize {
        self.data.len()
    }

    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }

    pub fn as_slice(&self) -> &[T] {
        &self.data
    }

    pub fn into_data(self) -> Vec<T> {
        self.data
    }

    pub fn get(&self, row: u32, col: u32) -> Option<&T> {
        if row < self.height && col < self.width {
            let idx = (row * self.width + col) as usize;
            Some(&self.data[idx])
        } else {
            None
        }
    }

    pub fn get_linear(&self, index: usize) -> Option<&T> {
        self.data.get(index)
    }
}

impl<T: Clone> Index<(u32, u32)> for Array2D<T> {
    type Output = T;

    fn index(&self, (row, col): (u32, u32)) -> &Self::Output {
        assert!(
            row < self.height && col < self.width,
            "Array2D index out of bounds: ({}, {}) for size ({}, {})",
            row,
            col,
            self.height,
            self.width
        );
        let idx = (row * self.width + col) as usize;
        &self.data[idx]
    }
}

impl<T: Clone> Index<usize> for Array2D<T> {
    type Output = T;

    fn index(&self, index: usize) -> &Self::Output {
        &self.data[index]
    }
}

pub fn sample_data_dir() -> PathBuf {
    if cfg!(target_os = "windows") {
        PathBuf::from("C:/ProgramData/Zivid")
    } else {
        PathBuf::from("/usr/share/Zivid/data")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_array2d_creation() {
        let data = vec![
            PointXYZ {
                x: 1.0,
                y: 2.0,
                z: 3.0,
            },
            PointXYZ {
                x: 4.0,
                y: 5.0,
                z: 6.0,
            },
        ];
        let array = Array2D {
            data,
            width: 2,
            height: 1,
        };

        assert_eq!(array.width(), 2);
        assert_eq!(array.height(), 1);
        assert_eq!(array.len(), 2);

        let point = array.get(0, 0).unwrap();
        assert_eq!(point.x, 1.0);
        assert_eq!(point.y, 2.0);
        assert_eq!(point.z, 3.0);
    }

    #[test]
    fn test_array2d_out_of_bounds() {
        let array: Array2D<PointXYZ> = Array2D {
            data: vec![PointXYZ::default()],
            width: 1,
            height: 1,
        };

        assert!(array.get(0, 0).is_some());
        assert!(array.get(1, 0).is_none());
        assert!(array.get(0, 1).is_none());
    }

    #[test]
    fn test_sample_data_dir() {
        let path = sample_data_dir();
        // Just verify it returns something reasonable
        assert!(!path.as_os_str().is_empty());
    }

    #[test]
    fn test_error_display() {
        let error = ZividError::new("test error".into());
        assert_eq!(format!("{}", error), "Zivid error: test error");
    }
}
