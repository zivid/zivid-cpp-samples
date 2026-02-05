use std::path::PathBuf;
use zivid::{sample_data_dir, Application, Settings};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // ====================
    // Connect to a camera
    // ====================
    let application = Application::new()?;
    let settings_path = {
        let mut path = sample_data_dir();
        path.push("Settings");
        path.push("Default.yml");
        path
    };
    let settings = Settings::from_file(settings_path)?;
    let camera = application.connect_camera()?;
    // =====================
    // Take a 2D+3D capture
    // =====================
    let frame = camera.capture(&settings)?;
    let output_path = PathBuf::from("output.zdf");
    frame.save(output_path.clone())?;
    println!("Saved frame to {}", output_path.display());
    // ====================
    // Get the point cloud
    // ====================
    let point_cloud = frame.get_point_cloud()?;
    let points = point_cloud.copy_points_xyz()?;
    let point_count = points.width() * points.height();
    println!(
        "Point cloud [{} x {}] has {} points",
        points.width(),
        points.height(),
        point_count
    );
    // ==================
    // Get the depth map
    // ==================
    let depth_map = point_cloud.copy_points_z()?;
    println!(
        "Depth map [{} x {}] has {} points",
        depth_map.width(),
        depth_map.height(),
        depth_map.width() * depth_map.height()
    );
    // ===========
    // Get colors
    // ===========
    let colors = point_cloud.copy_colors_rgba()?;
    println!(
        "Color map [{} x {}] has {} colors",
        colors.width(),
        colors.height(),
        colors.width() * colors.height()
    );
    Ok(())
}
