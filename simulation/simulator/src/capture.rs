pub mod depth;
pub mod driver;
pub mod view_copy;

use bevy::camera::RenderTarget;
use bevy::core_pipeline::prepass::DepthPrepass;
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::post_process::bloom::Bloom;
use bevy::prelude::*;

pub use driver::CaptureBundle;

#[derive(Component)]
pub struct CaptureSource;

#[derive(Component)]
pub struct CaptureCamera;

#[derive(Resource, Deref, Clone)]
pub struct ImageHandle(pub Handle<Image>);

#[derive(Resource, Clone, Copy)]
pub struct CameraFov(pub f32);

pub const CAPTURE_CAMERA_ORDER: isize = -100;

pub fn setup_capture_camera(world: &mut World) {
    let capture_camera_exists = {
        let mut query = world.query_filtered::<Entity, With<CaptureCamera>>();
        query.iter(world).next().is_some()
    };
    if capture_camera_exists {
        return;
    }

    let render_target_handle = world.resource::<ImageHandle>().0.clone();
    let fov = world.resource::<CameraFov>().0;
    let bloom_intensity = world
        .resource::<crate::config::SimulationConfig>()
        .vision
        .bloom_intensity;

    let camera = world
        .spawn((
            Camera3d::default(),
            Tonemapping::None,
            RenderTarget::Image(render_target_handle.into()),
            Camera {
                order: CAPTURE_CAMERA_ORDER,
                // clear_color: ClearColorConfig::Custom(Color::BLACK),
                ..default()
            },
            Projection::Perspective(PerspectiveProjection {
                fov,
                near: 0.1,
                far: 10000.0,
                ..default()
            }),
            Msaa::Off,
            DepthPrepass,
            CaptureCamera,
        ))
        .id();

    // A real lens spreads a blown-out LED into a halo wider than the emitter itself,
    // which is what makes armour bars easy to threshold. Off unless configured.
    if bloom_intensity.is_finite() && bloom_intensity > 0.0 {
        world.entity_mut(camera).insert(Bloom {
            intensity: bloom_intensity,
            ..default()
        });
    }
}

#[derive(Component)]
pub struct PreviewCamera;

pub fn setup_preview_window(world: &mut World) {
    let preview_enabled = world
        .resource::<crate::config::SimulationConfig>()
        .preview
        .enabled;
    if !preview_enabled {
        return;
    }

    let fov = world.resource::<CameraFov>().0;
    let preview_camera_exists = {
        let mut query = world.query_filtered::<Entity, With<PreviewCamera>>();
        query.iter(world).next().is_some()
    };
    if !preview_camera_exists {
        world.spawn((
            // Render the real scene directly to the window. The sensor image stays
            // off-screen for ROS2/Talos capture and is not used as a UI background.
            Camera3d::default(),
            Tonemapping::None,
            Camera {
                order: 1,
                ..default()
            },
            Projection::Perspective(PerspectiveProjection {
                fov,
                near: 0.1,
                far: 10000.0,
                ..default()
            }),
            Msaa::Off,
            PreviewCamera,
        ));
    }
}

pub fn copy_transform(target: &Transform, our: &mut Transform) {
    our.translation = target.translation;
    our.scale = target.scale;
    our.rotation = target.rotation;
}

pub fn sync_capture_camera(
    time: Res<Time>,
    target: Single<&Transform, (With<CaptureSource>, Without<CaptureCamera>)>,
    mut our: Single<&mut Transform, (With<CaptureCamera>, Without<CaptureSource>)>,
    mut preview: Query<
        &mut Transform,
        (With<PreviewCamera>, Without<CaptureSource>, Without<CaptureCamera>),
    >,
    mut last_log: Local<f32>,
) {
    copy_transform(&target, &mut our);
    for mut transform in &mut preview {
        copy_transform(&target, &mut transform);
    }
    if std::env::var_os("DAEDALUS_CAMERA_DIAGNOSTICS").is_some()
        && time.elapsed_secs() - *last_log >= 1.0
    {
        *last_log = time.elapsed_secs();
        info!(target: "daedalus::camera_diagnostics",
            "CAMERA_STATE t={:.3} position={:?} forward={:?}",
            time.elapsed_secs(), target.translation, target.forward());
    }
}

#[derive(Clone, Copy, Debug)]
pub struct CameraIntrinsics {
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    pub width: u32,
    pub height: u32,
}

pub fn compute_camera_intrinsics(width: u32, height: u32, fov_y: f32) -> CameraIntrinsics {
    let fov_y = fov_y as f64;
    let aspect = width as f64 / height as f64;
    let fov_x = 2.0 * ((fov_y / 2.0).tan() * aspect).atan();

    let fx = width as f64 / (2.0 * (fov_x / 2.0).tan());
    let fy = height as f64 / (2.0 * (fov_y / 2.0).tan());

    let cx = width as f64 / 2.0;
    let cy = height as f64 / 2.0;

    CameraIntrinsics {
        fx,
        fy,
        cx,
        cy,
        width,
        height,
    }
}

pub const IMAGE_WIDTH: u32 = 1440;
pub const IMAGE_HEIGHT: u32 = 1080;
