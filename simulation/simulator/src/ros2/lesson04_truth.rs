//! Evaluation-only ground truth, extracted from the same world snapshot as the RGB render.
//! Never supplied to the detector, PnP solver or tracker.
use crate::capture::CaptureCamera;
use crate::components::{Controlled, InfantryGimbal};
use crate::robomaster::prelude::{Armor, ArmorParts, ArmorRoot, MarkerData, Team};
use crate::training::TrainingTarget;
use bevy::prelude::*;
use bevy::render::Extract;

#[derive(Resource, Clone, Default)]
pub struct Lesson04Truth(pub serde_json::Value);

pub fn extract_truth(
    mut commands: Commands,
    armors: Extract<Query<(Entity, &Armor, &ArmorRoot, &ArmorParts)>>,
    markers: Extract<Query<(&GlobalTransform, &MarkerData)>>,
    parents: Extract<Query<&ChildOf>>,
    targets: Extract<Query<&TrainingTarget>>,
    camera: Extract<Single<&GlobalTransform, With<CaptureCamera>>>,
    gimbal: Extract<Single<&GlobalTransform, (With<Controlled>, With<InfantryGimbal>)>>,
    time: Extract<Res<Time<Fixed>>>,
    board: Extract<Res<crate::scoreboard::Board>>,
) {
    let cam = **camera;
    let gim = **gimbal;
    let camera_inverse = cam.to_matrix().inverse();
    let optical = |p: Vec3| {
        let v = camera_inverse.transform_point3(p);
        [v.x, -v.y, -v.z]
    };
    let mut plates = Vec::new();
    for (entity, armor, root, parts) in armors.iter() {
        if !matches!(armor.team, Team::Blue) {
            continue;
        }
        let mut ancestor = entity;
        let mut kind = None;
        for _ in 0..30 {
            if let Ok(t) = targets.get(ancestor) {
                kind = Some(t.kind);
                break;
            }
            let Ok(p) = parents.get(ancestor) else { break; };
            ancestor = p.parent();
        }
        let Some(kind) = kind else { continue; };
        let Ok((tf, MarkerData(local))) = markers.get(parts.marker()) else { continue; };
        let world = local.map(|p| tf.transform_point(p));
        let center = world.iter().copied().sum::<Vec3>() / 4.0;
        let points = world.map(optical);
        if points.iter().any(|p| p[2] <= 0.1) {
            continue;
        }
        plates.push(serde_json::json!({
            "id": root.id.as_usize(), "kind": kind,
            "label": format!("{:?}", armor.label),
            "center_camera": optical(center),
            "center_world_bevy": center.to_array(), "corners_camera": points
        }));
    }
    let q = gim.rotation();
    commands.insert_resource(Lesson04Truth(serde_json::json!({
        "sim_time": time.elapsed_secs_f64(), "plates": plates,
        "camera_world_bevy": cam.translation().to_array(),
        "gimbal_world_bevy": gim.translation().to_array(),
        "gimbal_quat_bevy_xyzw": q.to_array(),
        "scoreboard": {"selected":board.0.selected,"running":board.0.running,
            "scores":board.0.scores.iter().map(|s|serde_json::json!({
                "shots":s.shots,"hits":s.hits,"damage":s.damage,
                "off_target":s.off_target,"seconds":s.seconds
            })).collect::<Vec<_>>()}
    })));
}
