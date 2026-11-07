use super::*;
use crate::joint::row::RowDimension;
use crate::{
    constraint::SolverBody,
    joint::{FixedJoint, Joint, JointCommon, JointFrame},
    mass::MassProperties,
    rigid_body::RigidBody,
    transform::Transform,
    world::PhysicsWorld,
};
use kapitsa_math::{Mat3, Vec3};

fn test_joint_id() -> crate::world::JointId {
    let mut world = PhysicsWorld::new();
    let body_a = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .unwrap();
    let body_b = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .unwrap();
    world
        .add_joint(
            body_a,
            body_b,
            Joint::Fixed(FixedJoint::new(JointCommon::new(
                JointFrame::IDENTITY,
                JointFrame::IDENTITY,
            ))),
        )
        .unwrap()
}

fn row(key: u8) -> ConstraintRow {
    ConstraintRow {
        key,
        dimension: RowDimension::Linear,
        body_a: 0,
        body_b: 1,
        linear_a: -Vec3::X,
        angular_a: Vec3::ZERO,
        position_angular_a: Vec3::ZERO,
        linear_b: Vec3::X,
        angular_b: Vec3::ZERO,
        error: 0.0,
        target_velocity: 0.0,
        force_bias: 0.0,
        force_driven: false,
        position_enabled: true,
        compliance: 0.0,
        damping: 0.0,
        min_multiplier: f64::NEG_INFINITY,
        max_multiplier: f64::INFINITY,
        min_impulse: f64::NEG_INFINITY,
        max_impulse: f64::INFINITY,
        accumulated_multiplier: 0.0,
        accumulated_impulse: 0.0,
        cached_velocity_effective_mass: 0.0,
    }
}

fn dynamic_solver_body(position: Vec3, local_com: Vec3) -> SolverBody {
    let mass = MassProperties::new(1.0, local_com, Mat3::IDENTITY).expect("valid mass");
    let body = RigidBody::new_dynamic(
        Transform::new(position, kapitsa_math::Quat::IDENTITY).expect("transform"),
        mass,
    )
    .expect("dynamic body");
    let state = *body.state();
    SolverBody::from_prediction(body, state)
}

fn static_solver_body(position: Vec3) -> SolverBody {
    let body = RigidBody::new_static(
        Transform::new(position, kapitsa_math::Quat::IDENTITY).expect("transform"),
    );
    let state = *body.state();
    SolverBody::from_prediction(body, state)
}

fn reaction_constraint(common: JointCommon) -> JointConstraint {
    JointConstraint {
        id: test_joint_id(),
        body_a: 0,
        body_b: 1,
        joint: Joint::Fixed(FixedJoint::new(common)),
    }
}

fn reaction_row(
    dimension: RowDimension,
    linear_a: Vec3,
    angular_a: Vec3,
    linear_b: Vec3,
    angular_b: Vec3,
    impulse: f64,
) -> ConstraintRow {
    ConstraintRow {
        key: 0,
        dimension,
        body_a: 0,
        body_b: 1,
        linear_a,
        angular_a,
        position_angular_a: angular_a,
        linear_b,
        angular_b,
        error: 0.0,
        target_velocity: 0.0,
        force_bias: 0.0,
        force_driven: false,
        position_enabled: true,
        compliance: 0.0,
        damping: 0.0,
        min_multiplier: f64::NEG_INFINITY,
        max_multiplier: f64::INFINITY,
        min_impulse: f64::NEG_INFINITY,
        max_impulse: f64::INFINITY,
        accumulated_multiplier: 1.0e12,
        accumulated_impulse: impulse,
        cached_velocity_effective_mass: 0.0,
    }
}

#[test]
fn break_reaction_sums_linear_and_angular_components_at_anchor() {
    let mut common = JointCommon::new(
        JointFrame::new(Vec3::Y, kapitsa_math::Quat::IDENTITY),
        JointFrame::IDENTITY,
    );
    common.break_force = Some(0.1);
    common.break_torque = Some(0.1);
    let joints = [reaction_constraint(common)];
    let bodies = [
        dynamic_solver_body(Vec3::ZERO, Vec3::ZERO),
        static_solver_body(Vec3::X),
    ];
    let mut workspace = JointWorkspace::new();
    workspace.rows.push(reaction_row(
        RowDimension::Linear,
        Vec3::X,
        Vec3::Z,
        -Vec3::X,
        -Vec3::Z,
        2.0,
    ));
    workspace.rows.push(reaction_row(
        RowDimension::Angular,
        Vec3::Y,
        Vec3::Y,
        -Vec3::Y,
        -Vec3::Y,
        3.0,
    ));
    workspace.row_joint_indices.extend([0, 0]);

    workspace
        .collect_break_events(&joints, &bodies, 1.0, ReactionMode::VelocityOnly)
        .expect("collect fixed-joint reaction");

    let event = workspace.break_events().first().expect("break event");
    assert!((event.force() - 13.0_f64.sqrt()).abs() <= 1.0e-12);
    assert!((event.torque() - 5.0).abs() <= 1.0e-12);
}

#[test]
fn break_reaction_uses_dynamic_side_b_at_common_anchor() {
    let mut common = JointCommon::new(JointFrame::IDENTITY, JointFrame::IDENTITY);
    common.break_force = Some(0.1);
    common.break_torque = Some(0.1);
    let joints = [reaction_constraint(common)];
    let bodies = [
        static_solver_body(Vec3::ZERO),
        dynamic_solver_body(Vec3::ZERO, Vec3::Y),
    ];
    let mut workspace = JointWorkspace::new();
    workspace.rows.push(reaction_row(
        RowDimension::Linear,
        Vec3::X,
        Vec3::ZERO,
        -Vec3::X,
        -Vec3::new(0.0, 0.0, 2.5),
        2.0,
    ));
    workspace.row_joint_indices.push(0);

    workspace
        .collect_break_events(&joints, &bodies, 1.0, ReactionMode::VelocityOnly)
        .expect("collect side-B reaction");

    let event = workspace.break_events().first().expect("break event");
    assert!((event.force() - 2.0).abs() <= 1.0e-12);
    assert!((event.torque() - 3.0).abs() <= 1.0e-12);
}

#[test]
fn break_threshold_is_strict_and_load_is_averaged_over_dt() {
    let bodies = [
        dynamic_solver_body(Vec3::ZERO, Vec3::ZERO),
        static_solver_body(Vec3::X),
    ];
    let mut workspace = JointWorkspace::new();
    workspace.rows.push(reaction_row(
        RowDimension::Linear,
        Vec3::X,
        Vec3::ZERO,
        -Vec3::X,
        Vec3::ZERO,
        2.0,
    ));
    workspace.row_joint_indices.push(0);

    let mut common = JointCommon::new(JointFrame::IDENTITY, JointFrame::IDENTITY);
    common.break_force = Some(2.0);
    let at_threshold = [reaction_constraint(common)];
    workspace
        .collect_break_events(&at_threshold, &bodies, 1.0, ReactionMode::VelocityOnly)
        .expect("threshold equality");
    assert!(workspace.break_events().is_empty());

    common.break_force = Some(1.999);
    let below_threshold = [reaction_constraint(common)];
    workspace
        .collect_break_events(&below_threshold, &bodies, 1.0, ReactionMode::VelocityOnly)
        .expect("strict threshold crossing");
    assert_eq!(workspace.break_events().len(), 1);
    assert!((workspace.break_events()[0].force() - 2.0).abs() <= 1.0e-12);

    common.break_force = Some(3.0);
    let timestep_threshold = [reaction_constraint(common)];
    workspace
        .collect_break_events(
            &timestep_threshold,
            &bodies,
            0.5,
            ReactionMode::VelocityOnly,
        )
        .expect("shorter timestep load");
    assert_eq!(workspace.break_events().len(), 1);
    assert!((workspace.break_events()[0].force() - 4.0).abs() <= 1.0e-12);
}

#[test]
fn disabled_and_nondynamic_joints_do_not_break() {
    let bodies = [static_solver_body(Vec3::ZERO), static_solver_body(Vec3::X)];
    let mut workspace = JointWorkspace::new();
    workspace.rows.push(reaction_row(
        RowDimension::Linear,
        Vec3::X,
        Vec3::ZERO,
        -Vec3::X,
        Vec3::ZERO,
        100.0,
    ));
    workspace.row_joint_indices.push(0);

    let mut common = JointCommon::new(JointFrame::IDENTITY, JointFrame::IDENTITY);
    common.break_force = Some(0.1);
    common.enabled = false;
    let disabled = [reaction_constraint(common)];
    workspace
        .collect_break_events(&disabled, &bodies, 1.0, ReactionMode::VelocityOnly)
        .expect("disabled joint");
    assert!(workspace.break_events().is_empty());

    common.enabled = true;
    let nondynamic = [reaction_constraint(common)];
    workspace
        .collect_break_events(&nondynamic, &bodies, 1.0, ReactionMode::VelocityOnly)
        .expect("joint with no dynamic side");
    assert!(workspace.break_events().is_empty());
}

#[test]
fn constraint_estimate_reaction_mode_is_inactive() {
    let mut common = JointCommon::new(JointFrame::IDENTITY, JointFrame::IDENTITY);
    common.break_force = Some(1.0);
    let joints = [reaction_constraint(common)];
    let bodies = [
        dynamic_solver_body(Vec3::ZERO, Vec3::ZERO),
        static_solver_body(Vec3::X),
    ];
    let mut workspace = JointWorkspace::new();

    assert_eq!(
        workspace.collect_break_events(&joints, &bodies, 1.0, ReactionMode::ConstraintEstimate),
        Err(CollisionError::InvalidConfig)
    );
    assert!(workspace.break_events().is_empty());
}

#[test]
fn clear_reuses_row_and_owner_capacity_for_many_joints() {
    let mut workspace = JointWorkspace::new();
    let joint_id = test_joint_id();
    let add_rows = |workspace: &mut JointWorkspace| {
        workspace.begin_prepare();
        for joint_index in 0..32 {
            workspace.set_joint_index(joint_index, joint_id);
            workspace.push(ConstraintRow {
                key: joint_index as u8,
                dimension: RowDimension::Linear,
                body_a: 0,
                body_b: 1,
                linear_a: -Vec3::X,
                angular_a: Vec3::ZERO,
                position_angular_a: Vec3::ZERO,
                linear_b: Vec3::X,
                angular_b: Vec3::ZERO,
                error: 0.0,
                target_velocity: 0.0,
                force_bias: 0.0,
                force_driven: false,
                position_enabled: true,
                compliance: 0.0,
                damping: 0.0,
                min_multiplier: f64::NEG_INFINITY,
                max_multiplier: f64::INFINITY,
                min_impulse: f64::NEG_INFINITY,
                max_impulse: f64::INFINITY,
                accumulated_multiplier: 0.0,
                accumulated_impulse: 0.0,
                cached_velocity_effective_mass: 0.0,
            });
        }
        workspace.finish_prepare();
    };
    add_rows(&mut workspace);
    let row_capacity = workspace.rows.capacity();
    let owner_capacity = workspace.row_joint_indices.capacity();
    let id_capacity = workspace.joint_ids.capacity();
    assert!(row_capacity >= 32);
    assert!(owner_capacity >= 32);
    assert!(id_capacity >= 32);

    workspace.clear();

    assert!(workspace.rows.is_empty());
    assert!(workspace.row_joint_indices.is_empty());
    assert!(workspace.joint_ids.is_empty());
    assert_eq!(workspace.rows.capacity(), row_capacity);
    assert_eq!(workspace.row_joint_indices.capacity(), owner_capacity);
    assert_eq!(workspace.joint_ids.capacity(), id_capacity);

    add_rows(&mut workspace);
    assert_eq!(workspace.rows.capacity(), row_capacity);
    assert_eq!(workspace.row_joint_indices.capacity(), owner_capacity);
    assert_eq!(workspace.joint_ids.capacity(), id_capacity);
}

#[test]
fn multipliers_follow_joint_and_row_keys_and_reset_at_step_boundaries() {
    let mut workspace = JointWorkspace::new();
    let first_joint = test_joint_id();
    let replacement_joint = test_joint_id();

    workspace.begin_prepare();
    workspace.set_joint_index(0, first_joint);
    workspace.push(row(4));
    workspace.finish_prepare();
    workspace.rows_mut()[0].accumulated_multiplier = 3.0;
    workspace.rows_mut()[0].accumulated_impulse = 7.0;

    workspace.begin_prepare();
    workspace.set_joint_index(0, replacement_joint);
    workspace.push(row(4));
    workspace.finish_prepare();
    assert_eq!(workspace.rows()[0].accumulated_multiplier, 0.0);
    assert_eq!(workspace.rows()[0].accumulated_impulse, 0.0);

    workspace.rows_mut()[0].accumulated_multiplier = 5.0;
    workspace.rows_mut()[0].accumulated_impulse = 11.0;
    workspace.begin_prepare();
    workspace.set_joint_index(0, replacement_joint);
    workspace.push(row(5));
    workspace.finish_prepare();
    assert_eq!(workspace.rows()[0].accumulated_multiplier, 0.0);
    assert_eq!(workspace.rows()[0].accumulated_impulse, 0.0);

    workspace.rows_mut()[0].accumulated_multiplier = 13.0;
    workspace.rows_mut()[0].accumulated_impulse = 17.0;
    workspace.begin_prepare();
    workspace.set_joint_index(0, replacement_joint);
    workspace.push(row(5));
    workspace.finish_prepare();
    assert_eq!(workspace.rows()[0].accumulated_multiplier, 13.0);
    assert_eq!(workspace.rows()[0].accumulated_impulse, 17.0);

    workspace.begin_step();
    assert_eq!(workspace.rows()[0].accumulated_multiplier, 0.0);
    assert_eq!(workspace.rows()[0].accumulated_impulse, 0.0);
}

#[test]
fn sequential_updates_keep_row_owner_and_multiplier_identity() {
    let mut workspace = JointWorkspace::new();
    let first_joint = test_joint_id();
    let replacement_joint = test_joint_id();

    workspace.begin_joint_prepare(0, first_joint);
    workspace.push(row(4));
    workspace.finish_joint_prepare().unwrap();
    workspace.rows_mut()[0].accumulated_multiplier = 3.0;

    workspace.begin_joint_prepare(0, first_joint);
    workspace.push(row(4));
    workspace.finish_joint_prepare().unwrap();
    assert_eq!(workspace.rows()[0].accumulated_multiplier, 3.0);

    workspace.begin_joint_prepare(0, replacement_joint);
    workspace.push(row(4));
    workspace.finish_joint_prepare().unwrap();
    assert_eq!(workspace.rows()[0].accumulated_multiplier, 0.0);
    assert_eq!(workspace.joint_ids[0], Some(replacement_joint));
}
