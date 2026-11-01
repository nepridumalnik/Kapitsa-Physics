use super::{row::ConstraintRow, workspace::JointWorkspace};
use crate::{
    collision::{CollisionError, solver::SolverBody},
    constraint,
    transform::Transform,
};
use kapitsa_math::{Quat, Vec3};

/// Shared sequential Gauss–Seidel/XPBD solver for prepared joint rows.
pub(crate) struct JointSolver;

#[derive(Clone, Copy)]
pub(crate) struct JointConstraint {
    pub(crate) body_a: usize,
    pub(crate) body_b: usize,
    pub(crate) joint: super::Joint,
}

#[derive(Clone, Copy)]
struct RowSettings {
    body_a: usize,
    body_b: usize,
    common: super::JointCommon,
    dt: f64,
    position_enabled: bool,
    velocity_target: f64,
    multiplier_bounds: Option<(f64, f64)>,
    impulse_bounds: Option<(f64, f64)>,
}

impl JointSolver {
    pub(crate) fn solve<F>(
        workspace: &mut JointWorkspace,
        bodies: &mut [SolverBody],
        dt: f64,
        position_iterations: u32,
        velocity_iterations: u32,
        mut prepare_rows: F,
    ) -> Result<(), CollisionError>
    where
        F: FnMut(&[SolverBody], &mut JointWorkspace) -> Result<(), CollisionError>,
    {
        if !dt.is_finite() || dt <= 0.0 || position_iterations == 0 || velocity_iterations == 0 {
            return Err(CollisionError::InvalidConfig);
        }
        workspace.begin_step();
        for _ in 0..position_iterations {
            workspace.begin_prepare();
            prepare_rows(bodies, workspace)?;
            workspace.finish_prepare();
            solve_position_iteration(workspace, bodies, dt)?;
        }
        for _ in 0..velocity_iterations {
            workspace.begin_prepare();
            prepare_rows(bodies, workspace)?;
            workspace.finish_prepare();
            solve_velocity_iteration(workspace, bodies, dt)?;
        }
        Ok(())
    }
}

pub(crate) fn prepare_joint_rows(
    bodies: &[SolverBody],
    workspace: &mut JointWorkspace,
    joints: &[JointConstraint],
    dt: f64,
) -> Result<(), CollisionError> {
    for constraint in joints {
        if !constraint.joint.common().enabled {
            continue;
        }
        solver_bodies_at(bodies, constraint.body_a, constraint.body_b)?;
        match constraint.joint {
            super::Joint::Fixed(joint) => append_fixed_rows(
                bodies,
                workspace,
                constraint.body_a,
                constraint.body_b,
                joint,
                dt,
            )?,
            super::Joint::Ball(joint) => append_ball_rows(
                bodies,
                workspace,
                constraint.body_a,
                constraint.body_b,
                joint,
                dt,
            )?,
            super::Joint::Distance(joint) => append_distance_rows(
                bodies,
                workspace,
                constraint.body_a,
                constraint.body_b,
                joint,
                dt,
            )?,
            super::Joint::Hinge(joint) => append_hinge_rows(
                bodies,
                workspace,
                constraint.body_a,
                constraint.body_b,
                joint,
                dt,
            )?,
            super::Joint::Slider(joint) => append_slider_rows(
                bodies,
                workspace,
                constraint.body_a,
                constraint.body_b,
                joint,
                dt,
            )?,
            super::Joint::Cylindrical(joint) => append_cylindrical_rows(
                bodies,
                workspace,
                constraint.body_a,
                constraint.body_b,
                joint,
                dt,
            )?,
            super::Joint::Planar(_)
            | super::Joint::Spring(_)
            | super::Joint::ConeTwist(_)
            | super::Joint::D6(_) => {}
        }
    }
    Ok(())
}

fn append_fixed_rows(
    bodies: &[SolverBody],
    workspace: &mut JointWorkspace,
    body_a: usize,
    body_b: usize,
    joint: super::FixedJoint,
    dt: f64,
) -> Result<(), CollisionError> {
    let (anchor_a, rotation_a) = world_frame(bodies, body_a, joint.common.frame_a)?;
    let (anchor_b, rotation_b) = world_frame(bodies, body_b, joint.common.frame_b)?;
    let settings = row_settings(body_a, body_b, joint.common, dt);
    let position_error = anchor_b
        .checked_sub(anchor_a)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    for local_axis in [Vec3::X, Vec3::Y, Vec3::Z] {
        let axis = rotation_a
            .rotate_vector(local_axis)
            .ok_or(CollisionError::ArithmeticOverflow)?;
        let error = position_error
            .checked_dot(axis)
            .ok_or(CollisionError::ArithmeticOverflow)?;
        push_linear_row(
            bodies,
            workspace,
            settings,
            [anchor_a, anchor_b],
            axis,
            error,
            bilateral_bounds(),
        )?;
    }
    let rotation_error = relative_rotation_error(rotation_a, rotation_b)?;
    for local_axis in [Vec3::X, Vec3::Y, Vec3::Z] {
        let axis = rotation_a
            .rotate_vector(local_axis)
            .ok_or(CollisionError::ArithmeticOverflow)?;
        let error = rotation_error
            .checked_dot(local_axis)
            .ok_or(CollisionError::ArithmeticOverflow)?;
        push_angular_row(workspace, settings, axis, error, bilateral_bounds())?;
    }
    Ok(())
}

fn append_ball_rows(
    bodies: &[SolverBody],
    workspace: &mut JointWorkspace,
    body_a: usize,
    body_b: usize,
    joint: super::BallJoint,
    dt: f64,
) -> Result<(), CollisionError> {
    let (anchor_a, rotation_a) = world_frame(bodies, body_a, joint.common.frame_a)?;
    let (anchor_b, rotation_b) = world_frame(bodies, body_b, joint.common.frame_b)?;
    let settings = row_settings(body_a, body_b, joint.common, dt);
    let position_error = anchor_b
        .checked_sub(anchor_a)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    for axis in [Vec3::X, Vec3::Y, Vec3::Z] {
        let error = position_error
            .checked_dot(axis)
            .ok_or(CollisionError::ArithmeticOverflow)?;
        push_linear_row(
            bodies,
            workspace,
            settings,
            [anchor_a, anchor_b],
            axis,
            error,
            bilateral_bounds(),
        )?;
    }
    append_ball_angular_limits(workspace, settings, rotation_a, rotation_b, joint)?;
    Ok(())
}

fn append_ball_angular_limits(
    workspace: &mut JointWorkspace,
    settings: RowSettings,
    rotation_a: Quat,
    rotation_b: Quat,
    joint: super::BallJoint,
) -> Result<(), CollisionError> {
    let relative = canonical_relative_rotation(rotation_a, rotation_b)?;
    if let Some(limit) = joint.swing_limit {
        let axis_a = rotation_a
            .rotate_vector(Vec3::Z)
            .ok_or(CollisionError::ArithmeticOverflow)?;
        let axis_b = rotation_b
            .rotate_vector(Vec3::Z)
            .ok_or(CollisionError::ArithmeticOverflow)?;
        let cosine = axis_a
            .checked_dot(axis_b)
            .ok_or(CollisionError::ArithmeticOverflow)?
            .clamp(-1.0, 1.0);
        let angle = cosine.acos();
        if angle > limit {
            let swing_axis = axis_a.checked_cross(axis_b);
            let axis = match swing_axis.and_then(Vec3::try_normalize) {
                Some(axis) => axis,
                None => rotation_a
                    .rotate_vector(Vec3::X)
                    .ok_or(CollisionError::ArithmeticOverflow)?,
            };
            push_angular_row(
                workspace,
                settings,
                axis,
                angle - limit,
                (f64::NEG_INFINITY, 0.0),
            )?;
        }
    }
    if let Some(limit) = joint.twist_limit {
        let twist = twist_angle(relative)?;
        let (error, bounds) = if twist < limit.lower {
            (twist - limit.lower, (0.0, f64::INFINITY))
        } else if twist > limit.upper {
            (twist - limit.upper, (f64::NEG_INFINITY, 0.0))
        } else {
            return Ok(());
        };
        let axis = rotation_a
            .rotate_vector(Vec3::Z)
            .ok_or(CollisionError::ArithmeticOverflow)?;
        push_angular_row(workspace, settings, axis, error, bounds)?;
    }
    Ok(())
}

fn append_distance_rows(
    bodies: &[SolverBody],
    workspace: &mut JointWorkspace,
    body_a: usize,
    body_b: usize,
    joint: super::DistanceJoint,
    dt: f64,
) -> Result<(), CollisionError> {
    let (anchor_a, rotation_a) = world_frame(bodies, body_a, joint.common.frame_a)?;
    let (anchor_b, _) = world_frame(bodies, body_b, joint.common.frame_b)?;
    let settings = row_settings(body_a, body_b, joint.common, dt);
    let delta = anchor_b
        .checked_sub(anchor_a)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let distance = delta
        .checked_length()
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let (error, lower, upper) = if distance < joint.distance_limit.lower {
        (distance - joint.distance_limit.lower, 0.0, f64::INFINITY)
    } else if distance > joint.distance_limit.upper {
        (
            distance - joint.distance_limit.upper,
            f64::NEG_INFINITY,
            0.0,
        )
    } else {
        return Ok(());
    };
    let axis = if distance > f64::MIN_POSITIVE {
        delta / distance
    } else {
        rotation_a
            .rotate_vector(Vec3::Z)
            .ok_or(CollisionError::ArithmeticOverflow)?
    };
    push_linear_row(
        bodies,
        workspace,
        settings,
        [anchor_a, anchor_b],
        axis,
        error,
        (lower, upper),
    )
}

fn append_hinge_rows(
    bodies: &[SolverBody],
    workspace: &mut JointWorkspace,
    body_a: usize,
    body_b: usize,
    joint: super::HingeJoint,
    dt: f64,
) -> Result<(), CollisionError> {
    let (anchor_a, rotation_a) = world_frame(bodies, body_a, joint.common.frame_a)?;
    let (anchor_b, rotation_b) = world_frame(bodies, body_b, joint.common.frame_b)?;
    let settings = row_settings(body_a, body_b, joint.common, dt);
    append_anchor_lock(bodies, workspace, settings, anchor_a, anchor_b, rotation_a)?;
    append_axis_alignment(workspace, settings, rotation_a, rotation_b)?;

    let angle = twist_angle(canonical_relative_rotation(rotation_a, rotation_b)?)?;
    if let Some(settings) = drive_settings(settings, joint.drive)? {
        push_angular_row(
            workspace,
            settings,
            rotation_a
                .rotate_vector(Vec3::Z)
                .ok_or(CollisionError::ArithmeticOverflow)?,
            angle - joint.drive.target_position,
            bilateral_bounds(),
        )?;
    }
    if let Some(limit) = joint.angle_limit {
        append_angular_limit(bodies, workspace, settings, rotation_a, angle, limit)?;
    }
    Ok(())
}

fn append_slider_rows(
    bodies: &[SolverBody],
    workspace: &mut JointWorkspace,
    body_a: usize,
    body_b: usize,
    joint: super::SliderJoint,
    dt: f64,
) -> Result<(), CollisionError> {
    let (anchor_a, rotation_a) = world_frame(bodies, body_a, joint.common.frame_a)?;
    let (anchor_b, rotation_b) = world_frame(bodies, body_b, joint.common.frame_b)?;
    let settings = row_settings(body_a, body_b, joint.common, dt);
    append_linear_axes(
        bodies,
        workspace,
        settings,
        anchor_a,
        anchor_b,
        &perpendicular_basis(
            rotation_a
                .rotate_vector(Vec3::Z)
                .ok_or(CollisionError::ArithmeticOverflow)?,
        )?,
    )?;
    append_rotation_lock(workspace, settings, rotation_a, rotation_b)?;

    let axis = rotation_a
        .rotate_vector(Vec3::Z)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let coordinate = anchor_b
        .checked_sub(anchor_a)
        .and_then(|delta| delta.checked_dot(axis))
        .ok_or(CollisionError::ArithmeticOverflow)?;
    if let Some(drive) = drive_settings(settings, joint.drive)? {
        push_linear_row(
            bodies,
            workspace,
            drive,
            [anchor_a, anchor_b],
            axis,
            coordinate - joint.drive.target_position,
            bilateral_bounds(),
        )?;
    }
    if let Some(limit) = joint.translation_limit {
        append_linear_limit(
            bodies,
            workspace,
            settings,
            [anchor_a, anchor_b],
            axis,
            coordinate,
            limit,
        )?;
    }
    Ok(())
}

fn append_cylindrical_rows(
    bodies: &[SolverBody],
    workspace: &mut JointWorkspace,
    body_a: usize,
    body_b: usize,
    joint: super::CylindricalJoint,
    dt: f64,
) -> Result<(), CollisionError> {
    let (anchor_a, rotation_a) = world_frame(bodies, body_a, joint.common.frame_a)?;
    let (anchor_b, rotation_b) = world_frame(bodies, body_b, joint.common.frame_b)?;
    let settings = row_settings(body_a, body_b, joint.common, dt);
    let axis = rotation_a
        .rotate_vector(Vec3::Z)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    append_linear_axes(
        bodies,
        workspace,
        settings,
        anchor_a,
        anchor_b,
        &perpendicular_basis(axis)?,
    )?;
    append_axis_alignment(workspace, settings, rotation_a, rotation_b)?;

    let coordinate = anchor_b
        .checked_sub(anchor_a)
        .and_then(|delta| delta.checked_dot(axis))
        .ok_or(CollisionError::ArithmeticOverflow)?;
    if let Some(drive) = drive_settings(settings, joint.translation_drive)? {
        push_linear_row(
            bodies,
            workspace,
            drive,
            [anchor_a, anchor_b],
            axis,
            coordinate - joint.translation_drive.target_position,
            bilateral_bounds(),
        )?;
    }
    if let Some(limit) = joint.translation_limit {
        append_linear_limit(
            bodies,
            workspace,
            settings,
            [anchor_a, anchor_b],
            axis,
            coordinate,
            limit,
        )?;
    }

    let angle = twist_angle(canonical_relative_rotation(rotation_a, rotation_b)?)?;
    if let Some(drive) = drive_settings(settings, joint.angular_drive)? {
        push_angular_row(
            workspace,
            drive,
            axis,
            angle - joint.angular_drive.target_position,
            bilateral_bounds(),
        )?;
    }
    if let Some(limit) = joint.angle_limit {
        append_angular_limit(bodies, workspace, settings, rotation_a, angle, limit)?;
    }
    Ok(())
}

fn append_anchor_lock(
    bodies: &[SolverBody],
    workspace: &mut JointWorkspace,
    settings: RowSettings,
    anchor_a: Vec3,
    anchor_b: Vec3,
    frame_rotation: Quat,
) -> Result<(), CollisionError> {
    let axes = [Vec3::X, Vec3::Y, Vec3::Z].map(|axis| frame_rotation.rotate_vector(axis));
    let axes = [
        axes[0].ok_or(CollisionError::ArithmeticOverflow)?,
        axes[1].ok_or(CollisionError::ArithmeticOverflow)?,
        axes[2].ok_or(CollisionError::ArithmeticOverflow)?,
    ];
    append_linear_axes(bodies, workspace, settings, anchor_a, anchor_b, &axes)
}

fn append_linear_axes(
    bodies: &[SolverBody],
    workspace: &mut JointWorkspace,
    settings: RowSettings,
    anchor_a: Vec3,
    anchor_b: Vec3,
    axes: &[Vec3],
) -> Result<(), CollisionError> {
    let delta = anchor_b
        .checked_sub(anchor_a)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    for axis in axes.iter().copied() {
        let error = delta
            .checked_dot(axis)
            .ok_or(CollisionError::ArithmeticOverflow)?;
        push_linear_row(
            bodies,
            workspace,
            settings,
            [anchor_a, anchor_b],
            axis,
            error,
            bilateral_bounds(),
        )?;
    }
    Ok(())
}

fn append_rotation_lock(
    workspace: &mut JointWorkspace,
    settings: RowSettings,
    rotation_a: Quat,
    rotation_b: Quat,
) -> Result<(), CollisionError> {
    let error = relative_rotation_error(rotation_a, rotation_b)?;
    for local_axis in [Vec3::X, Vec3::Y, Vec3::Z] {
        let axis = rotation_a
            .rotate_vector(local_axis)
            .ok_or(CollisionError::ArithmeticOverflow)?;
        push_angular_row(
            workspace,
            settings,
            axis,
            error
                .checked_dot(local_axis)
                .ok_or(CollisionError::ArithmeticOverflow)?,
            bilateral_bounds(),
        )?;
    }
    Ok(())
}

fn append_axis_alignment(
    workspace: &mut JointWorkspace,
    settings: RowSettings,
    rotation_a: Quat,
    rotation_b: Quat,
) -> Result<(), CollisionError> {
    let axis_a = rotation_a
        .rotate_vector(Vec3::Z)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let axis_b = rotation_b
        .rotate_vector(Vec3::Z)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let dot = axis_a
        .checked_dot(axis_b)
        .ok_or(CollisionError::ArithmeticOverflow)?
        .clamp(-1.0, 1.0);
    let cross = axis_a
        .checked_cross(axis_b)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let sine = cross
        .checked_length()
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let error = if sine > f64::MIN_POSITIVE {
        cross * (sine.atan2(dot) / sine)
    } else if dot < 0.0 {
        let (axis, _) = crate::collision::solver::tangent_basis(axis_a)?;
        axis * std::f64::consts::PI
    } else {
        Vec3::ZERO
    };
    let (tangent_a, tangent_b) = crate::collision::solver::tangent_basis(axis_a)?;
    for axis in [tangent_a, tangent_b] {
        let component = error
            .checked_dot(axis)
            .ok_or(CollisionError::ArithmeticOverflow)?;
        if component != 0.0 {
            push_angular_row(workspace, settings, axis, component, bilateral_bounds())?;
        }
    }
    Ok(())
}

fn append_linear_limit(
    bodies: &[SolverBody],
    workspace: &mut JointWorkspace,
    settings: RowSettings,
    anchors: [Vec3; 2],
    axis: Vec3,
    coordinate: f64,
    limit: super::JointLimit,
) -> Result<(), CollisionError> {
    let mut row_settings = settings;
    let active = if let Some(violation) = limit_error(coordinate, limit) {
        Some(violation)
    } else {
        let body_a = bodies
            .get(settings.body_a)
            .ok_or(CollisionError::InvalidContact)?;
        let body_b = bodies
            .get(settings.body_b)
            .ok_or(CollisionError::InvalidContact)?;
        let speed =
            constraint::relative_velocity_at(body_a.body(), body_b.body(), anchors[0], anchors[1])
                .map_err(map_constraint_error)?
                .checked_dot(axis)
                .ok_or(CollisionError::ArithmeticOverflow)?;
        let predicted = coordinate + speed * settings.dt;
        if !predicted.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        if speed > 0.0 && predicted > limit.upper {
            row_settings.position_enabled = false;
            row_settings.velocity_target = (limit.upper - coordinate) / settings.dt;
            Some((0.0, (f64::NEG_INFINITY, 0.0)))
        } else if speed < 0.0 && predicted < limit.lower {
            row_settings.position_enabled = false;
            row_settings.velocity_target = (limit.lower - coordinate) / settings.dt;
            Some((0.0, (0.0, f64::INFINITY)))
        } else {
            None
        }
    };
    if let Some((error, bounds)) = active {
        push_linear_row(
            bodies,
            workspace,
            row_settings,
            anchors,
            axis,
            error,
            bounds,
        )?;
    }
    Ok(())
}

fn append_angular_limit(
    bodies: &[SolverBody],
    workspace: &mut JointWorkspace,
    settings: RowSettings,
    rotation_a: Quat,
    angle: f64,
    limit: super::JointLimit,
) -> Result<(), CollisionError> {
    let axis = rotation_a
        .rotate_vector(Vec3::Z)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let mut row_settings = settings;
    let active = if let Some(violation) = limit_error(angle, limit) {
        Some(violation)
    } else {
        let body_a = bodies
            .get(settings.body_a)
            .ok_or(CollisionError::InvalidContact)?;
        let body_b = bodies
            .get(settings.body_b)
            .ok_or(CollisionError::InvalidContact)?;
        let speed = body_b
            .state()
            .angular_velocity()
            .checked_sub(body_a.state().angular_velocity())
            .and_then(|velocity| velocity.checked_dot(axis))
            .ok_or(CollisionError::ArithmeticOverflow)?;
        let predicted = angle + speed * settings.dt;
        if !predicted.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        if speed > 0.0 && predicted > limit.upper {
            row_settings.position_enabled = false;
            row_settings.velocity_target = (limit.upper - angle) / settings.dt;
            Some((0.0, (f64::NEG_INFINITY, 0.0)))
        } else if speed < 0.0 && predicted < limit.lower {
            row_settings.position_enabled = false;
            row_settings.velocity_target = (limit.lower - angle) / settings.dt;
            Some((0.0, (0.0, f64::INFINITY)))
        } else {
            None
        }
    };
    if let Some((error, bounds)) = active {
        push_angular_row(workspace, row_settings, axis, error, bounds)?;
    }
    Ok(())
}

fn limit_error(value: f64, limit: super::JointLimit) -> Option<(f64, (f64, f64))> {
    if value < limit.lower {
        Some((value - limit.lower, (0.0, f64::INFINITY)))
    } else if value > limit.upper {
        Some((value - limit.upper, (f64::NEG_INFINITY, 0.0)))
    } else {
        None
    }
}

fn row_settings(body_a: usize, body_b: usize, common: super::JointCommon, dt: f64) -> RowSettings {
    RowSettings {
        body_a,
        body_b,
        common,
        dt,
        position_enabled: true,
        velocity_target: 0.0,
        multiplier_bounds: None,
        impulse_bounds: None,
    }
}

fn perpendicular_basis(axis: Vec3) -> Result<[Vec3; 2], CollisionError> {
    let (first, second) = crate::collision::solver::tangent_basis(axis)?;
    Ok([first, second])
}

fn drive_settings(
    mut settings: RowSettings,
    drive: super::JointDrive,
) -> Result<Option<RowSettings>, CollisionError> {
    if !drive.enabled || drive.max_force == 0.0 {
        return Ok(None);
    }
    let max_impulse = drive.max_force * settings.dt;
    let max_multiplier = max_impulse * settings.dt;
    if !max_impulse.is_finite() || !max_multiplier.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    let compliance = if drive.stiffness > 0.0 {
        drive.stiffness.recip()
    } else {
        0.0
    };
    if !compliance.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    settings.common.compliance = compliance;
    settings.common.damping = drive.damping;
    settings.position_enabled = drive.stiffness > 0.0;
    settings.velocity_target = drive.target_velocity;
    settings.multiplier_bounds = Some((-max_multiplier, max_multiplier));
    settings.impulse_bounds = Some((-max_impulse, max_impulse));
    Ok(Some(settings))
}

fn world_frame(
    bodies: &[SolverBody],
    body_index: usize,
    frame: super::JointFrame,
) -> Result<(Vec3, Quat), CollisionError> {
    let body = bodies
        .get(body_index)
        .ok_or(CollisionError::InvalidContact)?;
    let local = Transform::new(frame.local_anchor, frame.local_rotation)
        .map_err(|_| CollisionError::InvalidContact)?;
    let transform = body
        .state()
        .transform()
        .compose(&local)
        .map_err(|_| CollisionError::ArithmeticOverflow)?;
    Ok((transform.translation(), transform.rotation()))
}

fn relative_rotation_error(rotation_a: Quat, rotation_b: Quat) -> Result<Vec3, CollisionError> {
    let relative = canonical_relative_rotation(rotation_a, rotation_b)?;
    let vector = Vec3::new(relative.x, relative.y, relative.z);
    let sine = vector
        .checked_length()
        .ok_or(CollisionError::ArithmeticOverflow)?;
    if sine <= f64::MIN_POSITIVE {
        return Ok(Vec3::ZERO);
    }
    let angle = 2.0 * sine.atan2(relative.w);
    vector
        .checked_mul_scalar(angle / sine)
        .ok_or(CollisionError::ArithmeticOverflow)
}

fn canonical_relative_rotation(rotation_a: Quat, rotation_b: Quat) -> Result<Quat, CollisionError> {
    let mut relative = (rotation_a.conjugate() * rotation_b)
        .try_normalize()
        .ok_or(CollisionError::ArithmeticOverflow)?;
    if relative.w < 0.0 {
        relative = Quat::new(-relative.x, -relative.y, -relative.z, -relative.w);
    }
    Ok(relative)
}

fn twist_angle(relative: Quat) -> Result<f64, CollisionError> {
    let length = relative.z.hypot(relative.w);
    if !length.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    if length <= f64::MIN_POSITIVE {
        return Ok(0.0);
    }
    let angle = 2.0 * (relative.z / length).atan2(relative.w / length);
    let wrapped =
        (angle + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI;
    wrapped
        .is_finite()
        .then_some(wrapped)
        .ok_or(CollisionError::ArithmeticOverflow)
}

fn bilateral_bounds() -> (f64, f64) {
    (f64::NEG_INFINITY, f64::INFINITY)
}

fn push_linear_row(
    bodies: &[SolverBody],
    workspace: &mut JointWorkspace,
    settings: RowSettings,
    anchors: [Vec3; 2],
    axis: Vec3,
    error: f64,
    bounds: (f64, f64),
) -> Result<(), CollisionError> {
    if !axis.is_finite() || !error.is_finite() || !settings.dt.is_finite() || settings.dt <= 0.0 {
        return Err(CollisionError::ArithmeticOverflow);
    }
    let center_a = center_of_mass(
        bodies
            .get(settings.body_a)
            .ok_or(CollisionError::InvalidContact)?,
    )?;
    let center_b = center_of_mass(
        bodies
            .get(settings.body_b)
            .ok_or(CollisionError::InvalidContact)?,
    )?;
    let offset_a = anchors[0]
        .checked_sub(center_a)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let offset_b = anchors[1]
        .checked_sub(center_b)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let angular_a = offset_a
        .checked_cross(axis)
        .and_then(|value| value.checked_mul_scalar(-1.0))
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let angular_b = offset_b
        .checked_cross(axis)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    workspace.push(ConstraintRow {
        body_a: settings.body_a,
        body_b: settings.body_b,
        linear_a: -axis,
        angular_a,
        linear_b: axis,
        angular_b,
        error,
        target_velocity: -error * (0.2 / settings.dt) + settings.velocity_target,
        position_enabled: settings.position_enabled,
        compliance: settings.common.compliance,
        damping: settings.common.damping,
        min_multiplier: settings.multiplier_bounds.map_or(bounds.0, |value| value.0),
        max_multiplier: settings.multiplier_bounds.map_or(bounds.1, |value| value.1),
        min_impulse: settings.impulse_bounds.map_or(bounds.0, |value| value.0),
        max_impulse: settings.impulse_bounds.map_or(bounds.1, |value| value.1),
        accumulated_multiplier: 0.0,
        accumulated_impulse: 0.0,
    });
    Ok(())
}

fn push_angular_row(
    workspace: &mut JointWorkspace,
    settings: RowSettings,
    axis: Vec3,
    error: f64,
    bounds: (f64, f64),
) -> Result<(), CollisionError> {
    if !axis.is_finite() || !error.is_finite() || !settings.dt.is_finite() || settings.dt <= 0.0 {
        return Err(CollisionError::ArithmeticOverflow);
    }
    workspace.push(ConstraintRow {
        body_a: settings.body_a,
        body_b: settings.body_b,
        linear_a: Vec3::ZERO,
        angular_a: -axis,
        linear_b: Vec3::ZERO,
        angular_b: axis,
        error,
        target_velocity: -error * (0.2 / settings.dt) + settings.velocity_target,
        position_enabled: settings.position_enabled,
        compliance: settings.common.compliance,
        damping: settings.common.damping,
        min_multiplier: settings.multiplier_bounds.map_or(bounds.0, |value| value.0),
        max_multiplier: settings.multiplier_bounds.map_or(bounds.1, |value| value.1),
        min_impulse: settings.impulse_bounds.map_or(bounds.0, |value| value.0),
        max_impulse: settings.impulse_bounds.map_or(bounds.1, |value| value.1),
        accumulated_multiplier: 0.0,
        accumulated_impulse: 0.0,
    });
    Ok(())
}

fn center_of_mass(body: &SolverBody) -> Result<Vec3, CollisionError> {
    body.body()
        .world_center_of_mass()
        .map_err(|_| CollisionError::ArithmeticOverflow)
        .map(|center| center.unwrap_or(body.state().transform().translation()))
}

fn solve_position_iteration(
    workspace: &mut JointWorkspace,
    bodies: &mut [SolverBody],
    dt: f64,
) -> Result<(), CollisionError> {
    let dt_squared = dt * dt;
    if !dt_squared.is_finite() || dt_squared <= 0.0 {
        return Err(CollisionError::ArithmeticOverflow);
    }
    let alpha_scale = dt_squared.recip();
    for index in 0..workspace.rows().len() {
        let row = *workspace
            .rows()
            .get(index)
            .ok_or(CollisionError::InvalidContact)?;
        if !row.is_valid(bodies.len()) {
            return Err(CollisionError::InvalidContact);
        }
        if !row.position_enabled {
            continue;
        }
        let effective_mass = effective_mass_for_row(bodies, row)?;
        let alpha = row.compliance * alpha_scale;
        if !alpha.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        let denominator = effective_mass + alpha;
        if denominator <= f64::MIN_POSITIVE {
            continue;
        }
        let delta = -(row.error + alpha * row.accumulated_multiplier) / denominator;
        let next =
            (row.accumulated_multiplier + delta).clamp(row.min_multiplier, row.max_multiplier);
        let applied = next - row.accumulated_multiplier;
        if !next.is_finite() || !applied.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        if applied != 0.0 {
            apply_position_row(bodies, row, applied)?;
            workspace.rows_mut()[index].accumulated_multiplier = next;
        }
    }
    Ok(())
}

fn solve_velocity_iteration(
    workspace: &mut JointWorkspace,
    bodies: &mut [SolverBody],
    dt: f64,
) -> Result<(), CollisionError> {
    for index in 0..workspace.rows().len() {
        let row = *workspace
            .rows()
            .get(index)
            .ok_or(CollisionError::InvalidContact)?;
        if !row.is_valid(bodies.len()) {
            return Err(CollisionError::InvalidContact);
        }
        let effective_mass = effective_mass_for_row(bodies, row)?;
        if effective_mass <= f64::MIN_POSITIVE {
            continue;
        }
        let velocity = row_velocity(bodies, row)?;
        let speed_error = row.target_velocity - velocity;
        let alpha = row.compliance / (dt * dt);
        if !alpha.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        let delta = if row.compliance == 0.0 && row.damping == 0.0 {
            speed_error / effective_mass
        } else if row.damping > 0.0 {
            let scaled_damping = row.damping * dt;
            let denominator = 1.0_f64 + scaled_damping * (effective_mass + alpha);
            scaled_damping * speed_error / denominator
        } else if row.target_velocity != 0.0 {
            speed_error / (effective_mass + alpha)
        } else {
            0.0
        };
        let next = (row.accumulated_impulse + delta).clamp(row.min_impulse, row.max_impulse);
        let applied = next - row.accumulated_impulse;
        if !delta.is_finite() || !next.is_finite() || !applied.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        if applied != 0.0 {
            apply_velocity_row(bodies, row, applied)?;
            workspace.rows_mut()[index].accumulated_impulse = next;
        }
    }
    Ok(())
}

fn effective_mass_for_row(
    bodies: &[SolverBody],
    row: ConstraintRow,
) -> Result<f64, CollisionError> {
    let (body_a, body_b) = solver_bodies_at(bodies, row.body_a, row.body_b)?;
    let jacobians = [
        constraint::Jacobian {
            linear: row.linear_a,
            angular: row.angular_a,
        },
        constraint::Jacobian {
            linear: row.linear_b,
            angular: row.angular_b,
        },
    ];
    constraint::effective_cross_mass(body_a.body(), body_b.body(), jacobians, jacobians)
        .map_err(map_constraint_error)
}

fn row_velocity(bodies: &[SolverBody], row: ConstraintRow) -> Result<f64, CollisionError> {
    let (body_a, body_b) = solver_bodies_at(bodies, row.body_a, row.body_b)?;
    let state_a = body_a.state();
    let state_b = body_b.state();
    let value = row.linear_a.dot(state_a.linear_velocity())
        + row.angular_a.dot(state_a.angular_velocity())
        + row.linear_b.dot(state_b.linear_velocity())
        + row.angular_b.dot(state_b.angular_velocity());
    value
        .is_finite()
        .then_some(value)
        .ok_or(CollisionError::ArithmeticOverflow)
}

fn apply_position_row(
    bodies: &mut [SolverBody],
    row: ConstraintRow,
    multiplier: f64,
) -> Result<(), CollisionError> {
    let (body_a, body_b) = solver_bodies_at_mut(bodies, row.body_a, row.body_b)?;
    constraint::apply_generalized_position_impulse_pair(
        body_a.body_mut(),
        body_b.body_mut(),
        row.linear_a * multiplier,
        row.angular_a * multiplier,
        row.linear_b * multiplier,
        row.angular_b * multiplier,
    )
    .map_err(map_constraint_error)
}

fn apply_velocity_row(
    bodies: &mut [SolverBody],
    row: ConstraintRow,
    impulse: f64,
) -> Result<(), CollisionError> {
    let (body_a, body_b) = solver_bodies_at_mut(bodies, row.body_a, row.body_b)?;
    constraint::apply_generalized_impulse_pair(
        body_a.body_mut(),
        body_b.body_mut(),
        row.linear_a * impulse,
        row.angular_a * impulse,
        row.linear_b * impulse,
        row.angular_b * impulse,
    )
    .map_err(map_constraint_error)
}

fn solver_bodies_at(
    bodies: &[SolverBody],
    body_a: usize,
    body_b: usize,
) -> Result<(&SolverBody, &SolverBody), CollisionError> {
    if body_a == body_b {
        return Err(CollisionError::InvalidContact);
    }
    Ok((
        bodies.get(body_a).ok_or(CollisionError::InvalidContact)?,
        bodies.get(body_b).ok_or(CollisionError::InvalidContact)?,
    ))
}

fn solver_bodies_at_mut(
    bodies: &mut [SolverBody],
    body_a: usize,
    body_b: usize,
) -> Result<(&mut SolverBody, &mut SolverBody), CollisionError> {
    if body_a == body_b {
        return Err(CollisionError::InvalidContact);
    }
    if body_a < body_b {
        let (left, right) = bodies.split_at_mut(body_b);
        Ok((
            left.get_mut(body_a).ok_or(CollisionError::InvalidContact)?,
            right.first_mut().ok_or(CollisionError::InvalidContact)?,
        ))
    } else {
        let (left, right) = bodies.split_at_mut(body_a);
        Ok((
            right.first_mut().ok_or(CollisionError::InvalidContact)?,
            left.get_mut(body_b).ok_or(CollisionError::InvalidContact)?,
        ))
    }
}

fn map_constraint_error(error: constraint::ConstraintError) -> CollisionError {
    match error {
        constraint::ConstraintError::InvalidMass => CollisionError::InvalidContact,
        constraint::ConstraintError::ArithmeticOverflow => CollisionError::ArithmeticOverflow,
        constraint::ConstraintError::NonConvergent => CollisionError::NonConvergent,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{mass::MassProperties, rigid_body::RigidBody, transform::Transform};
    use kapitsa_math::{Quat, Vec3};

    fn solver_body(position: Vec3) -> SolverBody {
        solver_body_with_rotation(position, Quat::IDENTITY)
    }

    fn solver_body_with_rotation(position: Vec3, rotation: Quat) -> SolverBody {
        solver_body_with_velocities(position, rotation, Vec3::ZERO, Vec3::ZERO)
    }

    fn solver_body_with_velocities(
        position: Vec3,
        rotation: Quat,
        linear_velocity: Vec3,
        angular_velocity: Vec3,
    ) -> SolverBody {
        let mass =
            MassProperties::new(1.0, Vec3::ZERO, kapitsa_math::Mat3::IDENTITY).expect("valid mass");
        let mut body =
            RigidBody::new_dynamic(Transform::new(position, rotation).expect("transform"), mass)
                .expect("dynamic body");
        body.set_linear_velocity(linear_velocity)
            .expect("finite linear velocity");
        body.set_angular_velocity(angular_velocity)
            .expect("finite angular velocity");
        let state = *body.state();
        SolverBody::from_prediction(body, state)
    }

    fn solve_joint(joint: super::super::Joint, bodies: &mut [SolverBody]) {
        let constraints = [JointConstraint {
            body_a: 0,
            body_b: 1,
            joint,
        }];
        let mut workspace = JointWorkspace::new();
        let prepare = |bodies: &[SolverBody], workspace: &mut JointWorkspace| {
            prepare_joint_rows(bodies, workspace, &constraints, 0.01)
        };
        JointSolver::solve(&mut workspace, bodies, 0.01, 12, 8, prepare).expect("joint row solve");
    }

    fn anchor_row(error: f64) -> ConstraintRow {
        ConstraintRow {
            body_a: 0,
            body_b: 1,
            linear_a: -Vec3::X,
            angular_a: Vec3::ZERO,
            linear_b: Vec3::X,
            angular_b: Vec3::ZERO,
            error,
            target_velocity: 0.0,
            position_enabled: true,
            compliance: 0.0,
            damping: 0.0,
            min_multiplier: f64::NEG_INFINITY,
            max_multiplier: f64::INFINITY,
            min_impulse: f64::NEG_INFINITY,
            max_impulse: f64::INFINITY,
            accumulated_multiplier: 0.0,
            accumulated_impulse: 0.0,
        }
    }

    #[test]
    fn xpbd_row_corrects_position_and_reuses_rows_across_iterations() {
        let mut bodies = [solver_body(Vec3::ZERO), solver_body(Vec3::X * 2.0)];
        let mut workspace = JointWorkspace::new();
        let prepare = |bodies: &[SolverBody], workspace: &mut JointWorkspace| {
            let distance = bodies[1].state().transform().translation().x
                - bodies[0].state().transform().translation().x;
            workspace.push(anchor_row(distance - 1.0));
            Ok(())
        };
        JointSolver::solve(&mut workspace, &mut bodies, 0.01, 4, 1, prepare).expect("solve row");
        let distance = bodies[1].state().transform().translation().x
            - bodies[0].state().transform().translation().x;
        assert!((distance - 1.0).abs() < 1.0e-12, "distance={distance}");
        assert_eq!(workspace.rows().len(), 1);
    }

    #[test]
    fn compliant_row_scales_with_dt_and_preserves_finite_body_state() {
        let mut bodies = [solver_body(Vec3::ZERO), solver_body(Vec3::X * 2.0)];
        let mut workspace = JointWorkspace::new();
        let mut row = anchor_row(0.0);
        row.compliance = 1.0e-3;
        let prepare = |bodies: &[SolverBody], workspace: &mut JointWorkspace| {
            let distance = bodies[1].state().transform().translation().x
                - bodies[0].state().transform().translation().x;
            workspace.push(ConstraintRow {
                error: distance - 1.0,
                ..row
            });
            Ok(())
        };
        JointSolver::solve(&mut workspace, &mut bodies, 0.1, 8, 1, prepare)
            .expect("solve compliant row");
        assert!(bodies.iter().all(|body| {
            body.state().transform().translation().is_finite()
                && body.state().linear_velocity().is_finite()
                && body.state().angular_velocity().is_finite()
        }));
    }

    #[test]
    fn fixed_joint_locks_connector_translation_and_rotation() {
        let rotation = Quat::from_axis_angle(Vec3::Z, 0.6).expect("rotation");
        let mut bodies = [
            solver_body(Vec3::ZERO),
            solver_body_with_rotation(Vec3::new(2.0, -1.0, 0.5), rotation),
        ];
        let common = super::super::JointCommon::new(
            super::super::JointFrame::IDENTITY,
            super::super::JointFrame::IDENTITY,
        );
        solve_joint(
            super::super::Joint::Fixed(super::super::FixedJoint::new(common)),
            &mut bodies,
        );
        let position_delta = bodies[1]
            .state()
            .transform()
            .translation()
            .checked_sub(bodies[0].state().transform().translation())
            .expect("finite delta");
        let angle_delta = relative_rotation_error(
            bodies[0].state().transform().rotation(),
            bodies[1].state().transform().rotation(),
        )
        .expect("relative angle");
        assert!(position_delta.length() < 1.0e-10);
        assert!(angle_delta.length() < 1.0e-10);
    }

    #[test]
    fn ball_joint_keeps_anchors_together_and_allows_free_rotation() {
        let rotation = Quat::from_axis_angle(Vec3::Y, 0.7).expect("rotation");
        let mut bodies = [
            solver_body(Vec3::ZERO),
            solver_body_with_rotation(Vec3::X * 3.0, rotation),
        ];
        let common = super::super::JointCommon::new(
            super::super::JointFrame::IDENTITY,
            super::super::JointFrame::IDENTITY,
        );
        solve_joint(
            super::super::Joint::Ball(super::super::BallJoint::new(common)),
            &mut bodies,
        );
        let position_delta = bodies[1]
            .state()
            .transform()
            .translation()
            .checked_sub(bodies[0].state().transform().translation())
            .expect("finite delta");
        assert!(position_delta.length() < 1.0e-10);
        assert!(
            relative_rotation_error(
                bodies[0].state().transform().rotation(),
                bodies[1].state().transform().rotation()
            )
            .expect("relative angle")
            .length()
                > 0.6
        );
    }

    #[test]
    fn distance_joint_enforces_range_and_leaves_in_range_distance_unchanged() {
        let common = super::super::JointCommon::new(
            super::super::JointFrame::IDENTITY,
            super::super::JointFrame::IDENTITY,
        );
        let joint = super::super::Joint::Distance(super::super::DistanceJoint::new(
            common,
            super::super::JointLimit::new(1.0, 2.0),
        ));
        let mut outside = [solver_body(Vec3::ZERO), solver_body(Vec3::X * 3.0)];
        solve_joint(joint, &mut outside);
        let distance = (outside[1].state().transform().translation()
            - outside[0].state().transform().translation())
        .length();
        assert!((distance - 2.0).abs() < 1.0e-10, "distance={distance}");

        let mut inside = [solver_body(Vec3::ZERO), solver_body(Vec3::X * 1.5)];
        solve_joint(joint, &mut inside);
        let unchanged = (inside[1].state().transform().translation()
            - inside[0].state().transform().translation())
        .length();
        assert!((unchanged - 1.5).abs() < 1.0e-12);
    }

    #[test]
    fn ball_joint_twist_limit_clamps_relative_rotation() {
        let rotation = Quat::from_axis_angle(Vec3::Z, 1.0).expect("rotation");
        let mut bodies = [
            solver_body(Vec3::ZERO),
            solver_body_with_rotation(Vec3::ZERO, rotation),
        ];
        let common = super::super::JointCommon::new(
            super::super::JointFrame::IDENTITY,
            super::super::JointFrame::IDENTITY,
        );
        let mut joint = super::super::BallJoint::new(common);
        joint.twist_limit = Some(super::super::JointLimit::new(-0.25, 0.25));
        solve_joint(super::super::Joint::Ball(joint), &mut bodies);
        let error = relative_rotation_error(
            bodies[0].state().transform().rotation(),
            bodies[1].state().transform().rotation(),
        )
        .expect("relative angle");
        assert!((error.z - 0.25).abs() < 1.0e-10, "twist={}", error.z);
    }

    #[test]
    fn ball_joint_swing_limit_uses_the_connector_axes() {
        let swing = Quat::from_axis_angle(Vec3::Y, 0.8).expect("swing rotation");
        let twist = Quat::from_axis_angle(Vec3::Z, 1.0).expect("twist rotation");
        let mut bodies = [
            solver_body(Vec3::ZERO),
            solver_body_with_rotation(Vec3::ZERO, swing * twist),
        ];
        let common = super::super::JointCommon::new(
            super::super::JointFrame::IDENTITY,
            super::super::JointFrame::IDENTITY,
        );
        let mut joint = super::super::BallJoint::new(common);
        joint.swing_limit = Some(0.2);
        solve_joint(super::super::Joint::Ball(joint), &mut bodies);
        let axis_a = bodies[0]
            .state()
            .transform()
            .rotation()
            .rotate_vector(Vec3::Z)
            .expect("axis A");
        let axis_b = bodies[1]
            .state()
            .transform()
            .rotation()
            .rotate_vector(Vec3::Z)
            .expect("axis B");
        let swing_angle = axis_a.dot(axis_b).clamp(-1.0, 1.0).acos();
        assert!((swing_angle - 0.2).abs() < 1.0e-10, "swing={swing_angle}");
    }

    #[test]
    fn hinge_joint_aligns_axes_and_enforces_twist_limit() {
        let tilt = Quat::from_axis_angle(Vec3::X, 0.4).expect("tilt");
        let twist = Quat::from_axis_angle(Vec3::Z, 0.7).expect("twist");
        let mut bodies = [
            solver_body(Vec3::ZERO),
            solver_body_with_rotation(Vec3::new(2.0, 1.0, -1.0), tilt * twist),
        ];
        let common = super::super::JointCommon::new(
            super::super::JointFrame::IDENTITY,
            super::super::JointFrame::IDENTITY,
        );
        let mut joint = super::super::HingeJoint::new(common);
        joint.angle_limit = Some(super::super::JointLimit::new(-0.2, 0.2));
        solve_joint(super::super::Joint::Hinge(joint), &mut bodies);

        let position_delta = bodies[1]
            .state()
            .transform()
            .translation()
            .checked_sub(bodies[0].state().transform().translation())
            .expect("finite delta");
        let rotation_error = relative_rotation_error(
            bodies[0].state().transform().rotation(),
            bodies[1].state().transform().rotation(),
        )
        .expect("relative rotation");
        assert!(position_delta.length() < 1.0e-10);
        assert!(rotation_error.x.abs() < 1.0e-10);
        assert!(rotation_error.y.abs() < 1.0e-10);
        assert!((rotation_error.z - 0.2).abs() < 1.0e-10);
    }

    #[test]
    fn slider_joint_locks_sideways_motion_and_rotation() {
        let rotation = Quat::from_axis_angle(Vec3::X, 0.4).expect("rotation");
        let mut bodies = [
            solver_body(Vec3::ZERO),
            solver_body_with_rotation(Vec3::new(2.0, 3.0, 4.0), rotation),
        ];
        let common = super::super::JointCommon::new(
            super::super::JointFrame::IDENTITY,
            super::super::JointFrame::IDENTITY,
        );
        let mut joint = super::super::SliderJoint::new(common);
        joint.translation_limit = Some(super::super::JointLimit::new(-1.0, 2.0));
        solve_joint(super::super::Joint::Slider(joint), &mut bodies);

        let delta = bodies[1].state().transform().translation()
            - bodies[0].state().transform().translation();
        let rotation_error = relative_rotation_error(
            bodies[0].state().transform().rotation(),
            bodies[1].state().transform().rotation(),
        )
        .expect("relative rotation");
        let axis = bodies[0]
            .state()
            .transform()
            .rotation()
            .rotate_vector(Vec3::Z)
            .expect("slider axis");
        let axial = delta.dot(axis);
        let sideways = delta - axis * axial;
        assert!(sideways.length() < 1.0e-10, "slider sideways={sideways:?}");
        assert!((-1.0 - 1.0e-10..=2.0 + 1.0e-10).contains(&axial));
        assert!(rotation_error.length() < 1.0e-10);
    }

    #[test]
    fn cylindrical_joint_limits_slide_and_twist_independently() {
        let tilt = Quat::from_axis_angle(Vec3::X, 0.5).expect("tilt");
        let twist = Quat::from_axis_angle(Vec3::Z, 0.7).expect("twist");
        let mut bodies = [
            solver_body(Vec3::ZERO),
            solver_body_with_rotation(Vec3::new(2.0, 3.0, 4.0), tilt * twist),
        ];
        let common = super::super::JointCommon::new(
            super::super::JointFrame::IDENTITY,
            super::super::JointFrame::IDENTITY,
        );
        let mut joint = super::super::CylindricalJoint::new(common);
        joint.translation_limit = Some(super::super::JointLimit::new(-1.0, 2.0));
        joint.angle_limit = Some(super::super::JointLimit::new(-0.25, 0.25));
        solve_joint(super::super::Joint::Cylindrical(joint), &mut bodies);

        let delta = bodies[1].state().transform().translation()
            - bodies[0].state().transform().translation();
        let rotation_error = relative_rotation_error(
            bodies[0].state().transform().rotation(),
            bodies[1].state().transform().rotation(),
        )
        .expect("relative rotation");
        let axis = bodies[0]
            .state()
            .transform()
            .rotation()
            .rotate_vector(Vec3::Z)
            .expect("cylindrical axis");
        let axial = delta.dot(axis);
        let sideways = delta - axis * axial;
        assert!(sideways.length() < 1.0e-10, "sideways={sideways:?}");
        assert!((-1.0 - 1.0e-10..=2.0 + 1.0e-10).contains(&axial));
        assert!(rotation_error.x.abs() < 1.0e-10);
        assert!(rotation_error.y.abs() < 1.0e-10);
        assert!((rotation_error.z - 0.25).abs() < 1.0e-10);
    }

    #[test]
    fn joint_drives_respect_force_limit_and_target_velocity() {
        let drive = super::super::JointDrive {
            enabled: true,
            target_position: 0.0,
            target_velocity: 10.0,
            stiffness: 0.0,
            damping: 0.0,
            max_force: 100.0,
        };
        let common = super::super::JointCommon::new(
            super::super::JointFrame::IDENTITY,
            super::super::JointFrame::IDENTITY,
        );

        let mut hinge = super::super::HingeJoint::new(common);
        hinge.drive = drive;
        let mut hinge_bodies = [solver_body(Vec3::ZERO), solver_body(Vec3::ZERO)];
        solve_joint(super::super::Joint::Hinge(hinge), &mut hinge_bodies);
        let relative_angular_speed = hinge_bodies[1].state().angular_velocity().z
            - hinge_bodies[0].state().angular_velocity().z;
        assert!((relative_angular_speed - 2.0).abs() < 1.0e-10);

        let mut slider = super::super::SliderJoint::new(common);
        slider.drive = drive;
        let mut slider_bodies = [solver_body(Vec3::ZERO), solver_body(Vec3::ZERO)];
        solve_joint(super::super::Joint::Slider(slider), &mut slider_bodies);
        let relative_linear_speed = slider_bodies[1].state().linear_velocity().z
            - slider_bodies[0].state().linear_velocity().z;
        assert!((relative_linear_speed - 2.0).abs() < 1.0e-10);

        let mut cylindrical = super::super::CylindricalJoint::new(common);
        cylindrical.translation_drive = drive;
        cylindrical.angular_drive = drive;
        let mut cylindrical_bodies = [solver_body(Vec3::ZERO), solver_body(Vec3::ZERO)];
        solve_joint(
            super::super::Joint::Cylindrical(cylindrical),
            &mut cylindrical_bodies,
        );
        let linear_speed = cylindrical_bodies[1].state().linear_velocity().z
            - cylindrical_bodies[0].state().linear_velocity().z;
        let angular_speed = cylindrical_bodies[1].state().angular_velocity().z
            - cylindrical_bodies[0].state().angular_velocity().z;
        assert!((linear_speed - 2.0).abs() < 1.0e-10);
        assert!((angular_speed - 2.0).abs() < 1.0e-10);
    }

    #[test]
    fn position_drives_move_hinge_and_slider_toward_targets() {
        let drive = super::super::JointDrive {
            enabled: true,
            target_position: 0.4,
            target_velocity: 0.0,
            stiffness: 100_000.0,
            damping: 0.0,
            max_force: 1.0e7,
        };
        let common = super::super::JointCommon::new(
            super::super::JointFrame::IDENTITY,
            super::super::JointFrame::IDENTITY,
        );

        let mut hinge = super::super::HingeJoint::new(common);
        hinge.drive = drive;
        let mut hinge_bodies = [solver_body(Vec3::ZERO), solver_body(Vec3::ZERO)];
        solve_joint(super::super::Joint::Hinge(hinge), &mut hinge_bodies);
        let hinge_angle = relative_rotation_error(
            hinge_bodies[0].state().transform().rotation(),
            hinge_bodies[1].state().transform().rotation(),
        )
        .expect("hinge angle")
        .z;
        assert!(hinge_angle > 0.3, "hinge angle={hinge_angle}");

        let mut slider = super::super::SliderJoint::new(common);
        slider.drive = drive;
        let mut slider_bodies = [solver_body(Vec3::ZERO), solver_body(Vec3::ZERO)];
        solve_joint(super::super::Joint::Slider(slider), &mut slider_bodies);
        let slider_position = slider_bodies[1].state().transform().translation().z
            - slider_bodies[0].state().transform().translation().z;
        assert!(slider_position > 0.3, "slider position={slider_position}");
    }

    #[test]
    fn slider_limit_brakes_outward_velocity_before_crossing() {
        let common = super::super::JointCommon::new(
            super::super::JointFrame::IDENTITY,
            super::super::JointFrame::IDENTITY,
        );
        let mut joint = super::super::SliderJoint::new(common);
        joint.translation_limit = Some(super::super::JointLimit::new(-1.0, 1.0));
        let mut bodies = [
            solver_body(Vec3::ZERO),
            solver_body_with_velocities(
                Vec3::ZERO,
                Quat::IDENTITY,
                Vec3::new(0.0, 0.0, 200.0),
                Vec3::ZERO,
            ),
        ];

        solve_joint(super::super::Joint::Slider(joint), &mut bodies);

        let relative_speed =
            bodies[1].state().linear_velocity().z - bodies[0].state().linear_velocity().z;
        assert!(
            relative_speed <= 100.0 + 1.0e-10,
            "relative speed={relative_speed}"
        );
    }

    #[test]
    fn hinge_limit_brakes_outward_angular_velocity_before_crossing() {
        let common = super::super::JointCommon::new(
            super::super::JointFrame::IDENTITY,
            super::super::JointFrame::IDENTITY,
        );
        let mut joint = super::super::HingeJoint::new(common);
        joint.angle_limit = Some(super::super::JointLimit::new(-1.0, 1.0));
        let mut bodies = [
            solver_body(Vec3::ZERO),
            solver_body_with_velocities(
                Vec3::ZERO,
                Quat::IDENTITY,
                Vec3::ZERO,
                Vec3::new(0.0, 0.0, 200.0),
            ),
        ];

        solve_joint(super::super::Joint::Hinge(joint), &mut bodies);

        let relative_speed =
            bodies[1].state().angular_velocity().z - bodies[0].state().angular_velocity().z;
        assert!(
            relative_speed <= 100.0 + 1.0e-10,
            "relative speed={relative_speed}"
        );
    }
}
