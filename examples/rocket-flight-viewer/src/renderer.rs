use glium::glutin::surface::WindowSurface;
use glium::{
    Display, Program, Surface, VertexBuffer, implement_vertex,
    index::{NoIndices, PrimitiveType},
};
use kapitsa::math::Vec3;

use crate::{
    camera::{Camera, SceneFrame},
    flight::{FLAME_END, STAGE_ONE_BURN_END, STAGE_TWO_BURN_END},
};

#[derive(Clone, Copy)]
struct Vertex {
    position: [f32; 2],
    color: [f32; 3],
}

implement_vertex!(Vertex, position, color);

pub(crate) struct Renderer {
    program: Program,
}

impl Renderer {
    pub(crate) fn new(
        display: &Display<WindowSurface>,
    ) -> Result<Self, glium::ProgramCreationError> {
        let program = Program::from_source(
            display,
            "#version 140\nin vec2 position; in vec3 color; out vec3 v_color; void main() { v_color = color; gl_Position = vec4(position, 0.0, 1.0); }",
            "#version 140\nin vec3 v_color; out vec4 f_color; void main() { f_color = vec4(v_color, 1.0); }",
            None,
        )?;
        Ok(Self { program })
    }

    pub(crate) fn draw<S: Surface>(
        &self,
        display: &Display<WindowSurface>,
        target: &mut S,
        camera: &Camera,
        frame: &SceneFrame,
    ) -> Result<(), Box<dyn std::error::Error>> {
        target.clear_color(0.025, 0.035, 0.07, 1.0);
        let mut vertices = Vec::with_capacity(36);
        rect(
            &mut vertices,
            [-1.0, -1.0],
            [1.0, -0.91],
            [0.19, 0.21, 0.24],
        );
        rect(
            &mut vertices,
            [-0.12, -0.91],
            [0.12, -0.88],
            [0.42, 0.45, 0.48],
        );

        let stage_one_powered = frame.simulation_time < STAGE_ONE_BURN_END;
        let stage_two_powered =
            (STAGE_ONE_BURN_END..STAGE_TWO_BURN_END).contains(&frame.simulation_time);
        if stage_one_powered && let Some(state) = frame.stage_one {
            flame(&mut vertices, camera, state.transform(), FLAME_END);
        }
        if stage_two_powered && let Some(state) = frame.stage_two {
            flame(&mut vertices, camera, state.transform(), FLAME_END);
        }
        if let Some(state) = frame.stage_one {
            rocket(&mut vertices, camera, state.transform(), [0.92, 0.36, 0.12]);
        }
        if let Some(state) = frame.stage_two {
            rocket(&mut vertices, camera, state.transform(), [0.72, 0.83, 0.96]);
        }

        if vertices.is_empty() {
            return Ok(());
        }
        let vertex_buffer = VertexBuffer::new(display, &vertices)?;
        target.draw(
            &vertex_buffer,
            NoIndices(PrimitiveType::TrianglesList),
            &self.program,
            &glium::uniforms::EmptyUniforms,
            &Default::default(),
        )?;
        Ok(())
    }
}

fn rocket(
    vertices: &mut Vec<Vertex>,
    camera: &Camera,
    transform: kapitsa::Transform,
    color: [f32; 3],
) {
    let corners = [
        Vec3::new(-1.2, -2.0, 0.0),
        Vec3::new(1.2, -2.0, 0.0),
        Vec3::new(1.2, 2.0, 0.0),
        Vec3::new(-1.2, 2.0, 0.0),
    ]
    .map(|point| transform.transform_point(point));
    quad(vertices, camera, corners, color);
    let tip = transform.transform_point(Vec3::new(0.0, 2.8, 0.0));
    let left = transform.transform_point(Vec3::new(-1.2, 2.0, 0.0));
    let right = transform.transform_point(Vec3::new(1.2, 2.0, 0.0));
    triangle(
        vertices,
        [
            camera.project(left),
            camera.project(right),
            camera.project(tip),
        ],
        color,
    );
}

fn flame(vertices: &mut Vec<Vertex>, camera: &Camera, transform: kapitsa::Transform, length: f64) {
    let left = transform.transform_point(Vec3::new(-0.75, -2.0, 0.0));
    let right = transform.transform_point(Vec3::new(0.75, -2.0, 0.0));
    let tip = transform.transform_point(Vec3::new(0.0, -2.0 - length, 0.0));
    triangle(
        vertices,
        [
            camera.project(left),
            camera.project(right),
            camera.project(tip),
        ],
        [1.0, 0.38, 0.05],
    );
    let inner_left = transform.transform_point(Vec3::new(-0.35, -2.0, 0.0));
    let inner_right = transform.transform_point(Vec3::new(0.35, -2.0, 0.0));
    let inner_tip = transform.transform_point(Vec3::new(0.0, -2.0 - length * 0.65, 0.0));
    triangle(
        vertices,
        [
            camera.project(inner_left),
            camera.project(inner_right),
            camera.project(inner_tip),
        ],
        [1.0, 0.77, 0.18],
    );
}

fn quad(vertices: &mut Vec<Vertex>, camera: &Camera, points: [Vec3; 4], color: [f32; 3]) {
    let points = points.map(|point| camera.project(point));
    triangle(vertices, [points[0], points[1], points[2]], color);
    triangle(vertices, [points[0], points[2], points[3]], color);
}

fn triangle(vertices: &mut Vec<Vertex>, points: [[f32; 2]; 3], color: [f32; 3]) {
    vertices.extend(points.map(|position| Vertex { position, color }));
}

fn rect(vertices: &mut Vec<Vertex>, minimum: [f32; 2], maximum: [f32; 2], color: [f32; 3]) {
    triangle(
        vertices,
        [minimum, [maximum[0], minimum[1]], maximum],
        color,
    );
    triangle(
        vertices,
        [minimum, maximum, [minimum[0], maximum[1]]],
        color,
    );
}
