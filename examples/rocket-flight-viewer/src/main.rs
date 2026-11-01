mod camera;
mod flight;
mod renderer;

use std::{
    error::Error,
    time::{Duration, Instant},
};

use camera::Camera;
use flight::{FlightConfig, run};
use glium::{backend::glutin::SimpleWindowBuilder, glutin::surface::WindowSurface, winit};
use renderer::Renderer;
use tokio::runtime::Builder;
use winit::{
    application::ApplicationHandler,
    event_loop::ActiveEventLoop,
    window::{Window, WindowId},
};

const WIDTH: u32 = 1280;
const HEIGHT: u32 = 720;
const DEFAULT_FPS: u32 = 24;
const DEFAULT_WARP: f64 = 1.0;
const DEFAULT_STEPS: u64 = 4_200;

struct Args {
    width: u32,
    height: u32,
    fps: u32,
    warp: f64,
    steps: u64,
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = arguments()?;
    let runtime = Builder::new_multi_thread()
        .worker_threads(4)
        .enable_all()
        .build()?;
    let frames = runtime.block_on(run(FlightConfig {
        warp: args.warp,
        maximum_steps: args.steps,
        fps: args.fps,
    }))?;
    drop(runtime);
    let first = frames.first().ok_or("flight produced no frames")?;
    let last = frames.last().ok_or("flight produced no frames")?;
    println!(
        "captured {} snapshots: step {} to {}, {:.3} simulated seconds at requested {:.1}x",
        frames.len(),
        first.step,
        last.step,
        last.simulation_time - first.simulation_time,
        args.warp,
    );

    let event_loop = winit::event_loop::EventLoop::builder().build()?;
    let (window, display) = SimpleWindowBuilder::new()
        .with_title("Kapitsa two-stage rocket flight")
        .with_inner_size(args.width, args.height)
        .build(&event_loop);
    let renderer = Renderer::new(&display)?;
    let camera = Camera::for_frames(&frames, args.width, args.height);
    let mut viewer = Viewer {
        window,
        display,
        renderer,
        camera,
        frames,
        frame_index: 0,
        frame_period: Duration::from_secs_f64(1.0 / f64::from(args.fps)),
        next_redraw: Instant::now(),
    };
    event_loop.run_app(&mut viewer)?;
    Ok(())
}

struct Viewer {
    window: Window,
    display: glium::Display<WindowSurface>,
    renderer: Renderer,
    camera: Camera,
    frames: Vec<camera::SceneFrame>,
    frame_index: usize,
    frame_period: Duration,
    next_redraw: Instant,
}

impl ApplicationHandler for Viewer {
    fn resumed(&mut self, _event_loop: &ActiveEventLoop) {
        self.window.request_redraw();
    }

    fn window_event(
        &mut self,
        event_loop: &ActiveEventLoop,
        window_id: WindowId,
        event: winit::event::WindowEvent,
    ) {
        if window_id != self.window.id() {
            return;
        }
        match event {
            winit::event::WindowEvent::CloseRequested
            | winit::event::WindowEvent::KeyboardInput {
                event:
                    winit::event::KeyEvent {
                        logical_key: winit::keyboard::Key::Named(winit::keyboard::NamedKey::Escape),
                        state: winit::event::ElementState::Pressed,
                        ..
                    },
                ..
            } => event_loop.exit(),
            winit::event::WindowEvent::Resized(size) => self.display.resize(size.into()),
            winit::event::WindowEvent::RedrawRequested => {
                let frame = &self.frames[self.frame_index];
                let mut target = self.display.draw();
                if let Err(error) =
                    self.renderer
                        .draw(&self.display, &mut target, &self.camera, frame)
                {
                    eprintln!("OpenGL frame failed: {error}");
                    event_loop.exit();
                    return;
                }
                if let Err(error) = target.finish() {
                    eprintln!("OpenGL frame presentation failed: {error}");
                    event_loop.exit();
                    return;
                }
                self.frame_index = (self.frame_index + 1) % self.frames.len();
                self.next_redraw = Instant::now() + self.frame_period;
            }
            _ => {}
        }
    }

    fn about_to_wait(&mut self, event_loop: &ActiveEventLoop) {
        event_loop.set_control_flow(winit::event_loop::ControlFlow::WaitUntil(self.next_redraw));
        if Instant::now() >= self.next_redraw {
            self.window.request_redraw();
        }
    }
}

fn arguments() -> Result<Args, Box<dyn Error>> {
    let mut args = Args {
        width: WIDTH,
        height: HEIGHT,
        fps: DEFAULT_FPS,
        warp: DEFAULT_WARP,
        steps: DEFAULT_STEPS,
    };
    let mut options = std::env::args().skip(1);
    while let Some(option) = options.next() {
        let value = options
            .next()
            .ok_or_else(|| format!("missing value for {option}"))?;
        match option.as_str() {
            "--width" => args.width = value.parse()?,
            "--height" => args.height = value.parse()?,
            "--fps" => args.fps = value.parse()?,
            "--warp" => args.warp = value.parse()?,
            "--steps" => args.steps = value.parse()?,
            _ => return Err(format!("unknown option: {option}").into()),
        }
    }
    if args.width == 0
        || args.height == 0
        || args.fps == 0
        || args.fps > 120
        || args.steps == 0
        || !args.warp.is_finite()
        || args.warp <= 0.0
    {
        return Err("width/height/steps and warp must be positive; fps must be in 1..=120".into());
    }
    Ok(args)
}
