use std::{borrow::Cow, error::Error, fs::File, io::BufWriter, path::Path};

use glium::{
    Display,
    framebuffer::SimpleFrameBuffer,
    glutin::surface::WindowSurface,
    texture::{Texture2d, Texture2dDataSink},
};

use crate::{
    camera::{Camera, SceneFrame},
    renderer::Renderer,
};

struct RgbaImage {
    pixels: Vec<(u8, u8, u8, u8)>,
    width: u32,
}

impl Texture2dDataSink<(u8, u8, u8, u8)> for RgbaImage {
    fn from_raw(data: Cow<'_, [(u8, u8, u8, u8)]>, width: u32, _height: u32) -> Self {
        Self {
            pixels: data.into_owned(),
            width,
        }
    }
}

pub(crate) struct GifOptions<'a> {
    pub(crate) width: u32,
    pub(crate) height: u32,
    pub(crate) fps: u32,
    pub(crate) output: &'a Path,
}

pub(crate) fn export(
    display: &Display<WindowSurface>,
    renderer: &Renderer,
    camera: &Camera,
    frames: &[SceneFrame],
    options: GifOptions<'_>,
) -> Result<(), Box<dyn Error>> {
    let width = u16::try_from(options.width).map_err(|_| "GIF width exceeds 65535 pixels")?;
    let height = u16::try_from(options.height).map_err(|_| "GIF height exceeds 65535 pixels")?;
    let texture = Texture2d::empty(display, u32::from(width), u32::from(height))?;
    let mut encoder = gif::Encoder::new(
        BufWriter::new(File::create(options.output)?),
        width,
        height,
        &[],
    )?;
    encoder.set_repeat(gif::Repeat::Infinite)?;

    let mut elapsed_centiseconds = 0_u64;
    for (index, scene_frame) in frames.iter().enumerate() {
        let mut framebuffer = SimpleFrameBuffer::new(display, &texture)?;
        renderer.draw(display, &mut framebuffer, camera, scene_frame)?;
        drop(framebuffer);

        let image: RgbaImage = texture.read();
        let capacity = image
            .pixels
            .len()
            .checked_mul(3)
            .ok_or("GIF frame size overflows address space")?;
        let mut rgb = Vec::new();
        rgb.try_reserve_exact(capacity)?;
        for row in image.pixels.chunks_exact(image.width as usize).rev() {
            for &(red, green, blue, _) in row {
                rgb.extend_from_slice(&[red, green, blue]);
            }
        }
        let mut frame = gif::Frame::from_rgb_speed(width, height, &rgb, 10);
        let next_centisecond =
            ((index as u64 + 1) * 100 + u64::from(options.fps) / 2) / u64::from(options.fps);
        frame.delay = next_centisecond.saturating_sub(elapsed_centiseconds).max(1) as u16;
        elapsed_centiseconds = next_centisecond;
        encoder.write_frame(&frame)?;
    }
    Ok(())
}
