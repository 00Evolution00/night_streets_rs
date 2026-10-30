//! NIGHT STREETS - подпольные уличные гонки (Rust + macroquad).

// Макрос должен быть объявлен до `mod`, чтобы им могли пользоваться все модули.
// rgb!(30, 120, 255) -> [f32; 3] с компонентами 0..1
macro_rules! rgb {
    ($r:expr, $g:expr, $b:expr) => {
        [$r as f32 / 255.0, $g as f32 / 255.0, $b as f32 / 255.0]
    };
}

mod car;
mod carmodels;
mod focus_model;
mod game;
mod light;
mod math;
mod mesh;
mod save;
mod track;
mod world;

use macroquad::prelude::*;

fn window_conf() -> Conf {
    Conf {
        window_title: "NIGHT STREETS".to_owned(),
        window_width: 1280,
        window_height: 720,
        sample_count: 4,
        ..Default::default()
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    let mut game = game::Game::new();
    loop {
        let dt = get_frame_time().min(1.0 / 30.0);
        if !game.update(dt) {
            break;
        }
        game.draw();
        next_frame().await;
    }
}
