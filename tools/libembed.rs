// Подключение движок-независимых модулей игры без macroquad.
#[path = "../src/math.rs"]
mod math;
#[path = "../src/light.rs"]
mod light;
#[path = "../src/focus_model.rs"]
mod focus_model;
#[path = "../src/carmodels.rs"]
mod carmodels;

pub use carmodels::{all_car_specs, build_body, translate, wheel_polys, wheel_positions};
pub use focus_model::{Pal, FMat};
