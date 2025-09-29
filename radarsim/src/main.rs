use crate::radar::Radar;
use crate::render::Render;
use crate::utils::{Scalar, Vec2f, Vec3f};
use rand::prelude::ThreadRng;
use rand::Rng;
use tiny_skia::Color;

mod radar;
mod render;
mod utils;

const SIGMA: Scalar = 1.7; // measured directly from Stormworks using the 'jitter' finder

struct State {
    position: Vec3f,
    velocity: Vec3f,
}

impl State {
    pub fn new(position: Vec3f, velocity: Vec3f) -> State {
        State { position, velocity }
    }
}

struct Trace {
    history: Vec<Vec3f>,
    state: State,
    time_offs: Scalar,
}

impl Trace {
    pub fn new(init_state: State, time_offs: Scalar) -> Trace {
        Trace {
            history: vec![],
            state: init_state,
            time_offs,
        }
    }

    pub fn step(&mut self, dt: Scalar) {
        self.history.push(self.state.position);
        self.state.position += self.state.velocity * dt;
    }
}

struct RadarSim {
    radar: Radar,
    trajectories: Vec<Trace>,
    detections: Vec<Vec3f>,
    rng: ThreadRng,
}

impl RadarSim {
    const DOT_RADIUS: Scalar = 2.0;

    pub fn new(radar: Radar) -> RadarSim {
        Self {
            radar,
            trajectories: vec![],
            detections: vec![],
            rng: rand::rng(),
        }
    }

    pub fn add_trajectory(&mut self, state: State) {
        self.trajectories.push(Trace::new(state, 0.0));
    }

    pub fn step(&mut self, dt: Scalar) {
        self.radar.step(dt);
        for t in &mut self.trajectories {
            t.step(dt);
            if let Some(detect) = self.radar.try_detect(t.state.position, &mut self.rng) {
                self.detections.push(detect);
            }
        }
    }

    pub fn run(&mut self, time: Scalar, dt: Scalar) {
        let mut t = 0.0;
        while t < time {
            self.step(dt);
            t += dt;
        }
    }

    pub fn draw(&self, render: &mut Render) {
        for t in &self.trajectories {
            render.draw_poly_line(&t.history, 1.0, Color::from_rgba8(0, 0, 255, 255));
        }
        for d in &self.detections {
            render.draw_dot(*d, Self::DOT_RADIUS, Color::from_rgba8(255, 0, 0, 255));
        }
    }
}

fn main() {
    let radar = Radar::new(
        Radar::fov_to_rad(0.05),
        Radar::fov_to_rad(0.05),
        32_000.0,
        5.5,
    );
    const DT: Scalar = 1.0 / 60.0;

    let mut sim = RadarSim::new(radar);
    let state = State::new(
        Vec3f::new(0.0, 0.0, 10_000.0),
        Vec3f::new(-10.0, 0.0, -300.0),
    );
    sim.add_trajectory(state);
    sim.run(120.0, DT);

    let mut render = Render::new(
        1000,
        1000,
        Vec2f::new(-12000.0, 12000.0),
        Vec2f::new(300.0, -300.0),
    );
    sim.draw(&mut render);
    render.save("radarsim.png")

    //radar::sample_and_test(32_000.0, 0.0, 0.0, 10_000);
}
