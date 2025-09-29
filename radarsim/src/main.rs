use crate::kalman::Kalman;
use crate::radar::Radar;
use crate::render::Render;
use crate::utils::{Scalar, Vec2f, Vec3f};
use nalgebra::Matrix3;
use rand::prelude::ThreadRng;
use rand::Rng;
use tiny_skia::Color;

mod kalman;
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

struct RadarSim<R> {
    radar: Radar,
    trajectories: Vec<Trace>,
    detections: Vec<Vec3f>,
    kalman: Kalman<R>,
    predictions: Vec<Vec3f>,
    rng: ThreadRng,
    time: Scalar,
}

impl<R> RadarSim<R>
where
    R: Fn(Scalar) -> Matrix3<Scalar>,
{
    const DOT_RADIUS: Scalar = 2.0;

    pub fn new(radar: Radar, kalman: Kalman<R>) -> RadarSim<R> {
        Self {
            radar,
            trajectories: vec![],
            detections: vec![],
            kalman,
            predictions: vec![],
            rng: rand::rng(),
            time: 0.0,
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
                self.kalman.update(self.time, detect);
            }
        }
        if let Some((pos, vel, last_t)) = self.kalman.state() {
            println!(
                "Predict: t={:?} dt={:?} pos={:?} vel={:?}",
                self.time,
                self.time - last_t,
                pos,
                vel
            );
            let pos2 = pos + vel * (self.time - last_t);
            self.predictions.push(pos2);
        }
        self.time += dt;
    }

    pub fn run(&mut self, time: Scalar, dt: Scalar) {
        let mut t = 0.0;
        while t < time {
            self.step(dt);
            t += dt;
        }
    }

    pub fn draw(&self, render: &mut Render) {
        /*for t in &self.trajectories {
            render.draw_poly_line(&t.history, 1.0, Color::from_rgba8(0, 0, 255, 255));
        }*/
        for d in &self.detections {
            render.draw_dot(*d, Self::DOT_RADIUS, Color::from_rgba8(255, 0, 0, 255));
        }
        render.draw_poly_line(&self.predictions, 1.0, Color::from_rgba8(255, 0, 255, 255));
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

    let kalman = Kalman::new(0.1, |d| radar::covariance_cart_direct(d));
    let mut sim = RadarSim::new(radar, kalman);
    let state = State::new(Vec3f::new(0.0, 0.0, 2000.0), Vec3f::new(-1.0, 0.0, -30.0));
    sim.add_trajectory(state);
    sim.run(120.0, DT);

    let mut render = Render::new(
        1000,
        1000,
        Vec2f::new(-1200.0, 2200.0),
        Vec2f::new(300.0, -300.0),
    );
    sim.draw(&mut render);
    render.save("radarsim.png")

    //radar::sample_and_test(32_000.0, 0.0, 0.0, 10_000);
}
