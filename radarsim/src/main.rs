use crate::render::Render;
use crate::utils::{Polar, Scalar, Vec2f, Vec3f};
use rand::prelude::ThreadRng;
use rand::Rng;
use rand_distr::Normal;
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
    trajectories: Vec<Trace>,
    detections: Vec<Vec3f>,
    sweep_time: Scalar,
    rng: ThreadRng,
    distribution: Normal<Scalar>,
}

impl RadarSim {
    const DOT_RADIUS: Scalar = 2.0;

    pub fn new(sweep_time: Scalar, distribution: Normal<Scalar>) -> RadarSim {
        Self {
            trajectories: vec![],
            detections: vec![],
            sweep_time,
            rng: rand::rng(),
            distribution,
        }
    }

    pub fn add_trajectory(&mut self, state: State) {
        let time_offs = self.rng.random_range(0.0..self.sweep_time);
        self.trajectories.push(Trace::new(state, time_offs));
    }

    pub fn step(&mut self, dt: Scalar) {
        for t in &mut self.trajectories {
            t.step(dt);
            t.time_offs -= dt;
            if t.time_offs < 0.0 {
                t.time_offs += self.sweep_time;

                // add appropriate noise and introduce a detection
                println!("----------------------------------------");
                println!("posn: {:?}", t.state.position);
                let polar = Polar::from_vector(t.state.position);
                println!(
                    "polar: r={}, az={}, el={}",
                    polar.range, polar.azimuth, polar.elevation
                );
                let r_cart = radar::covariance_cart(polar);
                let detect = radar::perturb_position(r_cart, t.state.position, &mut self.rng);
                println!("perturb by: {:?}", detect - t.state.position);
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
    let distr = Normal::new(0.0, SIGMA).unwrap();
    let mut sim = RadarSim::new(5.0, distr);
    let state = State::new(Vec3f::new(0.0, 0.0, 10000.0), Vec3f::new(-5.0, 0.0, -5.0));
    sim.add_trajectory(state);
    sim.run(120.0, 1.0 / 60.0);

    let mut render = Render::new(
        1000,
        1000,
        Vec2f::new(-1200.0, 12000.0),
        Vec2f::new(300.0, -300.0),
    );
    sim.draw(&mut render);
    render.save("radarsim.png")

    //radar::sample_and_test(32_000.0, 0.0, 0.0, 10_000);
}
