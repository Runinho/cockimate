use esp_idf_svc::hal::gpio::{AnyOutputPin, Output, PinDriver};
use esp_idf_svc::sys::{esp, esp_task_wdt_add, esp_task_wdt_reset, esp_timer_get_time, vTaskDelay, xTaskGetCurrentTaskHandle};
use esp_idf_svc::systime::EspSystemTime;
use std::collections::VecDeque;
use std::sync::mpsc::{channel, Receiver, Sender, TryRecvError};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};
use esp_idf_svc::hal::delay::FreeRtos;

const STEP_PULSE_US: u64 = 10;
const MIN_SPEED: i32 = 10;

#[derive(Clone, Copy, Debug)]
pub enum Command {
  MoveTo { position: i32, speed: i32 },
  Stop,
  Resume,
  Reset,
  Wait { time_ms: u64 },
}

#[derive(Clone, Copy, Debug)]
pub struct AxisStatus {
  pub position: i32,
  pub target: i32,
  pub speed: i32,
  pub timestamp_us: i64,
}

pub struct Axis {
  position: i32,
  target: i32,
  speed: i32,
  desired_speed: i32,
  step_pin: PinDriver<'static, AnyOutputPin, Output>,
  dir_pin: PinDriver<'static, AnyOutputPin, Output>,
  enable_pin: PinDriver<'static, AnyOutputPin, Output>,
}

impl Axis {
  pub(crate) fn new(
    step_pin: PinDriver<'static, AnyOutputPin, Output>,
    dir_pin: PinDriver<'static, AnyOutputPin, Output>,
    enable_pin: PinDriver<'static, AnyOutputPin, Output>,
  ) -> Self {
    Self {
      position: 0,
      target: 0,
      speed: 0,
      desired_speed: 1000,
      step_pin,
      dir_pin,
      enable_pin,
    }
  }

  fn to_status(&self) -> AxisStatus {
    AxisStatus {
      position: self.position,
      target: self.target,
      speed: self.desired_speed,
      timestamp_us: unsafe { esp_timer_get_time() },
    }
  }

  fn set_enable(&mut self, enabled: bool) {
    if enabled {
      let _ = self.enable_pin.set_low();
    } else {
      let _ = self.enable_pin.set_high();
    }
    // Small delay for motor driver to respond
    delay_us(200);
  }
}

// Helper function for microsecond delays
fn delay_us(us: u64) {
  let start = now_micros();
  // if (us > 2000){
  //   FreeRtos::delay_ms((us / 1000) as u32); // Yields to scheduler
  // }
  while now_micros() - start < us {}
}
// Get current time in microseconds
fn now_micros() -> u64 {
  EspSystemTime {}.now().as_micros() as u64
}

enum State {
  Running,
  Paused,
  Waiting { wait_start: Instant },
}

impl PartialEq for State {
  fn eq(&self, other: &Self) -> bool {
    matches!(
            (self, other),
            (State::Running, State::Running) | (State::Paused, State::Paused) | (State::Waiting { .. }, State::Waiting { .. })
        )
  }
}

pub fn axis_thread(mut axis: Axis, cmd_rx: Receiver<Command>, status: Arc<Mutex<AxisStatus>>) {
  // Disable watchdog for Core 1's idle task
  unsafe {
    let handle = esp_idf_sys::xTaskGetIdleTaskHandleForCore(1);
    if !handle.is_null() {
      esp_idf_sys::esp_task_wdt_delete(handle);
    }

    //create own for me
    esp_idf_sys::esp_task_wdt_add(esp_idf_sys::xTaskGetCurrentTaskHandle());
  }
  println!("Axis thread started");
  println!(
    "Initial axis state: pos={}, target={}, speed={}",
    axis.position, axis.target, axis.speed
  );

  axis.set_enable(false);
  let _ = axis.step_pin.set_low();

  let mut state = State::Running;
  let mut command_buffer: VecDeque<Command> = VecDeque::with_capacity(8);
  let mut current: Option<Command> = None;
  let mut loop_count = 0u64;

  loop {
    unsafe {
      esp_idf_sys::esp_task_wdt_reset();
    }
    loop_count += 1;

    // Handle incoming commands (non-blocking)
    loop {
      match cmd_rx.try_recv() {
        Ok(cmd) => {
          println!(">>> Received command: {:?}", cmd);
          match cmd {
            Command::Stop => state = State::Paused,
            Command::Resume => state = State::Running,
            Command::Reset => {
              command_buffer.clear();
              current = None;
            }
            Command::MoveTo { position, speed } => {
              println!("  -> Adding MoveTo to buffer: pos={}, speed={}", position, speed);
              command_buffer.push_back(cmd);
            }
            Command::Wait { time_ms } => {
              println!("  -> Adding Wait to buffer: time_ms={}", time_ms);
              command_buffer.push_back(cmd);
            }
          }
        }
        Err(TryRecvError::Empty) => break,
        Err(TryRecvError::Disconnected) => {
          println!("!!! Command channel disconnected");
          break;
        }
      }
    }

    // Update shared status
    if let Ok(mut status_guard) = status.try_lock() {
      *status_guard = axis.to_status();
    }

    // Process commands if not paused
    if state != State::Paused {
      if current.is_none() {
        current = command_buffer.pop_front();
        if current.is_some() {
          println!("<<< Popped command from buffer: {:?}", current);
        }
      }

      if let Some(cmd) = &current {
        match cmd {
          Command::Wait { time_ms } => {
            if let State::Waiting { wait_start } = state {
              let elapsed = wait_start.elapsed().as_millis() as i64;
              let time_remaining = *time_ms as i64 - elapsed;

              if time_remaining <= 0 {
                println!("=== Wait complete");
                current = None;
                state = State::Running;
              }
            } else {
              println!("=== Starting wait for {} ms", time_ms);
              state = State::Waiting { wait_start: Instant::now() };
            }
          }
          Command::MoveTo { position, speed } => {
            // Reset to Running state if we were waiting
            if matches!(state, State::Waiting { .. }) {
              state = State::Running;
            }

            axis.target = *position;
            axis.desired_speed = *speed;

            if axis.position != *position {
              axis.set_enable(true);

              // Set direction
              if axis.position < axis.target {
                let _ = axis.dir_pin.set_high();
              } else {
                let _ = axis.dir_pin.set_low();
              }

              // Calculate distance to target
              let distance = (axis.position - axis.target).abs();
              let decel_distance = axis.speed;

              // Acceleration/deceleration logic
              if distance > decel_distance {
                if axis.speed < *speed {
                  axis.speed += 1;
                }
              } else {
                let target_speed = distance.max(MIN_SPEED);
                if axis.speed > target_speed {
                  axis.speed -= 1;
                }
              }

              // Calculate step delay
              let step_delay_us = 1_000_000 / axis.speed.max(10) as u64;

              // Wait for next step
              delay_us(step_delay_us - STEP_PULSE_US);

              // Generate step pulse
              let _ = axis.step_pin.set_high();
              delay_us(STEP_PULSE_US);
              let _ = axis.step_pin.set_low();

              // Update position
              if axis.position < axis.target {
                axis.position += 1;
              } else {
                axis.position -= 1;
              }

              // Continue immediately to next step
              continue;
            } else {
              println!("=== Movement complete at position: {}", axis.position);
              axis.set_enable(false);
              current = None;
            }
          }
          _ => {}
        }
      }
    } else {
      // Paused state
      axis.set_enable(false);
    }

    // Single delay point for all non-moving code paths
    delay_us(1000);
  }
}