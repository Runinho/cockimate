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

// Enum to identify which axis
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum AxisId {
  AxisX,
  AxisZ,
}

impl AxisId {
  fn as_usize(&self) -> usize {
    match self {
      AxisId::AxisX => 0,
      AxisId::AxisZ => 1,
    }
  }
}

#[derive(Clone, Copy, Debug)]
pub enum Command {
  MoveTo { position: i32, speed: i32 },
  Stop,
  Resume,
  Reset,
  Wait { time_ms: u64 },
}

#[derive(Clone, Copy, Debug)]
pub struct AxisCommand {
  pub axis: AxisId,
  pub command: Command,
}

#[derive(Clone, Copy, Debug)]
pub struct AxisStatus {
  pub position: i32,
  pub target: i32,
  pub speed: i32,
  pub timestamp_us: i64,
}

impl Default for AxisStatus {
  fn default() -> Self {
    Self {
      position: 0,
      target: 0,
      speed: 0,
      timestamp_us: 0,
    }
  }
}

// Struct to hold all pins for one axis
pub struct AxisPins {
  pub step_pin: PinDriver<'static, AnyOutputPin, Output>,
  pub dir_pin: PinDriver<'static, AnyOutputPin, Output>,
  pub enable_pin: PinDriver<'static, AnyOutputPin, Output>,
}

pub struct Axis {
  id: AxisId,
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
    id: AxisId,
    pins: AxisPins,
  ) -> Self {
    Self {
      id,
      position: 0,
      target: 0,
      speed: 0,
      desired_speed: 1000,
      step_pin: pins.step_pin,
      dir_pin: pins.dir_pin,
      enable_pin: pins.enable_pin,
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

// Struct to hold axis state and command queue
struct AxisState {
  state: State,
  command_buffer: VecDeque<Command>,
  current: Option<Command>,
  last_pulse_time: u64,
}

impl AxisState {
  fn new() -> Self {
    Self {
      state: State::Running,
      command_buffer: VecDeque::with_capacity(8),
      current: None,
      last_pulse_time: 0,
    }
  }
}

pub fn axis_thread(
  mut axis0: Axis,
  mut axis1: Axis,
  cmd_rx: Receiver<AxisCommand>,
  status: Arc<Mutex<[AxisStatus; 2]>>,
) {
  // Disable watchdog for Core 1's idle task
  unsafe {
    let handle = esp_idf_sys::xTaskGetIdleTaskHandleForCore(1);
    if !handle.is_null() {
      esp_idf_sys::esp_task_wdt_delete(handle);
    }

    //create own for me
    esp_idf_sys::esp_task_wdt_add(esp_idf_sys::xTaskGetCurrentTaskHandle());
  }
  println!("Dual axis thread started");
  println!(
    "Axis0 initial state: pos={}, target={}, speed={}",
    axis0.position, axis0.target, axis0.speed
  );
  println!(
    "Axis1 initial state: pos={}, target={}, speed={}",
    axis1.position, axis1.target, axis1.speed
  );

  axis0.set_enable(false);
  axis1.set_enable(false);
  let _ = axis0.step_pin.set_low();
  let _ = axis1.step_pin.set_low();

  let mut axis_state0 = AxisState::new();
  let mut axis_state1 = AxisState::new();
  let mut active_axis: Option<AxisId> = None;
  let mut loop_count = 0u64;

  loop {
    unsafe {
      esp_idf_sys::esp_task_wdt_reset();
    }
    loop_count += 1;

    let current_time = now_micros();

    // Handle incoming commands (non-blocking) and route to appropriate axis
    loop {
      match cmd_rx.try_recv() {
        Ok(axis_cmd) => {
          println!(">>> Received command: {:?}", axis_cmd);

          let axis_state = match axis_cmd.axis {
            AxisId::AxisX => &mut axis_state0,
            AxisId::AxisZ => &mut axis_state1,
          };

          match axis_cmd.command {
            Command::Stop => {
              axis_state.state = State::Paused;
            }
            Command::Resume => {
              axis_state.state = State::Running;
            }
            Command::Reset => {
              axis_state.command_buffer.clear();
              axis_state.current = None;
            }
            Command::MoveTo { position, speed } => {
              println!("[{:?}]   -> Adding MoveTo to buffer: pos={}, speed={}", axis_cmd.axis, position, speed);
              axis_state.command_buffer.push_back(axis_cmd.command);
            }
            Command::Wait { time_ms } => {
              println!("[{:?}]   -> Adding Wait to buffer: time_ms={}", axis_cmd.axis, time_ms);
              axis_state.command_buffer.push_back(axis_cmd.command);
            }
          }
        }
        Err(TryRecvError::Empty) => break,
        Err(TryRecvError::Disconnected) => {
          println!("!!! Command channel disconnected");
          return;
        }
      }
    }

    // Update shared status
    if let Ok(mut status_guard) = status.try_lock() {
      status_guard[0] = axis0.to_status();
      status_guard[1] = axis1.to_status();
    }

    // Process both axes and get time until next pulse
    let time_to_next_pulse_0 = process_axis_commands(
      &mut axis0,
      &mut axis_state0,
      &mut active_axis,
      current_time,
    );

    let time_to_next_pulse_1 = process_axis_commands(
      &mut axis1,
      &mut axis_state1,
      &mut active_axis,
      current_time,
    );

    // Determine which axis needs to pulse next and sort by time
    let (first_axis, first_time, second_axis, second_time) =
        match (time_to_next_pulse_0, time_to_next_pulse_1) {
          (Some(t0), Some(t1)) => {
            if t0 <= t1 {
              (Some(AxisId::AxisX), t0, Some(AxisId::AxisZ), Some(t1))
            } else {
              (Some(AxisId::AxisZ), t1, Some(AxisId::AxisX), Some(t0))
            }
          }
          (Some(t0), None) => (Some(AxisId::AxisX), t0, None, None),
          (None, Some(t1)) => (Some(AxisId::AxisZ), t1, None, None),
          (None, None) => (None, 0, None, None),
        };

    if let Some(first_id) = first_axis {
      // Wait for the first pulse
      delay_us(first_time);
      let after_first_wait = now_micros();

      // Execute first pulse
      match first_id {
        AxisId::AxisX => execute_pulse(&mut axis0, &mut axis_state0, after_first_wait),
        AxisId::AxisZ => execute_pulse(&mut axis1, &mut axis_state1, after_first_wait),
      }

      // Check if we should execute the second pulse
      if let (Some(second_id), Some(second_t)) = (second_axis, second_time) {
        let remaining_time = second_t - first_time;

        // If waiting would cause us to miss the next pulse of the faster axis (1.9x threshold)
        if remaining_time <= (first_time as f32 * 1.9) as u64 {
          // Wait and execute the second pulse
          delay_us(remaining_time);
          let after_second_wait = now_micros();

          match second_id {
            AxisId::AxisX => execute_pulse(&mut axis0, &mut axis_state0, after_second_wait),
            AxisId::AxisZ => execute_pulse(&mut axis1, &mut axis_state1, after_second_wait),
          }
        }
        // Otherwise, skip waiting for the second pulse and continue loop
      }
    } else {
      // No axes need to pulse
      delay_us(1000);
    }
  }
}

// Helper function to process commands for a single axis
// Returns Some(microseconds) until next pulse, or None if not moving
fn process_axis_commands(
  axis: &mut Axis,
  axis_state: &mut AxisState,
  active_axis: &mut Option<AxisId>,
  current_time: u64,
) -> Option<u64> {
  // If paused, disable and return
  if axis_state.state == State::Paused {
    axis.set_enable(false);
    return None;
  }

  // Pop next command if we don't have one
  if axis_state.current.is_none() {
    axis_state.current = axis_state.command_buffer.pop_front();
    if axis_state.current.is_some() {
      println!("[{:?}] <<< Popped command from buffer: {:?}", axis.id, axis_state.current);
    }
  }
  // Process current command
  if let Some(cmd) = &axis_state.current {
    match cmd {
      Command::Wait { time_ms } => {
        if let State::Waiting { wait_start } = axis_state.state {
          let elapsed = wait_start.elapsed().as_millis() as i64;
          let time_remaining = *time_ms as i64 - elapsed;

          if time_remaining <= 0 {
            println!("[{:?}] === Wait complete", axis.id);
            axis_state.current = None;
            axis_state.state = State::Running;
          }
        } else {
          println!("[{:?}] === Starting wait for {} ms", axis.id, time_ms);
          axis_state.state = State::Waiting { wait_start: Instant::now() };
        }
        return None;
      }
      Command::MoveTo { position, speed } => {
        // Reset to Running state if we were waiting
        if matches!(axis_state.state, State::Waiting { .. }) {
          axis_state.state = State::Running;
        }

        axis.target = *position;
        axis.desired_speed = *speed;

        if axis.position != *position {
          // Enable axis if not already enabled
          axis.set_enable(true);

          // Set direction
          if axis.position < axis.target {
            let _ = axis.dir_pin.set_high();
          } else {
            let _ = axis.dir_pin.set_low();
          }

          // Calculate timing for next pulse
          return Some(calculate_time_to_next_pulse(axis, axis_state, current_time));
        } else {
          // Movement complete
          println!("[{:?}] === Movement complete at position: {}", axis.id, axis.position);
          axis.set_enable(false);
          axis_state.current = None;
          return None;
        }
      }
      _ => {}
    }
  }
  None
}

// Calculate time until next pulse should occur
fn calculate_time_to_next_pulse(
  axis: &mut Axis,
  axis_state: &AxisState,
  current_time: u64,
) -> u64 {
  // // Calculate distance to target
  // let distance = (axis.position - axis.target).abs();
  // let decel_distance = axis.speed;
  //
  // // Acceleration/deceleration logic
  // let target_speed = if distance > decel_distance {
  //   axis.desired_speed.min(axis.speed + 1)
  // } else {
  //   distance.max(MIN_SPEED).min(axis.speed)
  // };

  // Calculate step delay based on current speed
  let step_delay_us = 1_000_000 / axis.speed.max(10) as u64;

  // Calculate time elapsed since last pulse
  let time_since_last_pulse = if axis_state.last_pulse_time > 0 {
    current_time.saturating_sub(axis_state.last_pulse_time)
  } else {
    step_delay_us // First pulse, return full delay
  };

  // Return remaining time until next pulse
  if time_since_last_pulse >= step_delay_us {
    0 // Ready to pulse now
  } else {
    step_delay_us - time_since_last_pulse
  }
}

// Execute a step pulse on the axis
fn execute_pulse(
  axis: &mut Axis,
  axis_state: &mut AxisState,
  current_time: u64,
) {
  // Update speed based on distance to target
  let distance = (axis.position - axis.target).abs();
  let decel_distance = axis.speed;

  if distance > decel_distance {
    if axis.speed < axis.desired_speed {
      axis.speed += 1;
    }
  } else {
    let target_speed = distance.max(MIN_SPEED);
    if axis.speed > target_speed {
      axis.speed -= 1;
    }
  }

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

  // Record pulse time
  axis_state.last_pulse_time = current_time;

  println!("[{:?}] Pulse executed, pos={}, target={}, speed={}",
           axis.id, axis.position, axis.target, axis.speed);
}

