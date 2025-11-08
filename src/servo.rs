use esp_idf_svc::hal::gpio::{AnyIOPin, AnyOutputPin, Input, Output, PinDriver, Pull};
use esp_idf_svc::sys::{esp, esp_task_wdt_add, esp_task_wdt_reset, esp_timer_get_time, vTaskDelay, xTaskGetCurrentTaskHandle};
use esp_idf_svc::systime::EspSystemTime;
use std::collections::VecDeque;
use std::sync::mpsc::{channel, Receiver, Sender, TryRecvError};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};
use esp_idf_svc::hal::delay::FreeRtos;
use serde::{Deserialize, Serialize};

const STEP_PULSE_US: u64 = 10;
const MIN_SPEED: i32 = 10;

impl AxisId {
  fn as_usize(&self) -> usize {
    match self {
      AxisId::AxisX => 0,
      AxisId::AxisZ => 1,
    }
  }
}

// Add Deserialize to your existing enums/structs
#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
#[serde(tag = "type", rename_all = "lowercase")]
pub enum Command {
  #[serde(rename = "moveto")]
  MoveTo { position: i32, speed: i32 },
  Home { direction: i32, speed: i32 },
  Stop,
  Resume,
  Reset,
  #[serde(rename = "wait")]
  Wait { time_ms: u64 },
  Sync { id: Option<u32> },
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
pub struct AxisCommand {
  #[serde(default)]
  pub axis: Option<AxisId>,
  #[serde(flatten)]
  pub command: Command,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum AxisId {
  #[serde(rename = "x")]
  AxisX,
  #[serde(rename = "z")]
  AxisZ,
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
  pub home_pin: PinDriver<'static, AnyIOPin, Input>, // Homing limit switch
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
  home_pin: PinDriver<'static, AnyIOPin, Input>,
  homing_direction: i32, // Used during homing
  is_enabled: bool,
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
      home_pin: pins.home_pin,
      homing_direction: 0,
      is_enabled: false,
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


  // return true when changed
  fn set_enable(&mut self, enabled: bool) -> bool{
    if enabled != self.is_enabled{
      if enabled {
        let _ = self.enable_pin.set_low();
      } else {
        let _ = self.enable_pin.set_high();
      }
      self.is_enabled = enabled;
      // Small delay for motor driver to respond
      //delay_us(200);
      return true;
    }
    false
  }

  // Check if home switch is pressed (active LOW with internal pullup)
  fn is_home_switch_pressed(&self) -> bool {
    self.home_pin.is_low()
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
  Homing, // New state for homing
}

impl PartialEq for State {
  fn eq(&self, other: &Self) -> bool {
    matches!(
            (self, other),
            (State::Running, State::Running)
            | (State::Paused, State::Paused)
            | (State::Waiting { .. }, State::Waiting { .. })
            | (State::Homing, State::Homing)
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

// Global synchronization state shared between axes
struct SyncState {
  next_sync_id: u32,              // The next sync ID to assign
  axes_at_sync: [Option<u32>; 2], // Which sync_id each axis is waiting at (None if not waiting)
}

impl SyncState {
  fn new() -> Self {
    Self {
      next_sync_id: 0,
      axes_at_sync: [None, None],
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

  let mut sync_id = 0;

  // Create shared sync state
  let sync_state = Arc::new(Mutex::new(SyncState::new()));

  loop {
    unsafe {
      esp_idf_sys::esp_task_wdt_reset();
    }
    loop_count += 1;

    let current_time = now_micros();

    // Handle incoming commands (non-blocking) and route to appropriate axis
    loop {
      match cmd_rx.try_recv() {
        Ok(mut axis_cmd) => {
          println!(">>> Received command: {:?}", axis_cmd);

          if let Command::Sync {id} = &axis_cmd.command {
            println!("adding sync id: {} to both buffers", sync_id);

            axis_state0.command_buffer.push_back(Command::Sync {id: Some(sync_id) });
            axis_state1.command_buffer.push_back(Command::Sync {id: Some(sync_id) });
            sync_id += 1;

          } else {
            let axis_state = match axis_cmd.axis {
              Some(AxisId::AxisX) => &mut axis_state0,
              Some(AxisId::AxisZ) => &mut axis_state1,
              None => {
                // TODO: we should apply that to both axis!
                axis_cmd.axis = Some(AxisId::AxisX);
                &mut axis_state0
              }
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

                // Handle sync state on reset
                let mut sync_guard = sync_state.lock().unwrap();
                let axis_idx = axis_cmd.axis.expect("None not supported for command").as_usize();

                // If this axis was at a sync, increment next_sync_id to invalidate it
                if sync_guard.axes_at_sync[axis_idx].is_some() {
                  println!("[{:?}] === Reset while at sync - invalidating sync_id={}",
                           axis_cmd.axis, sync_guard.next_sync_id);
                  sync_guard.next_sync_id += 1;
                }

                // Clear all axes from sync (invalidated now)
                sync_guard.axes_at_sync = [None, None];
              }
              Command::MoveTo { position, speed } => {
                println!("[{:?}]   -> Adding MoveTo to buffer: pos={}, speed={}", axis_cmd.axis, position, speed);
                axis_state.command_buffer.push_back(axis_cmd.command);
              }
              Command::Home { direction, speed } => {
                println!("[{:?}]   -> Adding Home to buffer: direction={}, speed={}", axis_cmd.axis, direction, speed);
                axis_state.command_buffer.push_back(axis_cmd.command);
              }
              Command::Wait { time_ms } => {
                println!("[{:?}]   -> Adding Wait to buffer: time_ms={}", axis_cmd.axis, time_ms);
                axis_state.command_buffer.push_back(axis_cmd.command);
              },
              Command::Sync{id} => {
                panic!("can never happen because handeld in case above")
              }
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
    let next_pulse_time_0 = process_axis_commands(
      &mut axis0,
      &mut axis_state0,
      &mut active_axis,
      current_time,
      sync_state.clone(),
    );

    let next_pulse_time_1 = process_axis_commands(
      &mut axis1,
      &mut axis_state1,
      &mut active_axis,
      current_time,
      sync_state.clone(),
    );


    let time_after_compute = now_micros();

    // Determine which axis needs to pulse next and sort by time
    let (first_axis, first_time, second_axis, second_time) =
        match (next_pulse_time_0, next_pulse_time_1) {
          (Some(t0), Some(t1)) => {
            if t0 as i64 - time_after_compute as i64 <= t1 as i64 - time_after_compute as i64 {
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
      let time_to_wait = (first_time as i64) - (time_after_compute as i64);

      // // debug only
      // let (first_axis_state, first_axis_s) = match &first_id {
      //   AxisId::AxisX => (&axis_state0, &axis0),
      //   AxisId::AxisZ => (&axis_state1, &axis1)
      // };
      // println!("waiting for {time_to_wait} for axis {:?}. last time: {} current_time: {}, speed: {}",
      //          first_id,
      //          first_axis_state.last_pulse_time,
      //          time_after_compute,
      //          first_axis_s.speed);
      if time_to_wait > 0 {
        delay_us(time_to_wait as u64);
      }
      let time_after_first_wait = now_micros();


      // Execute first pulse
      match first_id {
        AxisId::AxisX => execute_pulse(&mut axis0, &mut axis_state0, time_after_first_wait),
        AxisId::AxisZ => execute_pulse(&mut axis1, &mut axis_state1, time_after_first_wait),
      }

      // Check if we should execute the second pulse
      if let (Some(second_id), Some(second_t)) = (second_axis, second_time) {
        let remaining_time = second_t as i64 - now_micros() as i64;

        // If waiting would cause us to miss the next pulse of the faster axis (1.9x threshold)
        if remaining_time <= ((time_after_compute - current_time) + 20) as i64 {
          // Wait and execute the second pulse
          if (remaining_time > 0) {
            delay_us(remaining_time as u64);
          }
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
  sync_state: Arc<Mutex<SyncState>>,
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
    // reset the speed
    axis.speed = 0;
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
      Command::Home { direction, speed } => {
        // Check if home switch is already pressed at start
        if axis_state.state != State::Homing && axis.is_home_switch_pressed() {
          println!("[{:?}] === Homing complete - switch already pressed at start", axis.id);
          axis.position = 0;
          axis.set_enable(false);
          axis_state.current = None;
          axis_state.state = State::Running;
          return None;
        }

        // Set state to homing if not already
        if axis_state.state != State::Homing {
          println!("[{:?}] === Starting homing: direction={}, speed={}", axis.id, direction, speed);
          axis_state.state = State::Homing;
          axis.desired_speed = *speed;
          axis.speed = MIN_SPEED; // Start slow
          axis.homing_direction = *direction;

          // Set direction pin
          if *direction > 0 {
            let _ = axis.dir_pin.set_high();
          } else {
            let _ = axis.dir_pin.set_low();
          }

          axis.set_enable(true);
          // to force wait after enable/dir change.
          axis_state.last_pulse_time = now_micros();
        }

        // Check if home switch is pressed
        if axis.is_home_switch_pressed() {
          println!("[{:?}] === Homing complete - switch triggered at position {}", axis.id, axis.position);
          axis.position = 0; // Reset position to zero
          axis.set_enable(false);
          axis_state.current = None;
          axis_state.state = State::Running;
          return None;
        }

        // Continue homing movement
        return Some(calculate_time_to_next_pulse_homing(axis, axis_state, current_time));
      }
      Command::Sync {id} => {
        // Reset to Running state if we were waiting
        if matches!(axis_state.state, State::Waiting { .. }) {
          axis_state.state = State::Running;
        }

        let mut sync_guard = sync_state.lock().unwrap();
        let axis_idx = axis.id.as_usize();
        let sync_id = id.unwrap();

        // NEW: Check if this sync has been invalidated
        if sync_id < sync_guard.next_sync_id {
          println!("[{:?}] === Skipping outdated sync_id={} (next_sync_id={})",
                   axis.id, sync_id, sync_guard.next_sync_id);
          axis_state.current = None; // Skip this sync command
          return None;
        }

        // First time seeing this Sync command?
        if sync_guard.axes_at_sync[axis_idx].is_none() {
          sync_guard.axes_at_sync[axis_idx] = Some(sync_id);
          println!("[{:?}] === Waiting at sync_id={}", axis.id, sync_id);
        }

        // Check if both axes are at the same sync point
        let this_sync = sync_guard.axes_at_sync[axis_idx];
        let all_synced = sync_guard.axes_at_sync.iter()
            .all(|&s| s.is_some() && s == this_sync);

        if all_synced {
          println!("=== All axes synchronized at sync_id={:?}", this_sync);
          sync_guard.next_sync_id += 1;
          sync_guard.axes_at_sync = [None, None];
          axis_state.current = None;
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
          // Set direction
          if axis.position < axis.target {
            let _ = axis.dir_pin.set_high();
          } else {
            let _ = axis.dir_pin.set_low();
          }
          // enable if needed (this includes a wait)
          let changed =  axis.set_enable(true);
          let changed_time = match changed {true => {200} _ => 0};
          if changed {
            // we set the last pulse time to now, to force the wait.
            axis_state.last_pulse_time = now_micros();
          }

          // Calculate timing for next pulse
          return Some(calculate_time_to_next_pulse(axis, axis_state, current_time) + changed_time);
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
  // Calculate step delay based on current speed
  let step_delay_us = 1_000_000 / axis.speed.max(10) as u64;

  return step_delay_us + axis_state.last_pulse_time;
}

// Calculate time until next pulse during homing (simpler logic)
fn calculate_time_to_next_pulse_homing(
  axis: &mut Axis,
  axis_state: &AxisState,
  current_time: u64,
) -> u64 {
  // Use constant speed during homing
  let step_delay_us = 1_000_000 / axis.desired_speed.max(10) as u64;
  return step_delay_us + axis_state.last_pulse_time;
}

// Execute a step pulse on the axis
fn execute_pulse(
  axis: &mut Axis,
  axis_state: &mut AxisState,
  current_time: u64,
) {
  // Different behavior for homing vs normal movement
  if axis_state.state == State::Homing {
    // During homing, maintain constant speed
    axis.speed = axis.desired_speed;

    // Generate step pulse
    let _ = axis.step_pin.set_high();
    delay_us(STEP_PULSE_US);
    let _ = axis.step_pin.set_low();

    // Update position based on homing direction
    axis.position += axis.homing_direction;

    // Record pulse time
    axis_state.last_pulse_time = current_time;
  } else {
    // Normal movement with acceleration/deceleration
    // Update speed based on distance to target
    let distance = (axis.position - axis.target).abs();
    let decel_distance = axis.speed/5;

    if distance > decel_distance {
      if axis.speed < axis.desired_speed {
        axis.speed += 5;
      }
    } else {
      let target_speed = distance.max(MIN_SPEED);
      if axis.speed > target_speed {
        axis.speed -= 5;
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
  }

  // println!("[{:?}] Pulse executed, pos={}, target={}, speed={}",
  //          axis.id, axis.position, axis.target, axis.speed);
}