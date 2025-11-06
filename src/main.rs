//! HTTP Server with JSON POST handler
//!
//! Go to 192.168.71.1 to test

mod servo;

use core::convert::TryInto;
use std::sync::{Arc, Mutex};
use std::sync::mpsc::channel;
use embedded_svc::{http::{Headers, Method}, io::{Read, Write}, wifi, wifi::{AuthMethod, ClientConfiguration, Configuration}};

use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::{
  eventloop::EspSystemEventLoop,
  http::server::EspHttpServer,
  nvs::EspDefaultNvsPartition,
  wifi::{BlockingWifi, EspWifi},
};
use std::thread::{self, Builder, JoinHandle};
use std::time::Duration;
use embedded_svc::wifi::AccessPointConfiguration;
use esp_idf_svc::hal::cpu::Core;
use esp_idf_svc::hal::gpio::{IOPin, OutputPin, PinDriver};
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use log::*;

use serde::Deserialize;
use crate::servo::{axis_thread, Axis, AxisStatus, Command};

const SSID: &str = env!("WIFI_SSID");
const PASSWORD: &str = env!("WIFI_PASS");
static INDEX_HTML: &str = include_str!("servo-control-migrated.html");

// Max payload length
const MAX_LEN: usize = 128;

// Need lots of stack to parse JSON
const STACK_SIZE: usize = 10240;

// Wi-Fi channel, between 1 and 11
const CHANNEL: u8 = 11;

#[derive(Deserialize)]
struct FormData<'a> {
  first_name: &'a str,
  age: u32,
  birthplace: &'a str,
}

fn parse_query_param<T: core::str::FromStr>(query: &String, key: &'static str) -> Result<T, String> {
  query
    .split('&')
    .find_map(|param| {
      let (k, v) = param.split_once('=')?;
      if k == key {
        Some(v)
      } else {
        None
      }
    })
    .ok_or_else(|| format!("Missing parameter: {}", key))
    .and_then(|v| v.parse::<T>().map_err(|_| format!("Invalid parameter: {}", key)))
}

fn parse_move_params(query: String) -> Result<(i32, u32), String> {
  let position = parse_query_param::<i32>(&query, "position")?;
  let speed = parse_query_param::<u32>(&query, "speed")?;
  Ok((position, speed))
}

fn main() -> anyhow::Result<()> {
  esp_idf_svc::sys::link_patches();
  esp_idf_svc::log::EspLogger::initialize_default();

  // Setup Wifi

  let peripherals = Peripherals::take()?;
  let sys_loop = EspSystemEventLoop::take()?;
  let nvs = EspDefaultNvsPartition::take()?;

  let mut wifi = BlockingWifi::wrap(
    EspWifi::new(peripherals.modem, sys_loop.clone(), Some(nvs))?,
    sys_loop,
  )?;
  // wifi.wifi_mut()
  //     .sta_netif()
  //     .set_hostname("cockimate4")?;

  connect_wifi(&mut wifi)?;


  // Create communication channel
  let (cmd_tx, cmd_rx) = channel::<Command>();

  // Create shared status
  let status = Arc::new(Mutex::new(AxisStatus {
    position: 0,
    target: 0,
    speed: 0,
    timestamp_us: 0,
  }));

  let mut server = create_server()?;

  server.fn_handler("/", Method::Get, |req| {
    req.into_ok_response()?
      .write_all(INDEX_HTML.as_bytes())
      .map(|_| ())
  })?;

  // Clone the Arc and channel sender before moving into closure
  let status_clone = status.clone();
  let cmd_tx_clone = cmd_tx.clone();

  server.fn_handler::<anyhow::Error, _>("/post", Method::Post, move |mut req| {
    let len = req.content_len().unwrap_or(0) as usize;

    if len > MAX_LEN {
      req.into_status_response(413)?
        .write_all("Request too big".as_bytes())?;
      return Ok(());
    }

    let mut buf = vec![0; len];
    req.read_exact(&mut buf)?;
    let mut resp = req.into_ok_response()?;

    if let Ok(form) = serde_json::from_slice::<FormData>(&buf) {
      cmd_tx_clone.send(Command::MoveTo {
        position: form.age as i32,
        speed: form.birthplace.parse().unwrap_or(500)
      }).unwrap();

      write!(
        resp,
        "Hello, {}-year-old {} from {}! {:?}",
        form.age, form.first_name, form.birthplace, status_clone.lock().unwrap()
      )?;
    } else {
      resp.write_all("JSON error".as_bytes())?;
    }

    Ok(())
  })?;

  // MoveTo command
  let cmd_tx_moveto = cmd_tx.clone();
  server.fn_handler::<anyhow::Error, _>("/moveto", Method::Get, move |req| {
    let query = req.uri()
      .split_once('?')
      .map(|(_, q)| q.to_string())
      .unwrap_or_default();
    let mut resp = req.into_ok_response()?;

    match parse_move_params(query) {
      Ok((position, speed)) => {
        cmd_tx_moveto.send(Command::MoveTo { position, speed: speed.try_into().unwrap() }).unwrap();
        resp.write_all(b"OK: MoveTo queued")?;
      }
      Err(e) => {
        resp.write_all(format!("Error: {}", e).as_bytes())?;
      }
    }
    Ok(())
  })?;

  // Stop command
  let cmd_tx_stop = cmd_tx.clone();
  server.fn_handler::<anyhow::Error, _>("/stop", Method::Get, move |req| {
    cmd_tx_stop.send(Command::Stop).unwrap();
    req.into_ok_response()?
      .write_all(b"OK: Stopped")?;
    Ok(())
  })?;

  // Resume command
  let cmd_tx_resume = cmd_tx.clone();
  server.fn_handler::<anyhow::Error, _>("/resume", Method::Get, move |req| {
    cmd_tx_resume.send(Command::Resume).unwrap();
    let mut resp = req.into_ok_response()?;
    resp.write_all(b"OK: Resumed")?;
    Ok(())
  })?;

  // Reset
  let cmd_tx_reset = cmd_tx.clone();
  server.fn_handler::<anyhow::Error, _>("/reset", Method::Get, move |req| {
    cmd_tx_reset.send(Command::Reset).unwrap();
    let mut resp = req.into_ok_response()?;
    resp.write_all(b"OK: Reset")?;
    Ok(())
  })?;

  // Wait command
  let cmd_tx_wait = cmd_tx.clone();
  server.fn_handler::<anyhow::Error, _>("/wait", Method::Get, move |req| {
    let query = req.uri()
      .split_once('?')
      .map(|(_, q)| q.to_string())
      .unwrap_or_default();;
    let mut resp = req.into_ok_response()?;

    match parse_query_param::<u64>(&query, "time") {
      Ok(time_ms) => {
        cmd_tx_wait.send(Command::Wait { time_ms }).unwrap();
        resp.write_all(b"OK: Wait queued")?;
      }
      Err(e) => {
        resp.write_all(format!("Error: {}", e).as_bytes())?;
      }
    }
    Ok(())
  })?;

  let status_clone2 = status.clone();
  // Status endpoint
  server.fn_handler::<anyhow::Error, _>("/status", Method::Get, move |req| {
    let status = {
      let s = status_clone2.lock();
      s.unwrap() // Copy out
    }; // Lock released here

    let json = format!(
      "{{\"position\":{},\"target\":{},\"speed\":{},\"timestamp\":{}}}",
      status.position, status.target, status.speed, status.timestamp_us
    );

    let mut resp = req.into_ok_response()?;
    resp.write_all(json.as_bytes())?;
    Ok(())
  })?;;


  // SETUP servo driver
  // Configure GPIO pins (adjust pin numbers for your hardware)
  let step_pin = PinDriver::output(peripherals.pins.gpio32.downgrade_output())?;
  let dir_pin = PinDriver::output(peripherals.pins.gpio25.downgrade_output())?;
  let enable_pin = PinDriver::output(peripherals.pins.gpio26.downgrade_output())?;

  let axis = Axis::new(step_pin, dir_pin, enable_pin);




  let status_clone = status.clone();

  // Configure thread for Core 1
  ThreadSpawnConfiguration::set(&ThreadSpawnConfiguration {
    name: Some(b"axis_task\0"),
    stack_size: 8192, // Adjust as needed
    priority: 5,
    pin_to_core: Some(Core::Core1),
    ..Default::default()
  })?;

  let _axis_handle = std::thread::Builder::new()
    .spawn(move || {
      log::info!("Running on core: {:?}", esp_idf_svc::hal::cpu::core());

      axis_thread(axis, cmd_rx, status_clone);
    })?;

  // Main loop - send some test commands
  thread::sleep(Duration::from_secs(1));
  println!("Sending test command");
  cmd_tx.send(Command::MoveTo { position: 1000, speed: 500 })?;

  thread::sleep(Duration::from_secs(5));
  println!("Current status: {:?}", *status.lock().unwrap());

  // Keep wifi and the server running beyond when main() returns (forever)
  // Do not call this if you ever want to stop or access them later.
  // Otherwise you can either add an infinite loop so the main task
  // never returns, or you can move them to another thread.
  // https://doc.rust-lang.org/stable/core/mem/fn.forget.html
  core::mem::forget(wifi);
  core::mem::forget(server);

  // idk does this help lol?

  // Main task no longer needed, free up some memory
  Ok(())
}

fn connect_wifi(wifi: &mut BlockingWifi<EspWifi<'static>>) -> anyhow::Result<()> {
  // // If instead of creating a new network you want to serve the page
  // // on your local network, you can replace this configuration with
  // // the client configuration from the http_client example.
  // let wifi_configuration = wifi::Configuration::AccessPoint(AccessPointConfiguration {
  //     ssid: SSID.try_into().unwrap(),
  //     ssid_hidden: true,
  //     auth_method: AuthMethod::WPA2Personal,
  //     password: PASSWORD.try_into().unwrap(),
  //     channel: CHANNEL,
  //     ..Default::default()
  // });

  // wifi.set_configuration(&wifi_configuration)?;
  let wifi_configuration: Configuration = Configuration::Client(ClientConfiguration {
    ssid: SSID.try_into().unwrap(),
    bssid: None,
    auth_method: esp_idf_svc::wifi::AuthMethod::WPA2Personal,
    password: PASSWORD.try_into().unwrap(),
    channel: None,
    ..Default::default()
  });

  wifi.set_configuration(&wifi_configuration)?;

  wifi.start()?;
  info!("Wifi started");

  // If using a client configuration you need
  // to connect to the network with:
  //
  //  ```
  wifi.connect()?;
  info!("Wifi connected");

  wifi.wait_netif_up()?;
  info!("Wifi netif up");

  // info!("Created Wi-Fi with WIFI_SSID `{SSID}` and WIFI_PASS `{PASSWORD}`");

  Ok(())
}

fn create_server() -> anyhow::Result<EspHttpServer<'static>> {
  let server_configuration = esp_idf_svc::http::server::Configuration {
    stack_size: STACK_SIZE,
    ..Default::default()
  };

  Ok(EspHttpServer::new(&server_configuration)?)
}