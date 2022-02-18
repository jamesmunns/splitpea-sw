use phm::Machine;
use std::{time::{Duration, Instant}, thread::sleep};

fn main() -> Result<(), ()> {
    let mut dport = None;

    for port in serialport::available_ports().unwrap() {
        if let serialport::SerialPortType::UsbPort(serialport::UsbPortInfo {
            serial_number: Some(sn),
            ..
        }) = &port.port_type
        {
            if sn.as_str() == "ajm123" {
                dport = Some(port.clone());
                break;
            }
        }
    }

    let dport = if let Some(port) = dport {
        port
    } else {
        eprintln!("Error: No `Pretty hal machine` connected!");
        return Ok(());
    };

    let port = serialport::new(dport.port_name, 115200)
        .timeout(Duration::from_millis(5))
        .open()
        .map_err(drop)?;

    let ehal = Machine::from_port(port).unwrap();
    let mut last_send = Instant::now();

    let mut splitpea = splitpea_driver::Splitpea::new(ehal);

    loop {
        if last_send.elapsed() >= Duration::from_millis(100) {
            last_send = Instant::now();
            // TODO(AJM): `Debug` bounds on pretty hal machine error types?
            let ct = splitpea.get_event_cts().map_err(drop).unwrap();
            if ct == 0 {
                continue;
            }
            println!("Events pending: {}", ct);

            let evts = splitpea.get_all_events().map_err(drop).unwrap();
            println!("evts: {:?}", evts);
        } else {
            // Don't thrash the CPU
            sleep(Duration::from_millis(10));
        }
    }
}
