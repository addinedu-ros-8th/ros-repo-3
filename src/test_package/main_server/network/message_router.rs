use std::net::{TcpListener, TcpStream};
use std::io::{Read, Write};
use std::thread;
fn handle_client(mut stream: TcpStream) {
    let mut buffer = [0; 512];
    loop {
        match stream.read(&mut buffer) {
            Ok(0) => {
                println!("Client disconnected");
                break;
            }
            Ok(n) => {
                let msg = String::from_utf8_lossy(&buffer[..n]);
                println!("Received: {}", msg);
                stream.write_all(b"ACK").unwrap();
            }
            Err(e) => {
                eprintln!("Error: {}", e);
                break;
            }
        }
    }
}
fn main() {
    let listener = TcpListener::bind("0.0.0.0:9999").unwrap();
    println!("Message router listening on port 9999");
    for stream in listener.incoming() {
        match stream {
            Ok(stream) => {
                thread::spawn(|| handle_client(stream));
            }
            Err(e) => {
                eprintln!("Connection failed: {}", e);
            }
        }
    }
}