// message_router.rs

use std::net::TcpStream;
use std::io::Write;
use std::sync::Mutex;

// 메시지 타입 정의
enum MessageType {
    LoginRequest,
    CreateTaskRequest,
    SearchQRCodeData,
    RobotStatusRequest,
    Unknown,
}

// 메시지 파싱
fn parse_message(message: &str) -> MessageType {
    if message.contains("LoginRequest") {
        MessageType::LoginRequest
    } else if message.contains("CreateTaskRequest") {
        MessageType::CreateTaskRequest
    } else if message.contains("SearchQRCodeData") {
        MessageType::SearchQRCodeData
    } else if message.contains("RequestRobotStatus") {
        MessageType::RobotStatusRequest
    } else {
        MessageType::Unknown
    }
}

// 메인 라우팅 함수
pub fn route_message(message: &str, client_socket: &Mutex<TcpStream>) {
    match parse_message(message) {
        MessageType::LoginRequest => {
            println!("[Router] Handling LoginRequest...");
            handle_login_request(message, client_socket);
        }
        MessageType::CreateTaskRequest => {
            println!("[Router] Handling CreateTaskRequest...");
            handle_create_task_request(message, client_socket);
        }
        MessageType::SearchQRCodeData => {
            println!("[Router] Handling SearchQRCodeData...");
            handle_search_qrcode_data(message, client_socket);
        }
        MessageType::RobotStatusRequest => {
            println!("[Router] Handling RequestRobotStatus...");
            handle_robot_status_request(message, client_socket);
        }
        MessageType::Unknown => {
            println!("[Router] Unknown message type received.");
        }
    }
}

// 핸들러 함수들 (Stub)

fn handle_login_request(_message: &str, _client_socket: &Mutex<TcpStream>) {
    // TODO: MainService로 login 요청 넘기기
}

fn handle_create_task_request(_message: &str, _client_socket: &Mutex<TcpStream>) {
    // TODO: MainService로 task 생성 요청 넘기기
}

fn handle_search_qrcode_data(_message: &str, _client_socket: &Mutex<TcpStream>) {
    // TODO: QR 코드 검색 요청 처리
}

fn handle_robot_status_request(_message: &str, _client_socket: &Mutex<TcpStream>) {
    // TODO: 로봇 상태 요청 처리
}
