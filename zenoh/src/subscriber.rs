use clap::Parser;
use tokio::time::sleep;
use std::time::{Duration, SystemTime, UNIX_EPOCH};
use std::fs::File;
use std::io::{Write, BufWriter};
use std::sync::atomic::{AtomicUsize, AtomicBool, Ordering};
use chrono::{Local, Datelike, Timelike};
use zenoh::config::Config;

#[derive(Parser)]
#[command(name = "zenoh-benchmark-subscriber")]
struct Args {
    /// 예상되는 총 샘플 수
    #[arg(short = 's', long, default_value_t = 1000)]
    expected_samples: usize,
}

// 밀리초 단위까지 포함한 타임스탬프 생성
fn get_timestamp_with_ms() -> String {
    let now = Local::now();
    
    format!("{:04}-{:02}-{:02} {:02}:{:02}:{:02}.{:03}", 
        now.year(), now.month(), now.day(),
        now.hour(), now.minute(), now.second(),
        now.nanosecond() / 1_000_000)
}

#[tokio::main]
async fn main() {
    // 명령줄 인자 파싱
    let args = Args::parse();
    let expected_samples = args.expected_samples;
    
    // CSV 파일 생성
    let file = File::create("zenoh_timestamp.csv").expect("Failed to create CSV file");
    
    let mut writer = BufWriter::new(file);
    writeln!(writer, "Timestamp");
    
    let config = Config::default();
    let session = zenoh::open(config)
        .await
        .expect("Failed to open Zenoh session");

    let subscriber = session
        .declare_subscriber("zenoh_benchmark")
        .await
        .expect("Failed to declare subscriber");

    let mut received_count = 0;
    let mut total_bytes = 0;
    let mut start_time_ns = 0;

    loop {
        match subscriber.recv() {
            Ok(sample) => {
                if received_count == 0 {
                    start_time_ns = SystemTime::now()
                        .duration_since(UNIX_EPOCH)
                        .unwrap()
                        .as_nanos() as usize;
                }

                let timestamp = get_timestamp_with_ms();
                if let Err(e) = writeln!(writer, "{}", timestamp) {
                    eprintln!("CSV write failed: {}", e);
                }

                let zbytes = sample.payload();
                let data_size = zbytes.len();

                // 빈 메시지가 오면 종료
                if data_size == 0 {
                    let now = SystemTime::now().duration_since(UNIX_EPOCH)
                        .unwrap().as_nanos() as usize;

                    let duration_ns = now - start_time_ns;
                    let duration_us = duration_ns / 1_000;
                    let duration_sec = duration_ns as f64 / 1_000_000_000.0;

                    let throughput = total_bytes as f64 / duration_sec;
                    let loss_rate = (expected_samples - received_count) as f64 
                                    / expected_samples as f64;

                    println!("Received empty message - publisher finished");
                    println!("Total received: {}", received_count);
                    println!("Total time (microseconds): {}", duration_us);
                    println!("Throughput (bytes per second): {:.2}", throughput);
                    println!("Loss rate: {:.4}", loss_rate);

                    break; // 빈 메시지를 받았으므로 종료
                }
                
                received_count += 1;

                println!("Received message with [{} bytes] ({} / {})", 
                    data_size, received_count, expected_samples);

                total_bytes += data_size;
            }
            Err(e) => {
                eprintln!("Failed to receive message: {e}");
            }
        }
    }
}