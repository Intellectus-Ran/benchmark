use clap::Parser;
use std::time::{Duration, Instant};
use tokio::time::sleep;

#[derive(Parser)]
#[command(name = "zenoh-benchmark-publisher")]
struct Args {
    /// 메시지 발행 간격 (밀리초)
    #[arg(short = 'i', long, default_value_t = 33)]
    interval_ms: u64,
    
    /// 메시지 크기 (바이트)
    #[arg(short = 'm', long, default_value_t = 1048576)]
    message_size: usize,
    
    /// 전송할 샘플 개수
    #[arg(short = 's', long, default_value_t = 1000)]
    sample_count: u32,
}

#[tokio::main]
async fn main() {
    // 명령줄 인자 파싱
    let args = Args::parse();
    println!("시작 설정:");
    println!("  - 간격: {}ms", args.interval_ms);
    println!("  - 메시지 크기: {}바이트", args.message_size);
    println!("  - 샘플 수: {}", args.sample_count);

    // Zenoh 설정
    let config = zenoh::config::Config::default();
    
    // Zenoh 세션 생성
    let session = zenoh::open(config).await.expect("Failed to open Zenoh session");

    // 발행자 선언
    let publisher = session.declare_publisher("zenoh_benchmark").await.expect("Failed to declare publisher");

    // 메시지 데이터 생성 (바이트 배열)
    let message_data = vec![0u8; args.message_size];
    
    // 메시지 발행 카운터
    let mut count = 0;
    
    loop {
        let start = Instant::now();
        
        // 메시지 발행
        publisher.put(message_data.clone()).await.expect("Failed to publish message");
        
        count += 1;
        println!("Published message {} with size {}", count, message_data.len());
        
        // 지정된 샘플 수에 도달하면 종료
        if count >= args.sample_count {
            // 빈 메시지 전송
            let empty_data = vec![0u8; 0];
            publisher.put(empty_data).await.expect("Failed to publish empty message");
            println!("Done publishing {} messages, sent empty message, exiting...", args.sample_count);
            break;
        }
        
        // 다음 메시지 발행까지 대기
        let elapsed = start.elapsed();
        let interval = Duration::from_millis(args.interval_ms);
        
        if elapsed < interval {
            sleep(interval - elapsed).await;
        }
    }
}
