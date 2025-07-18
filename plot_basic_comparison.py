import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
from datetime import datetime

def load_timestamp_data(csv_path):
    """CSV 파일에서 타임스탬프 데이터를 로드"""
    try:
        df = pd.read_csv(csv_path)
        # 타임스탬프를 datetime으로 변환 후 Unix 타임스탬프(밀리초)로 변환
        timestamps = pd.to_datetime(df['Timestamp'])
        # Unix 타임스탬프(초)로 변환 후 밀리초로 변환
        timestamps_ms = (timestamps.astype('int64') // 1000000).values  # 나노초를 밀리초로 변환
        return timestamps_ms
    except Exception as e:
        print(f"Error loading {csv_path}: {e}")
        return None

def calculate_inter_arrival_times(timestamps):
    """타임스탬프로부터 inter-arrival time 계산 (단위: 밀리초)"""
    if len(timestamps) < 2:
        return np.array([])
    
    # 타임스탬프 차이를 계산 (밀리초 단위)
    inter_arrivals = np.diff(timestamps)
    return inter_arrivals

def calculate_messages_per_second(timestamps):
    """초당 메시지 수 계산"""
    if len(timestamps) == 0:
        return np.array([]), np.array([])
    
    # 타임스탬프를 초 단위로 변환
    timestamps_sec = timestamps / 1000.0
    
    # 시작과 끝 시간
    start_time = int(np.floor(timestamps_sec[0]))
    end_time = int(np.ceil(timestamps_sec[-1]))
    
    # 각 초별로 메시지 수 계산
    time_bins = np.arange(start_time, end_time + 1)
    message_counts = []
    
    for i in range(len(time_bins) - 1):
        count = np.sum((timestamps_sec >= time_bins[i]) & (timestamps_sec < time_bins[i + 1]))
        message_counts.append(count)
    
    # 중간점을 시간축으로 사용
    time_points = time_bins[:-1] + 0.5
    
    return time_points, np.array(message_counts)

def plot_inter_arrival_times(data_dict, save_path):
    """Inter-arrival time 그래프 생성"""
    plt.figure(figsize=(12, 8))
    
    colors = ['blue', 'red', 'green']
    middleware_names = list(data_dict.keys())
    
    for i, (middleware, timestamps) in enumerate(data_dict.items()):
        if timestamps is not None and len(timestamps) > 1:
            inter_arrivals = calculate_inter_arrival_times(timestamps)
            if len(inter_arrivals) > 0:
                # x축은 메시지 번호
                x_axis = range(1, len(inter_arrivals) + 1)
                plt.plot(x_axis, inter_arrivals, 
                        color=colors[i], 
                        label=f'{middleware}', 
                        alpha=0.7, 
                        linewidth=1)
    
    plt.xlabel('Message Number')
    plt.ylabel('Inter-arrival Time (ms)')
    plt.title('Inter-arrival Times Comparison Across Middleware')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    
    # 그래프 저장
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"Inter-arrival times graph saved to: {save_path}")
    plt.close()

def plot_messages_per_second(data_dict, save_path):
    """초당 메시지 수 그래프 생성"""
    plt.figure(figsize=(12, 8))
    
    colors = ['blue', 'red', 'green']
    middleware_names = list(data_dict.keys())
    
    for i, (middleware, timestamps) in enumerate(data_dict.items()):
        if timestamps is not None and len(timestamps) > 0:
            time_points, message_counts = calculate_messages_per_second(timestamps)
            if len(time_points) > 0:
                # 시작 시간을 0으로 normalize
                normalized_time = time_points - time_points[0]
                plt.plot(normalized_time, message_counts, 
                        color=colors[i], 
                        label=f'{middleware}', 
                        alpha=0.7, 
                        linewidth=2,
                        marker='o',
                        markersize=3)
    
    plt.xlabel('Time (seconds)')
    plt.ylabel('Messages per Second')
    plt.title('Messages per Second Comparison Across Middleware')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    
    # 그래프 저장
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"Messages per second graph saved to: {save_path}")
    plt.close()

def main():
    # CSV 파일 경로 설정
    csv_files = {
        'FastDDS': 'rt/fastdds_timestamp.csv',
        'ROS2': 'rt/ros2_timestamp.csv',
        'Zenoh': 'rt/zenoh_timestamp.csv'
    }
    
    # 현재 스크립트 디렉토리 기준으로 경로 설정
    base_dir = os.path.dirname(os.path.abspath(__file__))
    
    # 데이터 로드
    data_dict = {}
    for middleware, csv_path in csv_files.items():
        full_path = os.path.join(base_dir, csv_path)
        if os.path.exists(full_path):
            timestamps = load_timestamp_data(full_path)
            data_dict[middleware] = timestamps
            if timestamps is not None:
                print(f"Loaded {len(timestamps)} timestamps from {middleware}")
        else:
            print(f"Warning: CSV file not found for {middleware}: {full_path}")
            # 테스트를 위한 더미 데이터 생성
            print(f"Generating dummy data for {middleware}")
            base_time = 1000000  # 시작 시간 (밀리초)
            num_messages = 1000
            # 각 미들웨어마다 다른 패턴의 더미 데이터
            if middleware == 'FastDDS':
                intervals = np.random.exponential(10, num_messages-1)  # 평균 10ms 간격
            elif middleware == 'ROS2':
                intervals = np.random.normal(15, 3, num_messages-1)  # 평균 15ms, 표준편차 3ms
            else:  # Zenoh
                intervals = np.random.gamma(2, 5, num_messages-1)  # 감마 분포
            
            intervals = np.maximum(intervals, 1)  # 최소 1ms 간격
            timestamps = np.cumsum(np.concatenate([[base_time], intervals]))
            data_dict[middleware] = timestamps
    
    # 그래프 생성 및 저장
    if data_dict:
        # Inter-arrival times 그래프
        inter_arrival_path = os.path.join(base_dir, 'inter_arrival_times.png')
        plot_inter_arrival_times(data_dict, inter_arrival_path)
        
        # Messages per second 그래프
        messages_per_sec_path = os.path.join(base_dir, 'messages_per_second.png')
        plot_messages_per_second(data_dict, messages_per_sec_path)
        
        # 통계 정보 출력
        print("\n=== Statistics Summary ===")
        for middleware, timestamps in data_dict.items():
            if timestamps is not None and len(timestamps) > 1:
                inter_arrivals = calculate_inter_arrival_times(timestamps)
                if len(inter_arrivals) > 0:
                    print(f"\n{middleware}:")
                    print(f"  Total messages: {len(timestamps)}")
                    print(f"  Average inter-arrival time: {np.mean(inter_arrivals):.2f} ms")
                    print(f"  Std dev inter-arrival time: {np.std(inter_arrivals):.2f} ms")
                    print(f"  Min inter-arrival time: {np.min(inter_arrivals):.2f} ms")
                    print(f"  Max inter-arrival time: {np.max(inter_arrivals):.2f} ms")
                    
                    # 평균 메시지 레이트 계산
                    total_time = (timestamps[-1] - timestamps[0]) / 1000.0  # 초 단위
                    avg_rate = len(timestamps) / total_time if total_time > 0 else 0
                    print(f"  Average message rate: {avg_rate:.2f} messages/sec")
    else:
        print("No data loaded from any CSV files.")

if __name__ == "__main__":
    main()