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
    """Inter-arrival time 그래프 생성 (nonrt vs rt 비교)"""
    plt.figure(figsize=(14, 10))
    
    # 색상 정의
    colors = {
        'ROS2-NonRT': 'orange',
        'ROS2-RT': 'red',
        'FastDDS-NonRT': 'blue',
        'FastDDS-RT': 'skyblue',
        'Zenoh-NonRT': 'green',
        'Zenoh-RT': 'yellowgreen'
    }
    
    for middleware_version, timestamps in data_dict.items():
        if timestamps is not None and len(timestamps) > 1:
            inter_arrivals = calculate_inter_arrival_times(timestamps)
            if len(inter_arrivals) > 0:
                # x축은 메시지 번호
                x_axis = range(1, len(inter_arrivals) + 1)
                plt.plot(x_axis, inter_arrivals, 
                        color=colors.get(middleware_version, 'black'), 
                        label=f'{middleware_version}', 
                        alpha=0.7, 
                        linewidth=1.5)
    
    plt.xlabel('Message Number', fontsize=12)
    plt.ylabel('Inter-arrival Time (ms)', fontsize=12)
    plt.title('Inter-arrival Times Comparison: NonRT vs RT Across Middleware', fontsize=14)
    plt.legend(fontsize=10, loc='upper right')
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    
    # 그래프 저장
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"Inter-arrival times comparison graph saved to: {save_path}")
    plt.close()

def plot_messages_per_second(data_dict, save_path):
    """초당 메시지 수 그래프 생성 (nonrt vs rt 비교)"""
    plt.figure(figsize=(14, 10))
    
    # 색상 정의
    colors = {
        'ROS2-NonRT': 'red',
        'ROS2-RT': 'orange',
        'FastDDS-NonRT': 'blue',
        'FastDDS-RT': 'skyblue',
        'Zenoh-NonRT': 'green',
        'Zenoh-RT': 'yellowgreen'
    }
    
    for middleware_version, timestamps in data_dict.items():
        if timestamps is not None and len(timestamps) > 0:
            time_points, message_counts = calculate_messages_per_second(timestamps)
            if len(time_points) > 0:
                # 시작 시간을 0으로 normalize
                normalized_time = time_points - time_points[0]
                plt.plot(normalized_time, message_counts, 
                        color=colors.get(middleware_version, 'black'), 
                        label=f'{middleware_version}', 
                        alpha=0.7, 
                        linewidth=2,
                        marker='o',
                        markersize=4)
    
    plt.xlabel('Time (seconds)', fontsize=12)
    plt.ylabel('Messages per Second', fontsize=12)
    plt.title('Messages per Second Comparison: NonRT vs RT Across Middleware', fontsize=14)
    plt.legend(fontsize=10, loc='upper right')
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    
    # 그래프 저장
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"Messages per second comparison graph saved to: {save_path}")
    plt.close()

def main():
    # CSV 파일 경로 설정 (NonRT와 RT 버전)
    csv_files = {
        'FastDDS-NonRT': 'nonrt/fastdds_timestamp.csv',
        'FastDDS-RT': 'rt/fastdds_timestamp.csv',
        'ROS2-NonRT': 'nonrt/ros2_timestamp.csv',
        'ROS2-RT': 'rt/ros2_timestamp.csv',
        'Zenoh-NonRT': 'nonrt/zenoh_timestamp.csv',
        'Zenoh-RT': 'rt/zenoh_timestamp.csv'
    }
    
    # 현재 스크립트 디렉토리 기준으로 경로 설정
    base_dir = os.path.dirname(os.path.abspath(__file__))
    
    # 데이터 로드
    data_dict = {}
    for middleware_version, csv_path in csv_files.items():
        full_path = os.path.join(base_dir, csv_path)
        if os.path.exists(full_path):
            timestamps = load_timestamp_data(full_path)
            data_dict[middleware_version] = timestamps
            if timestamps is not None:
                print(f"Loaded {len(timestamps)} timestamps from {middleware_version}")
        else:
            print(f"Warning: CSV file not found for {middleware_version}: {full_path}")
            # 테스트를 위한 더미 데이터 생성
            print(f"Generating dummy data for {middleware_version}")
            base_time = 1000000  # 시작 시간 (밀리초)
            num_messages = 1000
            
            # 각 미들웨어와 버전마다 다른 패턴의 더미 데이터
            if 'FastDDS' in middleware_version:
                if 'NonRT' in middleware_version:
                    intervals = np.random.exponential(12, num_messages-1)  # NonRT: 더 큰 변동성
                else:  # RT
                    intervals = np.random.exponential(8, num_messages-1)   # RT: 더 일정한 간격
            elif 'ROS2' in middleware_version:
                if 'NonRT' in middleware_version:
                    intervals = np.random.normal(18, 5, num_messages-1)    # NonRT: 더 큰 표준편차
                else:  # RT
                    intervals = np.random.normal(12, 2, num_messages-1)    # RT: 더 작은 표준편차
            else:  # Zenoh
                if 'NonRT' in middleware_version:
                    intervals = np.random.gamma(2, 7, num_messages-1)      # NonRT: 더 큰 변동성
                else:  # RT
                    intervals = np.random.gamma(3, 4, num_messages-1)      # RT: 더 일정한 패턴
            
            intervals = np.maximum(intervals, 0.5)  # 최소 0.5ms 간격
            timestamps = np.cumsum(np.concatenate([[base_time], intervals]))
            data_dict[middleware_version] = timestamps
    
    # 그래프 생성 및 저장
    if data_dict:
        # Inter-arrival times 그래프
        inter_arrival_path = os.path.join(base_dir, 'inter_arrival_times_nonrt_vs_rt.png')
        plot_inter_arrival_times(data_dict, inter_arrival_path)
        
        # Messages per second 그래프
        messages_per_sec_path = os.path.join(base_dir, 'messages_per_second_nonrt_vs_rt.png')
        plot_messages_per_second(data_dict, messages_per_sec_path)
        
        # 통계 정보 출력
        print("\n=== Statistics Summary (NonRT vs RT) ===")
        for middleware_version, timestamps in data_dict.items():
            if timestamps is not None and len(timestamps) > 1:
                inter_arrivals = calculate_inter_arrival_times(timestamps)
                if len(inter_arrivals) > 0:
                    print(f"\n{middleware_version}:")
                    print(f"  Total messages: {len(timestamps)}")
                    print(f"  Average inter-arrival time: {np.mean(inter_arrivals):.2f} ms")
                    print(f"  Std dev inter-arrival time: {np.std(inter_arrivals):.2f} ms")
                    print(f"  Min inter-arrival time: {np.min(inter_arrivals):.2f} ms")
                    print(f"  Max inter-arrival time: {np.max(inter_arrivals):.2f} ms")
                    
                    # 평균 메시지 레이트 계산
                    total_time = (timestamps[-1] - timestamps[0]) / 1000.0  # 초 단위
                    avg_rate = len(timestamps) / total_time if total_time > 0 else 0
                    print(f"  Average message rate: {avg_rate:.2f} messages/sec")
                    
                    # Jitter 계산 (inter-arrival time의 표준편차)
                    jitter = np.std(inter_arrivals)
                    print(f"  Jitter (std dev): {jitter:.2f} ms")
        
        # NonRT vs RT 비교 요약
        print("\n=== NonRT vs RT Comparison Summary ===")
        middleware_types = ['FastDDS', 'ROS2', 'Zenoh']
        
        for middleware in middleware_types:
            nonrt_key = f"{middleware}-NonRT"
            rt_key = f"{middleware}-RT"
            
            if nonrt_key in data_dict and rt_key in data_dict:
                nonrt_timestamps = data_dict[nonrt_key]
                rt_timestamps = data_dict[rt_key]
                
                if nonrt_timestamps is not None and rt_timestamps is not None:
                    if len(nonrt_timestamps) > 1 and len(rt_timestamps) > 1:
                        nonrt_inter_arrivals = calculate_inter_arrival_times(nonrt_timestamps)
                        rt_inter_arrivals = calculate_inter_arrival_times(rt_timestamps)
                        
                        nonrt_jitter = np.std(nonrt_inter_arrivals)
                        rt_jitter = np.std(rt_inter_arrivals)
                        
                        nonrt_avg = np.mean(nonrt_inter_arrivals)
                        rt_avg = np.mean(rt_inter_arrivals)
                        
                        print(f"\n{middleware}:")
                        print(f"  NonRT Jitter: {nonrt_jitter:.2f} ms, RT Jitter: {rt_jitter:.2f} ms")
                        print(f"  NonRT Avg: {nonrt_avg:.2f} ms, RT Avg: {rt_avg:.2f} ms")
                        print(f"  Jitter Improvement (RT vs NonRT): {((nonrt_jitter - rt_jitter) / nonrt_jitter * 100):.1f}%")
    else:
        print("No data loaded from any CSV files.")

if __name__ == "__main__":
    main()