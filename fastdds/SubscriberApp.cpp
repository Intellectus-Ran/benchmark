// Copyright 2024 Proyectos y Sistemas de Mantenimiento SL (Intellectus).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file SubscriberApp.cpp
 *
 */

#include "SubscriberApp.hpp"

#include <condition_variable>
#include <fstream>
#include <stdexcept>
#include <thread>

#include <fastdds/dds/core/status/SubscriptionMatchedStatus.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/dds/subscriber/qos/SubscriberQos.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/rtps/transport/shared_mem/SharedMemTransportDescriptor.hpp>
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.hpp>
#include <fastdds/rtps/transport/UDPv6TransportDescriptor.hpp>

#include "Application.hpp"
#include "CLIParser.hpp"
#include "ConfigurationPubSubTypes.hpp"

using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::rtps;

namespace eprosima {
namespace fastdds {
namespace examples {
namespace configuration {

SubscriberApp::SubscriberApp(
        const CLIParser::subscriber_config& config)
    : participant_(nullptr)
    , subscriber_(nullptr)
    , topic_(nullptr)
    , reader_(nullptr)
    , type_(new ConfigurationPubSubType())
    , received_samples_(0)
    , samples_(config.samples)
    , stop_(false)
    , total_bytes_(0)
{
    csv_file_.open("fastdds_timestamp.csv", std::ios::app);
    if (csv_file_.tellp() == 0) // 파일이 비어있으면 헤더 작성
    {
        csv_file_ << "Timestamp\n";
    }

    // Create the participant
    DomainParticipantQos pqos = PARTICIPANT_QOS_DEFAULT;
    pqos.name("Configuration_sub_participant");

    // 버퍼 사이즈 5MB로 설정
    pqos.transport().listen_socket_buffer_size = 5242880;

    auto factory = DomainParticipantFactory::get_instance();
    if (config.profile_participant.empty())
    {
        // Include Participant QoS
        pqos.setup_transports(config.transport);
        for (auto& transportDescriptor : pqos.transport().user_transports)
        {
            SocketTransportDescriptor* pT = dynamic_cast<SocketTransportDescriptor*>(transportDescriptor.get());
            if (pT)
            {
                pT->TTL = config.ttl;
            }
        }
        participant_ = factory->create_participant(config.domain, pqos);
    }
    else
    {
        participant_ = factory->create_participant_with_profile(config.profile_participant);
    }
    if (participant_ == nullptr)
    {
        throw std::runtime_error("Participant initialization failed");
    }

    // Register the type
    type_.register_type(participant_);

    // Create the subscriber
    SubscriberQos sub_qos = SUBSCRIBER_QOS_DEFAULT;
    participant_->get_default_subscriber_qos(sub_qos);
    if (!config.partitions.empty())
    {
        std::stringstream partitions(config.partitions);
        std::string partition;
        while (std::getline(partitions, partition, ';'))
        {
            sub_qos.partition().push_back(partition.c_str());
        }
    }
    subscriber_ = participant_->create_subscriber(sub_qos, nullptr, StatusMask::none());
    if (subscriber_ == nullptr)
    {
        throw std::runtime_error("Subscriber initialization failed");
    }

    // Create the topic
    TopicQos topic_qos = TOPIC_QOS_DEFAULT;
    participant_->get_default_topic_qos(topic_qos);
    topic_ = participant_->create_topic(config.topic_name, type_.get_type_name(), topic_qos);
    if (topic_ == nullptr)
    {
        throw std::runtime_error("Topic initialization failed");
    }

    // Create the data reader
    if (config.profile_reader.empty())
    {
        DataReaderQos reader_qos = DATAREADER_QOS_DEFAULT;
        subscriber_->get_default_datareader_qos(reader_qos);
        reader_qos.reliability().kind = config.reliability;
        reader_qos.durability().kind = config.durability;
        reader_qos.history().kind = config.history_kind;
        reader_qos.history().depth = config.history_depth;
        reader_qos.resource_limits().max_samples = 2147483647;
        reader_qos.resource_limits().max_instances = 2147483647;
        reader_qos.resource_limits().max_samples_per_instance = 2147483647;
        reader_qos.ownership().kind = config.ownership;
        if (config.deadline > 0)
        {
            reader_qos.deadline().period = eprosima::fastdds::dds::Duration_t(config.deadline * 1e-3);
        }
        reader_qos.reliable_reader_qos().disable_positive_acks.enabled = config.disable_positive_ack;
        if (config.lifespan > 0)
        {
            reader_qos.lifespan().duration = eprosima::fastdds::dds::Duration_t(config.lifespan * 1e-3);
        }
        reader_qos.liveliness().kind = config.liveliness;
        if (config.liveliness_lease > 0)
        {
            reader_qos.liveliness().lease_duration = eprosima::fastdds::dds::Duration_t(config.liveliness_lease * 1e-3);
        }
        if (config.liveliness_assert > 0)
        {
            reader_qos.liveliness().announcement_period = eprosima::fastdds::dds::Duration_t(
                config.liveliness_assert * 1e-3);
        }
        reader_ = subscriber_->create_datareader(topic_, reader_qos, this, StatusMask::all());
    }
    else
    {
        reader_ = subscriber_->create_datareader_with_profile(topic_, config.profile_reader, this, StatusMask::all());
    }
    if (reader_ == nullptr)
    {
        throw std::runtime_error("DataReader initialization failed");
    }
}

SubscriberApp::~SubscriberApp()
{
    if (nullptr != participant_)
    {
        // Delete DDS entities contained within the DomainParticipant
        participant_->delete_contained_entities();

        // Delete DomainParticipant
        DomainParticipantFactory::get_instance()->delete_participant(participant_);
    }
}

void SubscriberApp::on_subscription_matched(
        DataReader* /*reader*/,
        const SubscriptionMatchedStatus& info)
{
    if (info.current_count_change == 1)
    {
        std::cout << "Subscriber matched." << std::endl;
    }
    else if (info.current_count_change == -1)
    {
        std::cout << "Subscriber unmatched." << std::endl;
    }
    else
    {
        std::cout << info.current_count_change
                  << " is not a valid value for SubscriptionMatchedStatus current count change" << std::endl;
    }
}

std::string get_timestamp_with_ms() {
    using namespace std::chrono;

    auto now = system_clock::now();
    auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;

    auto time_t_now = system_clock::to_time_t(now);
    std::tm local_tm = *std::localtime(&time_t_now);

    std::ostringstream oss;
    oss << std::put_time(&local_tm, "%Y-%m-%d %H:%M:%S");
    oss << '.' << std::setfill('0') << std::setw(3) << ms.count();

    return oss.str();
}

void SubscriberApp::on_data_available(
        DataReader* reader)
{
    SampleInfo info;

    while ((!is_stopped()) && (RETCODE_OK == reader->take_next_sample(&configuration_, &info)))
    {
            // 시작시간 기록
            if (received_samples_ == 0) {
                start_time_ = std::chrono::system_clock::now();
            }

            // 수신 바이트 누적
            total_bytes_ += configuration_.data().size();

            // CSV에 수신 시간 기록
            std::string timestamp = get_timestamp_with_ms();
            csv_file_ << timestamp << "\n";

            // 빈 배열인 경우 수신 종료
            if (configuration_.data().size() == 0) {
                std::cout << "Publisher done publishing (received empty message)" << std::endl;

                // 총 수신 갯수
                std::cout << "Total received: " << received_samples_ << std::endl;

                // 총 수신 시간
                end_time_ = std::chrono::system_clock::now();
                auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(end_time_ - start_time_).count();
                std::cout << "Total time (us): " << duration_us << std::endl;
                
                // 초당 처리량
                double duration_sec = duration_us / 1'000'000.0;
                std::cout << "Throughput per sec: " << static_cast<double>(total_bytes_) / duration_sec << std::endl;

                // 메시지 유실률
                std::cout << "Loss Rate: " << static_cast<double>(samples_ - received_samples_) / samples_ << std::endl;

                stop();
                break;
            }

            received_samples_++;

            // Print Hello world message data
            std::cout << "Received message with [" << configuration_.data().size() << " bytes] (" 
                      << received_samples_ << " / " << samples_ << ")" << std::endl;
    }
}

void SubscriberApp::on_requested_deadline_missed(
        DataReader* /*reader*/,
        const RequestedDeadlineMissedStatus& /*status*/)
{
    std::cout << "Requested deadline missed!" << std::endl;
}

void SubscriberApp::on_liveliness_changed(
        DataReader* /*reader*/,
        const LivelinessChangedStatus& /*status*/)
{
    std::cout << "Liveliness changed!" << std::endl;
}

void SubscriberApp::on_sample_rejected(
        DataReader* /*reader*/,
        const SampleRejectedStatus& /*status*/)
{
    std::cout << "Sample rejected!" << std::endl;
}

void SubscriberApp::on_requested_incompatible_qos(
        DataReader* /*reader*/,
        const RequestedIncompatibleQosStatus& /*status*/)
{
    std::cout << "Incompatible QoS detected!" << std::endl;
}

void SubscriberApp::on_sample_lost(
        DataReader* /*reader*/,
        const SampleLostStatus& /*status*/)
{
    std::cout << "Sample lost!" << std::endl;
}

void SubscriberApp::run()
{
    std::unique_lock<std::mutex> lock_(mutex_);
    cv_.wait(lock_, [&]
            {
                return is_stopped();
            });
}

bool SubscriberApp::is_stopped()
{
    return stop_.load();
}

void SubscriberApp::stop()
{
    stop_.store(true);
    cv_.notify_all();
}

} // namespace configuration
} // namespace examples
} // namespace fastdds
} // namespace eprosima
