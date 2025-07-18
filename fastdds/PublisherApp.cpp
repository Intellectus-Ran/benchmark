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
 * @file PublisherApp.cpp
 *
 */

#include "PublisherApp.hpp"

#include <algorithm>
#include <condition_variable>
#include <csignal>
#include <stdexcept>
#include <thread>
#include <sys/timerfd.h>

#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/log/Log.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>
#include <fastdds/dds/publisher/qos/PublisherQos.hpp>
#include <fastdds/rtps/transport/shared_mem/SharedMemTransportDescriptor.hpp>
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.hpp>
#include <fastdds/rtps/transport/UDPv6TransportDescriptor.hpp>
#include "ConfigurationPubSubTypes.hpp"

using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::rtps;


namespace eprosima {
    namespace fastdds {
        namespace examples {
            namespace configuration {
                PublisherApp::PublisherApp(
                        const CLIParser::publisher_config& config)
                    : participant_(nullptr)
                    , publisher_(nullptr)
                    , topic_(nullptr)
                    , writer_(nullptr)
                    , type_(new ConfigurationPubSubType())
                    , matched_(0)
                    , period_ms_(config.interval)
                    , samples_(config.samples)
                    , stop_(false)
                    , wait_(config.wait)
                {
                    // Set up the data type with initial values
                    configuration_.data(std::vector<uint8_t>(config.msg_size, 0xAA));

                    // Create the participant
                    DomainParticipantQos pqos = PARTICIPANT_QOS_DEFAULT;
                    pqos.name("Configuration_pub_participant");
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

                    // Create the publisher
                    PublisherQos pub_qos = PUBLISHER_QOS_DEFAULT;
                    participant_->get_default_publisher_qos(pub_qos);
                    if (!config.partitions.empty())
                    {
                        std::stringstream partitions(config.partitions);
                        std::string partition;
                        while (std::getline(partitions, partition, ';'))
                        {
                            pub_qos.partition().push_back(partition.c_str());
                        }
                    }
                    publisher_ = participant_->create_publisher(pub_qos, nullptr, StatusMask::none());
                    if (publisher_ == nullptr)
                    {
                        throw std::runtime_error("Publisher initialization failed");
                    }

                    // Create the topic
                    TopicQos topic_qos = TOPIC_QOS_DEFAULT;
                    participant_->get_default_topic_qos(topic_qos);
                    topic_ = participant_->create_topic(config.topic_name, type_.get_type_name(), topic_qos);
                    if (topic_ == nullptr)
                    {
                        throw std::runtime_error("Topic initialization failed");
                    }

                    // Create the data writer
                    if (config.profile_writer.empty())
                    {
                        DataWriterQos writer_qos = DATAWRITER_QOS_DEFAULT;
                        publisher_->get_default_datawriter_qos(writer_qos);
                        writer_qos.publish_mode().kind = config.publish_mode;
                        writer_qos.reliability().kind = config.reliability;
                        writer_qos.durability().kind = config.durability;
                        writer_qos.history().kind = config.history_kind;
                        writer_qos.history().depth = config.history_depth;
                        writer_qos.resource_limits().max_samples = 2147483647;
                        writer_qos.resource_limits().max_instances = 2147483647;
                        writer_qos.resource_limits().max_samples_per_instance = 2147483647;
                        writer_qos.ownership().kind = config.ownership;
                        if (config.ownership_strength > 0
                                && config.ownership != OwnershipQosPolicyKind::EXCLUSIVE_OWNERSHIP_QOS)
                        {
                            throw std::runtime_error(
                                      "DataWriter initialization failed: ownership strength is only valid with exclusive ownership");
                        }
                        writer_qos.ownership_strength().value = config.ownership_strength;
                        if (config.deadline > 0)
                        {
                            writer_qos.deadline().period = eprosima::fastdds::dds::Duration_t(config.deadline * 1e-3);
                        }
                        writer_qos.reliable_writer_qos().disable_positive_acks.enabled = config.disable_positive_ack;
                        if (config.ack_keep_duration > 0)
                        {
                            writer_qos.reliable_writer_qos().disable_positive_acks.duration = eprosima::fastdds::dds::Duration_t(
                                config.ack_keep_duration * 1e-3);
                        }
                        if (config.lifespan > 0)
                        {
                            writer_qos.lifespan().duration = eprosima::fastdds::dds::Duration_t(config.lifespan * 1e-3);
                        }
                        writer_qos.liveliness().kind = config.liveliness;
                        if (config.liveliness_lease > 0)
                        {
                            writer_qos.liveliness().lease_duration = eprosima::fastdds::dds::Duration_t(config.liveliness_lease * 1e-3);
                        }
                        if (config.liveliness_assert > 0)
                        {
                            writer_qos.liveliness().announcement_period = eprosima::fastdds::dds::Duration_t(
                                config.liveliness_assert * 1e-3);
                        }
                        writer_ = publisher_->create_datawriter(topic_, writer_qos, this, StatusMask::all());
                    }
                    else
                    {
                        writer_ = publisher_->create_datawriter_with_profile(topic_, config.profile_writer, this, StatusMask::all());
                    }
                    if (writer_ == nullptr)
                    {
                        throw std::runtime_error("DataWriter initialization failed");
                    }
                }

                PublisherApp::~PublisherApp()
                {
                    if (nullptr != participant_)
                    {
                        // Delete DDS entities contained within the DomainParticipant
                        participant_->delete_contained_entities();

                        // Delete DomainParticipant
                        DomainParticipantFactory::get_instance()->delete_participant(participant_);
                    }
                }

                void PublisherApp::on_publication_matched(
                        eprosima::fastdds::dds::DataWriter* /*writer*/,
                        const PublicationMatchedStatus& info)
                {
                    if (info.current_count_change == 1)
                    {
                        matched_ = info.current_count;
                        std::cout << "Publisher matched." << std::endl;
                        if (matched_ >= static_cast<int16_t>(wait_))
                        {
                            cv_.notify_one();
                        }
                    }
                    else if (info.current_count_change == -1)
                    {
                        matched_ = info.current_count;
                        std::cout << "Publisher unmatched." << std::endl;
                    }
                    else
                    {
                        std::cout << info.current_count_change
                                  << " is not a valid value for PublicationMatchedStatus current count change" << std::endl;
                    }
                }

                void PublisherApp::on_offered_deadline_missed(
                        eprosima::fastdds::dds::DataWriter* /*writer*/,
                        const OfferedDeadlineMissedStatus& /*status*/)
                {
                    std::cout << "Deadline missed!" << std::endl;
                }

                void PublisherApp::on_offered_incompatible_qos(
                        DataWriter* /*writer*/,
                        const OfferedIncompatibleQosStatus& /*status*/)
                {
                    std::cout << "Incompatible QoS detected!" << std::endl;
                }

                void PublisherApp::on_liveliness_lost(
                        DataWriter* /*writer*/,
                        const LivelinessLostStatus& /*status*/)
                {
                    std::cout << "Liveliness lost!" << std::endl;
                }

                void PublisherApp::on_unacknowledged_sample_removed(
                        DataWriter* /*writer*/,
                        const InstanceHandle_t& /*instance*/)
                {
                    std::cout << "Unacknowledged sample was removed!" << std::endl;
                }

                void PublisherApp::run()
                {
                    // Wait for the data endpoints discovery
                    std::unique_lock<std::mutex> matched_lock(mutex_);
                    cv_.wait(matched_lock, [&]()
                            {
                                // Publisher starts sending messages when enough entities have been discovered.
                                return ((matched_ >= static_cast<int16_t>(wait_)) || is_stopped());
                            });

                    // 타이머 파일 디스크립터 생성
                    int tfd = timerfd_create(CLOCK_MONOTONIC, 0);
                    if (tfd == -1) {
                        std::cerr << "timerfd_create 실패" << std::endl;
                        return;
                    }

                    // 타이머 주기 설정 (33ms 주기)
                    itimerspec timerSpec;
                    memset(&timerSpec, 0, sizeof(timerSpec));
                    timerSpec.it_value.tv_sec = 0;
                    timerSpec.it_value.tv_nsec = 1;  // 바로 시작
                    timerSpec.it_interval.tv_sec = 0;
                    timerSpec.it_interval.tv_nsec = 33000000;  // 33ms

                    if (timerfd_settime(tfd, 0, &timerSpec, NULL) == -1) {
                        std::cerr << "timerfd_settime 실패" << std::endl;
                        close(tfd);
                        return;
                    }

                    int index = 0;

                    while (!is_stopped() && ((samples_ == 0) || index < samples_))
                    {
                        uint64_t expirations;

                        // 타이머 대기 (블로킹)
                        ssize_t s = read(tfd, &expirations, sizeof(expirations));

                        if (s != sizeof(expirations)) {
                            std::cerr << "타이머 read 실패" << std::endl;
                            break;
                        }

                        if (publish())
                        {
                            std::cout << "Sample with index: "
                                      << index << " (" << static_cast<int>(configuration_.data().size())
                                      << " Bytes) SENT" << std::endl;
                        }

                        index++;
                    }
                    
                    // 모든 샘플을 발행한 후 빈 배열 전송
                    if (!is_stopped())
                    {
                        // 빈 배열 생성 및 전송
                        configuration_.data().resize(0);
                        if (RETCODE_OK == writer_->write(&configuration_))
                        {
                            std::cout << "Empty message sent to signal completion" << std::endl;
                        }
                    }
                }

                bool PublisherApp::publish()
                {
                    bool ret = false;
                    
                    ret = (RETCODE_OK == writer_->write(&configuration_));

                    return ret;
                }

                bool PublisherApp::is_stopped()
                {
                    return stop_.load();
                }

                void PublisherApp::stop()
                {
                    stop_.store(true);
                    cv_.notify_one();
                }
            }
        }
    }
}