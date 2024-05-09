//
// Created by waxz on 4/26/24.
//

#include "dds_handler.h"
#include "common/string_logger.h"
//#include <fastrtps/xmlparser/XMLProfileManager.h>
//#include <fastdds/rtps/transport/TCPv4TransportDescriptor.h>
//#include <fastdds/rtps/transport/UDPv4TransportDescriptor.h>
#include <fastrtps/types/DynamicDataHelper.hpp>

namespace dds_helper {


    void PubListener::on_publication_matched(
            eprosima::fastdds::dds::DataWriter *,
            const eprosima::fastdds::dds::PublicationMatchedStatus &info) {
        if (info.current_count_change == 1) {
            matched = info.total_count;
            std::cout << "DataWriter matched." << std::endl;
        } else if (info.current_count_change == -1) {
            matched = info.total_count;
            std::cout << "DataWriter unmatched." << std::endl;
        } else {
            std::cout << info.current_count_change
                      << " is not a valid value for PublicationMatchedStatus current count change" << std::endl;
        }
    }


    void SubListener::on_data_available(
            eprosima::fastdds::dds::DataReader *reader) {

        static_cast<void>(reader);
    }

    void SubListener::on_subscription_matched(
            eprosima::fastdds::dds::DataReader *reader,
            const eprosima::fastdds::dds::SubscriptionMatchedStatus &info) {

        static_cast<void>(reader);

        if (info.current_count_change == 1) {
            matched = info.total_count;
            std::cout << "Subscriber matched." << std::endl;
        } else if (info.current_count_change == -1) {
            matched = info.total_count;
            std::cout << "Subscriber unmatched." << std::endl;
        } else {
            std::cout << info.current_count_change
                      << " is not a valid value for SubscriptionMatchedStatus current count change" << std::endl;
        }
    }


    void ParticipantListener::on_type_dependencies_reply(
            eprosima::fastdds::dds::DomainParticipant *participant,
            const eprosima::fastrtps::rtps::SampleIdentity &request_sample_id,
            const eprosima::fastrtps::types::TypeIdentifierWithSizeSeq &dependencies) {
        std::cout << "on_type_dependencies_reply ";
        static_cast<void>(participant);
        static_cast<void>(request_sample_id);
        static_cast<void>(dependencies);


    }

    void ParticipantListener::on_type_information_received(
            eprosima::fastdds::dds::DomainParticipant *participant,
            const eprosima::fastrtps::string_255 topic_name,
            const eprosima::fastrtps::string_255 type_name,
            const eprosima::fastrtps::types::TypeInformation &type_information) {
        std::cout << "on_type_information_received ";
        static_cast<void>(participant);
        static_cast<void>(topic_name);
        static_cast<void>(type_name);
        static_cast<void>(type_information);

    }

    void ParticipantListener::on_publisher_discovery(
            eprosima::fastdds::dds::DomainParticipant *participant,
            eprosima::fastrtps::rtps::WriterDiscoveryInfo &&info) {
        static_cast<void>(participant);

        std::cout << "on_publisher_discovery ";

        if (info.status == eprosima::fastrtps::rtps::WriterDiscoveryInfo::DISCOVERY_STATUS::DISCOVERED_WRITER) {
            std::cout << "DISCOVERED_WRITER ";
        } else if (info.status == eprosima::fastrtps::rtps::WriterDiscoveryInfo::DISCOVERY_STATUS::CHANGED_QOS_WRITER) {
            std::cout << "CHANGED_QOS_WRITER ";

        } else if (info.status == eprosima::fastrtps::rtps::WriterDiscoveryInfo::DISCOVERY_STATUS::REMOVED_WRITER) {
            std::cout << "REMOVED_WRITER ";

        }


        std::cout << "WRITER guid" << info.info.guid() << ", topicName: " << info.info.topicName() << ", typeName: "
                  << info.info.typeName() << ", topicKind: " << info.info.topicKind() << std::endl;

    }

    void ParticipantListener::on_subscriber_discovery(
            eprosima::fastdds::dds::DomainParticipant *participant,
            eprosima::fastrtps::rtps::ReaderDiscoveryInfo &&info) {
        static_cast<void>(participant);

        std::cout << "on_subscriber_discovery ";

        if (info.status == eprosima::fastrtps::rtps::ReaderDiscoveryInfo::DISCOVERED_READER) {
            std::cout << "DISCOVERED_READER ";
        } else if (info.status == eprosima::fastrtps::rtps::ReaderDiscoveryInfo::CHANGED_QOS_READER) {
            std::cout << "CHANGED_QOS_READER ";

        } else if (info.status == eprosima::fastrtps::rtps::ReaderDiscoveryInfo::REMOVED_READER) {
            std::cout << "REMOVED_READER ";

        }
        std::cout << "READER guid" << info.info.guid() << ", topicName: " << info.info.topicName() << ", typeName: "
                  << info.info.typeName() << ", topicKind: " << info.info.topicKind() << std::endl;

    }

    void ParticipantListener::on_subscription_matched(
            eprosima::fastdds::dds::DataReader *,
            const eprosima::fastdds::dds::SubscriptionMatchedStatus &info) {
        if (info.current_count_change == 1) {
            std::cout << "Subscriber matched" << std::endl;
        } else if (info.current_count_change == -1) {
            std::cout << "Subscriber unmatched" << std::endl;
        } else {
            std::cout << info.current_count_change
                      << " is not a valid value for SubscriptionMatchedStatus current count change" << std::endl;
        }
    }

    void ParticipantListener::on_data_available(
            eprosima::fastdds::dds::DataReader *reader) {

        static_cast<void>(reader);

    }

    void ParticipantListener::on_type_discovery(
            eprosima::fastdds::dds::DomainParticipant *,
            const eprosima::fastrtps::rtps::SampleIdentity &,
            const eprosima::fastrtps::string_255 &topic_name,
            const eprosima::fastrtps::types::TypeIdentifier *,
            const eprosima::fastrtps::types::TypeObject *,
            eprosima::fastrtps::types::DynamicType_ptr dyn_type) {
        std::cout << "Discovered type: " << dyn_type->get_name() << " from topic " << topic_name << std::endl;
    }

    void ParticipantListener::on_participant_discovery(
            eprosima::fastdds::dds::DomainParticipant * /*participant*/,
            eprosima::fastrtps::rtps::ParticipantDiscoveryInfo &&info) {
        std::cout << "on_participant_discovery:";
        if (info.status == eprosima::fastrtps::rtps::ParticipantDiscoveryInfo::DISCOVERED_PARTICIPANT) {
            std::cout << "Discovered ";
        } else if (info.status == eprosima::fastrtps::rtps::ParticipantDiscoveryInfo::DROPPED_PARTICIPANT ||
                   info.status == eprosima::fastrtps::rtps::ParticipantDiscoveryInfo::REMOVED_PARTICIPANT) {
            std::cout << "Dropped ";

        }
        std::cout << "Participant GUID " << info.info.m_guid << ", Name: " << info.info.m_participantName << std::endl;

    }


    DdsSimpleParticipant::~DdsSimpleParticipant() {
        std::cout << "DdsSimpleHandler try drop: " << participant_ << ", use_count: " << participant_.use_count()
                  << std::endl;

    }

    void DdsSimpleParticipant::delete_topic(eprosima::fastdds::dds::Topic *topic) {
        if (participant_) {
            participant_->delete_topic(topic);
        }
    }

    eprosima::fastdds::dds::Topic *DdsSimpleParticipant::create_topic(const std::string &topic_name,
                                                                      const std::string &type_name,
                                                                      const eprosima::fastdds::dds::TopicQos &qos,
                                                                      eprosima::fastdds::dds::TopicListener *listener) {
        auto it = topic_list_.find(topic_name);
        if (it == topic_list_.end()) {

            auto rt = topic_list_.emplace(topic_name, participant_->create_topic(topic_name, type_name, qos, listener));
            it = rt.first;
        }
        MLOGI("create topic [%s], input: [%s], return: [%s]", topic_name.c_str(), type_name.c_str(),
              it->second->get_type_name().c_str());
        if (type_name == it->second->get_type_name()) {
            return it->second;
        } else {
            MLOGW("error topic [%s], input: [%s] is not equal to  return: [%s]", topic_name.c_str(), type_name.c_str(),
                  it->second->get_type_name().c_str());
            return 0;
        }
        return 0;
    }

    void DdsSimpleParticipant::delete_subscriber(eprosima::fastdds::dds::Subscriber *subscriber) {
        if (participant_) {
            participant_->delete_subscriber(subscriber);
        }
    }

    eprosima::fastdds::dds::Subscriber *DdsSimpleParticipant::create_subscriber(const char *profile) {

        eprosima::fastdds::dds::SubscriberQos sub_qos;
        participant_->get_subscriber_qos_from_profile(profile, sub_qos);
        eprosima::fastdds::dds::Subscriber *sub = participant_->create_subscriber(sub_qos);
        return sub;
    }


    int DdsSimpleParticipant::init(const ParticipantConfig &config) {

        auto &xml = config.xml;
        auto &profile = config.profile;
        auto &name = config.name;


        MLOGI("create participant: %s\n", name.c_str());
        participant_qos = eprosima::fastdds::dds::PARTICIPANT_QOS_DEFAULT;


        ReturnCode_t ret = ReturnCode_t::RETCODE_ERROR;
        if (!xml.empty() && !profile.empty()) {
            ret = eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->load_XML_profiles_file(xml);
        }
        if (ReturnCode_t::RETCODE_OK == ret) {
            auto load_rt = eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->get_participant_qos_from_profile(
                    profile,
                    participant_qos);
            if (ReturnCode_t::RETCODE_OK != load_rt) {
                MLOGW("load qos profile %s failed", profile.c_str());
            } else {
                MLOGI("load qos profile %s ok", profile.c_str());
            }

        } else {

            MLOGW("Open file %s failed, use default qos", xml.c_str());
        }

        {
            if (!name.empty()) {
                participant_qos.name(name);
            }
            eprosima::fastdds::dds::DomainParticipant *participant = eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->create_participant(
                    0, participant_qos, &listener_);
            participant_.reset(participant, [this](auto &p) {
                std::cout << "DdsSimpleHandler try delete_participant: " << p << std::endl;
                for (auto &t: topic_list_) {
                    participant_->delete_topic(t.second);
                }
                eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->delete_participant(p);
            });

        }
        return 0;

    }

    DdsSimpleParticipant::DdsSimpleParticipant(const DdsSimpleParticipant &handler) : participant_(
            handler.participant_) {

    }

#if 0

    DdsSimpleHandler::~DdsSimpleHandler() {
        std::cout << "DdsSimpleHandler try drop: " << participant_ << ", use_count: " << participant_.use_count() << std::endl;

    }

    void DdsSimpleHandler::delete_topic(eprosima::fastdds::dds::Topic* topic){
        if (participant_){
            participant_->delete_topic(topic);
        }
    }
    void DdsSimpleHandler::delete_subscriber(eprosima::fastdds::dds::Subscriber* subscriber){
        if (participant_){
            participant_->delete_subscriber(subscriber);
        }
    }
    eprosima::fastdds::dds::Subscriber* DdsSimpleHandler::create_subscriber(const char* profile){

        eprosima::fastdds::dds::SubscriberQos sub_qos;
        participant_->get_subscriber_qos_from_profile(profile,sub_qos);
        eprosima::fastdds::dds::Subscriber* sub =  participant_->create_subscriber(sub_qos);
        return sub;
    }


    bool DdsSimpleHandler::init(
//        const char* custom_name ,
            const char* xml,
            const char* profile){





        auto ret = eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->load_XML_profiles_file(xml);
        if (ReturnCode_t::RETCODE_OK == ret)
        {


            eprosima::fastdds::dds::DomainParticipantQos participant_qos = eprosima::fastdds::dds::PARTICIPANT_QOS_DEFAULT;
            eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->get_participant_qos_from_profile(
                    profile,
                    participant_qos);

            // Name obtained in another section of the code
//        participant_qos.name() = custom_name;


//        participant_ =  eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->create_participant_with_profile(   0, profile,&listener_);
            eprosima::fastdds::dds::DomainParticipant* participant =  eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->create_participant(
                    0, participant_qos, &listener_);
            participant_.reset(participant,[](auto& p){
                std::cout << "DdsSimpleHandler try delete_participant: " << p   << std::endl;

                eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->delete_participant(p);
            });

        }else{
            std::cout << "Load " << xml << ", " << profile << "Failed" << std::endl;
        }
        return ReturnCode_t::RETCODE_OK == ret;

    }

    DdsSimpleHandler::DdsSimpleHandler(const DdsSimpleHandler& handler):participant_(handler.participant_) {

    }
#endif


}
