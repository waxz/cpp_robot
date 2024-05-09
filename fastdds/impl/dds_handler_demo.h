//
// Created by waxz on 4/17/24.
//

#ifndef CMAKE_SUPER_BUILD_DDS_HANDLER_DEMO_H
#define CMAKE_SUPER_BUILD_DDS_HANDLER_DEMO_H


#include <thread>
#include <vector>
#include <memory>
#include <unordered_map>

//DDS
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/DomainParticipantListener.hpp>
#include <fastrtps/types/DynamicDataHelper.hpp>


#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>

#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>

class PubListener : public eprosima::fastdds::dds::DataWriterListener
{
public:

    PubListener() = default;

    ~PubListener() override = default;

    void on_publication_matched(
            eprosima::fastdds::dds::DataWriter* writer,
            const eprosima::fastdds::dds::PublicationMatchedStatus& info) override;

    int matched = 0;
};
void PubListener::on_publication_matched(
        eprosima::fastdds::dds::DataWriter*,
        const eprosima::fastdds::dds::PublicationMatchedStatus& info)
{
    if (info.current_count_change == 1)
    {
        matched = info.total_count;
        std::cout << "DataWriter matched." << std::endl;
    }
    else if (info.current_count_change == -1)
    {
        matched = info.total_count;
        std::cout << "DataWriter unmatched." << std::endl;
    }
    else
    {
        std::cout << info.current_count_change
                  << " is not a valid value for PublicationMatchedStatus current count change" << std::endl;
    }
}

class SubListener : public eprosima::fastdds::dds::DataReaderListener
{
public:

    SubListener() = default;

    ~SubListener() override = default;

    void on_data_available(
            eprosima::fastdds::dds::DataReader* reader) override;

    void on_subscription_matched(
            eprosima::fastdds::dds::DataReader* reader,
            const eprosima::fastdds::dds::SubscriptionMatchedStatus& info) override;

    int matched = 0;
    uint32_t samples = 0;
};
void SubListener::on_data_available(
        eprosima::fastdds::dds::DataReader* reader){

    static_cast<void>(reader);
}

void SubListener::on_subscription_matched(
        eprosima::fastdds::dds::DataReader* reader,
        const eprosima::fastdds::dds::SubscriptionMatchedStatus& info){

    static_cast<void>(reader);

    if (info.current_count_change == 1)
    {
        matched = info.total_count;
        std::cout << "Subscriber matched." << std::endl;
    }
    else if (info.current_count_change == -1)
    {
        matched = info.total_count;
        std::cout << "Subscriber unmatched." << std::endl;
    }
    else
    {
        std::cout << info.current_count_change
                  << " is not a valid value for SubscriptionMatchedStatus current count change" << std::endl;
    }
}
class ParticipantListener
        :  public eprosima::fastdds::dds::DomainParticipantListener
{
public:

    ParticipantListener()
    {
    }

    ~ParticipantListener() override
    {
    }

    void on_data_available(
            eprosima::fastdds::dds::DataReader* reader) override;

    void on_subscription_matched(
            eprosima::fastdds::dds::DataReader* reader,
            const eprosima::fastdds::dds::SubscriptionMatchedStatus& info) override;



    //! Callback executed when a DomainParticipant is discovered, dropped or removed
    void on_participant_discovery(
            eprosima::fastdds::dds::DomainParticipant* /*participant*/,
            eprosima::fastrtps::rtps::ParticipantDiscoveryInfo&& info) override;

    void on_subscriber_discovery(
            eprosima::fastdds::dds::DomainParticipant* participant,
            eprosima::fastrtps::rtps::ReaderDiscoveryInfo&& info)override;
    void on_publisher_discovery(
            eprosima::fastdds::dds::DomainParticipant* participant,
            eprosima::fastrtps::rtps::WriterDiscoveryInfo&& info)override;

    void on_type_discovery(
            eprosima::fastdds::dds::DomainParticipant* participant,
            const eprosima::fastrtps::rtps::SampleIdentity& request_sample_id,
            const eprosima::fastrtps::string_255& topic,
            const eprosima::fastrtps::types::TypeIdentifier* identifier,
            const eprosima::fastrtps::types::TypeObject* object,
            eprosima::fastrtps::types::DynamicType_ptr dyn_type) override;

    void on_type_information_received(
            eprosima::fastdds::dds::DomainParticipant* participant,
            const eprosima::fastrtps::string_255 topic_name,
            const eprosima::fastrtps::string_255 type_name,
            const eprosima::fastrtps::types::TypeInformation& type_information)override;

    void on_type_dependencies_reply(
            eprosima::fastdds::dds::DomainParticipant* participant,
            const eprosima::fastrtps::rtps::SampleIdentity& request_sample_id,
            const eprosima::fastrtps::types::TypeIdentifierWithSizeSeq& dependencies)override;


    eprosima::fastrtps::types::DynamicType_ptr received_type_;

    std::atomic<bool> reception_flag_{false};


};


struct DdsSimpleHandler{

    DdsSimpleHandler(const DdsSimpleHandler& handler);

    ParticipantListener listener_;
    std::shared_ptr<eprosima::fastdds::dds::DomainParticipant>  participant_;


    std::vector<std::string> subscriber_list;

    DdsSimpleHandler(){}
    bool init(
//            const char* custom_name ,
              const char* xml,
              const char* profile);

    void delete_topic(eprosima::fastdds::dds::Topic* topic);
    void delete_subscriber(eprosima::fastdds::dds::Subscriber* subscriber);

    eprosima::fastdds::dds::Subscriber* create_subscriber(const char* profile);
    ~ DdsSimpleHandler();
};

template<typename T, typename S>
struct DdsSimpleWriter{
    DdsSimpleHandler handler_;
    eprosima::fastdds::dds::Subscriber* subscriber_;
    eprosima::fastdds::dds::Topic* topic_;
    eprosima::fastdds::dds::Publisher* publisher_;
    eprosima::fastdds::dds::DataWriter* writer_;
    eprosima::fastdds::dds::TypeSupport  type_;

    DdsSimpleWriter(const DdsSimpleHandler& handler, const char* topic_name , const char* topic_profile , const char* pub_profile, const char * writer_profile);
    ~DdsSimpleWriter( );
    void write(void** buffer, size_t buffer_size);

};

template<typename T, typename S>
DdsSimpleWriter<T,S>::DdsSimpleWriter(const DdsSimpleHandler& handler, const char* topic_name , const char* topic_profile , const char* pub_profile, const char * writer_profile)
        : handler_(handler), type_(new S())
{

    //REGISTER THE TYPE
    type_.register_type(handler_.participant_.get());


    //CREATE THE PUBLISHER
    eprosima::fastdds::dds::PublisherQos pub_qos = eprosima::fastdds::dds::PUBLISHER_QOS_DEFAULT;
    handler_.participant_->get_publisher_qos_from_profile(pub_profile, pub_qos);
    publisher_ = handler_.participant_->create_publisher(pub_qos);
    if (publisher_ == nullptr)
    {
        throw std::runtime_error("create_publisher failed");

    }

    //CREATE THE TOPIC
    eprosima::fastdds::dds::TopicQos topic_qos = eprosima::fastdds::dds::TOPIC_QOS_DEFAULT;

    handler.participant_->get_topic_qos_from_profile(topic_profile,topic_qos);

    topic_ = handler.participant_->create_topic(
            topic_name,
            type_.get_type_name(),
            topic_qos);
    if (topic_ == nullptr)
    {
        throw std::runtime_error("create_publisher failed");
    }


    // CREATE THE WRITER
    eprosima::fastdds::dds::DataWriterQos writer_qos = eprosima::fastdds::dds::DATAWRITER_QOS_DEFAULT;
    publisher_->get_datawriter_qos_from_profile( writer_profile,writer_qos);


    writer_ = publisher_->create_datawriter(topic_, writer_qos);
    if (writer_ == nullptr)
    {
        throw std::runtime_error("create_publisher failed");

    }


}


template<typename T, typename S>
struct DdsSimpleReader{
    DdsSimpleHandler handler_;
    eprosima::fastdds::dds::Subscriber* subscriber_;
    eprosima::fastdds::dds::Topic* topic_;
    eprosima::fastdds::dds::DataReader* reader_;
    eprosima::fastdds::dds::TypeSupport type_;

    DdsSimpleReader(const DdsSimpleHandler& handler, const char* topic_name , const char* topic_profile , const char* sub_profile, const char * reader_profile);
    ~DdsSimpleReader( );

};




template<typename T, typename S>
DdsSimpleReader<T,S>::DdsSimpleReader(const DdsSimpleHandler& handler, const char* topic_name, const char* topic_profile  , const char* sub_profile, const char * reader_profile)
: handler_(handler), type_(new S())
{

    //REGISTER THE TYPE
    type_.register_type(handler_.participant_.get());


    //CREATE THE SUBSCRIBER
    eprosima::fastdds::dds::SubscriberQos sub_qos = eprosima::fastdds::dds::SUBSCRIBER_QOS_DEFAULT;
    handler.participant_->get_subscriber_qos_from_profile(sub_profile,sub_qos);
    subscriber_ =  handler.participant_->create_subscriber(sub_qos);
    if (subscriber_ == nullptr)
    {
        throw std::runtime_error("create_subscriber failed");

    }

    //CREATE THE TOPIC
    eprosima::fastdds::dds::TopicQos topic_qos = eprosima::fastdds::dds::TOPIC_QOS_DEFAULT;

    handler.participant_->get_topic_qos_from_profile(topic_profile,topic_qos);

    topic_ = handler.participant_->create_topic(
            topic_name,
            type_.get_type_name(),
            topic_qos);
    if (topic_ == nullptr)
    {
        throw std::runtime_error("create_subscriber failed");
    }

    //CREATE READER
    eprosima::fastdds::dds::DataReaderQos reader_qos = eprosima::fastdds::dds::DATAREADER_QOS_DEFAULT;
    subscriber_->get_datareader_qos_from_profile(reader_profile,reader_qos);
    reader_ = subscriber_->create_datareader(topic_, reader_qos);


    if (reader_ == nullptr)
    {
        throw std::runtime_error("create_subscriber failed");
    }
}
template<typename T, typename S>
DdsSimpleReader<T,S>::~DdsSimpleReader(){
    if (reader_ != nullptr)
    {
        if (subscriber_)
        subscriber_->delete_datareader(reader_);
    }
    if (topic_ != nullptr)
    {
        handler_.delete_topic(topic_);
    }
    if (subscriber_ != nullptr)
    {
        handler_.delete_subscriber(subscriber_);
    }
}



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

void ParticipantListener::on_type_dependencies_reply(
        eprosima::fastdds::dds::DomainParticipant* participant,
        const eprosima::fastrtps::rtps::SampleIdentity& request_sample_id,
        const eprosima::fastrtps::types::TypeIdentifierWithSizeSeq& dependencies){
    std::cout << "on_type_dependencies_reply ";
    static_cast<void>(participant);
    static_cast<void>(request_sample_id);
    static_cast<void>(dependencies);


}
void ParticipantListener::on_type_information_received(
        eprosima::fastdds::dds::DomainParticipant* participant,
        const eprosima::fastrtps::string_255 topic_name,
        const eprosima::fastrtps::string_255 type_name,
        const eprosima::fastrtps::types::TypeInformation& type_information){
    std::cout << "on_type_information_received ";
    static_cast<void>(participant);
    static_cast<void>(topic_name);
    static_cast<void>(type_name);
    static_cast<void>(type_information);

}
void ParticipantListener::on_publisher_discovery(
        eprosima::fastdds::dds::DomainParticipant* participant,
        eprosima::fastrtps::rtps::WriterDiscoveryInfo&& info){
    static_cast<void>(participant);

    std::cout << "on_publisher_discovery ";

    if ( info.status == eprosima::fastrtps::rtps::WriterDiscoveryInfo::DISCOVERY_STATUS::DISCOVERED_WRITER)
    {
        std::cout << "DISCOVERED_WRITER ";
    }
    else if ( info.status == eprosima::fastrtps::rtps::WriterDiscoveryInfo::DISCOVERY_STATUS::CHANGED_QOS_WRITER)
    {
        std::cout << "CHANGED_QOS_WRITER ";

    }
    else if ( info.status == eprosima::fastrtps::rtps::WriterDiscoveryInfo::DISCOVERY_STATUS::REMOVED_WRITER)
    {
        std::cout << "REMOVED_WRITER ";

    }


    std::cout << "WRITER guid" << info.info.guid() << ", topicName: " << info.info.topicName() << ", typeName: " << info.info.typeName() << ", topicKind: "<< info.info.topicKind()  << std::endl;

}
void ParticipantListener::on_subscriber_discovery(
        eprosima::fastdds::dds::DomainParticipant* participant,
        eprosima::fastrtps::rtps::ReaderDiscoveryInfo&& info)
{
    static_cast<void>(participant);

    std::cout << "on_subscriber_discovery ";

    if ( info.status == eprosima::fastrtps::rtps::ReaderDiscoveryInfo::DISCOVERED_READER)
    {
        std::cout << "DISCOVERED_READER ";
    }
    else if ( info.status == eprosima::fastrtps::rtps::ReaderDiscoveryInfo::CHANGED_QOS_READER)
    {
        std::cout << "CHANGED_QOS_READER ";

    }
    else if ( info.status == eprosima::fastrtps::rtps::ReaderDiscoveryInfo::REMOVED_READER)
    {
        std::cout << "REMOVED_READER ";

    }
    std::cout << "READER guid" << info.info.guid() << ", topicName: " << info.info.topicName()    << ", typeName: " << info.info.typeName()<< ", topicKind: "<< info.info.topicKind()  << std::endl;

}
void ParticipantListener::on_subscription_matched(
        eprosima::fastdds::dds:: DataReader*,
        const eprosima::fastdds::dds::SubscriptionMatchedStatus& info)
{
    if (info.current_count_change == 1)
    {
        std::cout << "Subscriber matched" << std::endl;
    }
    else if (info.current_count_change == -1)
    {
        std::cout << "Subscriber unmatched" << std::endl;
    }
    else
    {
        std::cout << info.current_count_change
                  << " is not a valid value for SubscriptionMatchedStatus current count change" << std::endl;
    }
}

void ParticipantListener::on_data_available(
        eprosima::fastdds::dds::DataReader* reader)
{

    static_cast<void>(reader);

}

void ParticipantListener::on_type_discovery(
        eprosima::fastdds::dds::DomainParticipant*,
        const eprosima::fastrtps::rtps::SampleIdentity&,
        const eprosima::fastrtps::string_255& topic_name,
        const eprosima::fastrtps::types::TypeIdentifier*,
        const eprosima::fastrtps::types::TypeObject*,
        eprosima::fastrtps::types::DynamicType_ptr dyn_type)
{
    std::cout << "Discovered type: " << dyn_type->get_name() << " from topic " << topic_name << std::endl;
}

void  ParticipantListener::on_participant_discovery(
        eprosima::fastdds::dds::DomainParticipant* /*participant*/,
        eprosima::fastrtps::rtps::ParticipantDiscoveryInfo&& info)
{
    std::cout << "on_participant_discovery:";
    if (info.status == eprosima::fastrtps::rtps::ParticipantDiscoveryInfo::DISCOVERED_PARTICIPANT)
    {
        std::cout << "Discovered ";
    }
    else if (info.status == eprosima::fastrtps::rtps::ParticipantDiscoveryInfo::DROPPED_PARTICIPANT ||
             info.status == eprosima::fastrtps::rtps::ParticipantDiscoveryInfo::REMOVED_PARTICIPANT)
    {
        std::cout << "Dropped ";

    }
    std::cout << "Participant GUID " << info.info.m_guid << ", Name: "<< info.info.m_participantName << std::endl;

}


#endif //CMAKE_SUPER_BUILD_DDS_HANDLER_DEMO_H
