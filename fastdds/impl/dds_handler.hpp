//
// Created by waxz on 4/17/24.
//

#ifndef MESSAGE_CENTER_BUILD_DDS_HANDLER_H
#define MESSAGE_CENTER_BUILD_DDS_HANDLER_H


#include <thread>
#include <vector>
#include <memory>
#include <unordered_map>

#include "common/string_logger.h"
//DDS
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/DomainParticipantListener.hpp>


#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>

#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>


#include "dds_message_convert.hpp"

#include "config/dds_builder_config_gen.hpp"

namespace dds_helper {

    class PubListener : public eprosima::fastdds::dds::DataWriterListener {
    public:

        PubListener() = default;

        ~PubListener() override = default;

        void on_publication_matched(
                eprosima::fastdds::dds::DataWriter *writer,
                const eprosima::fastdds::dds::PublicationMatchedStatus &info) override;

        int matched = 0;
    };


    class SubListener : public eprosima::fastdds::dds::DataReaderListener {
    public:

        SubListener() = default;

        ~SubListener() override = default;

        void on_data_available(
                eprosima::fastdds::dds::DataReader *reader) override;

        void on_subscription_matched(
                eprosima::fastdds::dds::DataReader *reader,
                const eprosima::fastdds::dds::SubscriptionMatchedStatus &info) override;

        int matched = 0;
        uint32_t samples = 0;
    };

    class ParticipantListener
            : public eprosima::fastdds::dds::DomainParticipantListener {
    public:

        ParticipantListener() {
        }

        ~ParticipantListener() override {
        }

        void on_data_available(
                eprosima::fastdds::dds::DataReader *reader) override;

        void on_subscription_matched(
                eprosima::fastdds::dds::DataReader *reader,
                const eprosima::fastdds::dds::SubscriptionMatchedStatus &info) override;


        //! Callback executed when a DomainParticipant is discovered, dropped or removed
        void on_participant_discovery(
                eprosima::fastdds::dds::DomainParticipant * /*participant*/,
                eprosima::fastrtps::rtps::ParticipantDiscoveryInfo &&info) override;

        void on_subscriber_discovery(
                eprosima::fastdds::dds::DomainParticipant *participant,
                eprosima::fastrtps::rtps::ReaderDiscoveryInfo &&info) override;

        void on_publisher_discovery(
                eprosima::fastdds::dds::DomainParticipant *participant,
                eprosima::fastrtps::rtps::WriterDiscoveryInfo &&info) override;

        void on_type_discovery(
                eprosima::fastdds::dds::DomainParticipant *participant,
                const eprosima::fastrtps::rtps::SampleIdentity &request_sample_id,
                const eprosima::fastrtps::string_255 &topic,
                const eprosima::fastrtps::types::TypeIdentifier *identifier,
                const eprosima::fastrtps::types::TypeObject *object,
                eprosima::fastrtps::types::DynamicType_ptr dyn_type) override;

        void on_type_information_received(
                eprosima::fastdds::dds::DomainParticipant *participant,
                const eprosima::fastrtps::string_255 topic_name,
                const eprosima::fastrtps::string_255 type_name,
                const eprosima::fastrtps::types::TypeInformation &type_information) override;

        void on_type_dependencies_reply(
                eprosima::fastdds::dds::DomainParticipant *participant,
                const eprosima::fastrtps::rtps::SampleIdentity &request_sample_id,
                const eprosima::fastrtps::types::TypeIdentifierWithSizeSeq &dependencies) override;


        eprosima::fastrtps::types::DynamicType_ptr received_type_;

        std::atomic<bool> reception_flag_{false};


    };

#if 0
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
#endif

    struct DdsSimpleParticipant {

        DdsSimpleParticipant(const DdsSimpleParticipant &rhv);

        ParticipantListener listener_;
        std::shared_ptr<eprosima::fastdds::dds::DomainParticipant> participant_ = nullptr;


        std::vector<std::string> subscriber_list;

        DdsSimpleParticipant() = default;

        eprosima::fastdds::dds::DomainParticipantQos participant_qos;// = eprosima::fastdds::dds::PARTICIPANT_QOS_DEFAULT;
        ParticipantConfig config_;

        int init(const ParticipantConfig &config);

        void delete_topic(eprosima::fastdds::dds::Topic *topic);

        std::unordered_map<std::string, eprosima::fastdds::dds::Topic *> topic_list_;

        eprosima::fastdds::dds::Topic *create_topic(const std::string &topic_name,
                                                    const std::string &type_name,
                                                    const eprosima::fastdds::dds::TopicQos &qos,
                                                    eprosima::fastdds::dds::TopicListener *listener = nullptr);

        void delete_subscriber(eprosima::fastdds::dds::Subscriber *subscriber);

        eprosima::fastdds::dds::Subscriber *create_subscriber(const char *profile);

        ~ DdsSimpleParticipant();
    };

    template<typename T, typename S>
    struct DdsSimpleWriter {
        std::shared_ptr<DdsSimpleParticipant> participant_;
        eprosima::fastdds::dds::Topic *topic_;
        eprosima::fastdds::dds::Publisher *publisher_;
        eprosima::fastdds::dds::DataWriter *writer_;
        eprosima::fastdds::dds::TypeSupport type_;
        WriterConfig config_;
        std::shared_ptr<T> data_;
//        thread_local static T data_;

//        DdsSimpleWriter(int history_depth, const std::shared_ptr<DdsSimpleParticipant>& paticipant, const char* topic_name , const char* topic_profile , const char* pub_profile, const char * writer_profile);
        DdsSimpleWriter(const std::shared_ptr<DdsSimpleParticipant> &participant, const WriterConfig &config);

#if 0
        DdsSimpleWriter(DdsSimpleWriter&& rhv);
#endif

        ~DdsSimpleWriter();

        int write_data(void **buffer, size_t buffer_size);

        int write_data_dynamic(void **buffer, size_t buffer_size);

//        T& get_data(){
//            return DdsSimpleWriter::data_;
//        }
    };

    template<typename T, typename S>
    DdsSimpleWriter<T, S>::~DdsSimpleWriter() {
        std::cout << "~DdsSimpleWriter try drop " << std::endl;

        if ((writer_ != nullptr) && (publisher_ != nullptr)) {
            publisher_->delete_datawriter(writer_);
        }
        if (topic_ != nullptr) {
            participant_->participant_->delete_topic(topic_);
        }
        if (publisher_ != nullptr) {
//            participant_->participant_->delete_publisher(publisher_);
        }

    }

    template<typename T, typename S>
    int DdsSimpleWriter<T, S>::write_data_dynamic(void **buffer, size_t buffer_size) {
        if (!data_) {
            data_ = std::make_shared<T>();
        }
        int rt = 0;
        for (size_t i = 0; i < buffer_size; ++i) {
            rt = to_dds(data_.get(), buffer[i]);
            if (rt == 0) {
                writer_->write(data_.get());
            }
        }
        return rt;
    }

    template<typename T, typename S>
    int DdsSimpleWriter<T, S>::write_data(void **buffer, size_t buffer_size) {

        int rt = 0;
        if (type_.is_plain()) {

            for (size_t i = 0; i < buffer_size; ++i) {
                void *sample = nullptr;

                if (ReturnCode_t::RETCODE_OK
                    == writer_->loan_sample(
                        sample,
                        eprosima::fastdds::dds::DataWriter::LoanInitializationKind::NO_LOAN_INITIALIZATION)) {
                    // initialize and send the sample
                    T *data = static_cast<T *>(sample);
                    rt = to_dds(data, buffer[i]);


                    if ((rt == 0) && (!writer_->write(sample))) {
                        writer_->discard_loan(sample);
                    }
                }

            }
        } else {
            rt = write_data_dynamic(buffer, buffer_size);

        }
        return rt;

    }

#if 0
    template<typename T, typename S>
    DdsSimpleWriter<T,S>::DdsSimpleWriter(DdsSimpleWriter&& rhv)
    : participant_(std::move (rhv.participant_)), type_(std::move(rhv.type_)),config_(std::move(rhv.config_)){

        std::cout << "DdsSimpleWriter move constructor" << std::endl;
    }
#endif

    template<typename T, typename S>
    DdsSimpleWriter<T, S>::DdsSimpleWriter(const std::shared_ptr<DdsSimpleParticipant> &participant,
                                           const WriterConfig &config)
            : participant_(participant), type_(new S()), config_(config) {

        int history_depth = config_.qos_queue_size;

        //REGISTER THE TYPE
        auto type_rt = type_.register_type(participant_->participant_.get());

        if (ReturnCode_t::RETCODE_OK != type_rt) {
            throw std::runtime_error("DdsSimpleWriter register type_ failed");
        }

        //CREATE THE TOPIC
        eprosima::fastdds::dds::TopicQos topic_qos = eprosima::fastdds::dds::TOPIC_QOS_DEFAULT;
        if (!config_.topic_profile.empty())
            participant->participant_->get_topic_qos_from_profile(config_.topic_profile, topic_qos);


        topic_ = participant->create_topic(config_.topic_name,
                                           type_.get_type_name(),
                                           topic_qos);
        if (topic_ == nullptr) {
            throw std::runtime_error("DdsSimpleWriter create topic failed");

        }

        //CREATE THE PUBLISHER
        eprosima::fastdds::dds::PublisherQos pub_qos = eprosima::fastdds::dds::PUBLISHER_QOS_DEFAULT;
        if (!config_.pub_profile.empty())
            participant_->participant_->get_publisher_qos_from_profile(config_.pub_profile, pub_qos);
        publisher_ = participant_->participant_->create_publisher(pub_qos);
        if (publisher_ == nullptr) {
            throw std::runtime_error("DdsSimpleWriter create_publisher failed");

        }




        // CREATE THE WRITER
        eprosima::fastdds::dds::DataWriterQos writer_qos = eprosima::fastdds::dds::DATAWRITER_QOS_DEFAULT;
        if (!config_.writer_profile.empty())
            publisher_->get_datawriter_qos_from_profile(config_.writer_profile, writer_qos);

        writer_qos.history().depth = history_depth;
        writer_qos.data_sharing().automatic("/tmp/");

        writer_ = publisher_->create_datawriter(topic_, writer_qos);
        if (writer_ == nullptr) {
            throw std::runtime_error("DdsSimpleWriter create_publisher failed");

        }

    }

#if 0
    template<typename T, typename S>
    DdsSimpleWriter<T,S>::DdsSimpleWriter(int history_depth, const std::shared_ptr<DdsSimpleParticipant>& paticipant, const char* topic_name , const char* topic_profile , const char* pub_profile, const char * writer_profile)
            : participant_(paticipant), type_(new S())
    {

        //REGISTER THE TYPE
        type_.register_type(participant_->participant_.get());


        //CREATE THE PUBLISHER
        eprosima::fastdds::dds::PublisherQos pub_qos = eprosima::fastdds::dds::PUBLISHER_QOS_DEFAULT;
        participant_->participant_->get_publisher_qos_from_profile(pub_profile, pub_qos);
        publisher_ = participant_->participant_->create_publisher(pub_qos);
        if (publisher_ == nullptr)
        {
            throw std::runtime_error("create_publisher failed");

        }

        //CREATE THE TOPIC
        eprosima::fastdds::dds::TopicQos topic_qos = eprosima::fastdds::dds::TOPIC_QOS_DEFAULT;

        paticipant->participant_->get_topic_qos_from_profile(topic_profile, topic_qos);

        topic_ = paticipant->participant_->create_topic(
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

        writer_qos.history().depth = history_depth;

        writer_ = publisher_->create_datawriter(topic_, writer_qos);
        if (writer_ == nullptr)
        {
            throw std::runtime_error("create_publisher failed");

        }


    }
#endif


    template<typename T, typename S>
    struct DdsSimpleReader {

        std::shared_ptr<DdsSimpleParticipant> participant_;
        eprosima::fastdds::dds::Subscriber *subscriber_;
        eprosima::fastdds::dds::Topic *topic_;
        eprosima::fastdds::dds::DataReader *reader_;
        eprosima::fastdds::dds::TypeSupport type_;
        FASTDDS_CONST_SEQUENCE(DataSeq, T);
        DataSeq dataseq_;
        MemPoolHandler mem_pool_;
        ChannelBuffer channel_buffer;

        eprosima::fastdds::dds::SampleInfoSeq infoseq_;
        ReaderConfig config_;

//        DdsSimpleReader( ta_cfg_t* cfg  , int history_depth ,const std::shared_ptr<DdsSimpleParticipant>& participant, const char* topic_name , const char* topic_profile , const char* sub_profile, const char * reader_profile);
        DdsSimpleReader(ta_cfg_t *cfg, const std::shared_ptr<DdsSimpleParticipant> &participant,
                        const ReaderConfig &config);

        ~DdsSimpleReader();

        ChannelBuffer_ptr read_data();

    };


    template<typename T, typename S>
    DdsSimpleReader<T, S>::DdsSimpleReader(ta_cfg_t *cfg, const std::shared_ptr<DdsSimpleParticipant> &participant,
                                           const ReaderConfig &config)
            : participant_(participant), type_(new S()), config_(config) {

        int history_depth = config_.qos_queue_size;


        mem_pool_.cfg = *cfg;

        //REGISTER THE TYPE
        auto type_rt = type_.register_type(participant_->participant_.get());

        if (ReturnCode_t::RETCODE_OK != type_rt) {
            throw std::runtime_error("DdsSimpleReader register type_ failed");
        }

        //CREATE THE TOPIC
        eprosima::fastdds::dds::TopicQos topic_qos = eprosima::fastdds::dds::TOPIC_QOS_DEFAULT;
        if (!config_.topic_profile.empty())
            participant_->participant_->get_topic_qos_from_profile(config_.topic_profile, topic_qos);
        topic_ = participant->create_topic(config_.topic_name,
                                           type_.get_type_name(),
                                           topic_qos);
        if (topic_ == nullptr) {
            throw std::runtime_error("DdsSimpleReader create topic failed");
        }


        //CREATE THE SUBSCRIBER
        eprosima::fastdds::dds::SubscriberQos sub_qos = eprosima::fastdds::dds::SUBSCRIBER_QOS_DEFAULT;
        if (!config_.sub_profile.empty())
            participant_->participant_->get_subscriber_qos_from_profile(config_.sub_profile, sub_qos);
        subscriber_ = participant_->participant_->create_subscriber(sub_qos);
        if (subscriber_ == nullptr) {
            throw std::runtime_error("DdsSimpleReader create_subscriber failed");

        }






        //CREATE READER
        eprosima::fastdds::dds::DataReaderQos reader_qos = eprosima::fastdds::dds::DATAREADER_QOS_DEFAULT;
        if (!config_.reader_profile.empty())

            subscriber_->get_datareader_qos_from_profile(config_.reader_profile, reader_qos);

        reader_qos.history().depth = history_depth;
        reader_qos.data_sharing().automatic("/tmp/");

        reader_ = subscriber_->create_datareader(topic_, reader_qos);


        if (reader_ == nullptr) {
            throw std::runtime_error("DdsSimpleReader create_subscriber failed");
        }

    }


    template<typename T, typename S>
    ChannelBuffer_ptr DdsSimpleReader<T, S>::read_data() {

        eprosima::fastrtps::Duration_t timeout(0.001);

        mem_pool_.count = 0;


//        MLOGI("read data %i\n",0);
        if (reader_->wait_for_unread_message(timeout)) {
            if (ReturnCode_t::RETCODE_OK ==
                reader_->take(dataseq_, infoseq_, 2, eprosima::fastdds::dds::NOT_READ_SAMPLE_STATE)) {
//                MLOGI("read infoseq_.length() %u \n",infoseq_.length());

                for (eprosima::fastdds::dds::LoanableCollection::size_type i = 0; i < infoseq_.length(); ++i) {
//                    MLOGI("read infoseq_[i].valid_data %i \n",infoseq_[i].valid_data);

                    if (infoseq_[i].valid_data) {

                        // Print your structure data here.
                        const T &sample = dataseq_[i];

                        from_dds(sample, &mem_pool_);

//                        MLOGI("read mem_pool_.count %i \n",mem_pool_.count);

                    }
                }
                reader_->return_loan(dataseq_, infoseq_);
            }

        }
        if (mem_pool_.count == 0) {

            return nullptr;
        } else {
            channel_buffer.buffer = mem_pool_.buffer.data();
            channel_buffer.buffer_size = mem_pool_.count;
            return &channel_buffer;
        }


        return nullptr;
    }

    template<typename T, typename S>
    DdsSimpleReader<T, S>::~DdsSimpleReader() {
        std::cout << "~DdsSimpleReader try drop " << std::endl;

        if ((reader_ != nullptr) && (subscriber_ != nullptr)) {
            subscriber_->delete_datareader(reader_);
        }
        if (topic_ != nullptr) {
            participant_->delete_topic(topic_);
        }
        if (subscriber_ != nullptr) {
//            participant_->delete_subscriber(subscriber_);
        }
    }


}


#endif //MESSAGE_CENTER_BUILD_DDS_HANDLER_H
