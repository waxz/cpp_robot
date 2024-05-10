//
// Created by waxz on 10/19/23.
//

#include "ros_helper_impl.hpp"


// string
#include "absl/strings/str_format.h"
#include "absl/strings/escaping.h"
#include "absl/strings/string_view.h"
#include "absl/strings/match.h"
#include "absl/strings/str_split.h"


// log
#include "absl/log/log.h"
#include "absl/log/check.h"

// common
//#include "ros_helper_message.h"
#include "common/string_logger.h"



//ROSTFReader
ROSTFReader::ROSTFReader(std::shared_ptr<tf::TransformListener> tfl) :m_tfl(tfl){}
int ROSTFReader::create(const ros_helper::Channel& channel){
    std::vector<std::string> parts = absl::StrSplit(channel.topic_type, ':');

    if (parts.size() != 2){
        LOG(INFO) << "topic_type: [" << channel.topic_type  << "]" ;
        return -1;
    }

    parent_frame = parts[0];
    child_frame = parts[1];

    if(parent_frame.empty() || child_frame.empty()){
        LOG(INFO) << "parent_frame: [" << parent_frame<<"], child_frame: [" << child_frame << "]" ;
        return -1;
    }


//    sample = PoseStampedT_create();
    target.frame_id_.assign(parent_frame);
    target.child_frame_id_.assign(child_frame);
    return 0;

}

//int ROSTFReader::create_from_toml(const toml::basic_value<toml::discard_comments> &config) {
//    // get parent frame and child frame
//    // get wait time
//    // init pool
//
//    std::string topic_type = toml::find_or(config,m_config_key_topic_type,"");
//
//    std::vector<std::string> parts = absl::StrSplit(topic_type, ':');
//
//    if (parts.size() != 2){
//        LOG(INFO) << "topic_type: [" << topic_type  << "]" ;
//        return -1;
//    }
//
//    parent_frame = parts[0];
//    child_frame = parts[1];
//
//    if(parent_frame.empty() || child_frame.empty()){
//        LOG(INFO) << "parent_frame: [" << parent_frame<<"], child_frame: [" << child_frame << "]" ;
//        return -1;
//    }
//
//
////    sample = PoseStampedT_create();
//    target.frame_id_.assign(parent_frame);
//    target.child_frame_id_.assign(child_frame);
//    return 0;
//}

//int ROSTFReader::process() {
//    tf::StampedTransform result;
//
//    if(wait_time_s > 0.0f){
//        m_tfl->waitForTransform(parent_frame, child_frame,ros::Time(0), ros::Duration(wait_time_s));
//    }
//
//
//    MemPool_reset_counter(&pool);
//    try{
//        ros_read_tf(m_tfl,target,&pool);
//        return 1;
//
//    }catch (tf::TransformException &ex) {
//        ROS_ERROR("%s", ex.what());
//        return -1;
//    }
//    return -1;
//}

ChannelBuffer_ptr ROSTFReader::read_data() {
    tf::StampedTransform result;

    if(wait_time_s > 0.0f){
        m_tfl->waitForTransform(parent_frame, child_frame,ros::Time(0), ros::Duration(wait_time_s));
    }

    try{
        pose = * ros_read_tf(m_tfl,target);
        buffer[0] = &pose;
        channel_buffer.buffer_size = 1;
        channel_buffer.buffer = buffer;
        return &channel_buffer;

    }catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        return 0;
    }
    return 0;
}

//ROSTFWriter
ROSTFWriter::ROSTFWriter(std::shared_ptr<tf::TransformBroadcaster> tfb) : m_tfb(tfb){}
int ROSTFWriter::create(const ros_helper::Channel& channel){

    std::vector<std::string> parts = absl::StrSplit(channel.topic_type, ':');

    if (parts.size() != 2){
        LOG(INFO) << "topic_type: [" << channel.topic_type  << "]" ;
        return -1;
    }
    // Output each part

    parent_frame = parts[0];
    child_frame = parts[1];

    if(parent_frame.empty() || child_frame.empty()){
        LOG(INFO) << "parent_frame: [" << parent_frame<<"], child_frame: [" << child_frame << "]" ;
        return -1;
    }



//    sample = PoseStampedT_create();
    target.frame_id_.assign(parent_frame);
    target.child_frame_id_.assign(child_frame);
    return 0;
}


int ROSTFWriter::write_data(void **buffer, u32_t buffer_len) {
    target.stamp_ = ros::Time::now();
    ros_write_tf_data(m_tfb, target, buffer,buffer_len);
    return 0;
}

//ROSWriter
ROSWriter::ROSWriter(): m_nh(), m_nh_private("~") {

}
int ROSWriter::create(const ros_helper::Channel& channel){
    std::strcpy(option.topic_type,channel.topic_type.c_str());
    std::strcpy(option.topic_name,channel.topic_name.c_str());
    option.queue_size = channel.qos_queue_size;
    option.keep_last = false;

    writer =  create_writer(m_nh,&option);
    if(!writer.valid){
        std::cout << "ROSWriter::create_from_toml: " << option.topic_type << " is not defined" << std::endl;
        return -1;
    }

    return 0;
}


int ROSWriter::write_data(void **buffer, u32_t buffer_len) {

    int ret = writer.write_data(writer.writer,buffer, buffer_len);
    return ret;
}

//ROSReader
ROSReader::ROSReader(ta_cfg_t * cfg): m_nh(), m_nh_private("~") {
    mem_pool_handler.cfg = *cfg;
}
int  ROSReader::create(const ros_helper::Channel& channel){

    std::strcpy(option.topic_type,channel.topic_type.c_str());
    std::strcpy(option.topic_name,channel.topic_name.c_str());
    option.queue_size =channel.qos_queue_size;

    option.mem_pool = &mem_pool_handler;

    queue = std::make_shared<ros::CallbackQueue>();

    m_nh.setCallbackQueue(queue.get());

    reader =  create_reader(m_nh,&option);
    if(!reader.valid){
        std::cout << "ROSReader::create_from_toml: " << option.topic_type << " is not defined" << std::endl;
        return -1;
    }



    return 0;
}


ChannelBuffer_ptr ROSReader::read_data() {

//    for(auto & p: mem_pool_handler.buffer){
//        ta_free(&mem_pool_handler.cfg,p);
//    }
//    mem_pool_handler.buffer.clear();

    mem_pool_handler.count = 0;

    queue->callAvailable(ros::WallDuration());

    if(mem_pool_handler.count == 0){
        return 0;
    }else{
        channel_buffer.buffer = mem_pool_handler.buffer.data();
        channel_buffer.buffer_size = mem_pool_handler.count;
        return &channel_buffer;
    }
    return 0;
}
int RosHandler::create(const char *filename,const ta_cfg_t* cfg) {
    if(!cfg){
        std::cout << "RosHandler::create: cfg" << cfg << std::endl;

        return -1;
    }
    mem_cfg = *cfg;

    std::cout << "RosHandler::create:" << filename << std::endl;
    toml::basic_value<toml::discard_comments, std::unordered_map, std::vector> toml_data;

    try{
        toml_data = toml::parse(filename);
    }catch (std::runtime_error & e){
        MLOGW("RosHandler::create:toml::parse(%s) failed, error: %s\n",filename, e.what());
        return -1;
    }catch (toml::syntax_error& e){
        MLOGW("RosHandler::create:toml::parse(%s) failed, error: %s\n",filename, e.what());
        return -1;
    }catch (...){
        MLOGW("RosHandler::create:toml::parse(%s) failed, error: %s\n",filename, "Unknown");
        return -1;
    }


    ros_helper::Config  config;

    try{
        config = ros_helper::Config(toml_data.at("ros"));
        toml::value config_data = config;
        std::cout << "config_data:\n" << config_data << "\n";
    }catch (std::exception& e){
        MLOGW("RosHandler::create:toml::parse(%s) failed, error: %s\n",filename, e.what());
        return -1;
    }




    if (!toml_data.contains(m_config_key_root)){
        MLOGW("%s does not contain %s", filename, m_config_key_root);
        return -1;
    }
    auto config_root = toml::find(toml_data, m_config_key_root);
    if (!config_root.contains(m_config_key_config)){
        MLOGW("%s does not contain %s", filename, m_config_key_config);
        return -1;
    }
    auto tom_config = toml::find(config_root, m_config_key_config);

    if(!tom_config.is_table()){
        MLOGW("%s does not contain %s as table", filename, m_config_key_config);
    }

    // ros.init
    MLOGI("%s","start process args\n");

    auto toml_config_table = tom_config.as_table();
    int argc = toml_config_table.size();

    MLOGI("%s argc = %i","start process ",argc);

    std::vector<char*> arg_vec(argc);

    std::vector<std::string> config_args(argc);
    char buffer[200];
    int arg_id = 0;

#if 0
    for(auto& it : toml_config_table){


        sprintf(buffer,"%s:=%s", it.first.c_str(),it.second.as_string().str.c_str());
        config_args[arg_id].assign(buffer);
        MLOGI("get arg: %s\n",buffer);
        arg_vec[arg_id] = (char*)(config_args[arg_id].data());
        arg_id++;
    }
#endif
    for(auto& it: config.config){
        sprintf(buffer,"%s:=%s", it.first.c_str(),it.second.c_str());
        config_args[arg_id].assign(buffer);
        MLOGI("get arg: %s\n",buffer);
        arg_vec[arg_id] = (char*)(config_args[arg_id].data());
        arg_id++;
    }


    char** argv = arg_vec.data();


    MLOGI("%s","ros::init\n");

    ros::init(argc,argv,"demo");

    // create tfl and tfb
    bool need_tfb = false, need_tfl = false;
    m_tfl = std::make_shared<tf::TransformListener>();
    m_tfb = std::make_shared<tf::TransformBroadcaster>();

    for(auto& channel: config.readers){
        auto& channel_name = channel.first;
        auto& channel_config = channel.second;
        const char* topic_name = channel_config.topic_name.c_str();
        if( channel_config.topic_name.empty() || channel_config.topic_type.empty() ){

            toml::value value =channel_config;
            std::cout << "Detect error in channel_config:\n" << value << std::endl;

            return -1;
        }

        if( std::strcmp(topic_name, m_config_predefine_topic_name_tf) ==0 ){
            need_tfl = true;

//                auto it = channel_holder_map.emplace(channel_name, ROSTFReader(m_tfl));
            auto it = channel_reader_map.emplace(channel_name, ROSTFReader(m_tfl));



            int ret = absl::get<ROSTFReader>(it.first->second).create(channel_config);

        }else{

//                auto it = channel_holder_map.emplace(channel_name, ROSReader(mem_pool_manager));
            auto it = channel_reader_map.emplace(channel_name, ROSReader(&mem_cfg));


            int ret = absl::get<ROSReader>(it.first->second).create(channel_config);

            if(ret < 0){
                return -1;
            }
        }



    }
    for(auto& channel: config.writers){
        auto& channel_name = channel.first;
        auto& channel_config = channel.second;
        const char* topic_name = channel_config.topic_name.c_str();
        if( channel_config.topic_name.empty() || channel_config.topic_type.empty() ){

            toml::value value =channel_config;
            std::cout << "Detect error in channel_config:\n" << value << std::endl;

            return -1;
        }

        if(std::strcmp(topic_name, m_config_predefine_topic_name_tf) ==0 ){
            need_tfb = true;

//                auto it = channel_holder_map.emplace(channel_name, ROSTFWriter(m_tfb,mem_pool_manager));

            auto it = channel_writer_map.emplace(channel_name, ROSTFWriter(m_tfb));


            int ret =  absl::get<ROSTFWriter>(it.first->second).create(channel_config);

        }else{

//                auto it = channel_holder_map.emplace(channel_name, ROSWriter(mem_pool_manager));
            auto it = channel_writer_map.emplace(channel_name, ROSWriter());


            int ret = absl::get<ROSWriter>(it.first->second).create(channel_config);

            if(ret < 0){
                return -1;
            }
        }



    }



    std::cout << "[RosHandler]: all channel is created done" << std::endl;


    if(!need_tfl){
        m_tfl.reset();
    }
    if(!need_tfb){
        m_tfb.reset();
    }
    return 0;
}

int RosHandler::stop() {
    std::cout << "RosHandler::stop" << std::endl;
    if(ros::isStarted()){
        ros::shutdown();
    }
    return 0;
}

int RosHandler::write_data(const char *channel_name, void **buffer, u32_t buffer_size) {

    if(buffer == nullptr || buffer_size == 0){
        return 0;
    }

    auto it = channel_writer_map.find(channel_name);
    if(it == channel_writer_map.end()){
        MLOGW("%s not exist\n", channel_name);
        return 0;
    }

    int ret = 0;
    absl::visit([buffer,buffer_size,&ret ](auto& x){
        ret = x.write_data(buffer, buffer_size);
    },it->second );
    return ret;

}

ChannelBuffer_ptr RosHandler::read_data(const char *channel_name) {

    auto it = channel_reader_map.find(channel_name);
    if(it == channel_reader_map.end()){
        MLOGW("%s not exist\n", channel_name);
        return 0;
    }

    ChannelBuffer_ptr ret = nullptr;
    absl::visit([&ret ](auto& x){
        ret = x.read_data();
    },it->second );
    return ret;
}

