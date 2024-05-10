//
// Created by waxz on 5/8/24.
//

#include "dds_helper.h"
#include "dds_handler_variant.hpp"
#include "common/string_logger.h"

bool dds_create(message_handler_ptr_t h, const char *filename, const ta_cfg_t *cfg);

int dds_write_data(message_handler_ptr_t h, const char *channel_name, void **buffer, u32_t buffer_size);

ChannelBuffer_ptr dds_read_data(message_handler_ptr_t h, const char *channel_name);

void dds_close(message_handler_ptr_t h);

bool dds_is_ok(struct message_handler_t *h);

message_handler_t dds_handler_create() {
    message_handler_t target = {0};

    target.create = dds_create;
    target.close = dds_close;
//    target.read = ros_read;
//    target.write = ros_write;

//    target.pool = ros_get_pool;
    target.is_ok = dds_is_ok;
    target.read_data = dds_read_data;
    target.write_data = dds_write_data;

    return target;
}

bool dds_create(message_handler_ptr_t h, const char *filename, const ta_cfg_t *cfg) {

    dds_helper::DdsHandlerVariant *handler = new dds_helper::DdsHandlerVariant();
    int rt = handler->create(filename, cfg);
    h->handler = handler;
    MLOGI("%s ret = %i, handler = %p\n", "dds_create", rt, handler);

    if (rt < 0) {
        h->close(h);
        return false;
    }
    return true;
}

void dds_close(message_handler_ptr_t h) {
    MLOGI("%s h->handler = %p\n", "dds_close", h->handler);
    if (h->handler) {
        dds_helper::DdsHandlerVariant *handler = (dds_helper::DdsHandlerVariant *) h->handler;
        handler->stop();
        MLOGI("dds_close at : %p\n", h->handler);
        delete handler;
        h->handler = nullptr;
    }

}

bool dds_is_ok(struct message_handler_t *h) {
    if (h->handler) {
        dds_helper::DdsHandlerVariant *handler = (dds_helper::DdsHandlerVariant *) h->handler;
        return handler->is_ok();
    }
    return false;
}

int dds_write_data(message_handler_ptr_t h, const char *channel_name, void **buffer, u32_t buffer_size) {
    if (h->handler) {
#if 1

        dds_helper::DdsHandlerVariant *handler = (dds_helper::DdsHandlerVariant *) h->handler;
        return handler->write_data(channel_name, buffer, buffer_size);
#endif

    }
    return 0;
}

ChannelBuffer_ptr dds_read_data(message_handler_ptr_t h, const char *channel_name) {
    if (h->handler) {
#if 1
        dds_helper::DdsHandlerVariant *handler = (dds_helper::DdsHandlerVariant *) h->handler;
        return handler->read_data(channel_name);
#endif
    }
    return 0;
}
