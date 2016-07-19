#define LOG_TAG "NanohubHAL_Test"

#include <iostream>
#include <iomanip>
#include <map>
#include <memory>
#include <cstddef>
#include <cstdint>

#include <unistd.h>

#include <sys/endian.h>
#include <utils/Log.h>

#include <hardware/hardware.h>
#include <hardware/context_hub.h>
#include <nanohub/nanoapp.h>

inline std::ostream &operator << (std::ostream &os, const hub_app_name_t &appId)
{
    char vendor[6];
    __be64 beAppId = htobe64(appId.id);
    uint32_t seqId = appId.id & NANOAPP_VENDOR_ALL_APPS;

    std::ios::fmtflags f(os.flags());
    memcpy(vendor, (void*)&beAppId, sizeof(vendor) - 1);
    vendor[sizeof(vendor) - 1] = 0;
    if (strlen(vendor) == 5)
        os << vendor << ", " << std::hex << std::setw(6)  << seqId;
    else
        os << "#" << std::hex << appId.id;
    os.flags(f);

    return os;
}

void dumpBuffer(std::ostream &os, const char *pfx, const hub_app_name_t &appId, uint32_t evtId, const void *data, size_t len, int status)
{
    const uint8_t *p = static_cast<const uint8_t *>(data);
    os << pfx << ": [ID=" << appId << "; SZ=" << std::dec << len;
    if (evtId)
        os << "; EVT=" << std::hex << evtId;
    os << "]:" << std::hex;
    for (size_t i = 0; i < len; ++i) {
        os << " "  << std::setfill('0') << std::setw(2) << (unsigned int)p[i];
    }
    if (status) {
        os << "; status=" << status << " [" << std::setfill('0') << std::setw(8) << status << "]";
    }
}

class CHub
{
public:
    class IClient {
    public:
        virtual void onMessage(const hub_message_t &msg) = 0;
        virtual ~IClient(){}
    };
    class Client : IClient {
        CHub *mParent;
        const context_hub_t *mHub;
        std::function<void(const hub_message_t &)> mHandler;

    public:
        explicit Client(const context_hub_t *hub, CHub *parent) {
            mHub = hub;
            mParent = parent;
        }
        ~Client() = default;

        void setHandler(std::function<void(const hub_message_t &)> handler) {
            mHandler = handler;
        }
        void onMessage(const hub_message_t &msg) {
            if((bool)mHandler == true) {
                mHandler(msg);
            }
        }
        void sendMessage(const hub_message_t &msg) {
            mParent->sendMessage(mHub->hub_id, msg);
        }
        void sendToSystem(uint32_t typ, void *data, uint32_t len) {
            mParent->sendMessage(mHub->hub_id, mHub->os_app_name, typ, data, len);
        }
        void sendToApp(hub_app_name_t app, void *data, uint32_t len) {
            mParent->sendMessage(mHub->hub_id, app, 0, data, len);
        }
        const hub_app_name_t getSystemApp() const { return mHub->os_app_name; }
    };
private:
    static int contextHubCallback(uint32_t id, const hub_message_t *msg, void *cookie)
    {
        CHub *hub = static_cast<CHub*>(cookie);
        hub->onMessage(id, msg);
        return 0;
    }

    CHub() {
        hw_get_module(CONTEXT_HUB_MODULE_ID, (const hw_module_t **)&mMod);
        if (!mMod)
            return;
        mMod->subscribe_messages(0, contextHubCallback, this);
        mHubArraySize = mMod->get_hubs(mMod, &mHubArray);
        for (size_t i = 0; i < mHubArraySize; ++i) {
            auto item = &mHubArray[i];
            mHubs[item->hub_id] = std::unique_ptr<Client>(new Client(item, this));
        }
    }

    void onMessage(uint32_t hubId, const hub_message_t *msg) {
        mHubs[hubId]->onMessage(*msg);
    }

    int sendMessage(uint32_t id, const hub_message_t &msg) {
        return mMod->send_message(id, &msg);
    }

    int sendMessage(uint32_t id, hub_app_name_t app, uint32_t typ, void *data, uint32_t len) {
        hub_message_t msg = {
            .app_name = app,
            .message_type = typ,
            .message = data,
            .message_len = len,
        };
        return mMod->send_message(id, &msg);
    }

    context_hub_module_t *mMod = nullptr;
    const context_hub_t  *mHubArray = nullptr;
    size_t                mHubArraySize = 0;
    std::map <size_t, std::unique_ptr<Client> > mHubs;

public:
    static CHub *instantiate() {
        static CHub instance;

        return &instance;
    }
    Client *getClientByIndex(size_t idx) {
        return idx < mHubArraySize && mHubArray != nullptr ?
               getClientById(mHubArray[idx].hub_id) : nullptr;
    }
    Client *getClientById(size_t id) { return mHubs.count(id) ? mHubs[id].get() : nullptr; }
};

class NanoClient
{
    CHub::Client *mClient;
    std::ostream &log;
    android::Mutex lock;
    void onMessage(const hub_message_t &msg){
        android::Mutex::Autolock _l(lock);
        dumpBuffer(log, "Rx", msg.app_name, msg.message_type, msg.message, msg.message_len, 0);
        log << std::endl;
    }
public:
    NanoClient(int idx = 0) : log(std::clog) {
        CHub *hub = CHub::instantiate();
        mClient = hub->getClientByIndex(idx);
        if (mClient)
            mClient->setHandler(std::function<void(const hub_message_t&)>([this] (const hub_message_t&msg) { onMessage(msg); }));
    }
    void sendMessage(const hub_message_t &msg) { mClient->sendMessage(msg); }
    void sendMessageToSystem(uint32_t cmd, void * data, size_t dataSize) {
        hub_message_t msg;
        msg.message = data;
        msg.message_len = dataSize;
        msg.message_type = cmd;
        msg.app_name = mClient->getSystemApp();
        {
            android::Mutex::Autolock _l(lock);
            dumpBuffer(log, "Tx", msg.app_name, msg.message_type, msg.message, msg.message_len, 0);
            log << std::endl;
        }
        sendMessage(msg);
    }
    void sendMessageToApp(const hub_app_name_t appName, void * data, size_t dataSize) {
        hub_message_t msg;
        msg.message = data;
        msg.message_len = dataSize;
        msg.message_type = 0;
        msg.app_name = appName;
        sendMessage(msg);
    }
};

int main(int argc, char *argv[])
{
    NanoClient cli;
    int opt;
    long cmd = 0;
    uint64_t appId = 0;
    const char *appFileName = NULL;
    uint32_t fileSize = 0;

    while((opt = getopt(argc, argv, "c:i:a:")) != -1) {
        char *end = NULL;
        switch(opt) {
        case 'c':
            cmd = strtol(optarg, &end, 10);
            break;
        case 'i':
            appId = strtoull(optarg, &end, 16);
            break;
        case 'a':
            appFileName = optarg;
            break;
        }
        if (end && *end != '\0') {
            std::clog << "Invalid argument: " << optarg << std::endl;
            exit(1);
        }
    }
    switch(cmd) {
    case CONTEXT_HUB_APPS_ENABLE:
    {
        apps_enable_request_t req;
        req.app_name.id = appId;
        cli.sendMessageToSystem(CONTEXT_HUB_APPS_ENABLE, &req, sizeof(req));
    }
    break;
    case CONTEXT_HUB_APPS_DISABLE:
    {
        apps_disable_request_t req;
        req.app_name.id = appId;
        cli.sendMessageToSystem(CONTEXT_HUB_APPS_DISABLE, &req, sizeof(req));
    }
    break;
    case CONTEXT_HUB_LOAD_APP:
    {
        load_app_request_t *req = NULL;
        if (appFileName)
            req = (load_app_request_t *)loadFile(appFileName, &fileSize);
        if (!req || fileSize < sizeof(*req) || req->app_binary.magic != NANOAPP_MAGIC) {
            std::clog << "Invalid nanoapp image: " << appFileName << std::endl;
            exit(1);
        }
        cli.sendMessageToSystem(CONTEXT_HUB_LOAD_APP, req, fileSize);
        free(req);
    }
    break;
    case CONTEXT_HUB_UNLOAD_APP:
    {
        unload_app_request_t req;
        req.app_name.id = appId;
        cli.sendMessageToSystem(CONTEXT_HUB_UNLOAD_APP, &req, sizeof(req));
    }
    break;
    case CONTEXT_HUB_QUERY_APPS:
    {
        query_apps_request_t req;
        req.app_name.id = appId;
        cli.sendMessageToSystem(CONTEXT_HUB_QUERY_APPS, &req, sizeof(req));
    }
    break;
    case CONTEXT_HUB_QUERY_MEMORY:
    default:
        std::clog << "Unknown command: " << cmd << std::endl;
        exit(1);
    }
    while(1) {
        sleep(1);
    }
    return 0;
}
