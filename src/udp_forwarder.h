#pragma once
#include <stdint.h>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <thread>
#include <atomic>
#include <cstdint>
#include "pctl.h"

struct MotorWebConfig {
    uint8_t       mode       = 1;            // 0=GOLDEN_SNAP, 1=CONSTANT_VELOCITY
    float         rpm        = 1.f;
    float         gear_ratio = 1.47435294f;
    pctl_params_t mctl_vq    = {};
};

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    using sock_t = SOCKET;
    static inline bool   sock_valid(sock_t s) { return s != INVALID_SOCKET; }
    static inline void   sock_close(sock_t s) { closesocket(s); }
    static inline sock_t sock_bad()            { return INVALID_SOCKET; }
#else
    #include <sys/socket.h>
    #include <sys/select.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    using sock_t = int;
    static inline bool   sock_valid(sock_t s) { return s >= 0; }
    static inline void   sock_close(sock_t s) { ::close(s); }
    static inline sock_t sock_bad()            { return -1; }
#endif

class UdpForwarder {
public:
    const char* settingsFile = "forwarder.ini";

    UdpForwarder()
    {
        udpSock_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (!sock_valid(udpSock_))
            perror("UdpForwarder: udp socket");
    }

    ~UdpForwarder()
    {
        webRunning_ = false;
        if (webThread_.joinable()) webThread_.join();
        if (sock_valid(udpSock_)) sock_close(udpSock_);
    }

    void setDestination(const char* ip, uint16_t port)
    {
        {
            std::lock_guard<std::mutex> lk(destMtx_);
            strncpy(dest_.ip, ip, sizeof(dest_.ip) - 1);
            dest_.ip[sizeof(dest_.ip) - 1] = '\0';
            dest_.port = port;
        }
        saveSettings();
    }

    // Load ip/port from settingsFile. Falls back to current dest_ if file is absent or malformed.
    void loadSettings()
    {
        FILE* f = fopen(settingsFile, "r");
        if (!f) return;
        char line[128];
        char loadedIp[64]  = {};
        uint16_t loadedPort = 0;
        while (fgets(line, sizeof(line), f))
        {
            char val[64] = {};
            if (sscanf(line, "ip=%63s", val) == 1)
                strncpy(loadedIp, val, sizeof(loadedIp) - 1);
            else if (sscanf(line, "port=%hu", &loadedPort) == 1)
                ;
        }
        fclose(f);
        if (loadedIp[0] && loadedPort)
        {
            std::lock_guard<std::mutex> lk(destMtx_);
            strncpy(dest_.ip, loadedIp, sizeof(dest_.ip) - 1);
            dest_.ip[sizeof(dest_.ip) - 1] = '\0';
            dest_.port = loadedPort;
            printf("Forwarder loaded destination: %s:%u\n", dest_.ip, dest_.port);
        }
    }

    // Thread-safe. Called from lidar recv thread via onPacketReady.
    void send(const uint8_t* data, size_t len)
    {
        if (!sock_valid(udpSock_)) return;
        sockaddr_in dst{};
        dst.sin_family = AF_INET;
        {
            std::lock_guard<std::mutex> lk(destMtx_);
            dst.sin_port = htons(dest_.port);
            inet_pton(AF_INET, dest_.ip, &dst.sin_addr);
        }
        ::sendto(udpSock_, reinterpret_cast<const char*>(data), static_cast<int>(len),
                 0, reinterpret_cast<sockaddr*>(&dst), sizeof(dst));
    }

    void startWebServer(uint16_t httpPort = 80)
    {
        loadSettings();
        webRunning_ = true;
        webThread_  = std::thread(&UdpForwarder::webServerLoop, this, httpPort);
    }

    void initMotorConfig(const MotorWebConfig& cfg)
    {
        std::lock_guard<std::mutex> lk(motorMtx_);
        motorCfg_ = cfg;
    }

    bool pollMotorConfig(MotorWebConfig& out)
    {
        if (!motorDirty_.load()) return false;
        std::lock_guard<std::mutex> lk(motorMtx_);
        out = motorCfg_;
        motorDirty_ = false;
        return true;
    }

private:
    struct Destination {
        char     ip[64] = "127.0.0.1";
        uint16_t port   = 9000;
    };

    Destination       dest_;
    std::mutex        destMtx_;
    sock_t            udpSock_    = sock_bad();
    std::thread       webThread_;
    std::atomic<bool> webRunning_ = false;

    MotorWebConfig    motorCfg_;
    std::mutex        motorMtx_;
    std::atomic<bool> motorDirty_{false};

    void saveSettings()
    {
        FILE* f = fopen(settingsFile, "w");
        if (!f) { perror("UdpForwarder: save settings"); return; }
        std::lock_guard<std::mutex> lk(destMtx_);
        fprintf(f, "ip=%s\nport=%u\n", dest_.ip, dest_.port);
        fclose(f);
    }

    static void parseField(const char* body, const char* key, char* out, size_t outLen)
    {
        const char* p = strstr(body, key);
        if (!p) { out[0] = '\0'; return; }
        p += strlen(key);
        size_t i = 0;
        while (*p && *p != '&' && i < outLen - 1)
            out[i++] = *p++;
        out[i] = '\0';
    }

    void handleClient(sock_t client)
    {
        char req[2048] = {};
        ::recv(client, req, sizeof(req) - 1, 0);

        bool isPost = strncmp(req, "POST", 4) == 0;

        if (isPost)
        {
            const char* body = strstr(req, "\r\n\r\n");
            if (body)
            {
                body += 4;
                char formType[16] = {};
                parseField(body, "form=", formType, sizeof(formType));

                if (strcmp(formType, "motor") == 0)
                {
                    char buf[32];
                    MotorWebConfig cfg;
                    {
                        std::lock_guard<std::mutex> lk(motorMtx_);
                        cfg = motorCfg_;
                    }
                    parseField(body, "mode=",          buf, sizeof(buf)); if (buf[0]) cfg.mode       = (uint8_t)atoi(buf);
                    parseField(body, "rpm=",           buf, sizeof(buf)); if (buf[0]) cfg.rpm         = (float)atof(buf);
                    parseField(body, "gear_ratio=",    buf, sizeof(buf)); if (buf[0]) cfg.gear_ratio  = (float)atof(buf);
                    parseField(body, "kp_i32=",        buf, sizeof(buf)); if (buf[0]) cfg.mctl_vq.kpki.kp.i32         = atoi(buf);
                    parseField(body, "kp_radix=",      buf, sizeof(buf)); if (buf[0]) cfg.mctl_vq.kpki.kp.radix       = atoi(buf);
                    parseField(body, "ki_i32=",        buf, sizeof(buf)); if (buf[0]) cfg.mctl_vq.kpki.ki.i32         = atoi(buf);
                    parseField(body, "ki_radix=",      buf, sizeof(buf)); if (buf[0]) cfg.mctl_vq.kpki.ki.radix       = atoi(buf);
                    parseField(body, "x_integral_div=",buf, sizeof(buf)); if (buf[0]) cfg.mctl_vq.kpki.x_integral_div = atoi(buf);
                    parseField(body, "x_sat=",         buf, sizeof(buf)); if (buf[0]) cfg.mctl_vq.kpki.x_sat          = atoi(buf);
                    parseField(body, "out_rshift=",    buf, sizeof(buf)); if (buf[0]) cfg.mctl_vq.kpki.out_rshift     = (uint8_t)atoi(buf);
                    parseField(body, "kd_i32=",        buf, sizeof(buf)); if (buf[0]) cfg.mctl_vq.kd.i32              = atoi(buf);
                    parseField(body, "kd_radix=",      buf, sizeof(buf)); if (buf[0]) cfg.mctl_vq.kd.radix            = atoi(buf);
                    parseField(body, "out_sat=",       buf, sizeof(buf)); if (buf[0]) cfg.mctl_vq.out_sat             = atoi(buf);
                    {
                        std::lock_guard<std::mutex> lk(motorMtx_);
                        motorCfg_ = cfg;
                    }
                    motorDirty_ = true;
                    printf("Motor config updated\n");
                }
                else
                {
                    char newIp[64]    = {};
                    char newPortStr[8] = {};
                    parseField(body, "ip=",   newIp,      sizeof(newIp));
                    parseField(body, "port=", newPortStr, sizeof(newPortStr));
                    uint16_t newPort = (uint16_t)atoi(newPortStr);
                    if (newIp[0] && newPort)
                    {
                        setDestination(newIp, newPort);
                        printf("Forwarder destination: %s:%u\n", newIp, newPort);
                    }
                }
            }
            const char* redir =
                "HTTP/1.1 303 See Other\r\nLocation: /\r\nContent-Length: 0\r\n\r\n";
            ::send(client, redir, static_cast<int>(strlen(redir)), 0);
        }
        else
        {
            char curIp[64]; uint16_t curPort;
            {
                std::lock_guard<std::mutex> lk(destMtx_);
                strncpy(curIp, dest_.ip, sizeof(curIp) - 1);
                curIp[sizeof(curIp) - 1] = '\0';
                curPort = dest_.port;
            }
            MotorWebConfig mc;
            {
                std::lock_guard<std::mutex> lk(motorMtx_);
                mc = motorCfg_;
            }

            char htmlbody[4096];
            snprintf(htmlbody, sizeof(htmlbody),
                "<html><head><title>Lidar Forwarder</title></head><body>"
                "<h3>UDP Forwarder</h3>"
                "<form method=post>"
                "<input name=form value=fwd type=hidden>"
                "IP:&nbsp;<input name=ip value='%s'><br>"
                "Port:&nbsp;<input name=port value='%u'><br>"
                "<input type=submit value='Update Forwarder'>"
                "</form>"
                "<p>Current: <b>%s:%u</b></p>"
                "<hr><h3>Motor Control</h3>"
                "<form method=post>"
                "<input name=form value=motor type=hidden>"
                "Mode:&nbsp;<select name=mode>"
                "<option value=0%s>GOLDEN_SNAP</option>"
                "<option value=1%s>CONSTANT_VELOCITY</option>"
                "</select><br>"
                "RPM:&nbsp;<input name=rpm value='%g'><br>"
                "Gear Ratio:&nbsp;<input name=gear_ratio value='%g'><br>"
                "<b>mctl_vq:</b><br>"
                "kp.i32:&nbsp;<input name=kp_i32 value='%d'>&nbsp;"
                "kp.radix:&nbsp;<input name=kp_radix value='%d'><br>"
                "ki.i32:&nbsp;<input name=ki_i32 value='%d'>&nbsp;"
                "ki.radix:&nbsp;<input name=ki_radix value='%d'><br>"
                "x_integral_div:&nbsp;<input name=x_integral_div value='%d'><br>"
                "x_sat:&nbsp;<input name=x_sat value='%d'><br>"
                "out_rshift:&nbsp;<input name=out_rshift value='%d'><br>"
                "kd.i32:&nbsp;<input name=kd_i32 value='%d'>&nbsp;"
                "kd.radix:&nbsp;<input name=kd_radix value='%d'><br>"
                "out_sat:&nbsp;<input name=out_sat value='%d'><br>"
                "<input type=submit value='Update Motor'>"
                "</form>"
                "</body></html>",
                curIp, curPort, curIp, curPort,
                mc.mode == 0 ? " selected" : "",
                mc.mode == 1 ? " selected" : "",
                mc.rpm, mc.gear_ratio,
                mc.mctl_vq.kpki.kp.i32, mc.mctl_vq.kpki.kp.radix,
                mc.mctl_vq.kpki.ki.i32, mc.mctl_vq.kpki.ki.radix,
                mc.mctl_vq.kpki.x_integral_div,
                mc.mctl_vq.kpki.x_sat,
                mc.mctl_vq.kpki.out_rshift,
                mc.mctl_vq.kd.i32, mc.mctl_vq.kd.radix,
                mc.mctl_vq.out_sat);

            char resp[4800];
            int  respLen = snprintf(resp, sizeof(resp),
                "HTTP/1.1 200 OK\r\n"
                "Content-Type: text/html\r\n"
                "Content-Length: %zu\r\n"
                "Connection: close\r\n"
                "\r\n%s",
                strlen(htmlbody), htmlbody);
            ::send(client, resp, respLen, 0);
        }

        sock_close(client);
    }

    void webServerLoop(uint16_t httpPort)
    {
        sock_t srv = ::socket(AF_INET, SOCK_STREAM, 0);
        if (!sock_valid(srv)) { perror("UdpForwarder: tcp socket"); return; }

        int opt = 1;
        setsockopt(srv, SOL_SOCKET, SO_REUSEADDR,
                   reinterpret_cast<const char*>(&opt), sizeof(opt));

        sockaddr_in addr{};
        addr.sin_family      = AF_INET;
        addr.sin_port        = htons(httpPort);
        addr.sin_addr.s_addr = INADDR_ANY;

        if (::bind(srv, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
        {
            perror("UdpForwarder: web bind");
            sock_close(srv);
            return;
        }
        ::listen(srv, 4);
        printf("Forwarder config UI: http://localhost:%u\n", httpPort);

        while (webRunning_)
        {
            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(srv, &rfds);
            struct timeval tv; tv.tv_sec = 1; tv.tv_usec = 0;
            int n = ::select(static_cast<int>(srv) + 1, &rfds, nullptr, nullptr, &tv);
            if (n > 0)
            {
                sock_t client = ::accept(srv, nullptr, nullptr);
                if (sock_valid(client))
                    handleClient(client);
            }
        }
        sock_close(srv);
    }
};
