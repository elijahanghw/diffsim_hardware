// FcRelay — see fc_relay.h. Ported from relay_diffsim/relay_radxa/src/relay.cpp
// (Lucassen, Ferede, Bahnam, Blaha, TU Delft, GPLv3), reduced to the on-board
// case: the CNN features arrive via publishFeatures() instead of the HITL DPTH
// datagram on UDP port 5010, so that listener and its parsing are gone.

#include "fc_relay.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <signal.h>
#include <termios.h>
#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

extern "C" {
#include "pi-protocol.h"
#include "pi-messages.h"
}

namespace {

#define OPTITRACK_PORT 5005
#define SETPOINT_PORT  5006
#define KEYBOARD_PORT  5007
#define PI_MSG_PAYLOAD_OFFSET (PI_MSG_ID_BYTES + PI_MSG_PAYLOAD_LEN_BYTES)

constexpr int NN_FEATURE_CHUNK_SIZE = 32;
static_assert(FcRelay::kFeatureDim % NN_FEATURE_CHUNK_SIZE == 0,
    "kFeatureDim must be an exact multiple of NN_FEATURE_CHUNK_SIZE");
static_assert(PI_MSG_NN_INPUT_CHUNK_PAYLOAD_LEN == 1 + NN_FEATURE_CHUNK_SIZE * sizeof(float),
    "NN_FEATURE_CHUNK_SIZE doesn't match the field count in msgs/NN_INPUT_CHUNK.yaml");

// pi-protocol's piSendMsg takes a plain void(*)(uint8_t) sink, so the serial fd
// it writes to has to live at file scope. There is a single FcRelay per process
// (one FC serial link), so a single global fd is sufficient.
int g_serialPortFd = -1;

void serialWriter(uint8_t byte) {
    if (g_serialPortFd >= 0) {
        ssize_t n = write(g_serialPortFd, &byte, 1);
        (void)n;
    }
}

// On-demand diagnostics, as in the standalone relay:
//   SIGUSR1 -> print last received messages,  SIGUSR2 -> print parser stats.
void catch_function(int signo) {
    switch (signo) {
        case SIGUSR1: piPrintMsgs(&printf); break;
        case SIGUSR2: piPrintStats(&printf); break;
    }
}

int openSerialPort(const char* port, int baudrate) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("FcRelay: unable to open serial port");
        return -1;
    }

    struct termios options;
    tcgetattr(fd, &options);
    options.c_cflag = baudrate | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &options);
    return fd;
}

int openUdpPort(uint16_t port) {
    int sockfd;
    struct sockaddr_in server_addr;

    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
        perror("FcRelay: socket creation failed");
        exit(EXIT_FAILURE);
    }

    int flags = fcntl(sockfd, F_GETFL, 0);
    fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(port);

    if (bind(sockfd, (const struct sockaddr*)&server_addr, sizeof(server_addr)) == -1) {
        perror("FcRelay: bind failed");
        exit(EXIT_FAILURE);
    }
    return sockfd;
}

float ntohf(float in) {
    uint32_t bits;
    std::memcpy(&bits, &in, sizeof(bits));
    bits = ntohl(bits);
    float out;
    std::memcpy(&out, &bits, sizeof(out));
    return out;
}

// External pose, as delivered over the optitrack UDP port.
typedef struct pose_s {
    uint64_t timeUs;
    float x, y, z;
    float qx, qy, qz, qw;
} pose_t;

typedef struct pose_der_s {
    uint64_t timeUs;
    float x, y, z;
    float wx, wy, wz;
} pose_der_t;

// Baud rate in Hz -> termios constant. Returns -1 if unsupported.
int baudToTermios(int baudHz) {
    switch (baudHz) {
        case 38400:   return B38400;
        case 57600:   return B57600;
        case 115200:  return B115200;
        case 230400:  return B230400;
        case 460800:  return B460800;
        case 500000:  return B500000;
        case 921600:  return B921600;
        case 1000000: return B1000000;
        case 1500000: return B1500000;
        default:      return -1;
    }
}

}  // namespace

bool FcRelay::start(const std::string& serialPort, int baudRateHz) {
    if (running_) return true;

    int baud = baudToTermios(baudRateHz);
    if (baud == -1) {
        fprintf(stderr, "FcRelay: baudrate %d not supported\n", baudRateHz);
        return false;
    }

    serialFd_ = openSerialPort(serialPort.c_str(), baud);
    if (serialFd_ == -1) return false;
    g_serialPortFd = serialFd_;

    if (signal(SIGUSR1, catch_function) == SIG_ERR ||
        signal(SIGUSR2, catch_function) == SIG_ERR) {
        fprintf(stderr, "FcRelay: failed to install SIGUSR handlers\n");
        // non-fatal: diagnostics only
    }

    optitrackFd_ = openUdpPort(OPTITRACK_PORT);
    printf("FcRelay: listening on port %d for Optitrack...\n", OPTITRACK_PORT);
    setpointFd_ = openUdpPort(SETPOINT_PORT);
    printf("FcRelay: listening on port %d for Setpoints...\n", SETPOINT_PORT);
    keyboardFd_ = openUdpPort(KEYBOARD_PORT);
    printf("FcRelay: listening on port %d for Keystrokes...\n", KEYBOARD_PORT);

    running_ = true;
    thread_ = std::thread(&FcRelay::run, this);
    return true;
}

void FcRelay::publishFeatures(const float* features, std::size_t n) {
    if (n != kFeatureDim) {
        fprintf(stderr, "FcRelay: publishFeatures got %zu floats, expected %d, dropping\n",
                n, kFeatureDim);
        return;
    }
    std::lock_guard<std::mutex> lock(featuresMutex_);
    std::memcpy(features_.data(), features, kFeatureDim * sizeof(float));
    ++featuresSeq_;
}

void FcRelay::sendNnFeatures(const std::array<float, kFeatureDim>& vals) {
    for (std::size_t chunk = 0; chunk < kFeatureDim / NN_FEATURE_CHUNK_SIZE; ++chunk) {
        const float* c = vals.data() + chunk * NN_FEATURE_CHUNK_SIZE;
        piMsgNnInputChunkTx.chunk_index = (uint8_t)chunk;
        piMsgNnInputChunkTx.f0  = c[0];  piMsgNnInputChunkTx.f1  = c[1];
        piMsgNnInputChunkTx.f2  = c[2];  piMsgNnInputChunkTx.f3  = c[3];
        piMsgNnInputChunkTx.f4  = c[4];  piMsgNnInputChunkTx.f5  = c[5];
        piMsgNnInputChunkTx.f6  = c[6];  piMsgNnInputChunkTx.f7  = c[7];
        piMsgNnInputChunkTx.f8  = c[8];  piMsgNnInputChunkTx.f9  = c[9];
        piMsgNnInputChunkTx.f10 = c[10]; piMsgNnInputChunkTx.f11 = c[11];
        piMsgNnInputChunkTx.f12 = c[12]; piMsgNnInputChunkTx.f13 = c[13];
        piMsgNnInputChunkTx.f14 = c[14]; piMsgNnInputChunkTx.f15 = c[15];
        piMsgNnInputChunkTx.f16 = c[16]; piMsgNnInputChunkTx.f17 = c[17];
        piMsgNnInputChunkTx.f18 = c[18]; piMsgNnInputChunkTx.f19 = c[19];
        piMsgNnInputChunkTx.f20 = c[20]; piMsgNnInputChunkTx.f21 = c[21];
        piMsgNnInputChunkTx.f22 = c[22]; piMsgNnInputChunkTx.f23 = c[23];
        piMsgNnInputChunkTx.f24 = c[24]; piMsgNnInputChunkTx.f25 = c[25];
        piMsgNnInputChunkTx.f26 = c[26]; piMsgNnInputChunkTx.f27 = c[27];
        piMsgNnInputChunkTx.f28 = c[28]; piMsgNnInputChunkTx.f29 = c[29];
        piMsgNnInputChunkTx.f30 = c[30]; piMsgNnInputChunkTx.f31 = c[31];
        piSendMsg(&piMsgNnInputChunkTx, &serialWriter);
    }
}

void FcRelay::run() {
    pi_parse_states_t parseStates{};
    uint8_t piBuffer[PI_MAX_PACKET_LEN];

    static constexpr size_t OPTITRACK_BUFFER_SIZE =
        sizeof(unsigned int) + sizeof(pose_t) + sizeof(pose_der_t);
    uint8_t optitrackBuffer[OPTITRACK_BUFFER_SIZE];
    uint8_t setpointBuffer[PI_MSG_POS_SETPOINT_PAYLOAD_LEN];
    uint8_t keyboardBuffer[PI_MSG_KEYBOARD_PAYLOAD_LEN];

    pose_t pose;
    pose_der_t pose_der;

    uint64_t lastSentSeq = 0;

    while (running_) {
        // ---- serial RX: we relay timestamped messages only once we have a
        // fresh EKF_INPUTS from the FC, which carries the time base. ----
        ssize_t numBytes = read(serialFd_, piBuffer, PI_MAX_PACKET_LEN);
        bool newMessage = false;
        if (numBytes > 0) {
            for (ssize_t i = 0; i < numBytes; i++) {
                if (piParse(&parseStates, piBuffer[i]) == PI_MSG_EKF_INPUTS_ID) {
                    newMessage = true;
                }
            }
        }

        // ---- NN input features ----
        // NN_INPUT_CHUNK carries no timestamp, so it is sent independently of the
        // EKF_INPUTS gate: whenever the CNN thread has produced a new vector.
        {
            std::array<float, kFeatureDim> local;
            uint64_t seq;
            {
                std::lock_guard<std::mutex> lock(featuresMutex_);
                seq = featuresSeq_;
                local = features_;
            }
            if (seq != lastSentSeq) {
                sendNnFeatures(local);
                lastSentSeq = seq;
            }
        }

        if ((piMsgEkfInputsRxState == PI_MSG_RX_STATE_NONE) || (!newMessage)) {
            usleep(100);  // no time base yet, or nothing new: ease CPU
            continue;
        }

        struct sockaddr_in client_addr;
        socklen_t client_addr_len = sizeof(client_addr);

        // ---- optitrack -> FAKE_GPS + EXTERNAL_POSE ----
        int optitrackBytes = recvfrom(optitrackFd_, optitrackBuffer, OPTITRACK_BUFFER_SIZE, 0,
                                      (struct sockaddr*)&client_addr, &client_addr_len);
        if (optitrackBytes > 0) {
            memcpy(&pose, optitrackBuffer + sizeof(unsigned int), sizeof(pose_t));
            memcpy(&pose_der, optitrackBuffer + sizeof(unsigned int) + sizeof(pose_t),
                   sizeof(pose_der_t));

            piMsgFakeGpsTx.time_us = piMsgEkfInputsRx->time_us;
            static constexpr double CYBERZOO_LAT = 51.99071002805145;
            static constexpr double CYBERZOO_LON = 4.376727452462819;
            static constexpr double RE = 6378137.;
            piMsgFakeGpsTx.lat = (int)1e7 * (CYBERZOO_LAT + 180. / M_PI * (pose.x / RE));
            piMsgFakeGpsTx.lon = (int)1e7 *
                (CYBERZOO_LON + 180 / M_PI * (pose.y / RE) / cos(CYBERZOO_LON * M_PI / 180.));
            piMsgFakeGpsTx.altCm = (int)(pose.z * 100.f);
            piMsgFakeGpsTx.hdop = (short)150;
            piMsgFakeGpsTx.groundSpeed = (short)(hypotf(pose_der.x, pose_der.y) * 100.f);
            piMsgFakeGpsTx.groundCourse = (short)(1800.f * atan2(pose_der.x, pose_der.y) / M_PI);
            piMsgFakeGpsTx.numSat = 8;
            piSendMsg(&piMsgFakeGpsTx, &serialWriter);

            piMsgExternalPoseTx.time_us = piMsgEkfInputsRx->time_us;
            piMsgExternalPoseTx.ned_x = pose.x;
            piMsgExternalPoseTx.ned_y = pose.y;
            piMsgExternalPoseTx.ned_z = pose.z;
            piMsgExternalPoseTx.ned_xd = pose_der.x;
            piMsgExternalPoseTx.ned_yd = pose_der.y;
            piMsgExternalPoseTx.ned_zd = pose_der.z;
            piMsgExternalPoseTx.body_qi = pose.qw;
            piMsgExternalPoseTx.body_qx = pose.qx;
            piMsgExternalPoseTx.body_qy = pose.qy;
            piMsgExternalPoseTx.body_qz = pose.qz;
            piSendMsg(&piMsgExternalPoseTx, &serialWriter);
        }

        // ---- setpoints ----
        pi_POS_SETPOINT_t msgPosSetpoint;
        int setpointBytes = recvfrom(setpointFd_, setpointBuffer, PI_MSG_POS_SETPOINT_PAYLOAD_LEN, 0,
                                     (struct sockaddr*)&client_addr, &client_addr_len);
        if (setpointBytes > 0) {
            memcpy((uint8_t*)&msgPosSetpoint + PI_MSG_PAYLOAD_OFFSET, setpointBuffer,
                   PI_MSG_POS_SETPOINT_PAYLOAD_LEN);
            piMsgPosSetpointTx.time_us = piMsgEkfInputsRx->time_us;
            piMsgPosSetpointTx.ned_x = ntohf(msgPosSetpoint.ned_x);
            piMsgPosSetpointTx.ned_y = ntohf(msgPosSetpoint.ned_y);
            piMsgPosSetpointTx.ned_z = ntohf(msgPosSetpoint.ned_z);
            piMsgPosSetpointTx.ned_xd = ntohf(msgPosSetpoint.ned_xd);
            piMsgPosSetpointTx.ned_yd = ntohf(msgPosSetpoint.ned_yd);
            piMsgPosSetpointTx.ned_zd = ntohf(msgPosSetpoint.ned_zd);
            piMsgPosSetpointTx.yaw = ntohf(msgPosSetpoint.yaw);
            piSendMsg(&piMsgPosSetpointTx, &serialWriter);
        }

        // ---- keyboard ----
        pi_KEYBOARD_t msgKeyboard;
        int keyboardBytes = recvfrom(keyboardFd_, keyboardBuffer, PI_MSG_KEYBOARD_PAYLOAD_LEN, 0,
                                     (struct sockaddr*)&client_addr, &client_addr_len);
        if (keyboardBytes > 0) {
            memcpy((uint8_t*)&msgKeyboard + PI_MSG_PAYLOAD_OFFSET, keyboardBuffer,
                   PI_MSG_KEYBOARD_PAYLOAD_LEN);
            piMsgKeyboardTx.time_us = piMsgEkfInputsRx->time_us;
            piMsgKeyboardTx.key = msgKeyboard.key;
            piSendMsg(&piMsgKeyboardTx, &serialWriter);
        }
    }
}

void FcRelay::stop() {
    if (running_.exchange(false)) {
        if (thread_.joinable()) thread_.join();
    }
    if (serialFd_ != -1)    { close(serialFd_);    serialFd_ = -1; }
    if (optitrackFd_ != -1) { close(optitrackFd_); optitrackFd_ = -1; }
    if (setpointFd_ != -1)  { close(setpointFd_);  setpointFd_ = -1; }
    if (keyboardFd_ != -1)  { close(keyboardFd_);  keyboardFd_ = -1; }
    g_serialPortFd = -1;
}

FcRelay::~FcRelay() {
    stop();
}
