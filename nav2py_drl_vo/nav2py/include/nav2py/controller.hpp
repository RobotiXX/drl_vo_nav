/*
 *  License: MIT
 *  Author(s): Shrijit Singh <shrijitsingh99@gmail.com>
 */

#ifndef NAV2PY__CONTROLLER_HPP_
#define NAV2PY__CONTROLLER_HPP_

#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <vector>
#include <sys/socket.h>
#include <unistd.h>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace nav2py
{
    class Controller : public nav2_core::Controller
    {

    private:
        int socket_ = -1;
        FILE * pipe_ = nullptr;

        static void send_all(int socket, const char * data, size_t size)
        {
            size_t sent = 0;
            while (sent < size) {
                const ssize_t result = send(socket, data + sent, size - sent, MSG_NOSIGNAL);
                if (result < 0 && errno == EINTR) {
                    continue;
                }
                if (result <= 0) {
                    throw std::runtime_error("nav2py socket send failed");
                }
                sent += static_cast<size_t>(result);
            }
        }

        static double recv_double(int socket, std::chrono::milliseconds timeout)
        {
            uint64_t network_value = 0;
            auto * output = reinterpret_cast<char *>(&network_value);
            size_t received = 0;
            const auto deadline = std::chrono::steady_clock::now() + timeout;

            while (received < sizeof(network_value)) {
                const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
                    deadline - std::chrono::steady_clock::now());
                if (remaining.count() <= 0) {
                    throw std::runtime_error("timed out waiting for nav2py response");
                }
                pollfd descriptor{socket, POLLIN, 0};
                int ready;
                do {
                    ready = poll(&descriptor, 1, static_cast<int>(remaining.count()));
                } while (ready < 0 && errno == EINTR);
                if (ready == 0) {
                    throw std::runtime_error("timed out waiting for nav2py response");
                }
                if (ready < 0 || (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL))) {
                    throw std::runtime_error("nav2py response socket failed");
                }
                const ssize_t count = recv(socket, output + received,
                    sizeof(network_value) - received, 0);
                if (count < 0 && errno == EINTR) {
                    continue;
                }
                if (count <= 0) {
                    throw std::runtime_error("nav2py response socket closed");
                }
                received += static_cast<size_t>(count);
            }

            network_value = be64toh(network_value);
            double value;
            std::memcpy(&value, &network_value, sizeof(value));
            return value;
        }

    protected:
        bool nav2py_bootstrap(const std::string cmd)
        {
            // determine port from client
            uint16_t port;
            uint16_t magic = 48073; // magic number

            auto logger = rclcpp::get_logger("TemplateController");

            RCLCPP_DEBUG(logger, "starting process %s", cmd.c_str());

            pipe_ = popen(cmd.c_str(), "r");

            if (!pipe_)
                throw std::runtime_error("popen() failed");

            constexpr size_t buffer_size = (sizeof port) + (sizeof magic); // port followed by magic nubmer
            char buffer[buffer_size];
            try
            {
                const size_t received = fread(buffer, 1, buffer_size, pipe_);
                if (received != buffer_size)
                    throw std::runtime_error("bootstrap failed");

                uint16_t n_magic;
                std::memcpy(&n_magic, buffer + (sizeof port), sizeof n_magic);

                if (ntohs(n_magic) != magic)
                    throw std::runtime_error("magic number mismatch");

                uint16_t n_port;
                std::memcpy(&n_port, buffer, sizeof n_port);
                port = ntohs(n_port);
            }
            catch (...)
            {
                pclose(pipe_);
                pipe_ = nullptr;
                throw;
            }

            // open socket client
            int sockfd = socket(AF_INET, SOCK_STREAM, 0);
            if (sockfd == -1)
                throw std::runtime_error("socket() failed");

            sockaddr_in serverAddress;
            serverAddress.sin_family = AF_INET;
            serverAddress.sin_port = htons(port);
            if (inet_pton(AF_INET, "127.0.0.1", &serverAddress.sin_addr) != 1) {
                close(sockfd);
                throw std::runtime_error("invalid nav2py loopback address");
            }

            if (connect(sockfd, (struct sockaddr *)&serverAddress, sizeof(serverAddress))) {
                close(sockfd);
                throw std::runtime_error("connect() failed");
            }

            socket_ = sockfd;
            RCLCPP_DEBUG(logger, "socket connected");

            return 0;
        }

        void nav2py_cleanup()
        {
            if (socket_ >= 0) {
                shutdown(socket_, SHUT_RDWR);
                close(socket_);
                socket_ = -1;
            }
            if (pipe_ != nullptr) {
                pclose(pipe_);
                pipe_ = nullptr;
            }
        }
        
        const std::string SEP_BYTE = "\x01";
        const std::string END_BYTE = "\x03";

        void nav2py_send(const std::string & name, const std::vector<std::string> & messages){
            if (socket_ < 0) {
                throw std::runtime_error("nav2py socket is not connected");
            }
            size_t frame_size = name.size() + END_BYTE.size();
            for (const auto & message : messages) {
                frame_size += SEP_BYTE.size() + message.size();
            }
            std::string frame;
            frame.reserve(frame_size);
            frame += name;
            for(const auto& message : messages){
                frame += SEP_BYTE;
                frame += message;
            }
            frame += END_BYTE;
            send_all(socket_, frame.c_str(), frame.size());
        }

        geometry_msgs::msg::Twist wait_for_cmd_vel(std::chrono::milliseconds timeout)
        {
            geometry_msgs::msg::Twist cmd_vel;
            const auto deadline = std::chrono::steady_clock::now() + timeout;
            cmd_vel.linear.x = recv_double(socket_, timeout);
            const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - std::chrono::steady_clock::now());
            cmd_vel.angular.z = recv_double(socket_, remaining);
            return cmd_vel;
        };
    };
}

#endif // NAV2PY__CONTROLLER_HPP_
