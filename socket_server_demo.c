/*
 * main.c
 *
 *  Created on: Mar 16, 2020
 *      Author: lay
 */
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>

#include <arpa/inet.h>
#include <netinet/in.h>

#include <sys/socket.h>
#include <sys/epoll.h>

#define LISTEN_Q_LEN        10
#define MAX_EPOLL_EVENTS    10

#define BUF_LEN             (1024 * 300)

static unsigned char _rx_buf[BUF_LEN];
static unsigned char _tx_buf[BUF_LEN];
static int _main_connection_fd = -1;

extern int accept4 (int __fd, __SOCKADDR_ARG __addr, socklen_t *__restrict __addr_len, int __flags);

static int _send_data(int fd)
{
    int ret;
    int n_bytes = write(fd, _tx_buf, sizeof(_tx_buf));
    if (n_bytes <= 0) {
        printf("send data failed, fd = %d\n", fd);
        ret = -1;
    } else {
        printf("send %d bytes, fd = %d\n", n_bytes, fd);
        ret = 0;
    }
    return ret;
}

static int _read_data(int fd)
{
    int ret;
    int n_bytes = read(fd, _rx_buf, sizeof(_rx_buf));
    if (n_bytes <= 0) {
        printf("read data failed, fd = %d\n", fd);
        ret = -1;
    } else {
        printf("read %d bytes, fd = %d\n", n_bytes, fd);
        ret = 0;
    }
    return ret;
}

static void _disconnect(int epollfd, int fd)
{
    struct epoll_event event;
    int ret = epoll_ctl(epollfd, EPOLL_CTL_DEL, fd, &event);
    if (ret != 0) {
        printf("delete socket fd frome epoll failed\n");
    }

    if (fd == _main_connection_fd) {
        _main_connection_fd = -1;
    }
    close(fd);
    printf("connection is closed, fd = %d\n", fd);
}

int main(int argc, const char *argv[])
{
    int ret = -1;
    int port = 18063;
    int sockfd;
    int epollfd;
    struct epoll_event event;
    struct epoll_event events[MAX_EPOLL_EVENTS];
    struct sockaddr_in6 server_addr;
    struct sockaddr_in6 client_addr;
    socklen_t client_addr_len = 0;


    sockfd = socket(AF_INET6, SOCK_STREAM | SOCK_NONBLOCK, 0);
    if (sockfd < 0) {
        printf("create socket failed\n");
    }
    else {
        ret = 0;
    }

    if (ret == 0) {
        int val = 1;
        ret = setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (void*) (&val), sizeof(val));
    }

    if (ret == 0) {
        bzero(&server_addr, sizeof(server_addr));
        server_addr.sin6_family = PF_INET6;
        server_addr.sin6_port = htons(port);
        server_addr.sin6_addr = in6addr_any;
        ret = bind(sockfd, (struct sockaddr*) &server_addr, sizeof(server_addr));
    }
    else {
        printf("set socket to reuse address mode failed\n");
    }

    if (ret == 0) {
        ret = listen(sockfd, LISTEN_Q_LEN);
    }
    else {
        printf("bind socket failed\n");
    }

    if (ret == 0) {
        /* finished of socket initialization */
        printf("socket initialization [finished]\n");
        epollfd = epoll_create(MAX_EPOLL_EVENTS);
        if (epollfd < 0) {
            ret = -1;
        }
    } else {
        printf("listen socket failed\n");
    }

    if (ret == 0) {
        event.events = EPOLLIN;
        event.data.fd = sockfd;
        ret = epoll_ctl(epollfd, EPOLL_CTL_ADD, sockfd, &event);
    } else {
        printf("crate epoll failed\n");
    }

    if (ret == 0) {
        int timeout = -1; /* no timeout */
        printf("main loop [stared]\n");
        for(; ret == 0;) {
            int num = epoll_wait(epollfd, events, MAX_EPOLL_EVENTS, timeout);
            if (num > 0) {
                int i;
                for (i = 0; i < num; i++) {
                    if (events[i].data.fd == sockfd) {
                        /* connect event */
                        if (events[i].events & (EPOLLIN | EPOLLPRI)) {
                            char client_addr_str[128];
                            bzero(&client_addr, sizeof(client_addr));
                            /* accept client connection */
                            int confd = accept4(sockfd, (struct sockaddr *)&client_addr, &client_addr_len, SOCK_NONBLOCK);
                            if (confd < 0) {
                                ret = -1;
                                printf("accept socket failed\n");
                                break;
                            }

                            if (inet_ntop( AF_INET6, &client_addr.sin6_addr, client_addr_str, sizeof(client_addr_str) - 1)) {
                                printf("new client(%s) connected\n", client_addr_str);
                            } else {
                                printf("convert net data to ip string failed\n");
                            }

                            if (_main_connection_fd > 0) {
                                printf("old connection is valid [refused]\n");
                                close(confd);
                            } else {
                                /* add new client socket fd */
                                event.data.fd = confd;
                                event.events = EPOLLIN | EPOLLOUT | EPOLLERR | EPOLLHUP;
                                if (epoll_ctl(epollfd, EPOLL_CTL_ADD, confd, &event) < 0) {
                                    ret = -1;
                                    close(confd);
                                    printf("add new client socket to epoll failed\n");
                                    break;
                                } else {
                                    _main_connection_fd = confd;
                                    timeout = 500; /* 500ms timeout to check the client connection */
                                }
                            }
                        }
                    } else {
                        /* data event */
                        if ((events[i].data.fd > 0)) {
                            if (events[i].events & EPOLLIN) {
                                /* read client data */
                                if (_read_data(events[i].data.fd) == 0) {

                                } else {
                                    _disconnect(epollfd, events[i].data.fd);
                                    events[i].data.fd = -1;
                                    timeout = -1;
                                }
                            } else if (events[i].events & (EPOLLERR | EPOLLHUP)) {
                                /* socket error event */
                                _disconnect(epollfd, events[i].data.fd);
                                events[i].data.fd = -1;
                                timeout = -1;
                            }
                        }

                        if ((events[i].events & EPOLLOUT) && (events[i].data.fd > 0)) {
                            /* send data to client */
                            if (_send_data(events[i].data.fd) == 0) {

                            } else {
                                _disconnect(epollfd, events[i].data.fd);
                                events[i].data.fd = -1;
                                timeout = -1;
                            }
                        }
                    }
                }
            } else if (num == 0) {
                /* timeout code */
                if (_main_connection_fd > 0) {
                    printf("timeout for connection, fd = %d\n", _main_connection_fd);
                    _disconnect(epollfd, _main_connection_fd);
                    timeout = -1;
                }
            } else {
                printf("epoll wait failed\n");
                ret = -1;
                break;
            }
        }
    } else {
        printf("add socket fd to epoll failed\n");
    }

    if (epollfd > 0) {
        printf("close epoll fd\n");
        close(epollfd);
    }

    if (_main_connection_fd > 0) {
        printf("close main connection fd\n");
        close(_main_connection_fd);
    }

    if (sockfd > 0) {
        printf("close socket fd\n");
        close(sockfd);
    }

    printf("main loop [exited]\n");
    return ret;
}
