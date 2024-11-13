//
// Created by 陈鑫龙 on 2021/10/15.
//

#ifndef USBL_COMMON_H
#define USBL_COMMON_H

#include <iostream>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <string>
#include <unistd.h>
#include <errno.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#define Max_Buffer_Size 1024

/* error - print a diagnostic and optionally exit */
void error(int status, int err, char *fmt, ...) {
    va_list ap;

    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    if (err) {
        fprintf(stderr, ": %s (%d)\n", strerror(err), err);
    }
    if (status) {
        exit(status);
    }
}

int tcp_client(char *address, int port) {
    int socket_fd;
    socket_fd = socket(AF_INET, SOCK_STREAM, 0);

    struct sockaddr_in server_addr;
    bzero(&server_addr, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    inet_pton(AF_INET, address, &server_addr.sin_addr);

    socklen_t server_len = sizeof(server_addr);
    int connect_rt = connect(socket_fd, (struct sockaddr *) &server_addr, server_len);
    if (connect_rt < 0) {
        error(1, errno, "connect failed ");
    }

    return socket_fd;
}

int read_data(int fd, char* buffer, int size) {
    int ret = recv(fd, buffer, size - 1, 0);
    if (ret == -1 || (strncmp(buffer, "end", 3) == 0))
    {
        return -1;
    }
    return ret;
}

void send_data(int fd, const char* buffer, int size)
{
    int ret = send(fd, buffer, size, 0);
    printf ("send size: %d, real size: %d\n",ret, size);
}

#endif //USBL_COMMON_H
