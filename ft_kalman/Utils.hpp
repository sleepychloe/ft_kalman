/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   Utils.hpp                                          :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2024/04/06 07:16:15 by yhwang            #+#    #+#             */
/*   Updated: 2024/04/24 14:34:26 by yhwang           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef UTILS_HPP
# define UTILS_HPP

#include <iostream>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstring>
#include <string>
#include <csignal>

# define BLACK			"\x1b[0m"
# define RED			"\x1b[31m"
# define CYAN			"\x1b[36m"
# define YELLOW			"\x1b[33m"
# define MAGENTA		"\x1b[35m"

extern bool		g_running_flag;

void			signal_handler(int signo);
int			create_sock(void);
struct sockaddr_in	create_sockaddr_in(int port);
bool			send_handshake(int sock, struct sockaddr_in servaddr);
bool			is_serv_available(int sock, int time_sec);
bool			recv_from_serv(int sock, std::string &buf);

#endif
