/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   Utils.hpp                                          :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2024/04/06 07:16:15 by yhwang            #+#    #+#             */
/*   Updated: 2024/04/26 16:44:42 by yhwang           ###   ########.fr       */
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
#include "../incs/Color.hpp"

extern bool		g_running_flag;

void			signal_handler(int signo);
int			create_sock(void);
struct sockaddr_in	create_sockaddr_in(int port);
bool			send_msg(int sock, struct sockaddr_in servaddr, std::string msg);
bool			is_serv_available(int sock, int time_sec);
bool			recv_from_serv(int sock, std::string &buf);

#endif
