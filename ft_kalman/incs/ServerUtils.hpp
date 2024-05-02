/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ServerUtils.hpp                                    :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2024/04/06 07:16:15 by yhwang            #+#    #+#             */
/*   Updated: 2024/05/02 08:47:51 by yhwang           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef SERVER_UTILS_HPP
# define SERVER_UTILS_HPP

#include <iostream>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstring>
#include <string>
#include <csignal>
#include <vector>
#include <iomanip>
#include <limits>
#include "../incs/Color.hpp"

extern bool		g_running_flag;

void			signalHandler(int signo);
int			createSock(void);
struct sockaddr_in	createServAddr(int port);
bool			sendMsg(int sock, struct sockaddr_in servaddr, std::string msg);
bool			isServAvailable(int sock, int time_sec);
bool			recvFromServ(int sock, std::string &buf);
bool			sendPos(int sock, struct sockaddr_in servaddr, std::vector<double> pos, int timeout);

#endif
