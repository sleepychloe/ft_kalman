/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   Utils.hpp                                          :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2024/04/06 07:16:15 by yhwang            #+#    #+#             */
/*   Updated: 2024/05/02 09:46:22 by yhwang           ###   ########.fr       */
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
#include "../matrix/incs/Vector.hpp"
#include "../matrix/incs/Matrix.hpp"
#include "../incs/Parse.hpp"
#include "../incs/KalmanFilter.hpp"
#include "../incs/Color.hpp"

# define	DT			0.01
# define 	ACCELEROMETER_NOISE	0.001
# define	GYROSCOPE_NOISE		0.01
# define	GPS_NOISE		0.1

extern bool		g_running_flag;

/* ServerUtils.cpp */
void			signalHandler(int signo);
int			createSock(void);
struct sockaddr_in	createServAddr(int port);
bool			sendMsg(int sock, struct sockaddr_in servaddr, std::string msg);
bool			isServAvailable(int sock, int time_sec);
bool			recvFromServ(int sock, std::string &buf);
bool			sendPos(int sock, struct sockaddr_in servaddr, std::vector<double> pos, int timeout);
bool			parseElement(int sock, Parse &p, std::string element);

/* FilterUtils.cpp */
void			computeVelocity(std::vector<double> d,
				std::vector<double> a, std::vector<double> &v);
void			initFilter(Parse &p, std::vector<double> v, KalmanFilter<double> &kalman);

#endif
