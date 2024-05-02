/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   main.cpp                                           :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2024/04/24 01:27:37 by yhwang            #+#    #+#             */
/*   Updated: 2024/05/02 08:56:08 by yhwang           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "../incs/Parse.hpp"
#include "../incs/KalmanFilter.hpp"
#include "../incs/ServerUtils.hpp"
#include "../incs/FilterUtils.hpp"

bool	g_running_flag;

int	main(int argc, char **argv)
{
	if (argc != 1)
	{
		std::cerr << RED << "error: invalid argument" << BLACK << std::endl;
		return (1);
	}
	(void)argv;

	g_running_flag = true;

	int			client_sock = createSock();
	struct sockaddr_in	servaddr = createServAddr(4242);

	signal(SIGINT, signalHandler);
    	signal(SIGQUIT, signalHandler);

	std::string	msg = "READY";
	if (!sendMsg(client_sock, servaddr, msg) || !isServAvailable(client_sock, 1))
		return (close(client_sock), 1);
	std::cout << CYAN << "successfully sent message to server: " << msg << BLACK << std::endl;

	std::string	buf;
	Parse		p;

	for (size_t i = 0; g_running_flag && i < 7; i++)
	{		
		if (!recvFromServ(client_sock, buf) || !isServAvailable(client_sock, 3))
			return (close(client_sock), 1);
		std::cout << buf << std::endl;
		p.parse(buf);
	}
	std::cout << YELLOW << "initial values" << BLACK << std::endl;
	p.print();

	std::vector<double>	velocity({p.getSpeed(), 0, 0});
	computeVelocity(p.getDir(), p.getAcc(), velocity);

	KalmanFilter<double>	kalman;
	initFilter(p, velocity, kalman);

	if (!sendPos(client_sock, servaddr, kalman.getState().getVector(), 1))
		return (close(client_sock), 1);

	while (g_running_flag)
	{
		/* predict */
		for (size_t i = 0; i < 299; i++)
		{
			while (buf.find("ACCELERATION") == std::string::npos)
			{
				if (!recvFromServ(client_sock, buf) || !isServAvailable(client_sock, 1))
					return (close(client_sock), 1);
				std::cout << buf << std::endl;
			}
			p.parse(buf);
			while (buf.find("DIRECTION") == std::string::npos)
			{
				if (!recvFromServ(client_sock, buf) || !isServAvailable(client_sock, 1))
					return (close(client_sock), 1);
				std::cout << buf << std::endl;
			}
			p.parse(buf);
			computeVelocity(p.getDir(), p.getAcc(), velocity);

			kalman.predict();

			if (!sendPos(client_sock, servaddr, kalman.getState().getVector(), 1))
				return (close(client_sock), 1);
		}

		/* update */
		while (buf.find("POSITION") == std::string::npos)
		{
			if (!recvFromServ(client_sock, buf) || !isServAvailable(client_sock, 1))
				return (close(client_sock), 1);
			std::cout << buf << std::endl;
		}
		p.parse(buf);

		while (buf.find("ACCELERATION") == std::string::npos)
		{
			if (!recvFromServ(client_sock, buf) || !isServAvailable(client_sock, 1))
				return (close(client_sock), 1);
			std::cout << buf << std::endl;
		}
		p.parse(buf);
		while (buf.find("DIRECTION") == std::string::npos)
		{
			if (!recvFromServ(client_sock, buf) || !isServAvailable(client_sock, 1))
				return (close(client_sock), 1);
			std::cout << buf << std::endl;
		}
		p.parse(buf);
		computeVelocity(p.getDir(), p.getAcc(), velocity);

		// measurement: 1 * m
		Vector<double>	measurement({p.getPos()[0], p.getPos()[1], p.getPos()[2], p.getAcc()[0], p.getAcc()[1], p.getAcc()[2]});
		kalman.update(measurement);
		kalman.predict();

		if (!sendPos(client_sock, servaddr, kalman.getState().getVector(), 1))
			return (close(client_sock), 1);
	}
	close(client_sock);
	return (0);
}
