/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   main.cpp                                           :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2024/04/24 01:27:37 by yhwang            #+#    #+#             */
/*   Updated: 2024/04/25 17:06:17 by yhwang           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "../incs/Parse.hpp"
#include "../incs/Utils.hpp"
// #include "../incs/KalmanFilter.hpp"

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

	int			client_sock = create_sock();
	struct sockaddr_in	servaddr = create_sockaddr_in(4242);

	signal(SIGINT, signal_handler);
    	signal(SIGQUIT, signal_handler);

	if (!send_handshake(client_sock, servaddr) || !is_serv_available(client_sock, 1))
		return (close(client_sock), 1);

	std::string	buf;
	Parse		p;
	int		i = -1;

	while (g_running_flag && ++i < 7)
	{		
		if (!recv_from_serv(client_sock, buf) || !is_serv_available(client_sock, 3))
			return (close(client_sock), 1);
		std::cout << buf << std::endl;
		p.parse(buf);
	}
	std::cout << YELLOW << "initial values" << BLACK << std::endl;
	p.print();
	/*
		1. make kalman filter class -done
		2. test kalman filter class
		3. set matrices for calculating
		4. make function to send position data to the server
		5. make another while loop to send calculated data to server, 
			recieve it, re-calculate, send it back, ...
	*/
	close(client_sock);
	return (0);
}
