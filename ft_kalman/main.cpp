/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   main.cpp                                           :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2024/04/24 01:27:37 by yhwang            #+#    #+#             */
/*   Updated: 2024/04/24 14:37:44 by yhwang           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "./Utils.hpp"

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

	while (g_running_flag)
	{
		if (!recv_from_serv(client_sock, buf) || !is_serv_available(client_sock, 3))
			return (close(client_sock), 1);
		//parse
	}
	close(client_sock);
	return (0);
}
