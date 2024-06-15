/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   main.cpp                                           :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2024/04/24 01:27:37 by yhwang            #+#    #+#             */
/*   Updated: 2024/06/15 11:31:26 by yhwang           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "../incs/Parse.hpp"
#include "../incs/KalmanFilter.hpp"
#include "../incs/AdaptiveKalmanFilter.hpp"
#include "../incs/Utils.hpp"
#include "../incs/OpenGLUtils.hpp"

bool	g_running_flag;

int	main(int argc, char **argv)
{
	/* parse argv */
	std::function<void(void)>	print_errmsg = [&](void)
	{
		std::cerr << RED << "error: invalid argument" << BLACK << std::endl << std::endl;
		std::cerr << "USAGE:\n\t./ft_kalman [DURATION] ([OPTIONS])" << std::endl << std::endl;
		std::cerr << "DURATION:\n\tSpecify trajectory duration in minutes from 1 to 90" << std::endl << std::endl;
		std::cerr << "OPTIONS:\n\t--graph:" << std::endl;
		std::cerr << "\t\tshow trajectory visualizer with position and covariance display" << std::endl << std::endl;
		std::cerr << "\t--adaptive:" << std::endl;
		std::cerr << "\t\tcompute with adaptive kalman filter" << std::endl;
	};

	if (!(argc == 2 || argc == 3 || argc == 4))
		return (print_errmsg(), 1);

	if (!(argv[1] && 1 <= atoi(argv[1]) && atoi(argv[1]) <= 90))
		return (print_errmsg(), 1);

	int	duration = atoi(argv[1]);
	bool	flag_graph = false;
	bool	flag_adaptive = false;

	if (argv[2])
	{
		if (strlen(argv[2]) == strlen("--graph") && !strncmp(argv[2], "--graph", strlen("--graph")))
			flag_graph = true;
		else if (strlen(argv[2]) == strlen("--adaptive") && !strncmp(argv[2], "--adaptive", strlen("--adaptive")))
			flag_adaptive = true;
		else
			return (print_errmsg(), 1);

		if (argv[3])
		{
			if (flag_graph == true
				&& strlen(argv[3]) == strlen("--adaptive") && !strncmp(argv[3], "--adaptive", strlen("--adaptive")))
				flag_adaptive = true;
			else if (flag_adaptive == true
				&& strlen(argv[3]) == strlen("--graph") && !strncmp(argv[3], "--graph", strlen("--graph")))
				flag_graph = true;
			else
				return (print_errmsg(), 1);
		}
	}

	/* recieve init values from server */
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
	Parse		p(duration);

	for (size_t i = 0; g_running_flag && i < 7; i++)
	{		
		if (!recvFromServ(client_sock, buf) || !isServAvailable(client_sock, 1))
			return (close(client_sock), 1);
		std::cout << buf << std::endl;
		p.parse(buf);
	}

	std::vector<double>	velocity({p.getSpeed(), 0, 0});
	computeVelocity(p.getDir(), p.getAcc(), velocity);

	/* kalman filter */
	KalmanFilter<double>	*kalman = initFilter(p, velocity, flag_adaptive);

	std::vector<double>	position({kalman->getState().getVector()[0], kalman->getState().getVector()[1], kalman->getState().getVector()[2]});
	std::vector<std::vector<double>>	covariance = kalman->getCovariance().getMatrix();
	int		end_flag = 0;

	Vector<double>	control_input;
	Vector<double>	measurement;

	std::function<bool(void)>	predict = [&](void) -> bool
	{
		/* predict */
		for (size_t i = 0; i < 299; i++)
		{
			if (!parseElement(client_sock, p, "ACCELERATION", end_flag)
				|| !parseElement(client_sock, p, "DIRECTION", end_flag))
				return (false);
			computeVelocity(p.getDir(), p.getAcc(), velocity);

			/* control input: n */
			control_input = Vector<double>({p.getAcc()[0], p.getAcc()[1], p.getAcc()[2]});
			kalman->predict(control_input);
			position = std::vector<double>({kalman->getState().getVector()[0], kalman->getState().getVector()[1], kalman->getState().getVector()[2]});
			if (!sendPos(client_sock, servaddr, position, 1))
				return (false);

			if (end_flag)
				return (false);
		}
		return (true);
	};

	std::function<bool(void)>	update = [&](void) -> bool
	{
		/* update */
		if (!parseElement(client_sock, p, "POSITION", end_flag)
			|| !parseElement(client_sock, p, "ACCELERATION", end_flag)
			|| !parseElement(client_sock, p, "DIRECTION", end_flag))
			return (false);
		computeVelocity(p.getDir(), p.getAcc(), velocity);
		control_input = Vector<double>({p.getAcc()[0], p.getAcc()[1], p.getAcc()[2]});
		kalman->predict(control_input);

		/* measurement: m */
		measurement = Vector<double>({p.getPos()[0], p.getPos()[1], p.getPos()[2]});
		kalman->update(measurement);
		position = std::vector<double>({kalman->getState().getVector()[0], kalman->getState().getVector()[1], kalman->getState().getVector()[2]});
		covariance = kalman->getCovariance().getMatrix();
		if (!sendPos(client_sock, servaddr, position, 1))
			return (false);
		return (true);	
	};

	if (!flag_graph) /* without graph */
	{
		std::function<void(int)>	exit_program = [&](int exit_code)
		{
			delete kalman;
			close(client_sock);
			exit(exit_code);
		};

		if (!sendPos(client_sock, servaddr, position, 1))
			exit_program(1);

		while (g_running_flag)
		{
			if (!predict() || !update())
			{
				if (end_flag)
				{
					std::cout << YELLOW << "* TRAJECTORY END *" << BLACK << std::endl;
					break ;
				}
				else
					exit_program(1);
			}
		}
		exit_program(0);
	}
	else /* with graph */
	{
		std::function<void(t_opengl, int)>	exit_program = [&](t_opengl ctx, int exit_code)
		{
			delete kalman;
			close(client_sock);
			if (flag_graph)
			{
				glDeleteBuffers(1, &ctx.VBO_position);
				glDeleteBuffers(1, &ctx.VBO_covariance_p);
				glDeleteBuffers(1, &ctx.VBO_covariance_v);
				glfwDestroyWindow(ctx.window);
				glfwTerminate();
			}
			exit(exit_code);
		};

		t_opengl	ctx;

		if (!init_opengl(ctx))
			return (close(client_sock), 1);

		if (!sendPos(client_sock, servaddr, position, 1))
			exit_program(ctx, 1);
		
		update_position_graph(ctx, position);
		update_covariance_graph(ctx, covariance);
		render(ctx);

		while (!glfwWindowShouldClose(ctx.window) && g_running_flag)
		{
			glfwPollEvents();
			if (!predict() || !update())
			{
				if (end_flag)
				{
					std::cout << YELLOW << "* TRAJECTORY END *" << BLACK << std::endl;
					break ;
				}
				else
					exit_program(ctx, 1);
			}
			update_position_graph(ctx, position);
			update_covariance_graph(ctx, covariance);
			render(ctx);
		}
		g_running_flag = false;
		while (!glfwWindowShouldClose(ctx.window))
		{
			glfwPollEvents();
			render(ctx);
		}
		exit_program(ctx, 0);
	}
	return (0);
}
