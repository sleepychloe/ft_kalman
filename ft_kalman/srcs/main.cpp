/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   main.cpp                                           :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2024/04/24 01:27:37 by yhwang            #+#    #+#             */
/*   Updated: 2024/06/14 01:51:36 by yhwang           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "../incs/Parse.hpp"
#include "../incs/KalmanFilter.hpp"
#include "../incs/Utils.hpp"
#include "../incs/OpenGLUtils.hpp"

bool	g_running_flag;

int	main(int argc, char **argv)
{
	if (!(argc == 2 || argc == 3))
	{
		std::cerr << RED << "error: invalid argument" << BLACK << std::endl;
		std::cerr << RED << "usage: ./ft_kalman [duration] (--graph)" << BLACK << std::endl;
		std::cerr << RED << "       (1 <= duration <= 90)" << BLACK << std::endl;
		return (1);
	}

	if (!(argv[1] && 1 <= atoi(argv[1]) && atoi(argv[1]) <= 90))
	{
		std::cerr << RED << "error: invalid range of duration" << BLACK << std::endl;
		std::cerr << RED << "usage: ./ft_kalman [duration] (--graph)" << BLACK << std::endl;
		std::cerr << RED << "       (1 <= duration <= 90)" << BLACK << std::endl;
		return (1);
	}
	int	duration = atoi(argv[1]);

	bool	flag = false;

	if (argv[2] && strlen(argv[2]) == 7 && !strncmp(argv[2], "--graph", 7))
		flag = true;
	else if (argc == 2)
		flag = false;
	else
	{
		std::cerr << RED << "error: invalid argument" << BLACK << std::endl;
		std::cerr << RED << "usage: ./ft_kalman [duration] (--graph)" << BLACK << std::endl;
		std::cerr << RED << "       (1 <= duration <= 90)" << BLACK << std::endl;
		return (1);
	}

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

	std::function<void(t_opengl, int)>	exit_program = [&](t_opengl ctx, int exit_code)
	{
		close(client_sock);
		if (flag)
		{
			glDeleteBuffers(1, &ctx.VBO_position);
			glDeleteBuffers(1, &ctx.VBO_covariance_p);
			glDeleteBuffers(1, &ctx.VBO_covariance_v);
			glfwDestroyWindow(ctx.window);
			glfwTerminate();
		}
		exit(exit_code);
	};

	KalmanFilter<double>	kalman;
	initFilter(p, velocity, kalman);

	std::vector<double>	position({kalman.getState().getVector()[0], kalman.getState().getVector()[1], kalman.getState().getVector()[2]});
	std::vector<std::vector<double>>	covariance = kalman.getCovariance().getMatrix();
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
			kalman.predict(control_input);
			position = std::vector<double>({kalman.getState().getVector()[0], kalman.getState().getVector()[1], kalman.getState().getVector()[2]});
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
		kalman.predict(control_input);

		/* measurement: m */
		measurement = Vector<double>({p.getPos()[0], p.getPos()[1], p.getPos()[2]});
		kalman.update(measurement);
		position = std::vector<double>({kalman.getState().getVector()[0], kalman.getState().getVector()[1], kalman.getState().getVector()[2]});
		covariance = kalman.getCovariance().getMatrix();
		if (!sendPos(client_sock, servaddr, position, 1))
			return (false);
		return (true);	
	};

	if (!flag)
	{
		std::function<void(int)>	exit_program = [&](int exit_code)
		{
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
	else
	{
		std::function<void(t_opengl, int)>	exit_program = [&](t_opengl ctx, int exit_code)
		{
			close(client_sock);
			if (flag)
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
