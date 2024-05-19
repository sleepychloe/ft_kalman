/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   main.cpp                                           :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2024/04/24 01:27:37 by yhwang            #+#    #+#             */
/*   Updated: 2024/05/19 08:10:30 by yhwang           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "../incs/Parse.hpp"
#include "../incs/KalmanFilter.hpp"
#include "../incs/Utils.hpp"

bool	g_running_flag;

// int	main(int argc, char **argv)
// {
// 	if (argc != 1)
// 	{
// 		std::cerr << RED << "error: invalid argument" << BLACK << std::endl;
// 		return (1);
// 	}
// 	(void)argv;

// 	g_running_flag = true;

// 	int			client_sock = createSock();
// 	struct sockaddr_in	servaddr = createServAddr(4242);

// 	signal(SIGINT, signalHandler);
//     	signal(SIGQUIT, signalHandler);

// 	std::string	msg = "READY";
// 	if (!sendMsg(client_sock, servaddr, msg) || !isServAvailable(client_sock, 1))
// 		return (close(client_sock), 1);
// 	std::cout << CYAN << "successfully sent message to server: " << msg << BLACK << std::endl;

// 	std::string	buf;
// 	Parse		p;

// 	for (size_t i = 0; g_running_flag && i < 7; i++)
// 	{		
// 		if (!recvFromServ(client_sock, buf) || !isServAvailable(client_sock, 1))
// 			return (close(client_sock), 1);
// 		std::cout << buf << std::endl;
// 		p.parse(buf);
// 	}

// 	std::vector<double>	velocity({p.getSpeed(), 0, 0});
// 	computeVelocity(p.getDir(), p.getAcc(), velocity);

// 	KalmanFilter<double>	kalman;
// 	initFilter(p, velocity, kalman);

// 	if (!sendPos(client_sock, servaddr, kalman.getState().getVector(), 1))
// 		return (close(client_sock), 1);

// 	Vector<double>	control_input;
// 	Vector<double>	measurement;
// 	std::vector<double>	keep_a;

// 	while (g_running_flag)
// 	{
// 		/* predict */
// 		for (size_t i = 0; i < 299; i++)
// 		{
// 			if (!parseElement(client_sock, p, "ACCELERATION")
// 				|| !parseElement(client_sock, p, "DIRECTION"))
// 				return (close(client_sock), 1);
// 			computeVelocity(p.getDir(), p.getAcc(), velocity);
// 			keep_a = p.getAcc();

// 			/* control input: n */
// 			control_input = Vector<double>({p.getAcc()[0], p.getAcc()[1], p.getAcc()[2]});
// 			kalman.predict(control_input);
// 			if (!sendPos(client_sock, servaddr, kalman.getState().getVector(), 1))
// 				return (close(client_sock), 1);
// 		}
// 		/* update */
// 		if (!parseElement(client_sock, p, "POSITION")
// 			|| !parseElement(client_sock, p, "ACCELERATION")
// 			|| !parseElement(client_sock, p, "DIRECTION"))
// 			return (close(client_sock), 1);
// 		computeVelocity(p.getDir(), p.getAcc(), velocity);

// 		control_input = Vector<double>({p.getAcc()[0], p.getAcc()[1], p.getAcc()[2]});
// 		kalman.predict(control_input);

// 		/* measurement: m */
// 		measurement = Vector<double>({p.getPos()[0], p.getPos()[1], p.getPos()[2]});
// 		kalman.update(measurement);
// 		if (!sendPos(client_sock, servaddr, kalman.getState().getVector(), 1))
// 			return (close(client_sock), 1);
// 	}
// 	close(client_sock);
// 	return (0);
// }


#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <stdio.h>

void error_callback(int error, const char* description) {
    fprintf(stderr, "Error: %s\n", description);
    (void)error;
}

void setup_triangle() {
    GLfloat vertices[] = {
        -0.6f, -0.4f, 0.0f,
         0.6f, -0.4f, 0.0f,
         0.0f,  0.6f, 0.0f
    };

    GLuint VBO;
    glGenBuffers(1, &VBO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
}

void render_triangle() {
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glDisableVertexAttribArray(0);
}

int main(void) {
    GLFWwindow* window;

    glfwSetErrorCallback(error_callback);
    if (!glfwInit()) {
        return -1;
    }

    // Ensure the window will be decorated
    glfwWindowHint(GLFW_DECORATED, GLFW_TRUE);

    window = glfwCreateWindow(640, 480, "Simple Triangle", NULL, NULL);
    if (!window) {
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    glewInit();

    setup_triangle();

    while (!glfwWindowShouldClose(window)) {
        glClear(GL_COLOR_BUFFER_BIT);
        render_triangle();
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
