/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   OpenGLUtils.cpp                                    :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2024/05/20 16:06:59 by yhwang            #+#    #+#             */
/*   Updated: 2024/05/20 17:59:47 by yhwang           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "../incs/OpenGLUtils.hpp"
#include "../incs/Color.hpp"

bool	init_opengl(OpenGL &ctx)
{
	if (!glfwInit())
	{
		std::cerr << RED << "error: failed to initialize GLFW" << BLACK << std::endl;
		return (false);
	}

	ctx.window = glfwCreateWindow(WINODW_WIDTH, WINDOW_HEIGHT, "Kalman Filter", NULL, NULL);
	if (!ctx.window)
	{
		glfwTerminate();
		std::cerr << RED << "error: failed to create GLFW window" << BLACK << std::endl;
		return (false);
	}

	glfwMakeContextCurrent(ctx.window);
	if (glewInit() != GLEW_OK)
	{
		std::cerr << RED << "error: failed to initialize GLFW" << BLACK << std::endl;
		return (false);
	}

	glGenVertexArrays(1, &ctx.VAO);
	glGenBuffers(1, &ctx.VBO);
	glBindVertexArray(ctx.VAO);
	glBindBuffer(GL_ARRAY_BUFFER, ctx.VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Point3) * ctx.position.size(), nullptr, GL_DYNAMIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Point3), (void *)0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	glClearColor(1.0f, 1.0f, 1.0f, 1.0f); // white background
	return (true);
}

void	setup_view(void)
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(90, WINODW_WIDTH / WINDOW_HEIGHT, 0.1, 100000);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(12000, -10000, 10000, // camera position
		3000, -6000, -10000, // looks at the origin
		0, 0, 1); // Carmera vector
}

void	update_graph(OpenGL &ctx, const std::vector<double> &position)
{
	Point3	p = {static_cast<GLfloat>(position[0]), static_cast<GLfloat>(position[1]), static_cast<GLfloat>(position[2])};

	ctx.position.push_back(p);
	glBindBuffer(GL_ARRAY_BUFFER, ctx.VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Point3) * ctx.position.size(), ctx.position.data(), GL_DYNAMIC_DRAW);
}

void	draw_axis(double length)
{
	glBegin(GL_LINES);

	glColor3f(1, 0, 0); // red
	glVertex3f(0, 0, 0);
	glVertex3f(length, 0, 0); // +x

	// glVertex3f(0, 0, 0);
	// glVertex3f(-length, 0, 0); // -x

	glColor3f(0, 1, 0); // green
	glVertex3f(0, 0, 0);
	glVertex3f(0, length, 0); // +y

	// glVertex3f(0, 0, 0);
	// glVertex3f(0, -length, 0); // -y

	glColor3f(0, 0, 1); // blue
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0, length); // +z

	// glVertex3f(0, 0, 0);
	// glVertex3f(0, 0, -length); // -z

	glEnd();
}

void	render(OpenGL &ctx)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	setup_view();
	draw_axis(1000000.0);

	glBindVertexArray(ctx.VAO);
	glPointSize(2);
	glEnableVertexAttribArray(0);

	glColor3f(1, 0, 1);
	glDrawArrays(GL_POINTS, 0, ctx.position.size());
	glBindVertexArray(0);

	glfwSwapBuffers(ctx.window);
}
