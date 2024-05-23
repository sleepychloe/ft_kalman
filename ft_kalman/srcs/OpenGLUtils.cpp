/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   OpenGLUtils.cpp                                    :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2024/05/20 16:06:59 by yhwang            #+#    #+#             */
/*   Updated: 2024/05/24 01:20:56 by yhwang           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "../incs/OpenGLUtils.hpp"
#include "../incs/Color.hpp"

void	scroll_callback(GLFWwindow *window, double xoffset, double yoffset)
{
	t_opengl	*ctx = reinterpret_cast<t_opengl *>(glfwGetWindowUserPointer(window));
	if (yoffset > 0)
		ctx->data.zoom *= exp(-0.1);
	else if (yoffset < 0)
		ctx->data.zoom *= exp(0.1);
	(void)xoffset;
	(void)yoffset;
}

void	key_callback(GLFWwindow *window, int key, int scancode, int action, int mods)
{
	t_opengl	*ctx = reinterpret_cast<t_opengl *>(glfwGetWindowUserPointer(window));
	if (action == GLFW_PRESS || action == GLFW_REPEAT)
	{
		if (key == GLFW_KEY_A)
			ctx->data.camera_lookat_x -= 200;
		else if (key == GLFW_KEY_D)
			ctx->data.camera_lookat_x += 200;
		else if (key == GLFW_KEY_S)
			ctx->data.camera_lookat_y -= 200;
		else if (key == GLFW_KEY_W)
			ctx->data.camera_lookat_y += 200;
		else if (key == GLFW_KEY_DOWN)
			ctx->data.camera_lookat_z -= 200;
		else if (key == GLFW_KEY_UP)
			ctx->data.camera_lookat_z += 200;
	}
	(void)scancode;
	(void)mods;
}

void	init_data(t_opengl &ctx)
{
	ctx.data.zoom = 1.0f;
	ctx.data.camera_lookat_x = 14500;
	ctx.data.camera_lookat_y = -1000;
	ctx.data.camera_lookat_z = -2000;
}

bool	init_opengl(t_opengl &ctx)
{
	init_data(ctx);
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
	glfwSetWindowUserPointer(ctx.window, &ctx);
	glfwSetScrollCallback(ctx.window, scroll_callback);
	glfwSetKeyCallback(ctx.window, key_callback);

	if (glewInit() != GLEW_OK)
	{
		std::cerr << RED << "error: failed to initialize GLFW" << BLACK << std::endl;
		return (false);
	}

	glGenVertexArrays(1, &ctx.VAO);
	glGenBuffers(1, &ctx.VBO);
	glBindVertexArray(ctx.VAO);
	glBindBuffer(GL_ARRAY_BUFFER, ctx.VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(t_point3) * ctx.pos_estimate.size(), nullptr, GL_DYNAMIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(t_point3), (void *)0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	glClearColor(1.0f, 1.0f, 1.0f, 1.0f); // white background
	return (true);
}

void	setup_view(t_opengl &ctx, int view_type)
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluPerspective(20 * ctx.data.zoom, 1 / 1, 1, 10000000000);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	if (view_type == 0) // 3d view
		gluLookAt(45000 / ctx.data.zoom, -45000 / ctx.data.zoom, 45000 / ctx.data.zoom, // camera pos_estimate
			ctx.data.camera_lookat_x / ctx.data.zoom, ctx.data.camera_lookat_y / ctx.data.zoom, ctx.data.camera_lookat_z / ctx.data.zoom, // looks at
			0, 0, 1); // Carmera vector
	if (view_type == 1)
		gluLookAt(0, 0, 90000 / ctx.data.zoom, // camera pos_estimate
			14000, 0, 0, // looks at
			0, 1, 0); // Carmera vector
	if (view_type == 2)
		gluLookAt(0, -90000 / ctx.data.zoom, 0,
			14000, 0, 0,
			0, 0, 1);
}

void	update_graph(t_opengl &ctx, const std::vector<double> &pos)
{
	t_point3	p = {static_cast<GLfloat>(pos[0]), static_cast<GLfloat>(pos[1]), static_cast<GLfloat>(pos[2])};

	ctx.pos_estimate.push_back(p);
	glBindBuffer(GL_ARRAY_BUFFER, ctx.VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(t_point3) * ctx.pos_estimate.size(), ctx.pos_estimate.data(), GL_DYNAMIC_DRAW);
}

void	draw_axis(double length)
{
	glBegin(GL_LINES);

	glColor3f(1, 0, 0); // red
	glVertex3f(0, 0, 0);
	glVertex3f(length, 0, 0); // +x

	glColor3f(0, 1, 0); // green
	glVertex3f(0, 0, 0);
	glVertex3f(0, length, 0); // +y
	glVertex3f(0, 0, 0);
	glVertex3f(0, -length, 0); // -y

	glColor3f(0, 0, 1); // blue
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0, length); // +z
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0, -length); // -z
	glEnd();
}

void	render(t_opengl &ctx)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glViewport(0, 200, 400, 400);
	setup_view(ctx, XYZ);
	draw_axis(10000000000);
	glBindVertexArray(ctx.VAO);
	glPointSize(3);
	glColor3f(1, 0, 1);
	glDrawArrays(GL_POINTS, 0, ctx.pos_estimate.size());
	glBindVertexArray(0);

	glViewport(0, 0, 200, 200);
	setup_view(ctx, XY);
	draw_axis(10000000000);
	glBindVertexArray(ctx.VAO);
	glPointSize(3);
	glColor3f(1, 0, 1);
	glDrawArrays(GL_POINTS, 0, ctx.pos_estimate.size());
	glBindVertexArray(0);

	glViewport(200, 0, 200, 200);
	setup_view(ctx, XZ);
	draw_axis(10000000000);
	glBindVertexArray(ctx.VAO);
	glPointSize(3);
	glColor3f(1, 0, 1);
	glDrawArrays(GL_POINTS, 0, ctx.pos_estimate.size());
	glBindVertexArray(0);

	glfwSwapBuffers(ctx.window);
}
