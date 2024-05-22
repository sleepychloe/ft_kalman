/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   OpenGLUtils.hpp                                    :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2024/05/20 09:05:26 by yhwang            #+#    #+#             */
/*   Updated: 2024/05/20 17:24:33 by yhwang           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef OPENGL_UTILS_HPP
# define OPENGL_UTILS_HPP

#include <iostream>
#include <vector>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

# define	WINODW_WIDTH	800
# define	WINDOW_HEIGHT	600

struct Point3
{
	GLfloat	x;
	GLfloat	y;
	GLfloat	z;
};

struct OpenGL
{
	GLFWwindow		*window;
	GLuint			VAO;
	GLuint			VBO;
	std::vector<Point3>	position;
};

bool			init_opengl(OpenGL &ctx);
void			setup_view(void);
void			update_graph(OpenGL &ctx, const std::vector<double> &position);
void			draw_axis(double length);
void			render(OpenGL &ctx);

#endif
