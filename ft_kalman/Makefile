# **************************************************************************** #
#                                                                              #
#                                                         :::      ::::::::    #
#    Makefile                                           :+:      :+:    :+:    #
#                                                     +:+ +:+         +:+      #
#    By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+         #
#                                                 +#+#+#+#+#+   +#+            #
#    Created: 2024/04/23 23:34:22 by yhwang            #+#    #+#              #
#    Updated: 2024/06/15 17:18:32 by yhwang           ###   ########.fr        #
#                                                                              #
# **************************************************************************** #

NAME = ft_kalman
CPP = c++

SRC_DIR = ./srcs/
INCS_DIR = ./incs/

CPPFLAGS = -Wall -Wextra -Werror -std=c++17 -g3 -I$(INCS_DIR)
LDFLAGS = -lGLEW -lGL -lglfw -lGLU -ldl -lm

RM = rm -f

FILES = $(SRC_DIR)main \
		$(SRC_DIR)Parse \
		$(SRC_DIR)ServerUtils \
		$(SRC_DIR)FilterUtils \
		$(SRC_DIR)OpenGLUtils

SRCS = $(addsuffix .cpp, $(FILES))
OBJS = $(addsuffix .o, $(FILES))

%.o: %.cpp $(INCS_DIR)
	$(CPP) $(CPPFLAGS) -c $< -o $@

all: $(NAME)

$(NAME): $(OBJS)
	$(CPP) $(CPPFLAGS) $^ -o $@ $(LDFLAGS)

clean:
	$(RM) $(OBJS)

fclean: clean
	$(RM) $(NAME)

re: fclean all

.PHONY: all clean fclean re
