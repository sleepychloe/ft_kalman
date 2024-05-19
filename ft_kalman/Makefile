# **************************************************************************** #
#                                                                              #
#                                                         :::      ::::::::    #
#    Makefile                                           :+:      :+:    :+:    #
#                                                     +:+ +:+         +:+      #
#    By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+         #
#                                                 +#+#+#+#+#+   +#+            #
#    Created: 2024/04/23 23:34:22 by yhwang            #+#    #+#              #
#    Updated: 2024/05/19 15:16:28 by yhwang           ###   ########.fr        #
#                                                                              #
# **************************************************************************** #

NAME = ft_kalman
CPP = c++

SRC_DIR = ./srcs/
INCS_DIR = ./incs/

CPPFLAGS = -Wall -Wextra -Werror -std=c++17 -I$(INCS_DIR)
LDFLAGS = -lGLEW -lGL -lglfw -lGLU -pthread -ldl -lm

RM = rm -f

FILES = $(SRC_DIR)main \
		$(SRC_DIR)Parse \
		$(SRC_DIR)ServerUtils \
		$(SRC_DIR)FilterUtils

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
