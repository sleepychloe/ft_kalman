# **************************************************************************** #
#                                                                              #
#                                                         :::      ::::::::    #
#    Makefile                                           :+:      :+:    :+:    #
#                                                     +:+ +:+         +:+      #
#    By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+         #
#                                                 +#+#+#+#+#+   +#+            #
#    Created: 2024/05/19 14:24:38 by yhwang            #+#    #+#              #
#    Updated: 2024/06/15 17:21:50 by yhwang           ###   ########.fr        #
#                                                                              #
# **************************************************************************** #

RESET		:= $(shell tput -Txterm sgr0)
YELLOW		:= $(shell tput -Txterm setaf 3)
BLUE		:= $(shell tput -Txterm setaf 6)

COMPOSE_FILE	= ./docker-compose.yml

ENTROPY		?= 0
DURATION	?= 90
FILTERSPEED	?= 0
GRAPH		?= 1
NOISE		?= 1
ADAPTIVE	?= 0

# allow overriding variables via command line
ifneq ($(d),)
	DURATION = $(d)
endif

ifneq ($(e),)
	ENTROPY = $(e)
endif

ifneq ($(s),)
	FILTERSPEED = $(s)
endif

ifneq ($(g),)
	GRAPH = $(g)
endif

ifneq ($(n),)
	NOISE = $(n)
endif

ifneq ($(adaptive),)
	ADAPTIVE = $(adaptive)
endif

all: up

up:
	@ENTROPY=${ENTROPY} \
		DURATION=$(DURATION) \
		FILTERSPEED=$(FILTERSPEED) \
		GRAPH=$(GRAPH) \
		NOISE=$(NOISE) \
		ADAPTIVE=$(ADAPTIVE) \
		docker-compose -f $(COMPOSE_FILE) up --build -d
	@echo "$(YELLOW)Containers succesfully created and started$(RESET)"
	@docker attach kalman
ifneq ($(FILTERSPEED), 0)
	@make logs | grep imu-sensor-stream | tail -n 1
endif

list:
	@echo "$(BLUE)LIST OF CONTAINERS:$(RESET)"
	@docker ps -a
	@echo ""

	@echo "$(BLUE)LIST OF VOLUMES:$(RESET)"
	@docker volume ls
	@echo ""

	@echo "$(BLUE)LIST OF IMAGES:$(RESET)"
	@docker image ls
	@echo ""

	@echo "$(BLUE)LIST OF NETWORKS:$(RESET)"
	@docker network ls
	@echo ""

logs:
	@echo "$(BLUE)OUTPUTS OF CONTAINERS:$(RESET)"
	@docker-compose -f $(COMPOSE_FILE) logs

stop:
ifneq ($(shell docker ps -a | wc -l), 1)
	@echo "$(BLUE)Stopping container..$(RESET)"
	@docker-compose -f $(COMPOSE_FILE) stop -t1
	@echo "$(YELLOW)Containers succesfully stopped$(RESET)"
else
	@echo "$(YELLOW)There is no container to stop$(RESET)"
endif

restart:
ifneq ($(shell docker ps -a | wc -l), 1)
	@echo "$(BLUE)Restarting container..$(RESET)"
	@docker-compose -f $(COMPOSE_FILE) restart
	@echo "$(YELLOW)Containers succesfully restarted$(RESET)"
else
	@echo "$(YELLOW)There is no container to restart$(RESET)"
endif

fclean:
	@echo "$(BLUE)Removing everything..$(RESET)"
	@docker-compose -f $(COMPOSE_FILE) stop -t1
	@echo "$(YELLOW)Containers succesfully stopped$(RESET)"

ifneq ($(shell docker container ls -a | wc -l), 1)
	@docker container prune -f
	@echo "$(YELLOW)Containers successfully removed$(RESET)"
else
	@echo "$(YELLOW)There is no container to remove$(RESET)"
endif

ifneq ($(shell docker volume ls | wc -l), 1)
	@docker volume rm $(shell docker volume ls | awk 'NR>1' | cut -c 21-40) -f
	@echo "$(YELLOW)Volumes successfully removed$(RESET)"
else
	@echo "$(YELLOW)There is no volume to remove$(RESET)"
endif

ifneq ($(shell docker image ls | wc -l), 1)
	@docker rmi $(shell docker images -q) -f
	@echo "$(YELLOW)Images succesfully removed$(RESET)"
else
	@echo "$(YELLOW)There is no image to remove$(RESET)"
endif

ifneq ($(shell docker network ls | grep kalman | wc -l), 0)
	@docker network prune -f
	@echo "$(YELLOW)Networks successfully removed$(RESET)"
else
	@echo "$(YELLOW)There is no network to remove$(RESET)"
endif

	@echo "$(BLUE)Everything is successfully removed!$(RESET)"

.PHONY: all up list logs stop fclean
