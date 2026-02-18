# ╔══════════════════════════════════════════════════════════════════════════════╗
# ║  Makefile — All commands use docker compose                                ║
# ╚══════════════════════════════════════════════════════════════════════════════╝

include .env
export

COMPOSE_SIM    := -f compose/docker-compose.simulation.yml
COMPOSE_JETSON := -f compose/docker-compose.jetson.yml
COMPOSE_RPI5   := -f compose/docker-compose.rpi5.yml

COMPOSE_ENV_FILES := --env-file .env
ifneq ("$(wildcard .env.auth)","")
COMPOSE_ENV_FILES += --env-file .env.auth
endif

# ── Build ────────────────────────────────────────────────────────────────────

build-sim:
	docker compose $(COMPOSE_ENV_FILES) $(COMPOSE_SIM) build

build-jetson:
	docker compose $(COMPOSE_ENV_FILES) $(COMPOSE_JETSON) build

build-rpi5:
	docker compose $(COMPOSE_ENV_FILES) $(COMPOSE_RPI5) build

build-all: build-sim build-jetson build-rpi5

rebuild-sim:
	docker compose $(COMPOSE_ENV_FILES) $(COMPOSE_SIM) build --no-cache

rebuild-jetson:
	docker compose $(COMPOSE_ENV_FILES) $(COMPOSE_JETSON) build --no-cache

rebuild-rpi5:
	docker compose $(COMPOSE_ENV_FILES) $(COMPOSE_RPI5) build --no-cache

rebuild-all: rebuild-sim rebuild-jetson rebuild-rpi5

# ── Run ──────────────────────────────────────────────────────────────────────

sim:
	xhost +local:docker 2>/dev/null || true
	docker compose $(COMPOSE_ENV_FILES) $(COMPOSE_SIM) up

jetson:
	docker compose $(COMPOSE_ENV_FILES) $(COMPOSE_JETSON) up

rpi5:
	docker compose $(COMPOSE_ENV_FILES) $(COMPOSE_RPI5) up

# ── Dev shells (workspace is always mounted) ─────────────────────────────────

dev:
	xhost +local:docker 2>/dev/null || true
	docker compose $(COMPOSE_ENV_FILES) $(COMPOSE_SIM) run --rm sim bash

dev-jetson:
	docker compose $(COMPOSE_ENV_FILES) $(COMPOSE_JETSON) run --rm aircraft bash

dev-rpi5:
	docker compose $(COMPOSE_ENV_FILES) $(COMPOSE_RPI5) run --rm aircraft bash

# ── Shell into running container ─────────────────────────────────────────────

shell-sim:
	xhost +local:docker 2>/dev/null || true
	docker compose $(COMPOSE_ENV_FILES) $(COMPOSE_SIM) up -d sim
	docker compose $(COMPOSE_ENV_FILES) $(COMPOSE_SIM) exec sim bash

shell-jetson:
	docker compose $(COMPOSE_ENV_FILES) $(COMPOSE_JETSON) exec aircraft bash

shell-rpi5:
	docker compose $(COMPOSE_ENV_FILES) $(COMPOSE_RPI5) exec aircraft bash

# ── Cleanup ──────────────────────────────────────────────────────────────────

down:
	docker compose $(COMPOSE_ENV_FILES) $(COMPOSE_SIM) down 2>/dev/null || true
	docker compose $(COMPOSE_ENV_FILES) $(COMPOSE_JETSON) down 2>/dev/null || true
	docker compose $(COMPOSE_ENV_FILES) $(COMPOSE_RPI5) down 2>/dev/null || true

clean: down
	docker image rm -f \
		$(PROJECT_NAME)-simulation:latest \
		$(PROJECT_NAME)-jetson:latest \
		$(PROJECT_NAME)-rpi5:latest \
		2>/dev/null || true

nuke: clean
	docker system prune -af --volumes

# ── Info ─────────────────────────────────────────────────────────────────────

info:
	@echo "Project:      $(PROJECT_NAME)"
	@echo "ROS_DISTRO:   $(ROS_DISTRO)"
	@echo "PX4_VERSION:  $(PX4_VERSION)"
	@echo "DRONE_ID:     $(DRONE_ID)"
	@echo "ROS_LOCALHOST_ONLY: $(ROS_LOCALHOST_ONLY)"
	@echo "ROS_WS:       $(ROS_WS)"
	@echo "User:         $(CONTAINER_USER) ($(USER_UID):$(USER_GID))"

.PHONY: build-sim build-jetson build-rpi5 build-all \
        rebuild-sim rebuild-jetson rebuild-rpi5 rebuild-all \
        sim jetson rpi5 dev dev-jetson dev-rpi5 \
        shell-sim shell-jetson shell-rpi5 \
        down clean nuke info
