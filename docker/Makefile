# ╔══════════════════════════════════════════════════════════════════════════════╗
# ║  Makefile — All commands use docker compose                                ║
# ╚══════════════════════════════════════════════════════════════════════════════╝

include .env
export

COMPOSE_SIM    := -f compose/docker-compose.simulation.yml
COMPOSE_MULTI  := -f compose/docker-compose.multi-sitl.yml
COMPOSE_JETSON := -f compose/docker-compose.jetson.yml
COMPOSE_RPI5   := -f compose/docker-compose.rpi5.yml

COMPOSE_ENV_FILES := --env-file .env $(if $(wildcard .env.local),--env-file .env.local,)

# ── Build ────────────────────────────────────────────────────────────────────

build-sim:
	docker compose $(COMPOSE_ENV_FILES) $(COMPOSE_SIM) build

build-jetson:
	docker compose $(COMPOSE_ENV_FILES) $(COMPOSE_JETSON) build

build-rpi5:
	docker compose $(COMPOSE_ENV_FILES) $(COMPOSE_RPI5) build

build-all: build-sim build-jetson build-rpi5

build-sim-nocache:
	docker compose $(COMPOSE_ENV_FILES) $(COMPOSE_SIM) build --no-cache

build-jetson-nocache:
	docker compose $(COMPOSE_ENV_FILES) $(COMPOSE_JETSON) build --no-cache

build-rpi5-nocache:
	docker compose $(COMPOSE_ENV_FILES) $(COMPOSE_RPI5) build --no-cache

build-all-nocache: build-sim-nocache build-jetson-nocache build-rpi5-nocache

# ── Run ──────────────────────────────────────────────────────────────────────

generate-multi:
	python3 scripts/generate_multi_sitl.py --num-uavs $(NUM_UAVS)

multi-sitl: generate-multi
	xhost +local:docker 2>/dev/null || true
	docker compose $(COMPOSE_ENV_FILES) $(COMPOSE_MULTI) up

multi-sitl-down:
	docker compose $(COMPOSE_ENV_FILES) $(COMPOSE_MULTI) down

multi-dev:
	xhost +local:docker 2>/dev/null || true
	docker compose $(COMPOSE_ENV_FILES) $(COMPOSE_MULTI) run --rm sim bash

shell-gcs:
	xhost +local:docker 2>/dev/null || true
	docker compose $(COMPOSE_ENV_FILES) $(COMPOSE_MULTI) up -d gcs
	docker compose $(COMPOSE_ENV_FILES) $(COMPOSE_MULTI) exec gcs bash

tmux-gcs:
	docker compose $(COMPOSE_ENV_FILES) $(COMPOSE_MULTI) exec gcs tmux attach -t gcs

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
	docker compose $(COMPOSE_ENV_FILES) $(COMPOSE_JETSON) up -d aircraft
	docker compose $(COMPOSE_ENV_FILES) $(COMPOSE_JETSON) exec aircraft bash

shell-rpi5:
	docker compose $(COMPOSE_ENV_FILES) $(COMPOSE_RPI5) up -d aircraft
	docker compose $(COMPOSE_ENV_FILES) $(COMPOSE_RPI5) exec aircraft bash

# ── Cleanup ──────────────────────────────────────────────────────────────────

down:
	docker compose $(COMPOSE_ENV_FILES) $(COMPOSE_SIM) down 2>/dev/null || true
	docker compose $(COMPOSE_ENV_FILES) $(COMPOSE_MULTI) down 2>/dev/null || true
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

# ── Flight auto-start (systemd) ────────────────────────────────────────────

PLATFORM ?= jetson
SERVICE_FILE := /etc/systemd/system/peregrine.service

install-service:
	@echo "Installing peregrine.service for platform: $(PLATFORM)"
	sudo cp config/peregrine.service $(SERVICE_FILE)
	@# Patch compose file path for platform
	sudo sed -i 's/docker-compose.jetson.yml/docker-compose.$(PLATFORM).yml/g' $(SERVICE_FILE)
	@# Patch working directory to current location
	sudo sed -i 's|WorkingDirectory=.*|WorkingDirectory=$(CURDIR)|g' $(SERVICE_FILE)
	@# Patch .env.local path to current location
	sudo sed -i 's|EnvironmentFile=-.*|EnvironmentFile=-$(CURDIR)/.env.local|g' $(SERVICE_FILE)
	sudo systemctl daemon-reload
	@echo "Service installed. Enable with: sudo systemctl enable --now peregrine"

uninstall-service:
	sudo systemctl disable --now peregrine 2>/dev/null || true
	sudo rm -f $(SERVICE_FILE)
	sudo systemctl daemon-reload
	@echo "Service removed."

enable-flight:
	sudo systemctl enable --now peregrine

disable-flight:
	sudo systemctl disable --now peregrine

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
        build-sim-nocache build-jetson-nocache build-rpi5-nocache build-all-nocache \
	        sim jetson rpi5 generate-multi multi-sitl multi-sitl-down multi-dev \
	        shell-gcs tmux-gcs \
	        dev dev-jetson dev-rpi5 \
        shell-sim shell-jetson shell-rpi5 \
        install-service uninstall-service enable-flight disable-flight \
        down clean nuke info
