# ╔══════════════════════════════════════════════════════════════════════════════╗
# ║  Makefile — All commands use docker compose                                ║
# ╚══════════════════════════════════════════════════════════════════════════════╝

include .env
export

COMPOSE_SIM    := -f compose/docker-compose.simulation.yml
COMPOSE_JETSON := -f compose/docker-compose.jetson.yml
COMPOSE_RPI5   := -f compose/docker-compose.rpi5.yml

# ── Build ────────────────────────────────────────────────────────────────────

build-sim:
	docker compose $(COMPOSE_SIM) build

build-jetson:
	docker compose $(COMPOSE_JETSON) build

build-rpi5:
	docker compose $(COMPOSE_RPI5) build

build-all: build-sim build-jetson build-rpi5

# ── Run ──────────────────────────────────────────────────────────────────────

sim:
	xhost +local:docker 2>/dev/null || true
	docker compose $(COMPOSE_SIM) up

jetson:
	docker compose $(COMPOSE_JETSON) up

rpi5:
	docker compose $(COMPOSE_RPI5) up

# ── Dev shells (workspace is always mounted) ─────────────────────────────────

dev:
	xhost +local:docker 2>/dev/null || true
	docker compose $(COMPOSE_SIM) run --rm sim bash

dev-jetson:
	docker compose $(COMPOSE_JETSON) run --rm aircraft bash

dev-rpi5:
	docker compose $(COMPOSE_RPI5) run --rm aircraft bash

# ── Shell into running container ─────────────────────────────────────────────

shell-sim:
	docker exec -it $(PROJECT_NAME)-sim bash

shell-jetson:
	docker exec -it $(PROJECT_NAME)-aircraft-$(DRONE_ID) bash

shell-rpi5:
	docker exec -it $(PROJECT_NAME)-aircraft-$(DRONE_ID) bash

# ── Cleanup ──────────────────────────────────────────────────────────────────

down:
	docker compose $(COMPOSE_SIM) down 2>/dev/null || true
	docker compose $(COMPOSE_JETSON) down 2>/dev/null || true
	docker compose $(COMPOSE_RPI5) down 2>/dev/null || true

clean: down
	docker image rm -f \
		$(REGISTRY)/$(PROJECT_NAME)-simulation:latest \
		$(REGISTRY)/$(PROJECT_NAME)-jetson:latest \
		$(REGISTRY)/$(PROJECT_NAME)-rpi5:latest \
		2>/dev/null || true

nuke: clean
	docker system prune -af --volumes

# ── Info ─────────────────────────────────────────────────────────────────────

info:
	@echo "Project:      $(PROJECT_NAME)"
	@echo "Registry:     $(REGISTRY)"
	@echo "ROS_DISTRO:   $(ROS_DISTRO)"
	@echo "PX4_VERSION:  $(PX4_VERSION)"
	@echo "DRONE_ID:     $(DRONE_ID)"
	@echo "ROS_WS:       $(ROS_WS)"
	@echo "User:         $(CONTAINER_USER) ($(USER_UID):$(USER_GID))"

.PHONY: build-sim build-jetson build-rpi5 build-all \
        sim jetson rpi5 dev dev-jetson dev-rpi5 \
        shell-sim shell-jetson shell-rpi5 \
        down clean nuke info
