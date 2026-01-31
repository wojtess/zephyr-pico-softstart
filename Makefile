APP_DIR    := $(CURDIR)
WS_DIR     := $(APP_DIR)/.zephyr
BUILD_DIR  := $(APP_DIR)/build
VENV_DIR   := $(APP_DIR)/venv

BOARD      ?= rpi_pico

WEST       := $(VENV_DIR)/bin/west
PIP        := $(VENV_DIR)/bin/pip
PYTHON     := $(VENV_DIR)/bin/python3

UF2        := $(BUILD_DIR)/zephyr/zephyr.uf2

.PHONY: deps build flash clean distclean deepclean docs doxygen

# ---------------- deps (runs once; file-stamped) ----------------
deps: $(WS_DIR)

$(VENV_DIR):
	python3 -m venv "$(VENV_DIR)";
	$(PIP) install -U pip
	$(PIP) install -U west
	$(PIP) install -r scripts/requirements.txt


$(WS_DIR): | $(VENV_DIR) 
	mkdir -p $(WS_DIR)

	@if [ ! -d "$(WS_DIR)/.west" ]; then \
		cd "$(APP_DIR)" && "$(WEST)" init "$(WS_DIR)"; \
	fi
	cd "$(WS_DIR)" && "$(WEST)" update
	cd "$(WS_DIR)" && "$(WEST)" zephyr-export
	cd "$(WS_DIR)" && "$(WEST)" packages pip --install


# ---------------- build / flash (always run same commands) ----------------
build: deps
	@cd "$(WS_DIR)" && "$(WEST)" build -b "$(BOARD)" -s "$(APP_DIR)" -d "$(BUILD_DIR)"

flash: deps
	@cd "$(WS_DIR)" && "$(WEST)" build -b "$(BOARD)" -s "$(APP_DIR)" -d "$(BUILD_DIR)"
	@picotool load -x "$(UF2)"

# ---------------- housekeeping ----------------
clean:
	rm -rf "$(BUILD_DIR)"

distclean deepclean: clean
	rm -rf $(WS_DIR)
	rm -rf $(VENV_DIR)

# ---------------- documentation ----------------
docs doxygen:
	@echo "Generating Doxygen documentation..."
	@doxygen Doxyfile
	@echo "Documentation generated in docs/html/index.html"

