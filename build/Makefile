SDK_PATH:=$(shell pwd)/../ARDroneLib
PC_TARGET=yes
USE_LINUX=yes

ifdef MYKONOS
   include $(ARDRONE_CUSTOM_CONFIG)
   include $(ARDRONE_BUILD_CONFIG)
else
   include $(SDK_PATH)/Soft/Build/custom.makefile
   include $(SDK_PATH)/Soft/Build/config.makefile
endif

ARDRONE_TARGET_DIR=$(shell pwd)

TARGET=controllertest

SRC_DIR:=$(shell pwd)/../src

# Define application source files
GENERIC_BINARIES_SOURCE_DIR:=$(SRC_DIR)

GENERIC_BINARIES_COMMON_SOURCE_FILES+=			\
   Navdata/navdata.c    \
   Video/video_processing.c \
   Video/vision_stage.c \
   Control/joypad_control.c \
   Control/automated_control.c 

GENERIC_INCLUDES+=					\
	$(SRC_DIR) \
	$(LIB_DIR) \
	$(SDK_PATH)/Soft/Common \
	$(SDK_PATH)/Soft/Lib

GENERIC_TARGET_BINARIES_PREFIX=

GENERIC_TARGET_BINARIES_DIR=$(ARDRONE_TARGET_DIR)

GENERIC_BINARIES_SOURCE_ENTRYPOINTS+=			\
   main.c

GENERIC_INCLUDES:=$(addprefix -I,$(GENERIC_INCLUDES))

GENERIC_LIB_PATHS=-L$(GENERIC_TARGET_BINARIES_DIR)
GENERIC_LIBS=-lpc_ardrone -lgtk-x11-2.0 -lrt -lSDL -lm
GENERIC_LIBS+=$(shell pkg-config --libs opencv)

SDK_FLAGS+="USE_APP=yes"
SDK_FLAGS+="APP_ID=linux_sdk_demo"

export GENERIC_CFLAGS
export GENERIC_LIBS
export GENERIC_LIB_PATHS
export GENERIC_INCLUDES
export GENERIC_BINARIES_SOURCE_DIR
export GENERIC_BINARIES_COMMON_SOURCE_FILES
export GENERIC_TARGET_BINARIES_PREFIX
export GENERIC_TARGET_BINARIES_DIR
export GENERIC_BINARIES_SOURCE_ENTRYPOINTS

# Bug fix ...
export GENERIC_LIBRARY_SOURCE_DIR=$(GENERIC_BINARIES_SOURCE_DIR)


.PHONY: $(TARGET) build_libs

all: build_libs $(TARGET)

$(TARGET):
	@$(MAKE) -C $(SDK_PATH)/VP_SDK/Build $(TMP_SDK_FLAGS) $(SDK_FLAGS) $(MAKECMDGOALS) USE_LINUX=yes
	mv $(ARDRONE_TARGET_DIR)/main $(TARGET)
#	mv $(TARGET) $(ARDRONE_TARGET_DIR)/

$(MAKECMDGOALS): build_libs
	@$(MAKE) -C $(SDK_PATH)/VP_SDK/Build $(TMP_SDK_FLAGS) $(SDK_FLAGS) $(MAKECMDGOALS) USE_LINUX=yes

build_libs:
	@$(MAKE) -C $(SDK_PATH)/Soft/Build $(TMP_SDK_FLAGS) $(SDK_FLAGS) $(MAKECMDGOALS) USE_LINUX=yes

