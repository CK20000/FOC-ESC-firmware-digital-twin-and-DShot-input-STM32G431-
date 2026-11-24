TARGET = foc_g431
BUILD_DIR = build_hardware
OBJ_DIR = $(BUILD_DIR)/obj

PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size

MCU = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard

C_DEFS = -DUSE_HAL_DRIVER -DSTM32G431xx
C_INCLUDES = \
-IInc \
-ISTM32CubeG4/Projects/NUCLEO-G431KB/Templates/Inc \
-ISTM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Inc \
-ISTM32CubeG4/Drivers/CMSIS/Device/ST/STM32G4xx/Include \
-ISTM32CubeG4/Drivers/CMSIS/Include \
-ISTM32CubeG4/Middlewares/ST/STM32_USB_Device_Library/Core/Inc \
-ISTM32CubeG4/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) -O2 -Wall -fdata-sections -ffunction-sections -fno-math-errno
ASFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) -O2 -Wall

LDSCRIPT = foc_g431.ld
LIBS = -lc -lm -lnosys
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# ----------- OBJECT LISTS -----------
USER_OBJS = \
$(OBJ_DIR)/main.o \
$(OBJ_DIR)/foc_core.o \
$(OBJ_DIR)/foc_dshot.o \
$(OBJ_DIR)/system_stm32g4xx.o

USB_SRCS = \
STM32CubeG4/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c \
STM32CubeG4/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c \
STM32CubeG4/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c \
STM32CubeG4/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c

USB_OBJS = $(patsubst %.c,$(OBJ_DIR)/%.o,$(notdir $(USB_SRCS)))

HAL_SRCS = $(wildcard STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Src/*.c)
HAL_OBJS = $(patsubst STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Src/%.c,$(OBJ_DIR)/%.o,$(HAL_SRCS))

ASM_OBJ = $(OBJ_DIR)/startup_stm32g431kbtx.o

OBJECTS = $(USER_OBJS) $(USB_OBJS) $(HAL_OBJS) $(ASM_OBJ)

# ----------- RULES -----------
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).bin

$(OBJ_DIR)/main.o: Src/main.c Makefile | $(OBJ_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(OBJ_DIR)/foc_core.o: Src/foc_core.c Makefile | $(OBJ_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(OBJ_DIR)/foc_dshot.o: Src/foc_dshot.c Makefile | $(OBJ_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(OBJ_DIR)/usb_cdc.o: Src/usb_cdc.c Makefile | $(OBJ_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(OBJ_DIR)/system_stm32g4xx.o: STM32CubeG4/Projects/NUCLEO-G431KB/Templates/Src/system_stm32g4xx.c Makefile | $(OBJ_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(OBJ_DIR)/%.o: STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Src/%.c Makefile | $(OBJ_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

vpath %.c STM32CubeG4/Middlewares/ST/STM32_USB_Device_Library/Core/Src
vpath %.c STM32CubeG4/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src

$(OBJ_DIR)/%.o: %.c Makefile | $(OBJ_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(OBJ_DIR)/startup_stm32g431kbtx.o: STM32CubeG4/Projects/NUCLEO-G431KB/Templates/STM32CubeIDE/Example/Startup/startup_stm32g431kbtx.s Makefile | $(OBJ_DIR)
	$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(CP) -O binary -S $< $@

# Directory creation
$(BUILD_DIR):
	mkdir -p $@

$(OBJ_DIR): | $(BUILD_DIR)
	mkdir -p $@

clean:
	rm -rf $(BUILD_DIR)