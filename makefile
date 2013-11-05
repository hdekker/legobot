PRJ_ROOT_DIR := $(shell pwd)
ANOTHER_PROJECT_HOME := /usr/share/codesourcery/
YET_ANOTHER_PROJECT_HOME := /usr/share/codesourcery/lib/gcc/arm-none-linux-gnueabi/4.7.3

# Target Name
TARGET_NAME := program
PROJECT_TYPE := app

#Version Number
MAJOR := 0
MINOR := 01

#Include directories
INCLUDES_SRC = -I$(ANOTHER_PROJECT_HOME) -I$(YET_ANOTHER_PROJECT_HOME)/include

#Libraries
LIBRARIES := -ldl -L$(ANOTHER_PROJECT_HOME) -lpthread -lm

#Add more App Object files
OBJFILES :=

#Add more Dependency files
#These files are generated automatically. This is only needed to delete them all
DEPFILES :=

# Doxygen Configuration file
DOXYGEN_CONFIG := doxyfile

# ----------------------------------------------------------------------------
# DO NOT MODIFY BELOW
# ----------------------------------------------------------------------------
#Set for Debug mode or non-debug
#BUILD := rel
BUILD := devel

#Set Architecture
ARCH := arm

#Compilers
ifeq ($(ARCH),arm)
CC := /usr/share/codesourcery/bin/arm-none-linux-gnueabi-g++
GCC := /usr/share/codesourcery/bin/arm-none-linux-gnueabi-gcc
else
CC := g++
GCC := gcc
endif

#Set the file extension and prefix if lib
ifeq ($(PROJECT_TYPE),lib)
TARGET_PREFIX = lib
TARGET_POSTFIX = .so
CC += -shared
else
TARGET_PREFIX =
TARGET_POSTFIX =
endif

#BuildTool flags
ifeq ($(BUILD),devel)
DEBUGFLAGS_Assembler = -g
DEBUGFLAGS_C++-Compiler = -g -O0 -fno-omit-frame-pointer -pipe -Wall -DDEBUG
DEBUGFLAGS_C++-Linker = -g
DEBUGFLAGS_C-Compiler = -g -O0 -std=gnu99 -fno-omit-frame-pointer -pipe -Wall
DEBUGFLAGS_C-Linker = -g
DEBUGFLAGS_Librarian = -g
DEBUGFLAGS_Shared-Library-Linker = -g
TARGET_VER = $(TARGET_PREFIX)$(TARGET_NAME)_$(MAJOR).$(MINOR)_debug$(TARGET_POSTFIX)
TARGET = $(TARGET_PREFIX)$(TARGET_NAME)$(TARGET_POSTFIX)
else
DEBUGFLAGS_Assembler =
DEBUGFLAGS_C++-Compiler = -O3 -Wall -Werror
DEBUGFLAGS_C++-Linker =
DEBUGFLAGS_C-Compiler = -O2 -fomit-frame-pointer -D__USE_STRING_INLINES -pipe -Wall
DEBUGFLAGS_C-Linker =
DEBUGFLAGS_Librarian =
DEBUGFLAGS_Shared-Library-Linker =
TARGET_VER = $(TARGET_PREFIX)$(TARGET_NAME)_$(MAJOR).$(MINOR)$(TARGET_POSTFIX)
TARGET = $(TARGET_PREFIX)$(TARGET_NAME)$(TARGET_POSTFIX)
endif

#Global Build Macros
DEFINES +=

#Global Include directories
INCLUDES_SRC += -I$(PRJ_ROOT_DIR)/include

#Global Libraries
LIBRARIES +=

#Global App Object files
OBJFILES += $(patsubst %.cpp,%.o,$(wildcard *.cpp)) $(patsubst %.cpp,%.o,$(wildcard src/*.cpp))
OBJFILES += $(patsubst %.c,%.o,$(wildcard *.c)) $(patsubst %.c,%.o,$(wildcard src/*.c))

#Global Dependency files
#These files are generated automatically. This is only needed to delete them all
DEPFILES += $(patsubst %.cpp,%.d,$(wildcard *.cpp)) $(patsubst %.cpp,%.d,$(wildcard src/*.cpp))
DEPFILES += $(patsubst %.c,%.d,$(wildcard *.c)) $(patsubst %.c,%.d,$(wildcard src/*.c))

#Compile object files
%.o: %.cpp
	$(CC) $(DEBUGFLAGS_C++-Compiler) $(INCLUDES_SRC) $(DEFINES) -c -fmessage-length=0 -Wold-style-cast -Woverloaded-virtual -o $@ $<

#Compile object files
#May need to replace $(CC) with $(GCC)
%.o: %.c
	$(GCC) $(DEBUGFLAGS_C-Compiler) $(INCLUDES_SRC) $(DEFINES) -c -fmessage-length=0 -o $@ $<

#Build application
#Create two version of the file so when including the library you do not have to worry about a version
all: $(OBJFILES)
	$(GCC) -o $(TARGET) $(OBJFILES) $(LIBRARIES)
	#$(CC) -o $(TARGET_VER) $(OBJFILES) $(LIBRARIES)

#Clean files
clean:
	rm -f $(OBJFILES) rm -f $(DEPFILES) rm -f $(TARGET) rm -rf $(TARGET_VER)

#Make Doxygen files
doxygen:
	cd docs && doxygen $(DOXYGEN_CONFIG)

#Clean Doxygen files
clean_doxygen:
	rm -rf docs/html
	rm -rf docs/latex

#Help option
help:
	@echo
	@echo " make [target] [OPTIONS]"
	@echo
	@echo " Targets:"
	@echo " all Builds the app. This is the default target."
	@echo " clean Clean all the objects, apps and dependencies."
	@echo " help Prints this message."
	@echo " doxygen Create doxygen files"
	@echo " doxygen_clean Clean doxygen files"
	@echo " Options:"
	@echo " BUILD=rel Build a release build (default)"
	@echo " BUILD=devel Build a debug build"
	@echo " "
	@echo " ARCH=arm Build an ARM build (default)"
	@echo " ARCH=x86 Build an X86 build"
	@echo " "
	@echo
