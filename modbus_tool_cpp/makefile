#ARCH=ARM
OTHERSLIBPWD=/home/usrc

ifeq ($(ARCH),ARM)
CROSS=arm-linux-
endif

SRC:=$(shell ls *.c)
SRC+=$(shell ls *.cpp)

CFLAG:=-DMEMWATCH -DMW_STDIO
LDFLAG:=-static

ifeq ($(ARCH),ARM)
IPATH:=-I$(OTHERSLIBPWD)/sqlite-3.7.3/output/arm/include
else
IPATH:=-I$(OTHERSLIBPWD)/sqlite-3.7.3/output/ubuntu/include
endif

ifeq ($(ARCH),ARM)
LPATH:=-L$(OTHERSLIBPWD)/sqlite-3.7.3/output/arm/lib
else
LPATH:=-L$(OTHERSLIBPWD)/sqlite-3.7.3/output/ubuntu/lib
endif

ifeq ($(ARCH),ARM)
LIBS+=-lpthread
else
LIBS+=-lpthread
endif

ifeq ($(ARCH),ARM)
TARGET:=modbus.bin
else
TARGET:=modbus.bin
endif

$(TARGET) : $(SRC)
	$(CROSS)g++ $(CFLAG) -o $(TARGET) $^ $(LPATH) $(IPATH) $(LIBS) $(LDFLAG)
	#push modbus.bin /system/bin

clean:
	rm -f  *.bin  *.dis  *.elf  *.o
