CC := gcc
CFLAGS += -D_REENTRANT -pthread
LDLIBS += -pthread

TARGET_SRV := iot_server
OBJECT_SRV := $(TARGET_SRV).o

.PHONY: all clean

all: $(TARGET_SRV)

$(TARGET_SRV): $(OBJECT_SRV)
	$(CC) $(LDFLAGS) -o $@ $^ $(LDLIBS)

%.o: %.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJECT_SRV) $(TARGET_SRV)
