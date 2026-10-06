#CC:=arm-linux-gcc
CC:=gcc

TARGET_SRV=iot_server
OBJECT_SRV=$(TARGET_SRV).o 

#TARGET_CLN=iot_client
#OBJECT_CLN=$(TARGET_CLN).o

#TARGET_SQL=sql_client
#OBJECT_SQL=$(TARGET_SQL).o

#LDFLAGS=-D_REENTRANT -pthread -lmysqlclient
LDFLAGS=-D_REENTRANT -pthread

all : $(TARGET_SRV) $(TARGET_CLN) $(TARGET_SQL)

$(TARGET_SRV):$(OBJECT_SRV)
	$(CC) -o $@ $(OBJECT_SRV) $(LDFLAGS)
#$(TARGET_CLN):$(OBJECT_CLN)
#	$(CC) -o $@ $(OBJECT_CLN) $(LDFLAGS)
#$(TARGET_SQL):$(OBJECT_SQL)
#	$(CC) -o $@ $(OBJECT_SQL) -D_REENTRANT -pthread -lmysqlclient
%.o:%.c
	$(CC) -c -o $@ $<
clean:
	rm -f *.o $(TARGET_SRV) #$(TARGET_CLN) $(TARGET_SQL)
