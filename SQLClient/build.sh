#sudo apt-get install libmariadb-dev-compat
gcc central_client.c -o central_client -lpthread -lmysqlclient -lm
