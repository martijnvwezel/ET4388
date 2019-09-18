EXEC=tcpserver
CFLAGS= -lm  -lpthread -Wall
INC=-I header/  



helloworld: tcp_command.o $(EXEC).o
	gcc -o  $(EXEC) obj/$(EXEC).o $(CFLAGS) obj/tcp_command.o

tcp_command.o: src/tcp_command.c 
	gcc -c src/tcp_command.c $(INC) 
	mv *.o obj/
$(EXEC).o: src/$(EXEC).c -lm
	gcc -c  src/$(EXEC).c  $(INC) $(CFLAGS)
	mv *.o obj/

clean:
	rm obj/*.o ./$(EXEC)
 
 